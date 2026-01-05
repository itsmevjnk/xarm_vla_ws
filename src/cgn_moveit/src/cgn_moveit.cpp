#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <cgn_msgs/srv/infer.hpp>

#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <future>
#include <cmath>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "cgn_moveit", rclcpp::NodeOptions()
    );

    /* TF2 buffer and listener */
    auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);

    /* spin node in background */
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    /* MoveIt interface */
    node->declare_parameter("move_group", "move_group");
    std::string group_name = node->get_parameter("move_group").as_string();
    node->declare_parameter("ee_link", "");
    std::string ee_link = node->get_parameter("ee_link").as_string();
    node->declare_parameter("root_frame", "link_base");
    std::string root_frame = node->get_parameter("root_frame").as_string();
    moveit::planning_interface::MoveGroupInterface move_group(node, group_name);
    // Basic planning configuration / robustness defaults
    move_group.setPoseReferenceFrame(root_frame);
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.05);

    // If ee_link param wasn't provided, fall back to MoveIt end-effector link
    if (ee_link.empty()) {
        ee_link = move_group.getEndEffectorLink();
        RCLCPP_INFO(node->get_logger(), "ee_link param not set; using MoveIt end-effector link: '%s'", ee_link.c_str());
    } else {
        RCLCPP_INFO(node->get_logger(), "using ee_link from param: '%s'", ee_link.c_str());
    }

    auto grasp_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("grasp_target", 10);

    /* classifier index */
    node->declare_parameter("mask_cls", 1);
    int cls = node->get_parameter("mask_cls").as_int();

    /* get camera info message */
    sensor_msgs::msg::CameraInfo::SharedPtr info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
    RCLCPP_INFO(node->get_logger(), "waiting for camera info");
    bool success = rclcpp::wait_for_message(*info_msg, node, "camera_info");
    if (!success) {
        RCLCPP_ERROR(node->get_logger(), "cannot receive camera info - exiting");
        rclcpp::shutdown();
        return 1;
    }

    /* get image mask (from yolo_instseg::yolo_node) */
    sensor_msgs::msg::Image::SharedPtr mask_msg = std::make_shared<sensor_msgs::msg::Image>();
    RCLCPP_INFO(node->get_logger(), "waiting for segmentation mask");
    success = rclcpp::wait_for_message(*mask_msg, node, "mask");
    if (!success) {
        RCLCPP_ERROR(node->get_logger(), "cannot receive segmentation mask - exiting");
        rclcpp::shutdown();
        return 1;
    }

    /* get depth image */
    sensor_msgs::msg::Image::SharedPtr depth_msg = std::make_shared<sensor_msgs::msg::Image>();
    RCLCPP_INFO(node->get_logger(), "waiting for depth image");
    success = rclcpp::wait_for_message(*depth_msg, node, "depth", 1s); // timeout to ensure synchronisation with mask
    if (!success) {
        RCLCPP_ERROR(node->get_logger(), "cannot receive depth image - exiting");
        rclcpp::shutdown();
        return 1;
    }

    /* filter mask */
    cv::Mat mask_img = cv_bridge::toCvShare(mask_msg, sensor_msgs::image_encodings::TYPE_8UC1)->image;
    mask_img = (mask_img == cls) & 1; // comparison operation gives us 255/0, but we need 1/0 so we'll extract bit 0

    size_t mask_sum = cv::sum(mask_img).val[0];
    RCLCPP_INFO(node->get_logger(), "got %lu/%lu pixels in mask", mask_sum, mask_img.total());
    
    /* infer Contact-GraspNet */
    rclcpp::Client<cgn_msgs::srv::Infer>::SharedPtr cgn_client = node->create_client<cgn_msgs::srv::Infer>("infer");
    while (!cgn_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "interrupted while waiting for Contact-GraspNet service - exiting");
            return 1;
        }
        RCLCPP_WARN(node->get_logger(), "waiting for Contact-GraspNet service");
    }
    
    auto cgn_request = std::make_shared<cgn_msgs::srv::Infer::Request>();
    cgn_request->depth = *depth_msg;
    cgn_request->info = *info_msg;

    if (!mask_img.isContinuous()) mask_img = mask_img.reshape(1, mask_img.total());
    cgn_request->mask.assign(mask_img.datastart, mask_img.dataend);

    RCLCPP_INFO(node->get_logger(), "calling Contact-GraspNet inference service");
    auto cgn_result = cgn_client->async_send_request(cgn_request);
    // NOTE: the node is already being spun by the background executor thread above, so we must
    // NOT call rclcpp::spin_until_future_complete(node, ...) here (it would try to add the node
    // to a new executor and throw).
    if (cgn_result.wait_for(30s) != std::future_status::ready) {
        RCLCPP_ERROR(node->get_logger(), "timeout while waiting for Contact-GraspNet inference service response");
        rclcpp::shutdown();
        return 1;
    }
    auto cgn_grasps = cgn_result.get()->grasps;
    RCLCPP_INFO(node->get_logger(), "received %lu grasps from Contact-GraspNet", cgn_grasps.size());

    if (cgn_grasps.size() == 0) {
        RCLCPP_WARN(node->get_logger(), "no grasps received - exiting");
        rclcpp::shutdown();
        return 0;
    }

    /* select best grasp and perform transformation to root frame (e.g. link_base) */
    auto grasp = cgn_grasps[0];
    grasp.header.stamp = node->get_clock()->now(); // to avoid weirdness

    geometry_msgs::msg::PoseStamped grasp_tf; // transformed to root frame reference
    success = false;
    for (int attempt = 0; attempt < 10; attempt++) {
        try {
            grasp_tf = tf_buffer->transform(grasp, root_frame, 1s);
            success = true;
            break;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(node->get_logger(), "cannot transform grasp pose from %s to %s: %s", grasp.header.frame_id.c_str(), root_frame.c_str(), ex.what());
        }
    }
    if (!success) {
        RCLCPP_ERROR(node->get_logger(), "cannot obtain transform - exiting");
        rclcpp::shutdown();
        return 1;
    }

    grasp_pub->publish(grasp_tf); // publish for visualisation

    /* plan movement to best grasp */
    // Defensive: normalize quaternion and validate goal pose numbers
    auto & q = grasp_tf.pose.orientation;
    const double qn = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(qn) || qn < 1e-9) {
        RCLCPP_WARN(node->get_logger(), "grasp orientation quaternion invalid; resetting to identity");
        q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;
    } else {
        q.x /= qn; q.y /= qn; q.z /= qn; q.w /= qn;
    }

    const auto & p = grasp_tf.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        RCLCPP_ERROR(node->get_logger(), "grasp position contains NaN/Inf (x=%f y=%f z=%f) - exiting", p.x, p.y, p.z);
        rclcpp::shutdown();
        return 1;
    }

    // Ensure we're planning from the latest robot state (joint_states)
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(grasp_tf, ee_link);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
            node->get_logger(),
            "cannot plan movement to grasp target (frame=%s, ee_link=%s, pos=[%.3f %.3f %.3f]) - exiting",
            grasp_tf.header.frame_id.c_str(), ee_link.c_str(), p.x, p.y, p.z
        );
        rclcpp::shutdown();
        return 1;
    }
    
    /* execute movement */
    RCLCPP_INFO(node->get_logger(), "executing movement");
    if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "cannot execute movement - exiting");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "finished execution");

    rclcpp::shutdown();

    return 0;
}