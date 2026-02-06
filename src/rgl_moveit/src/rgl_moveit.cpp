#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/srv/grasp_planning.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <future>
#include <cmath>
#include <cstdlib>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "rgl_moveit", rclcpp::NodeOptions()
  );

  /* TF2 buffer, listener and static broadcaster */
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);
  auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  node->declare_parameter("grasp_pose_frame", "grasp_pose"); // for visualisation
  std::string grasp_pose_frame = node->get_parameter("grasp_pose_frame").as_string();

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

  /* Intel ROS2 Grasp Library interface */
  rclcpp::Client<moveit_msgs::srv::GraspPlanning>::SharedPtr rgl_client = node->create_client<moveit_msgs::srv::GraspPlanning>("plan_grasps");
  while (!rgl_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "interrupted while waiting for ROS2 Grasp Library service - exiting");
      return 1;
    }
    RCLCPP_WARN(node->get_logger(), "waiting for ROS2 Grasp Library service");
  }

  RCLCPP_INFO(node->get_logger(), "calling ROS2 Grasp Library inference service");
  auto rgl_request = std::make_shared<moveit_msgs::srv::GraspPlanning::Request>();
  // no need for anything else here I guess, we're doing random grasps
  auto rgl_result = rgl_client->async_send_request(rgl_request);
  // NOTE: the node is already being spun by the background executor thread above, so we must
  // NOT call rclcpp::spin_until_future_complete(node, ...) here (it would try to add the node
  // to a new executor and throw).
  if (rgl_result.wait_for(30s) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "timeout while waiting for ROS2 Grasp Library inference service response");
    rclcpp::shutdown();
  return 1;
  }
  auto rgl_grasps = rgl_result.get()->grasps;
  RCLCPP_INFO(node->get_logger(), "received %lu grasps from ROS2 Grasp Library", rgl_grasps.size());

  if (rgl_grasps.size() == 0) {
    RCLCPP_WARN(node->get_logger(), "no grasps received - exiting");
    rclcpp::shutdown();
    return 0;
  }

  /* select random grasp */
  srand(time(NULL)); // seed RNG
  auto grasp = rgl_grasps[0].grasp_pose; // alternatively we can select rgl_grasps[0]
  grasp.header.stamp = node->get_clock()->now(); // to avoid weirdness

  RCLCPP_INFO(
    node->get_logger(),
    "selecting grasp pose wrt %s: position (%.4f, %.4f, %.4f) orientation (%.4f, %.4f, %.4f, %.4f)",
    grasp.header.frame_id.c_str(),
    grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z,
    grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z, grasp.pose.orientation.w
  );

  geometry_msgs::msg::PoseStamped grasp_tf; // transformed to root frame reference
  bool success = false;
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

  /* publish grasp pose for visualisation */
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header = grasp.header;
  tf_msg.child_frame_id = grasp_pose_frame;
  tf_msg.transform.translation.x = grasp.pose.position.x;
  tf_msg.transform.translation.y = grasp.pose.position.y;
  tf_msg.transform.translation.z = grasp.pose.position.z;
  tf_msg.transform.rotation = grasp.pose.orientation;
  tf_broadcaster->sendTransform(tf_msg);

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