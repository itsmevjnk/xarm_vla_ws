#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "xarm_msgs/msg/robot_msg.hpp"
#include "tf2/LinearMath/Quaternion.h"

class XArmPublisherNode : public rclcpp::Node {
public:
    XArmPublisherNode() : Node("xarm_tf_pub") {
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        tcp_frame_ = this->declare_parameter<std::string>("tcp_frame", "ee_link");
        arm_ns_ = this->declare_parameter<std::string>("arm_ns", "xarm");

        tf_bcast_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        std::ostringstream topic_name_stream;
        topic_name_stream << "/" << arm_ns_ << "/robot_states";
        std::string topic_name = topic_name_stream.str();

        state_sub_ = this->create_subscription<xarm_msgs::msg::RobotMsg>(
            topic_name, rclcpp::SensorDataQoS(),
            std::bind(&XArmPublisherNode::state_cb, this, std::placeholders::_1)
        );
    }   

private:
    void state_cb(const std::shared_ptr<xarm_msgs::msg::RobotMsg> msg) {
        geometry_msgs::msg::TransformStamped t;

        t.header = msg->header;
        t.header.frame_id = base_frame_; // frame_id is empty from /robot_states
        t.child_frame_id = tcp_frame_;

        /* convert translation from mm to m */
        t.transform.translation.x = msg->pose[0] / 1000;
        t.transform.translation.y = msg->pose[1] / 1000;
        t.transform.translation.z = msg->pose[2] / 1000;

        /* convert Euler rotation to quaternion */
        tf2::Quaternion q;
        q.setRPY(msg->pose[3], msg->pose[4], msg->pose[5]); // RPY order, and already in radians
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_bcast_->sendTransform(t);
    }

    std::string base_frame_;
    std::string tcp_frame_;
    std::string arm_ns_;

    rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr state_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_bcast_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<XArmPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
