#include <rclcpp/rclcpp.hpp>

#include <xarm_msgs/srv/set_int16.hpp>

#include <std_msgs/msg/float32.hpp>
#include <xarm_msgs/srv/gripper_move.hpp>
#include <xarm_msgs/srv/get_float32.hpp>

using namespace std::chrono_literals;

class GripperNode : public rclcpp::Node {
public:
    GripperNode(): Node("gripper_node") {
        /* declare and get parameters */
        auto freq_desc = rcl_interfaces::msg::ParameterDescriptor();
        freq_desc.description = "Position polling frequency (in Hz)";
        this->declare_parameter("freq", 30.0, freq_desc);

        auto wait_desc = rcl_interfaces::msg::ParameterDescriptor();
        wait_desc.description = "Whether to wait for set_gripper_position service completion before accepting new command";
        this->declare_parameter("wait", false, wait_desc);

        /* create separate callback groups for services and topics */
        topic_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        /* create service clients (xArm API side) */
        set_position_cli_ = this->create_client<xarm_msgs::srv::GripperMove>("set_gripper_position", rmw_qos_profile_default, service_cb_group_);
        get_position_cli_ = this->create_client<xarm_msgs::srv::GetFloat32>("get_gripper_position", rmw_qos_profile_default, service_cb_group_);

        /* create position publisher */
        position_pub_ = this->create_publisher<std_msgs::msg::Float32>("gripper/current_pos", 10);

        /* set up timer for polling current position */
        double freq = this->get_parameter("freq").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / freq),
            std::bind(&GripperNode::timer_cb, this)
        );

        /* create position subscriber */
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = topic_cb_group_;
        position_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "gripper/target_pos", 10,
            std::bind(&GripperNode::set_position_cb, this, std::placeholders::_1),
            sub_options            
        );
    }

    void enable_gripper() { // to be called by main() before spinning
        auto client = this->create_client<xarm_msgs::srv::SetInt16>("set_gripper_enable");
        
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "interrupted while waiting for set_gripper_enable service");
                return;
            }
            RCLCPP_WARN(this->get_logger(), "waiting for set_gripper_enable service");
        }

        auto request = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
        request->data = 1;
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "failed to call set_gripper_enable service");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "enabled gripper");
    }

private:
    void timer_cb() {
        auto request = std::make_shared<xarm_msgs::srv::GetFloat32::Request>();

        auto future = get_position_cli_->async_send_request(request, std::bind(&GripperNode::get_position_cb, this, std::placeholders::_1));
    }

    void get_position_cb(rclcpp::Client<xarm_msgs::srv::GetFloat32>::SharedFuture future) {
        auto response = future.get();
        if (!response) {
            RCLCPP_ERROR(this->get_logger(), "get_gripper_position service call failed");
            return;
        }
        
        std_msgs::msg::Float32 msg;
        msg.data = response->data;
        position_pub_->publish(msg);
    }

    void set_position_cb(const std_msgs::msg::Float32::SharedPtr msg) {
        if (set_position_pending_.load()) {
            RCLCPP_WARN(this->get_logger(), "previous set_gripper_position service call is still pending - ignoring this message (%f)", msg->data);
            return;
        }

        auto request = std::make_shared<xarm_msgs::srv::GripperMove::Request>();
        request->pos = msg->data;
        request->wait = this->get_parameter("wait").as_bool();

        set_position_pending_.store(true);
        auto future = set_position_cli_->async_send_request(request, std::bind(&GripperNode::set_position_complete_cb, this, std::placeholders::_1));
    }

    void set_position_complete_cb(rclcpp::Client<xarm_msgs::srv::GripperMove>::SharedFuture future) {
        auto response = future.get();
        if (!response) {
            RCLCPP_ERROR(this->get_logger(), "set_gripper_position service call failed");
            // return;
        }
        
        set_position_pending_.store(false);
    }

    rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;

    rclcpp::Client<xarm_msgs::srv::GripperMove>::SharedPtr set_position_cli_;
    rclcpp::Client<xarm_msgs::srv::GetFloat32>::SharedPtr get_position_cli_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::atomic<bool> set_position_pending_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GripperNode>();
    node->enable_gripper();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}