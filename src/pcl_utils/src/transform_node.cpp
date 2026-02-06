#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class PointCloudTransformer : public rclcpp::Node {
public:
  PointCloudTransformer() : Node("transform_node") {
    // Declare parameters for flexibility
    this->declare_parameter<std::string>("target_frame", "base_link");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_out", rclcpp::SystemDefaultsQoS());
    
    subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_in", rclcpp::SensorDataQoS(),
      std::bind(&PointCloudTransformer::cloud_callback, this, std::placeholders::_1));
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::string target_frame = this->get_parameter("target_frame").as_string();
    sensor_msgs::msg::PointCloud2 transformed_cloud;

    try {
      // transform() automatically looks up the transform from msg->header.frame_id
      // to target_frame at the specific timestamp of the message.
      tf_buffer_->transform(*msg, transformed_cloud, target_frame, 
                            tf2::durationFromSec(0.1));
      
      publisher_->publish(transformed_cloud);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform cloud: %s", ex.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}