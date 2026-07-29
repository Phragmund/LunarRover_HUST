#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lunar_rover_navigation
{

class TwistStamperNode : public rclcpp::Node
{
public:
  TwistStamperNode()
  : Node("twist_stamper")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cmd_vel");
    output_topic_ = declare_parameter<std::string>("output_topic", "/chassis/cmd_vel");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.22);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 0.45);
    validate_parameters();

    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      output_topic_, rclcpp::QoS(1).reliable());
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        publish_stamped(*message);
      });
  }

private:
  void validate_parameters() const
  {
    if (input_topic_.empty() || output_topic_.empty() || frame_id_.empty()) {
      throw std::invalid_argument("Topics and frame_id must not be empty");
    }
    if (!std::isfinite(max_linear_x_) || max_linear_x_ <= 0.0 ||
      !std::isfinite(max_angular_z_) || max_angular_z_ <= 0.0)
    {
      throw std::invalid_argument("Velocity limits must be finite and positive");
    }
  }

  void publish_stamped(const geometry_msgs::msg::Twist & twist)
  {
    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = now();
    stamped.header.frame_id = frame_id_;
    stamped.twist = twist;
    stamped.twist.linear.x = std::clamp(
      stamped.twist.linear.x, -max_linear_x_, max_linear_x_);
    stamped.twist.linear.y = 0.0;
    stamped.twist.linear.z = 0.0;
    stamped.twist.angular.x = 0.0;
    stamped.twist.angular.y = 0.0;
    stamped.twist.angular.z = std::clamp(
      stamped.twist.angular.z, -max_angular_z_, max_angular_z_);
    publisher_->publish(stamped);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double max_linear_x_{0.22};
  double max_angular_z_{0.45};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

}  // namespace lunar_rover_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lunar_rover_navigation::TwistStamperNode>());
  rclcpp::shutdown();
  return 0;
}
