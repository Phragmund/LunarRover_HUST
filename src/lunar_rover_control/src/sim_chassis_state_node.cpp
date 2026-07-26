#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lunar_rover_msgs/msg/chassis_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace lunar_rover_control
{

class SimChassisStateNode : public rclcpp::Node
{
public:
  SimChassisStateNode()
  : Node("sim_chassis_state_node")
  {
    const double publish_rate = declare_parameter<double>("publish_rate", 20.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.25);
    enabled_ = declare_parameter<bool>("default_enabled", true);
    const std::string operating_mode =
      declare_parameter<std::string>("operating_mode", "autonomous");
    const std::string command_topic =
      declare_parameter<std::string>("command_topic", "/chassis/cmd_vel");
    const std::string state_topic =
      declare_parameter<std::string>("state_topic", "/chassis/state");
    const std::string estop_topic =
      declare_parameter<std::string>("estop_topic", "/chassis/emergency_stop");
    const std::string enable_topic =
      declare_parameter<std::string>("enable_topic", "/chassis/controller_enabled");
    const std::string fault_topic =
      declare_parameter<std::string>("fault_topic", "/chassis/fault_bits");

    if (!std::isfinite(publish_rate) || publish_rate <= 0.0 ||
      !std::isfinite(command_timeout_) || command_timeout_ <= 0.0)
    {
      throw std::invalid_argument("publish_rate and command_timeout must be positive");
    }
    if (operating_mode == "manual") {
      requested_mode_ = lunar_rover_msgs::msg::ChassisState::MODE_MANUAL;
    } else if (operating_mode == "autonomous") {
      requested_mode_ = lunar_rover_msgs::msg::ChassisState::MODE_AUTONOMOUS;
    } else {
      throw std::invalid_argument("operating_mode must be 'manual' or 'autonomous'");
    }

    state_publisher_ = create_publisher<lunar_rover_msgs::msg::ChassisState>(
      state_topic, rclcpp::QoS(10));
    command_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      command_topic, rclcpp::QoS(1).reliable(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr) {
        last_command_time_ = now();
        have_command_ = true;
      });
    estop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic, rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        estop_ = message->data;
      });
    enable_subscription_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic, rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        enabled_ = message->data;
      });
    fault_subscription_ = create_subscription<std_msgs::msg::UInt32>(
      fault_topic, rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::UInt32::SharedPtr message) {
        fault_bits_ = message->data;
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SimChassisStateNode::publish_state, this));
  }

private:
  void publish_state()
  {
    const rclcpp::Time stamp = now();
    const double command_age = (stamp - last_command_time_).seconds();
    const bool watchdog_ok =
      have_command_ && command_age >= -0.05 && command_age <= command_timeout_;

    lunar_rover_msgs::msg::ChassisState state;
    state.header.stamp = stamp;
    state.header.frame_id = "base_link";
    state.fault_bits = fault_bits_;
    state.estop = estop_;
    state.enabled = enabled_;
    state.watchdog_ok = watchdog_ok;
    if (estop_ || fault_bits_ != 0U) {
      state.mode = lunar_rover_msgs::msg::ChassisState::MODE_FAULT;
    } else if (!enabled_) {
      state.mode = lunar_rover_msgs::msg::ChassisState::MODE_DISABLED;
    } else {
      state.mode = requested_mode_;
    }
    state_publisher_->publish(state);
  }

  uint8_t requested_mode_{
    lunar_rover_msgs::msg::ChassisState::MODE_AUTONOMOUS};
  uint32_t fault_bits_{0U};
  bool estop_{false};
  bool enabled_{true};
  bool have_command_{false};
  double command_timeout_{0.25};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<lunar_rover_msgs::msg::ChassisState>::SharedPtr state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr fault_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lunar_rover_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lunar_rover_control::SimChassisStateNode>());
  rclcpp::shutdown();
  return 0;
}
