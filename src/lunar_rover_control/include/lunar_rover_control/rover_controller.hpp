#ifndef LUNAR_ROVER_CONTROL__ROVER_CONTROLLER_HPP_
#define LUNAR_ROVER_CONTROL__ROVER_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "rclcpp/subscription.hpp"

#include "lunar_rover_control/kinematics.hpp"

namespace lunar_rover_control
{

class RoverController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct TwistCommand
  {
    double linear_x{0.0};
    double angular_z{0.0};
    double received_at_seconds{0.0};
    bool valid{false};
  };

  void command_callback(const geometry_msgs::msg::TwistStamped::SharedPtr message);
  bool read_parameters();
  void halt();

  std::vector<std::string> steering_joints_;
  std::vector<std::string> wheel_joints_;
  std::string command_topic_{"/chassis/cmd_vel"};
  double command_timeout_seconds_{0.25};
  double steering_alignment_tolerance_{0.08};
  double max_steering_rate_{1.2};
  double max_wheel_angular_acceleration_{10.0};
  bool stop_wheels_until_steered_{true};

  std::array<double, STEERING_COUNT> last_steering_commands_{};
  std::array<double, WHEEL_COUNT> last_wheel_commands_{};
  std::unique_ptr<RoverKinematics> kinematics_;
  realtime_tools::RealtimeBuffer<TwistCommand> command_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_;
};

}  // namespace lunar_rover_control

#endif  // LUNAR_ROVER_CONTROL__ROVER_CONTROLLER_HPP_
