#ifndef LUNAR_ROVER_CONTROL__ROCKER_DIFFERENTIAL_CONTROLLER_HPP_
#define LUNAR_ROVER_CONTROL__ROCKER_DIFFERENTIAL_CONTROLLER_HPP_

#include <string>

#include "controller_interface/controller_interface.hpp"

namespace lunar_rover_control
{

struct RockerDifferentialCommand
{
  double left_effort{0.0};
  double right_effort{0.0};
  bool valid{false};
};

RockerDifferentialCommand compute_rocker_differential_effort(
  double left_position,
  double right_position,
  double left_velocity,
  double right_velocity,
  double coupling_sign,
  double equilibrium_offset,
  double stiffness,
  double damping,
  double max_effort);

class RockerDifferentialController : public controller_interface::ControllerInterface
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
  void halt();

  std::string left_joint_;
  std::string right_joint_;
  double coupling_sign_{1.0};
  double equilibrium_offset_{0.0};
  double stiffness_{120.0};
  double damping_{18.0};
  double max_effort_{120.0};
};

}  // namespace lunar_rover_control

#endif  // LUNAR_ROVER_CONTROL__ROCKER_DIFFERENTIAL_CONTROLLER_HPP_
