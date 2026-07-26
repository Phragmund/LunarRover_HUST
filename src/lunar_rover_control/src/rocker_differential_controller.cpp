#include "lunar_rover_control/rocker_differential_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace lunar_rover_control
{

controller_interface::CallbackReturn RockerDifferentialController::on_init()
{
  try {
    auto_declare<std::string>("left_rocker_joint", "left_rocker_joint");
    auto_declare<std::string>("right_rocker_joint", "right_rocker_joint");
    auto_declare<double>("coupling_sign", 1.0);
    auto_declare<double>("equilibrium_offset", 0.0);
    auto_declare<double>("stiffness", 120.0);
    auto_declare<double>("damping", 18.0);
    auto_declare<double>("max_effort", 120.0);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RockerDifferentialController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names = {
    left_joint_ + "/" + hardware_interface::HW_IF_EFFORT,
    right_joint_ + "/" + hardware_interface::HW_IF_EFFORT};
  return configuration;
}

controller_interface::InterfaceConfiguration
RockerDifferentialController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  configuration.names = {
    left_joint_ + "/" + hardware_interface::HW_IF_POSITION,
    right_joint_ + "/" + hardware_interface::HW_IF_POSITION,
    left_joint_ + "/" + hardware_interface::HW_IF_VELOCITY,
    right_joint_ + "/" + hardware_interface::HW_IF_VELOCITY};
  return configuration;
}

controller_interface::CallbackReturn RockerDifferentialController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  left_joint_ = node->get_parameter("left_rocker_joint").as_string();
  right_joint_ = node->get_parameter("right_rocker_joint").as_string();
  coupling_sign_ = node->get_parameter("coupling_sign").as_double();
  equilibrium_offset_ = node->get_parameter("equilibrium_offset").as_double();
  stiffness_ = node->get_parameter("stiffness").as_double();
  damping_ = node->get_parameter("damping").as_double();
  max_effort_ = node->get_parameter("max_effort").as_double();

  const bool valid =
    !left_joint_.empty() && !right_joint_.empty() && left_joint_ != right_joint_ &&
    std::isfinite(coupling_sign_) &&
    std::abs(std::abs(coupling_sign_) - 1.0) < 1.0e-9 &&
    std::isfinite(equilibrium_offset_) &&
    std::isfinite(stiffness_) && stiffness_ >= 0.0 &&
    std::isfinite(damping_) && damping_ >= 0.0 &&
    std::isfinite(max_effort_) && max_effort_ > 0.0;
  if (!valid) {
    RCLCPP_ERROR(node->get_logger(), "Invalid rocker differential parameters");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RockerDifferentialController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != 2U || state_interfaces_.size() != 4U) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Rocker controller expected 2 command and 4 state interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  halt();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RockerDifferentialController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  halt();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RockerDifferentialController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const double left_position = state_interfaces_[0].get_value();
  const double right_position = state_interfaces_[1].get_value();
  const double left_velocity = state_interfaces_[2].get_value();
  const double right_velocity = state_interfaces_[3].get_value();
  if (!std::isfinite(left_position) || !std::isfinite(right_position) ||
    !std::isfinite(left_velocity) || !std::isfinite(right_velocity))
  {
    halt();
    return controller_interface::return_type::ERROR;
  }

  const double error =
    left_position + coupling_sign_ * right_position - equilibrium_offset_;
  const double error_rate = left_velocity + coupling_sign_ * right_velocity;
  const double left_effort = std::clamp(
    -stiffness_ * error - damping_ * error_rate, -max_effort_, max_effort_);
  const double right_effort = std::clamp(
    coupling_sign_ * left_effort, -max_effort_, max_effort_);
  command_interfaces_[0].set_value(left_effort);
  command_interfaces_[1].set_value(right_effort);
  return controller_interface::return_type::OK;
}

void RockerDifferentialController::halt()
{
  for (auto & interface : command_interfaces_) {
    interface.set_value(0.0);
  }
}

}  // namespace lunar_rover_control

PLUGINLIB_EXPORT_CLASS(
  lunar_rover_control::RockerDifferentialController,
  controller_interface::ControllerInterface)
