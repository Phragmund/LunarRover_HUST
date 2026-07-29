#include "lunar_rover_control/rover_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"

namespace lunar_rover_control
{
namespace
{

const std::vector<std::string> kDefaultSteeringJoints = {
  "left_front_steering_joint",
  "right_front_steering_joint",
  "left_rear_steering_joint",
  "right_rear_steering_joint"};

const std::vector<std::string> kDefaultWheelJoints = {
  "left_front_wheel_joint",
  "right_front_wheel_joint",
  "left_middle_wheel_joint",
  "right_middle_wheel_joint",
  "left_rear_wheel_joint",
  "right_rear_wheel_joint"};

}  // namespace

controller_interface::CallbackReturn RoverController::on_init()
{
  try {
    steering_joints_ = kDefaultSteeringJoints;
    wheel_joints_ = kDefaultWheelJoints;
    auto_declare<std::vector<std::string>>(
      "steering_joints", kDefaultSteeringJoints);
    auto_declare<std::vector<std::string>>("wheel_joints", kDefaultWheelJoints);
    auto_declare<double>("wheel_radius", 0.0625);
    auto_declare<double>("wheelbase", 0.52);
    auto_declare<double>("track_width", 0.50);
    auto_declare<double>("max_steering_angle", 1.0471975511965976);
    auto_declare<double>("max_wheel_angular_velocity", 8.0);
    auto_declare<std::string>("command_topic", "/chassis/cmd_vel");
    auto_declare<double>("command_timeout", 0.25);
    auto_declare<double>("steering_alignment_tolerance", 0.08);
    auto_declare<double>("max_steering_rate", 1.2);
    auto_declare<double>("max_wheel_angular_acceleration", 10.0);
    auto_declare<bool>("stop_wheels_until_steered", true);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  command_buffer_.writeFromNonRT(TwistCommand{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RoverController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : steering_joints_) {
    configuration.names.emplace_back(
      joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (const auto & joint : wheel_joints_) {
    configuration.names.emplace_back(
      joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
RoverController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : steering_joints_) {
    configuration.names.emplace_back(
      joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return configuration;
}

controller_interface::CallbackReturn RoverController::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!read_parameters()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  command_subscription_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
    command_topic_,
    rclcpp::QoS(1).reliable(),
    std::bind(&RoverController::command_callback, this, std::placeholders::_1));
  command_buffer_.writeFromNonRT(TwistCommand{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoverController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != STEERING_COUNT + WHEEL_COUNT ||
    state_interfaces_.size() != STEERING_COUNT)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces and %zu steering state interfaces, got %zu and %zu",
      static_cast<std::size_t>(STEERING_COUNT + WHEEL_COUNT),
      static_cast<std::size_t>(STEERING_COUNT),
      command_interfaces_.size(), state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < STEERING_COUNT; ++i) {
    const double current = state_interfaces_[i].get_value();
    last_steering_commands_[i] = std::isfinite(current) ? current : 0.0;
    command_interfaces_[i].set_value(last_steering_commands_[i]);
  }
  last_wheel_commands_.fill(0.0);
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    command_interfaces_[STEERING_COUNT + i].set_value(last_wheel_commands_[i]);
  }
  command_buffer_.writeFromNonRT(TwistCommand{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RoverController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  halt();
  command_buffer_.writeFromNonRT(TwistCommand{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RoverController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!kinematics_) {
    return controller_interface::return_type::ERROR;
  }

  const TwistCommand command = *command_buffer_.readFromRT();
  const double age = time.seconds() - command.received_at_seconds;
  const bool fresh =
    command.valid && std::isfinite(age) && age >= -0.05 &&
    age <= command_timeout_seconds_;

  RoverCommand targets = fresh ?
    kinematics_->inverse(command.linear_x, command.angular_z) :
    kinematics_->inverse(0.0, 0.0);

  const double dt = period.seconds();
  const double steering_step =
    std::isfinite(dt) && dt > 0.0 ? max_steering_rate_ * dt : 0.0;
  double maximum_error = 0.0;
  for (std::size_t i = 0; i < STEERING_COUNT; ++i) {
    const double current = state_interfaces_[i].get_value();
    if (!std::isfinite(current)) {
      halt();
      return controller_interface::return_type::ERROR;
    }
    if (!fresh) {
      targets.steering_positions[i] = current;
    }
    const double current_error = RoverKinematics::shortest_angular_distance(
      current, targets.steering_positions[i]);
    maximum_error = std::max(maximum_error, std::abs(current_error));
    if (!fresh) {
      last_steering_commands_[i] = current;
    } else {
      const double command_error = RoverKinematics::shortest_angular_distance(
        last_steering_commands_[i], targets.steering_positions[i]);
      last_steering_commands_[i] += std::clamp(
        command_error, -steering_step, steering_step);
    }
    command_interfaces_[i].set_value(last_steering_commands_[i]);
  }

  const bool can_drive =
    fresh && (!stop_wheels_until_steered_ ||
    maximum_error <= steering_alignment_tolerance_);
  const bool ackermann_turn_requested =
    fresh && std::abs(command.linear_x) > 1e-6 && std::abs(command.angular_z) > 1e-6;
  const bool hold_wheels_while_steering =
    ackermann_turn_requested && stop_wheels_until_steered_ && !can_drive;
  const double wheel_step =
    std::isfinite(dt) && dt > 0.0 ? max_wheel_angular_acceleration_ * dt : 0.0;
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i) {
    if (!fresh) {
      // Watchdog stops intentionally bypass acceleration limiting.
      last_wheel_commands_[i] = 0.0;
    } else if (hold_wheels_while_steering) {
      // Preserve the current drive speed while steering catches up for Ackermann turns.
    } else if (!can_drive) {
      // Non-Ackermann alignment waits still stop the drive wheels.
      last_wheel_commands_[i] = 0.0;
    } else {
      const double difference =
        targets.wheel_angular_velocities[i] - last_wheel_commands_[i];
      last_wheel_commands_[i] += std::clamp(difference, -wheel_step, wheel_step);
    }
    command_interfaces_[STEERING_COUNT + i].set_value(
      last_wheel_commands_[i]);
  }
  return controller_interface::return_type::OK;
}

void RoverController::command_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr message)
{
  TwistCommand command;
  command.linear_x = message->twist.linear.x;
  command.angular_z = message->twist.angular.z;
  command.received_at_seconds = get_node()->now().seconds();
  command.valid =
    std::isfinite(command.linear_x) && std::isfinite(command.angular_z);
  if (!command.valid) {
    RCLCPP_WARN(get_node()->get_logger(), "Discarding non-finite chassis command");
    command.linear_x = 0.0;
    command.angular_z = 0.0;
  }
  command_buffer_.writeFromNonRT(command);
}

bool RoverController::read_parameters()
{
  auto node = get_node();
  steering_joints_ = node->get_parameter("steering_joints").as_string_array();
  wheel_joints_ = node->get_parameter("wheel_joints").as_string_array();
  command_topic_ = node->get_parameter("command_topic").as_string();
  command_timeout_seconds_ = node->get_parameter("command_timeout").as_double();
  steering_alignment_tolerance_ =
    node->get_parameter("steering_alignment_tolerance").as_double();
  max_steering_rate_ = node->get_parameter("max_steering_rate").as_double();
  max_wheel_angular_acceleration_ =
    node->get_parameter("max_wheel_angular_acceleration").as_double();
  stop_wheels_until_steered_ =
    node->get_parameter("stop_wheels_until_steered").as_bool();

  if (steering_joints_.size() != STEERING_COUNT ||
    wheel_joints_.size() != WHEEL_COUNT)
  {
    RCLCPP_ERROR(
      node->get_logger(), "steering_joints must contain four names and wheel_joints six names");
    return false;
  }
  std::unordered_set<std::string> unique_joints;
  for (const auto & joint : steering_joints_) {
    if (joint.empty() || !unique_joints.insert(joint).second) {
      RCLCPP_ERROR(node->get_logger(), "Joint names must be non-empty and unique");
      return false;
    }
  }
  for (const auto & joint : wheel_joints_) {
    if (joint.empty() || !unique_joints.insert(joint).second) {
      RCLCPP_ERROR(node->get_logger(), "Joint names must be non-empty and unique");
      return false;
    }
  }
  if (command_topic_.empty() || !std::isfinite(command_timeout_seconds_) ||
    command_timeout_seconds_ <= 0.0 ||
    !std::isfinite(steering_alignment_tolerance_) ||
    steering_alignment_tolerance_ < 0.0 ||
    !std::isfinite(max_steering_rate_) || max_steering_rate_ <= 0.0 ||
    !std::isfinite(max_wheel_angular_acceleration_) ||
    max_wheel_angular_acceleration_ <= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid controller topic or timeout/alignment parameter");
    return false;
  }

  RoverGeometry geometry;
  geometry.wheel_radius = node->get_parameter("wheel_radius").as_double();
  geometry.wheelbase = node->get_parameter("wheelbase").as_double();
  geometry.track_width = node->get_parameter("track_width").as_double();
  geometry.max_steering_angle =
    node->get_parameter("max_steering_angle").as_double();
  geometry.max_wheel_angular_velocity =
    node->get_parameter("max_wheel_angular_velocity").as_double();

  std::string reason;
  if (!RoverKinematics::geometry_is_valid(geometry, &reason)) {
    RCLCPP_ERROR(node->get_logger(), "Invalid rover geometry: %s", reason.c_str());
    return false;
  }
  kinematics_ = std::make_unique<RoverKinematics>(geometry);
  return true;
}

void RoverController::halt()
{
  last_wheel_commands_.fill(0.0);
  for (std::size_t i = 0; i < WHEEL_COUNT &&
    STEERING_COUNT + i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[STEERING_COUNT + i].set_value(last_wheel_commands_[i]);
  }
}

}  // namespace lunar_rover_control

PLUGINLIB_EXPORT_CLASS(
  lunar_rover_control::RoverController,
  controller_interface::ControllerInterface)
