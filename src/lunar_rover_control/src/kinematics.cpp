#include "lunar_rover_control/kinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace lunar_rover_control
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-12;

double normalize_angle(double angle) noexcept
{
  return std::remainder(angle, 2.0 * kPi);
}

struct WheelLocation
{
  double x;
  double y;
  bool steerable;
  std::size_t steering_index;
};

std::array<WheelLocation, WHEEL_COUNT> wheel_locations(const RoverGeometry & geometry)
{
  const double front_x = geometry.wheelbase * 0.5;
  const double rear_x = -front_x;
  const double left_y = geometry.track_width * 0.5;
  const double right_y = -left_y;

  return {{
      {front_x, left_y, true, FRONT_LEFT_STEERING},
      {front_x, right_y, true, FRONT_RIGHT_STEERING},
      {0.0, left_y, false, 0U},
      {0.0, right_y, false, 0U},
      {rear_x, left_y, true, REAR_LEFT_STEERING},
      {rear_x, right_y, true, REAR_RIGHT_STEERING},
    }};
}

std::pair<double, double> minimal_steering(double heading, double speed)
{
  heading = normalize_angle(heading);
  if (heading > kPi * 0.5) {
    heading -= kPi;
    speed = -speed;
  } else if (heading < -kPi * 0.5) {
    heading += kPi;
    speed = -speed;
  }
  return {heading, speed};
}

}  // namespace

RoverKinematics::RoverKinematics(const RoverGeometry & geometry)
: geometry_(geometry)
{
  std::string reason;
  if (!geometry_is_valid(geometry_, &reason)) {
    throw std::invalid_argument("Invalid rover geometry: " + reason);
  }
}

const RoverGeometry & RoverKinematics::geometry() const noexcept
{
  return geometry_;
}

RoverCommand RoverKinematics::inverse(double linear_x, double angular_z) const
{
  RoverCommand result;
  if (!std::isfinite(linear_x) || !std::isfinite(angular_z)) {
    result.saturated = true;
    return result;
  }

  const auto locations = wheel_locations(geometry_);
  for (std::size_t i = 0; i < locations.size(); ++i) {
    const auto & location = locations[i];
    const double velocity_x = linear_x - angular_z * location.y;
    const double velocity_y = angular_z * location.x;
    double steering = 0.0;
    double wheel_speed = velocity_x / geometry_.wheel_radius;

    if (location.steerable) {
      if (std::hypot(velocity_x, velocity_y) > kEpsilon) {
        const auto steering_and_speed = minimal_steering(
          std::atan2(velocity_y, velocity_x),
          std::hypot(velocity_x, velocity_y) / geometry_.wheel_radius);
        steering = steering_and_speed.first;
        wheel_speed = steering_and_speed.second;
      }

      const double limited_steering = std::clamp(
        steering, -geometry_.max_steering_angle, geometry_.max_steering_angle);
      if (limited_steering != steering) {
        // Project the desired contact velocity onto the reachable wheel heading.
        wheel_speed =
          (velocity_x * std::cos(limited_steering) +
          velocity_y * std::sin(limited_steering)) /
          geometry_.wheel_radius;
        steering = limited_steering;
        result.saturated = true;
      }
      result.steering_positions[location.steering_index] = steering;
    }

    const double limited_speed = std::clamp(
      wheel_speed,
      -geometry_.max_wheel_angular_velocity,
      geometry_.max_wheel_angular_velocity);
    if (limited_speed != wheel_speed) {
      result.saturated = true;
    }
    result.wheel_angular_velocities[i] = limited_speed;
  }

  return result;
}

PlanarTwist RoverKinematics::forward(
  const std::array<double, STEERING_COUNT> & steering_positions,
  const std::array<double, WHEEL_COUNT> & wheel_angular_velocities) const
{
  // Least-squares fit of r*w_i = cos(theta_i)*vx +
  // (-y_i*cos(theta_i) + x_i*sin(theta_i))*wz.
  double ata_00 = 0.0;
  double ata_01 = 0.0;
  double ata_11 = 0.0;
  double atb_0 = 0.0;
  double atb_1 = 0.0;

  const auto locations = wheel_locations(geometry_);
  for (std::size_t i = 0; i < locations.size(); ++i) {
    if (!std::isfinite(wheel_angular_velocities[i])) {
      return {};
    }
    const auto & location = locations[i];
    const double steering =
      location.steerable ? steering_positions[location.steering_index] : 0.0;
    if (!std::isfinite(steering)) {
      return {};
    }

    const double a0 = std::cos(steering);
    const double a1 = -location.y * std::cos(steering) +
      location.x * std::sin(steering);
    const double b = geometry_.wheel_radius * wheel_angular_velocities[i];
    ata_00 += a0 * a0;
    ata_01 += a0 * a1;
    ata_11 += a1 * a1;
    atb_0 += a0 * b;
    atb_1 += a1 * b;
  }

  const double determinant = ata_00 * ata_11 - ata_01 * ata_01;
  if (!std::isfinite(determinant) || std::abs(determinant) <= kEpsilon) {
    return {};
  }

  PlanarTwist result;
  result.linear_x = (atb_0 * ata_11 - atb_1 * ata_01) / determinant;
  result.angular_z = (ata_00 * atb_1 - ata_01 * atb_0) / determinant;
  result.valid = std::isfinite(result.linear_x) && std::isfinite(result.angular_z);
  return result;
}

double RoverKinematics::shortest_angular_distance(double from, double to) noexcept
{
  return normalize_angle(to - from);
}

bool RoverKinematics::geometry_is_valid(
  const RoverGeometry & geometry, std::string * reason)
{
  const auto fail = [reason](const std::string & message) {
      if (reason != nullptr) {
        *reason = message;
      }
      return false;
    };

  if (!std::isfinite(geometry.wheel_radius) || geometry.wheel_radius <= 0.0) {
    return fail("wheel_radius must be finite and positive");
  }
  if (!std::isfinite(geometry.wheelbase) || geometry.wheelbase <= 0.0) {
    return fail("wheelbase must be finite and positive");
  }
  if (!std::isfinite(geometry.track_width) || geometry.track_width <= 0.0) {
    return fail("track_width must be finite and positive");
  }
  if (!std::isfinite(geometry.max_steering_angle) ||
    geometry.max_steering_angle <= 0.0 ||
    geometry.max_steering_angle > kPi * 0.5)
  {
    return fail("max_steering_angle must be in (0, pi/2]");
  }
  if (!std::isfinite(geometry.max_wheel_angular_velocity) ||
    geometry.max_wheel_angular_velocity <= 0.0)
  {
    return fail("max_wheel_angular_velocity must be finite and positive");
  }
  if (reason != nullptr) {
    reason->clear();
  }
  return true;
}

}  // namespace lunar_rover_control
