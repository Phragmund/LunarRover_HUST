#ifndef LUNAR_ROVER_CONTROL__KINEMATICS_HPP_
#define LUNAR_ROVER_CONTROL__KINEMATICS_HPP_

#include <array>
#include <cstddef>
#include <string>

namespace lunar_rover_control
{

enum SteeringIndex : std::size_t
{
  FRONT_LEFT_STEERING = 0,
  FRONT_RIGHT_STEERING = 1,
  REAR_LEFT_STEERING = 2,
  REAR_RIGHT_STEERING = 3,
  STEERING_COUNT = 4
};

// Wheel arrays use this order everywhere in this package.
enum WheelIndex : std::size_t
{
  FRONT_LEFT_WHEEL = 0,
  FRONT_RIGHT_WHEEL = 1,
  MIDDLE_LEFT_WHEEL = 2,
  MIDDLE_RIGHT_WHEEL = 3,
  REAR_LEFT_WHEEL = 4,
  REAR_RIGHT_WHEEL = 5,
  WHEEL_COUNT = 6
};

struct RoverGeometry
{
  double wheel_radius{0.09};
  double wheelbase{0.52};
  double track_width{0.50};
  double max_steering_angle{1.0471975511965976};
  double max_wheel_angular_velocity{8.0};
};

struct RoverCommand
{
  std::array<double, STEERING_COUNT> steering_positions{};
  std::array<double, WHEEL_COUNT> wheel_angular_velocities{};
  bool saturated{false};
};

struct PlanarTwist
{
  double linear_x{0.0};
  double angular_z{0.0};
  bool valid{false};
};

class RoverKinematics
{
public:
  explicit RoverKinematics(const RoverGeometry & geometry = RoverGeometry{});

  const RoverGeometry & geometry() const noexcept;
  RoverCommand inverse(double linear_x, double angular_z) const;

  PlanarTwist forward(
    const std::array<double, STEERING_COUNT> & steering_positions,
    const std::array<double, WHEEL_COUNT> & wheel_angular_velocities) const;

  static double shortest_angular_distance(double from, double to) noexcept;
  static bool geometry_is_valid(const RoverGeometry & geometry, std::string * reason = nullptr);

private:
  RoverGeometry geometry_;
};

}  // namespace lunar_rover_control

#endif  // LUNAR_ROVER_CONTROL__KINEMATICS_HPP_
