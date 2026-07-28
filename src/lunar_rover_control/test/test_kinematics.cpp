#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"

#include "lunar_rover_control/kinematics.hpp"
#include "lunar_rover_control/rocker_differential_controller.hpp"

namespace lunar_rover_control
{
namespace
{

constexpr double kTolerance = 1.0e-9;

RoverGeometry unrestricted_geometry()
{
  RoverGeometry geometry;
  geometry.wheel_radius = 0.1;
  geometry.wheelbase = 0.6;
  geometry.track_width = 0.5;
  geometry.max_steering_angle = 1.4;
  geometry.max_wheel_angular_velocity = 100.0;
  return geometry;
}

TEST(RoverKinematics, StraightDriving)
{
  RoverKinematics kinematics(unrestricted_geometry());
  const RoverCommand command = kinematics.inverse(0.5, 0.0);

  for (const double steering : command.steering_positions) {
    EXPECT_NEAR(steering, 0.0, kTolerance);
  }
  for (const double speed : command.wheel_angular_velocities) {
    EXPECT_NEAR(speed, 5.0, kTolerance);
  }
  EXPECT_FALSE(command.saturated);
}

TEST(RoverKinematics, CoordinatedTurn)
{
  RoverKinematics kinematics(unrestricted_geometry());
  const RoverCommand command = kinematics.inverse(0.5, 0.4);

  EXPECT_GT(command.steering_positions[FRONT_LEFT_STEERING], 0.0);
  EXPECT_GT(command.steering_positions[FRONT_RIGHT_STEERING], 0.0);
  EXPECT_LT(command.steering_positions[REAR_LEFT_STEERING], 0.0);
  EXPECT_LT(command.steering_positions[REAR_RIGHT_STEERING], 0.0);
  EXPECT_LT(
    command.wheel_angular_velocities[MIDDLE_LEFT_WHEEL],
    command.wheel_angular_velocities[MIDDLE_RIGHT_WHEEL]);

  const PlanarTwist recovered = kinematics.forward(
    command.steering_positions, command.wheel_angular_velocities);
  ASSERT_TRUE(recovered.valid);
  EXPECT_NEAR(recovered.linear_x, 0.5, kTolerance);
  EXPECT_NEAR(recovered.angular_z, 0.4, kTolerance);
}

TEST(RoverKinematics, InPlaceTurnUsesXSteering)
{
  RoverKinematics kinematics(unrestricted_geometry());
  const RoverCommand command = kinematics.inverse(0.0, 0.5);

  EXPECT_LT(command.steering_positions[FRONT_LEFT_STEERING], 0.0);
  EXPECT_GT(command.steering_positions[FRONT_RIGHT_STEERING], 0.0);
  EXPECT_GT(command.steering_positions[REAR_LEFT_STEERING], 0.0);
  EXPECT_LT(command.steering_positions[REAR_RIGHT_STEERING], 0.0);
  EXPECT_LT(command.wheel_angular_velocities[FRONT_LEFT_WHEEL], 0.0);
  EXPECT_GT(command.wheel_angular_velocities[FRONT_RIGHT_WHEEL], 0.0);
  EXPECT_LT(command.wheel_angular_velocities[MIDDLE_LEFT_WHEEL], 0.0);
  EXPECT_GT(command.wheel_angular_velocities[MIDDLE_RIGHT_WHEEL], 0.0);
  EXPECT_LT(command.wheel_angular_velocities[REAR_LEFT_WHEEL], 0.0);
  EXPECT_GT(command.wheel_angular_velocities[REAR_RIGHT_WHEEL], 0.0);

  const PlanarTwist recovered = kinematics.forward(
    command.steering_positions, command.wheel_angular_velocities);
  ASSERT_TRUE(recovered.valid);
  EXPECT_NEAR(recovered.linear_x, 0.0, kTolerance);
  EXPECT_NEAR(recovered.angular_z, 0.5, kTolerance);
}

TEST(RoverKinematics, SaturatesSteeringAndWheelSpeed)
{
  RoverGeometry geometry = unrestricted_geometry();
  geometry.max_steering_angle = 0.35;
  geometry.max_wheel_angular_velocity = 2.0;
  RoverKinematics kinematics(geometry);
  const RoverCommand command = kinematics.inverse(2.0, 5.0);

  EXPECT_TRUE(command.saturated);
  for (const double steering : command.steering_positions) {
    EXPECT_LE(std::abs(steering), geometry.max_steering_angle + kTolerance);
  }
  for (const double speed : command.wheel_angular_velocities) {
    EXPECT_LE(std::abs(speed), geometry.max_wheel_angular_velocity + kTolerance);
  }
}

TEST(RoverKinematics, ReverseMotionRoundTrip)
{
  RoverKinematics kinematics(unrestricted_geometry());
  const RoverCommand command = kinematics.inverse(-0.35, -0.25);
  const PlanarTwist recovered = kinematics.forward(
    command.steering_positions, command.wheel_angular_velocities);

  ASSERT_TRUE(recovered.valid);
  EXPECT_NEAR(recovered.linear_x, -0.35, kTolerance);
  EXPECT_NEAR(recovered.angular_z, -0.25, kTolerance);
}

TEST(RoverKinematics, RejectsInvalidGeometry)
{
  RoverGeometry geometry = unrestricted_geometry();
  geometry.wheel_radius = 0.0;
  EXPECT_THROW(RoverKinematics kinematics(geometry), std::invalid_argument);
}

TEST(RoverKinematics, NonFiniteCommandFailsClosed)
{
  RoverKinematics kinematics(unrestricted_geometry());
  const RoverCommand command = kinematics.inverse(
    std::numeric_limits<double>::quiet_NaN(), 0.0);

  EXPECT_TRUE(command.saturated);
  for (const double steering : command.steering_positions) {
    EXPECT_DOUBLE_EQ(steering, 0.0);
  }
  for (const double speed : command.wheel_angular_velocities) {
    EXPECT_DOUBLE_EQ(speed, 0.0);
  }
}

TEST(RockerDifferentialController, HoldsOppositeRockerAnglesAtEquilibrium)
{
  const RockerDifferentialCommand command = compute_rocker_differential_effort(
    0.12, -0.12, 0.0, 0.0, 1.0, 0.0, 120.0, 18.0, 120.0);

  ASSERT_TRUE(command.valid);
  EXPECT_NEAR(command.left_effort, 0.0, kTolerance);
  EXPECT_NEAR(command.right_effort, 0.0, kTolerance);
}

TEST(RockerDifferentialController, AppliesEqualEffortForSameAxisDifferential)
{
  const RockerDifferentialCommand command = compute_rocker_differential_effort(
    0.10, 0.05, 0.02, 0.01, 1.0, 0.0, 100.0, 10.0, 120.0);

  ASSERT_TRUE(command.valid);
  EXPECT_LT(command.left_effort, 0.0);
  EXPECT_NEAR(command.right_effort, command.left_effort, kTolerance);
}

TEST(RockerDifferentialController, SaturatesAndRejectsInvalidState)
{
  const RockerDifferentialCommand saturated = compute_rocker_differential_effort(
    2.0, 2.0, 0.0, 0.0, 1.0, 0.0, 120.0, 18.0, 30.0);
  ASSERT_TRUE(saturated.valid);
  EXPECT_DOUBLE_EQ(saturated.left_effort, -30.0);
  EXPECT_DOUBLE_EQ(saturated.right_effort, -30.0);

  const RockerDifferentialCommand invalid = compute_rocker_differential_effort(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0, 1.0, 0.0,
    120.0, 18.0, 30.0);
  EXPECT_FALSE(invalid.valid);
  EXPECT_DOUBLE_EQ(invalid.left_effort, 0.0);
  EXPECT_DOUBLE_EQ(invalid.right_effort, 0.0);
}

}  // namespace
}  // namespace lunar_rover_control
