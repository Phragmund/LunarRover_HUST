#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "lunar_rover_control/kinematics.hpp"

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

void set_diagonal(
  std::array<double, 36> & covariance,
  const std::vector<double> & diagonal)
{
  covariance.fill(0.0);
  for (std::size_t i = 0; i < 6U; ++i) {
    covariance[i * 6U + i] = diagonal[i];
  }
}

}  // namespace

class WheelOdometryNode : public rclcpp::Node
{
public:
  WheelOdometryNode()
  : Node("wheel_odometry_node")
  {
    steering_joints_ = declare_parameter<std::vector<std::string>>(
      "steering_joints", kDefaultSteeringJoints);
    wheel_joints_ = declare_parameter<std::vector<std::string>>(
      "wheel_joints", kDefaultWheelJoints);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    const std::string joint_states_topic =
      declare_parameter<std::string>("joint_states_topic", "/joint_states");
    const std::string odometry_topic =
      declare_parameter<std::string>("odometry_topic", "/wheel/odometry");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    maximum_integration_dt_ = declare_parameter<double>("maximum_integration_dt", 0.5);

    RoverGeometry geometry;
    geometry.wheel_radius = declare_parameter<double>("wheel_radius", 0.09);
    geometry.wheelbase = declare_parameter<double>("wheelbase", 0.52);
    geometry.track_width = declare_parameter<double>("track_width", 0.50);
    geometry.max_steering_angle =
      declare_parameter<double>("max_steering_angle", 1.0471975511965976);
    // Odometry never clamps feedback, but the shared geometry validator requires this field.
    geometry.max_wheel_angular_velocity =
      declare_parameter<double>("max_wheel_angular_velocity", 1000.0);

    const auto pose_covariance = declare_parameter<std::vector<double>>(
      "pose_covariance_diagonal",
      std::vector<double>{0.02, 0.02, 1.0e6, 1.0e6, 1.0e6, 0.05});
    const auto twist_covariance = declare_parameter<std::vector<double>>(
      "twist_covariance_diagonal",
      std::vector<double>{0.04, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 0.08});

    if (steering_joints_.size() != STEERING_COUNT ||
      wheel_joints_.size() != WHEEL_COUNT)
    {
      throw std::invalid_argument(
              "steering_joints must contain four names and wheel_joints six names");
    }
    std::unordered_set<std::string> unique_joints;
    for (const auto & joint : steering_joints_) {
      if (joint.empty() || !unique_joints.insert(joint).second) {
        throw std::invalid_argument("Joint names must be non-empty and unique");
      }
    }
    for (const auto & joint : wheel_joints_) {
      if (joint.empty() || !unique_joints.insert(joint).second) {
        throw std::invalid_argument("Joint names must be non-empty and unique");
      }
    }
    if (odom_frame_.empty() || base_frame_.empty() ||
      joint_states_topic.empty() || odometry_topic.empty())
    {
      throw std::invalid_argument("Odometry topics and frames must not be empty");
    }
    if (!std::isfinite(maximum_integration_dt_) || maximum_integration_dt_ <= 0.0) {
      throw std::invalid_argument("maximum_integration_dt must be positive");
    }
    if (pose_covariance.size() != 6U || twist_covariance.size() != 6U) {
      throw std::invalid_argument("Covariance diagonal parameters must contain six values");
    }
    for (const double value : pose_covariance) {
      if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument("Pose covariance values must be finite and non-negative");
      }
    }
    for (const double value : twist_covariance) {
      if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument("Twist covariance values must be finite and non-negative");
      }
    }
    kinematics_ = std::make_unique<RoverKinematics>(geometry);
    set_diagonal(pose_covariance_, pose_covariance);
    set_diagonal(twist_covariance_, twist_covariance);

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      odometry_topic, rclcpp::QoS(10));
    if (publish_tf_) {
      transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&WheelOdometryNode::joint_state_callback, this, std::placeholders::_1));
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    std::unordered_map<std::string, std::size_t> indices;
    indices.reserve(message->name.size());
    for (std::size_t i = 0; i < message->name.size(); ++i) {
      indices.emplace(message->name[i], i);
    }

    std::array<double, STEERING_COUNT> steering_positions{};
    for (std::size_t i = 0; i < steering_joints_.size(); ++i) {
      const auto found = indices.find(steering_joints_[i]);
      if (found == indices.end() || found->second >= message->position.size()) {
        warn_missing_joint(steering_joints_[i]);
        return;
      }
      steering_positions[i] = message->position[found->second];
    }

    std::array<double, WHEEL_COUNT> wheel_velocities{};
    for (std::size_t i = 0; i < wheel_joints_.size(); ++i) {
      const auto found = indices.find(wheel_joints_[i]);
      if (found == indices.end() || found->second >= message->velocity.size()) {
        warn_missing_joint(wheel_joints_[i]);
        return;
      }
      wheel_velocities[i] = message->velocity[found->second];
    }

    const PlanarTwist twist = kinematics_->forward(steering_positions, wheel_velocities);
    if (!twist.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Wheel odometry rejected invalid joint feedback");
      return;
    }

    rclcpp::Time stamp(message->header.stamp);
    if (stamp.nanoseconds() == 0) {
      stamp = now();
    }
    if (have_previous_stamp_) {
      const double dt = (stamp - previous_stamp_).seconds();
      if (dt < -maximum_integration_dt_) {
        RCLCPP_WARN(
          get_logger(),
          "Detected backward ROS time jump; resetting wheel odometry timestamp");
        previous_stamp_ = stamp;
        publish_odometry(stamp, twist);
        return;
      }
      if (dt <= 0.0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping non-monotonic JointState timestamp with dt=%.3f", dt);
        return;
      }
      if (dt <= maximum_integration_dt_) {
        const double midpoint_yaw = yaw_ + 0.5 * twist.angular_z * dt;
        x_ += twist.linear_x * std::cos(midpoint_yaw) * dt;
        y_ += twist.linear_x * std::sin(midpoint_yaw) * dt;
        yaw_ = std::remainder(
          yaw_ + twist.angular_z * dt, 2.0 * 3.14159265358979323846);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping wheel odometry integration with dt=%.3f", dt);
      }
    }
    previous_stamp_ = stamp;
    have_previous_stamp_ = true;
    publish_odometry(stamp, twist);
  }

  void publish_odometry(const rclcpp::Time & stamp, const PlanarTwist & twist)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = x_;
    odometry.pose.pose.position.y = y_;
    odometry.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
    odometry.pose.pose.orientation.w = std::cos(yaw_ * 0.5);
    odometry.twist.twist.linear.x = twist.linear_x;
    odometry.twist.twist.angular.z = twist.angular_z;
    odometry.pose.covariance = pose_covariance_;
    odometry.twist.covariance = twist_covariance_;
    odometry_publisher_->publish(odometry);

    if (transform_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.rotation = odometry.pose.pose.orientation;
      transform_broadcaster_->sendTransform(transform);
    }
  }

  void warn_missing_joint(const std::string & joint)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "JointState is missing required feedback for '%s'", joint.c_str());
  }

  std::vector<std::string> steering_joints_;
  std::vector<std::string> wheel_joints_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{false};
  double maximum_integration_dt_{0.5};
  std::unique_ptr<RoverKinematics> kinematics_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  std::array<double, 36> pose_covariance_{};
  std::array<double, 36> twist_covariance_{};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  bool have_previous_stamp_{false};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
};

}  // namespace lunar_rover_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lunar_rover_control::WheelOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
