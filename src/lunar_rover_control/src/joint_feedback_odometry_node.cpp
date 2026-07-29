#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
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

const std::array<std::string, STEERING_COUNT> kSteeringJoints = {
  "left_front_steering_joint",
  "right_front_steering_joint",
  "left_rear_steering_joint",
  "right_rear_steering_joint"};

const std::array<std::string, WHEEL_COUNT> kDriveJoints = {
  "left_front_wheel_joint",
  "right_front_wheel_joint",
  "left_middle_wheel_joint",
  "right_middle_wheel_joint",
  "left_rear_wheel_joint",
  "right_rear_wheel_joint"};

void fill_covariance(
  std::array<double, 36> & covariance,
  const std::vector<double> & diagonal)
{
  covariance.fill(0.0);
  for (std::size_t index = 0; index < diagonal.size(); ++index) {
    covariance[index * 6U + index] = diagonal[index];
  }
}

}  // namespace

class JointFeedbackOdometryNode : public rclcpp::Node
{
public:
  JointFeedbackOdometryNode()
  : Node("joint_feedback_odometry")
  {
    joint_topic_ = declare_parameter<std::string>("joint_topic", "/joint_states");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/localization/odometry");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    maximum_dt_ = declare_parameter<double>("maximum_dt", 0.25);

    RoverGeometry geometry;
    geometry.wheel_radius = declare_parameter<double>("wheel_radius", 0.0625);
    geometry.wheelbase = declare_parameter<double>("wheelbase", 0.39);
    geometry.track_width = declare_parameter<double>("track_width", 0.345);
    geometry.max_steering_angle =
      declare_parameter<double>("max_steering_angle", 1.0471975512);
    geometry.max_wheel_angular_velocity =
      declare_parameter<double>("feedback_velocity_limit", 100.0);
    kinematics_ = std::make_unique<RoverKinematics>(geometry);

    const auto pose_diagonal = declare_parameter<std::vector<double>>(
      "pose_covariance_diagonal",
      {0.03, 0.03, 1.0e6, 1.0e6, 1.0e6, 0.06});
    const auto twist_diagonal = declare_parameter<std::vector<double>>(
      "twist_covariance_diagonal",
      {0.04, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 0.08});
    validate_parameters(pose_diagonal, twist_diagonal);
    fill_covariance(pose_covariance_, pose_diagonal);
    fill_covariance(twist_covariance_, twist_diagonal);

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, rclcpp::QoS(20));
    if (publish_tf_) {
      broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_topic_, rclcpp::SensorDataQoS(),
      std::bind(
        &JointFeedbackOdometryNode::joint_callback, this,
        std::placeholders::_1));
  }

private:
  void validate_parameters(
    const std::vector<double> & pose_diagonal,
    const std::vector<double> & twist_diagonal) const
  {
    if (joint_topic_.empty() || output_topic_.empty() ||
      odom_frame_.empty() || base_frame_.empty())
    {
      throw std::invalid_argument("Topics and frame names must not be empty");
    }
    if (!std::isfinite(maximum_dt_) || maximum_dt_ <= 0.0) {
      throw std::invalid_argument("maximum_dt must be finite and positive");
    }
    if (pose_diagonal.size() != 6U || twist_diagonal.size() != 6U) {
      throw std::invalid_argument("Covariance diagonals must have six values");
    }
    for (const auto * diagonal : {&pose_diagonal, &twist_diagonal}) {
      for (const double value : *diagonal) {
        if (!std::isfinite(value) || value < 0.0) {
          throw std::invalid_argument(
                  "Covariance values must be finite and non-negative");
        }
      }
    }
  }

  template<std::size_t Count>
  bool read_joint_values(
    const std::unordered_map<std::string, std::size_t> & indices,
    const std::array<std::string, Count> & names,
    const std::vector<double> & values,
    std::array<double, Count> & output)
  {
    for (std::size_t index = 0; index < Count; ++index) {
      const auto found = indices.find(names[index]);
      if (found == indices.end() || found->second >= values.size()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Joint feedback is missing '%s'", names[index].c_str());
        return false;
      }
      output[index] = values[found->second];
    }
    return true;
  }

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    std::unordered_map<std::string, std::size_t> indices;
    indices.reserve(message->name.size());
    for (std::size_t index = 0; index < message->name.size(); ++index) {
      indices[message->name[index]] = index;
    }

    std::array<double, STEERING_COUNT> steering{};
    std::array<double, WHEEL_COUNT> drive_velocity{};
    if (!read_joint_values(
        indices, kSteeringJoints, message->position, steering) ||
      !read_joint_values(
        indices, kDriveJoints, message->velocity, drive_velocity))
    {
      return;
    }

    const PlanarTwist twist = kinematics_->forward(steering, drive_velocity);
    if (!twist.valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected non-finite joint feedback");
      return;
    }

    rclcpp::Time stamp(message->header.stamp);
    if (stamp.nanoseconds() == 0) {
      stamp = now();
    }
    if (have_stamp_) {
      const double dt = (stamp - previous_stamp_).seconds();
      if (dt > 0.0 && dt <= maximum_dt_) {
        const double midpoint_yaw = yaw_ + 0.5 * twist.angular_z * dt;
        x_ += twist.linear_x * std::cos(midpoint_yaw) * dt;
        y_ += twist.linear_x * std::sin(midpoint_yaw) * dt;
        yaw_ = std::remainder(
          yaw_ + twist.angular_z * dt,
          2.0 * 3.14159265358979323846);
      } else if (dt < 0.0) {
        RCLCPP_WARN(
          get_logger(),
          "ROS time moved backwards; restarting odometry integration");
      } else if (dt > maximum_dt_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipped odometry integration gap of %.3f seconds", dt);
      }
    }
    previous_stamp_ = stamp;
    have_stamp_ = true;
    publish(stamp, twist);
  }

  void publish(const rclcpp::Time & stamp, const PlanarTwist & twist)
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = x_;
    odometry.pose.pose.position.y = y_;
    odometry.pose.pose.orientation.z = std::sin(0.5 * yaw_);
    odometry.pose.pose.orientation.w = std::cos(0.5 * yaw_);
    odometry.twist.twist.linear.x = twist.linear_x;
    odometry.twist.twist.angular.z = twist.angular_z;
    odometry.pose.covariance = pose_covariance_;
    odometry.twist.covariance = twist_covariance_;
    publisher_->publish(odometry);

    if (broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.rotation = odometry.pose.pose.orientation;
      broadcaster_->sendTransform(transform);
    }
  }

  std::string joint_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};
  double maximum_dt_{0.25};
  std::unique_ptr<RoverKinematics> kinematics_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::array<double, 36> pose_covariance_{};
  std::array<double, 36> twist_covariance_{};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  bool have_stamp_{false};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
};

}  // namespace lunar_rover_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<lunar_rover_control::JointFeedbackOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
