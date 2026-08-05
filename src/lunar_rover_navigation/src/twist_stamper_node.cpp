#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lunar_rover_navigation
{

class TwistStamperNode : public rclcpp::Node
{
public:
  TwistStamperNode()
  : Node("twist_stamper")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cmd_vel");
    output_topic_ = declare_parameter<std::string>("output_topic", "/chassis/cmd_vel");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.22);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 0.45);
    clearance_guard_enabled_ = declare_parameter<bool>("clearance_guard_enabled", true);
    costmap_topic_ = declare_parameter<std::string>("costmap_topic", "/local_costmap/costmap");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/localization/composite_odometry");
    front_check_start_x_ = declare_parameter<double>("front_check_start_x", 0.25);
    front_check_distance_ = declare_parameter<double>("front_check_distance", 0.35);
    rear_check_distance_ = declare_parameter<double>("rear_check_distance", 0.30);
    check_half_width_ = declare_parameter<double>("check_half_width", 0.20);
    lethal_cost_threshold_ = declare_parameter<int>("lethal_cost_threshold", 90);
    backup_speed_ = declare_parameter<double>("backup_speed", -0.08);
    backup_distance_ = declare_parameter<double>("backup_distance", 0.10);
    backup_hold_time_ = declare_parameter<double>("backup_hold_time", 1.5);
    backup_cooldown_time_ = declare_parameter<double>("backup_cooldown_time", 8.0);
    max_data_age_ = declare_parameter<double>("max_data_age", 1.0);
    validate_parameters();

    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      output_topic_, rclcpp::QoS(1).reliable());
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        publish_stamped(*message);
      });
    if (clearance_guard_enabled_) {
      costmap_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic_, rclcpp::QoS(1),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr message) {
          latest_costmap_ = message;
        });
      odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(20),
        [this](const nav_msgs::msg::Odometry::SharedPtr message) {
          latest_odom_ = message;
        });
    }
  }

private:
  void validate_parameters() const
  {
    if (input_topic_.empty() || output_topic_.empty() || frame_id_.empty()) {
      throw std::invalid_argument("Topics and frame_id must not be empty");
    }
    if (!std::isfinite(max_linear_x_) || max_linear_x_ <= 0.0 ||
      !std::isfinite(max_angular_z_) || max_angular_z_ <= 0.0)
    {
      throw std::invalid_argument("Velocity limits must be finite and positive");
    }
    if (clearance_guard_enabled_ &&
      (!std::isfinite(front_check_start_x_) || front_check_start_x_ < 0.0 ||
      !std::isfinite(front_check_distance_) || front_check_distance_ <= 0.0 ||
      !std::isfinite(rear_check_distance_) || rear_check_distance_ <= 0.0 ||
      !std::isfinite(check_half_width_) || check_half_width_ <= 0.0 ||
      !std::isfinite(backup_speed_) || backup_speed_ >= 0.0 ||
      !std::isfinite(backup_distance_) || backup_distance_ <= 0.0 ||
      !std::isfinite(backup_hold_time_) || backup_hold_time_ <= 0.0 ||
      !std::isfinite(backup_cooldown_time_) || backup_cooldown_time_ < 0.0 ||
      !std::isfinite(max_data_age_) || max_data_age_ <= 0.0 ||
      lethal_cost_threshold_ < 0 || lethal_cost_threshold_ > 100))
    {
      throw std::invalid_argument("Invalid clearance guard parameters");
    }
  }

  static double yaw_from_odom(const nav_msgs::msg::Odometry & odom)
  {
    const auto & q = odom.pose.pose.orientation;
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  bool data_fresh(const rclcpp::Time & now_time) const
  {
    if (!latest_costmap_ || !latest_odom_) {
      return false;
    }
    const auto costmap_age = (now_time - rclcpp::Time(latest_costmap_->header.stamp)).seconds();
    const auto odom_age = (now_time - rclcpp::Time(latest_odom_->header.stamp)).seconds();
    return costmap_age >= 0.0 && odom_age >= 0.0 &&
      costmap_age <= max_data_age_ && odom_age <= max_data_age_;
  }

  bool region_blocked(
    const nav_msgs::msg::OccupancyGrid & grid,
    const nav_msgs::msg::Odometry & odom,
    const double min_x,
    const double max_x) const
  {
    const auto & origin = grid.info.origin.position;
    const auto & pose = odom.pose.pose.position;
    const double yaw = yaw_from_odom(odom);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    for (unsigned int row = 0; row < grid.info.height; ++row) {
      const double world_y = origin.y + (static_cast<double>(row) + 0.5) * grid.info.resolution;
      for (unsigned int column = 0; column < grid.info.width; ++column) {
        const int value = grid.data[row * grid.info.width + column];
        if (value < lethal_cost_threshold_) {
          continue;
        }

        const double world_x =
          origin.x + (static_cast<double>(column) + 0.5) * grid.info.resolution;
        const double dx = world_x - pose.x;
        const double dy = world_y - pose.y;
        const double base_x = cos_yaw * dx + sin_yaw * dy;
        const double base_y = -sin_yaw * dx + cos_yaw * dy;
        if (min_x <= base_x && base_x <= max_x && std::abs(base_y) <= check_half_width_) {
          return true;
        }
      }
    }
    return false;
  }

  bool backup_distance_reached() const
  {
    if (!latest_odom_) {
      return false;
    }
    const auto & pose = latest_odom_->pose.pose.position;
    return std::hypot(pose.x - backup_start_x_, pose.y - backup_start_y_) >= backup_distance_;
  }

  void finish_backup(const rclcpp::Time & now_time)
  {
    backup_active_ = false;
    backup_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    cooldown_until_ = now_time + rclcpp::Duration::from_seconds(backup_cooldown_time_);
  }

  bool should_backup(const geometry_msgs::msg::Twist & twist, const rclcpp::Time & now_time)
  {
    if (!clearance_guard_enabled_ || !data_fresh(now_time)) {
      return false;
    }
    if (now_time < cooldown_until_) {
      return false;
    }
    if (twist.linear.x < -0.01) {
      return false;
    }
    const bool front_blocked = region_blocked(
      *latest_costmap_, *latest_odom_,
      front_check_start_x_, front_check_start_x_ + front_check_distance_);
    if (!front_blocked) {
      return false;
    }
    const bool rear_blocked = region_blocked(
      *latest_costmap_, *latest_odom_, -rear_check_distance_, -front_check_start_x_);
    if (rear_blocked) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Front clearance blocked but rear is also occupied; not backing up");
      return false;
    }
    backup_active_ = true;
    backup_start_x_ = latest_odom_->pose.pose.position.x;
    backup_start_y_ = latest_odom_->pose.pose.position.y;
    backup_until_ = now_time + rclcpp::Duration::from_seconds(backup_hold_time_);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Front clearance blocked; backing up %.2fm to avoid steering against obstacle",
      backup_distance_);
    return true;
  }

  void publish_stamped(const geometry_msgs::msg::Twist & twist)
  {
    const auto stamp = now();
    geometry_msgs::msg::Twist output = twist;
    bool backing_up = false;
    if (clearance_guard_enabled_ && backup_active_) {
      if (stamp < backup_until_ && !backup_distance_reached()) {
        backing_up = true;
      } else {
        finish_backup(stamp);
      }
    }
    if (!backing_up && should_backup(twist, stamp)) {
      backing_up = true;
    }
    if (backing_up) {
      output.linear.x = backup_speed_;
      output.angular.z = 0.0;
    }

    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = stamp;
    stamped.header.frame_id = frame_id_;
    stamped.twist = output;
    stamped.twist.linear.x = std::clamp(
      stamped.twist.linear.x, -max_linear_x_, max_linear_x_);
    stamped.twist.linear.y = 0.0;
    stamped.twist.linear.z = 0.0;
    stamped.twist.angular.x = 0.0;
    stamped.twist.angular.y = 0.0;
    stamped.twist.angular.z = std::clamp(
      stamped.twist.angular.z, -max_angular_z_, max_angular_z_);
    publisher_->publish(stamped);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  std::string costmap_topic_;
  std::string odom_topic_;
  double max_linear_x_{0.22};
  double max_angular_z_{0.45};
  bool clearance_guard_enabled_{true};
  double front_check_start_x_{0.25};
  double front_check_distance_{0.35};
  double rear_check_distance_{0.30};
  double check_half_width_{0.20};
  int lethal_cost_threshold_{90};
  double backup_speed_{-0.08};
  double backup_distance_{0.10};
  double backup_hold_time_{1.5};
  double backup_cooldown_time_{8.0};
  double max_data_age_{1.0};
  bool backup_active_{false};
  double backup_start_x_{0.0};
  double backup_start_y_{0.0};
  rclcpp::Time backup_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time cooldown_until_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

}  // namespace lunar_rover_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lunar_rover_navigation::TwistStamperNode>());
  rclcpp::shutdown();
  return 0;
}
