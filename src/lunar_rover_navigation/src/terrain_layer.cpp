#include "lunar_rover_navigation/terrain_layer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"

namespace lunar_rover_navigation
{
namespace
{

tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose)
{
  return tf2::Transform(
    tf2::Quaternion(
      pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w),
    tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

tf2::Transform stampedToTransform(
  const geometry_msgs::msg::TransformStamped & transform)
{
  return tf2::Transform(
    tf2::Quaternion(
      transform.transform.rotation.x, transform.transform.rotation.y,
      transform.transform.rotation.z, transform.transform.rotation.w),
    tf2::Vector3(
      transform.transform.translation.x, transform.transform.translation.y,
      transform.transform.translation.z));
}

}  // namespace

void TerrainLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter(
    "topic", rclcpp::ParameterValue(std::string("/terrain/traversability")));
  declareParameter("lethal_threshold", rclcpp::ParameterValue(100));
  declareParameter("unknown_is_lethal", rclcpp::ParameterValue(true));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.10));
  declareParameter("maximum_map_age", rclcpp::ParameterValue(0.60));

  const auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TerrainLayer cannot lock its lifecycle node");
  }
  int lethal_threshold = 100;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".lethal_threshold", lethal_threshold);
  node->get_parameter(name_ + ".unknown_is_lethal", cost_parameters_.unknown_is_lethal);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".maximum_map_age", maximum_map_age_);
  cost_parameters_.lethal_threshold = static_cast<std::uint8_t>(
    std::clamp(lethal_threshold, 1, 100));
  if (transform_tolerance_ < 0.0 || maximum_map_age_ <= 0.0) {
    throw std::invalid_argument("TerrainLayer timing parameters are invalid");
  }

  global_frame_ = layered_costmap_->getGlobalFrameID();
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.reliable().transient_local();
  subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_, qos,
    std::bind(&TerrainLayer::incomingMap, this, std::placeholders::_1));
  current_ = false;
  RCLCPP_INFO(
    logger_, "TerrainLayer subscribes to %s in costmap frame %s",
    topic_.c_str(), global_frame_.c_str());
}

void TerrainLayer::incomingMap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  const std::size_t expected_size =
    static_cast<std::size_t>(message->info.width) *
    static_cast<std::size_t>(message->info.height);
  if (
    message->header.frame_id.empty() || message->info.resolution <= 0.0F ||
    message->info.width == 0U || message->info.height == 0U ||
    message->data.size() != expected_size)
  {
    RCLCPP_ERROR(logger_, "TerrainLayer rejected a malformed OccupancyGrid");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  map_ = message;
  last_received_ = clock_->now();
  current_ = true;
}

bool TerrainLayer::mapIsFresh() const
{
  if (!map_ || last_received_.nanoseconds() == 0) {
    return false;
  }
  const double age = (clock_->now() - last_received_).seconds();
  return age <= maximum_map_age_ && age >= -transform_tolerance_;
}

void TerrainLayer::updateBounds(
  double, double, double,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr map;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_ = mapIsFresh();
    map = map_;
  }
  if (!map) {
    return;
  }

  geometry_msgs::msg::TransformStamped frame_transform;
  try {
    frame_transform = tf_->lookupTransform(
      global_frame_, map->header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & exception) {
    current_ = false;
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "TerrainLayer cannot transform %s to %s: %s",
      map->header.frame_id.c_str(), global_frame_.c_str(), exception.what());
    return;
  }

  const tf2::Transform transform =
    stampedToTransform(frame_transform) * poseToTransform(map->info.origin);
  const double width =
    static_cast<double>(map->info.width) * map->info.resolution;
  const double height =
    static_cast<double>(map->info.height) * map->info.resolution;
  const std::array<tf2::Vector3, 4> corners{
    transform * tf2::Vector3(0.0, 0.0, 0.0),
    transform * tf2::Vector3(width, 0.0, 0.0),
    transform * tf2::Vector3(0.0, height, 0.0),
    transform * tf2::Vector3(width, height, 0.0)};
  for (const auto & corner : corners) {
    *min_x = std::min(*min_x, corner.x());
    *min_y = std::min(*min_y, corner.y());
    *max_x = std::max(*max_x, corner.x());
    *max_y = std::max(*max_y, corner.y());
  }
}

void TerrainLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  nav_msgs::msg::OccupancyGrid::SharedPtr map;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map = map_;
  }
  if (!map) {
    return;
  }

  geometry_msgs::msg::TransformStamped frame_transform;
  try {
    frame_transform = tf_->lookupTransform(
      global_frame_, map->header.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException &) {
    current_ = false;
    return;
  }
  const tf2::Transform transform =
    stampedToTransform(frame_transform) * poseToTransform(map->info.origin);
  const double resolution = static_cast<double>(map->info.resolution);

  for (std::uint32_t y = 0U; y < map->info.height; ++y) {
    for (std::uint32_t x = 0U; x < map->info.width; ++x) {
      const std::size_t input_index =
        static_cast<std::size_t>(y) * map->info.width + x;
      const auto terrain_cost =
        terrainValueToCost(map->data[input_index], cost_parameters_);
      if (terrain_cost == kNoInformation) {
        continue;
      }

      const tf2::Vector3 global = transform * tf2::Vector3(
        (static_cast<double>(x) + 0.5) * resolution,
        (static_cast<double>(y) + 0.5) * resolution, 0.0);
      unsigned int master_x = 0U;
      unsigned int master_y = 0U;
      if (!master_grid.worldToMap(global.x(), global.y(), master_x, master_y)) {
        continue;
      }
      if (
        static_cast<int>(master_x) < min_i ||
        static_cast<int>(master_y) < min_j ||
        static_cast<int>(master_x) >= max_i ||
        static_cast<int>(master_y) >= max_j)
      {
        continue;
      }
      const auto previous = master_grid.getCost(master_x, master_y);
      if (
        previous == nav2_costmap_2d::NO_INFORMATION ||
        terrain_cost > previous)
      {
        master_grid.setCost(master_x, master_y, terrain_cost);
      }
    }
  }
}

void TerrainLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  map_.reset();
  last_received_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  current_ = false;
}

bool TerrainLayer::isClearable()
{
  return false;
}

}  // namespace lunar_rover_navigation

PLUGINLIB_EXPORT_CLASS(
  lunar_rover_navigation::TerrainLayer,
  nav2_costmap_2d::Layer)
