#include "lunar_rover_navigation/terrain_layer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace lunar_rover_navigation
{

void TerrainLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TerrainLayer lifecycle node expired during initialization");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(topic_));
  declareParameter("lethal_threshold", rclcpp::ParameterValue(lethal_threshold_));
  declareParameter("unknown_cost", rclcpp::ParameterValue(unknown_cost_));
  declareParameter("cost_scale", rclcpp::ParameterValue(cost_scale_));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(transform_tolerance_));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "topic", topic_);
  node->get_parameter(name_ + "." + "lethal_threshold", lethal_threshold_);
  node->get_parameter(name_ + "." + "unknown_cost", unknown_cost_);
  node->get_parameter(name_ + "." + "cost_scale", cost_scale_);
  node->get_parameter(name_ + "." + "transform_tolerance", transform_tolerance_);

  rclcpp::QoS qos(1);
  qos.reliable().transient_local();
  subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_, qos, std::bind(&TerrainLayer::gridCallback, this, std::placeholders::_1));

  current_ = false;
  RCLCPP_INFO(logger_, "TerrainLayer subscribed to %s", topic_.c_str());
}

void TerrainLayer::gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_grid_ = message;
  current_ = true;
}

void TerrainLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_grid_) {
    return;
  }
  const auto & info = latest_grid_->info;
  const auto costmap_frame = layered_costmap_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped grid_to_costmap;
  if (!lookupTransform(costmap_frame, latest_grid_->header.frame_id, grid_to_costmap)) {
    current_ = false;
    return;
  }

  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const double size_x = static_cast<double>(info.width) * info.resolution;
  const double size_y = static_cast<double>(info.height) * info.resolution;
  const std::array<std::pair<double, double>, 4> corners{{
    {origin_x, origin_y},
    {origin_x + size_x, origin_y},
    {origin_x, origin_y + size_y},
    {origin_x + size_x, origin_y + size_y},
  }};

  for (const auto & corner : corners) {
    double transformed_x = 0.0;
    double transformed_y = 0.0;
    transformPoint(grid_to_costmap, corner.first, corner.second, transformed_x, transformed_y);
    *min_x = std::min(*min_x, transformed_x);
    *min_y = std::min(*min_y, transformed_y);
    *max_x = std::max(*max_x, transformed_x);
    *max_y = std::max(*max_y, transformed_y);
  }
}

void TerrainLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr grid;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid = latest_grid_;
  }
  if (!grid) {
    return;
  }
  const auto costmap_frame = layered_costmap_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped costmap_to_grid;
  if (!lookupTransform(grid->header.frame_id, costmap_frame, costmap_to_grid)) {
    current_ = false;
    return;
  }

  const auto & info = grid->info;
  if (info.width == 0 || info.height == 0 || info.resolution <= 0.0) {
    return;
  }

  const int bounded_min_i = std::max(0, min_i);
  const int bounded_min_j = std::max(0, min_j);
  const int bounded_max_i = std::min(static_cast<int>(master_grid.getSizeInCellsX()), max_i);
  const int bounded_max_j = std::min(static_cast<int>(master_grid.getSizeInCellsY()), max_j);

  for (int j = bounded_min_j; j < bounded_max_j; ++j) {
    for (int i = bounded_min_i; i < bounded_max_i; ++i) {
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(i, j, wx, wy);
      double grid_x = wx;
      double grid_y = wy;
      transformPoint(costmap_to_grid, wx, wy, grid_x, grid_y);
      const int terrain_i = static_cast<int>(
        std::floor((grid_x - info.origin.position.x) / info.resolution));
      const int terrain_j = static_cast<int>(
        std::floor((grid_y - info.origin.position.y) / info.resolution));
      if (
        terrain_i < 0 || terrain_j < 0 ||
        terrain_i >= static_cast<int>(info.width) ||
        terrain_j >= static_cast<int>(info.height))
      {
        continue;
      }
      const auto index = static_cast<std::size_t>(
        terrain_j * static_cast<int>(info.width) + terrain_i);
      if (index >= grid->data.size()) {
        continue;
      }
      const unsigned char terrain_cost = terrainCostToCostmap(grid->data[index]);
      if (terrain_cost == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }
      const unsigned char old_cost = master_grid.getCost(i, j);
      const unsigned char new_cost =
        (old_cost == nav2_costmap_2d::NO_INFORMATION)
        ? terrain_cost
        : std::max(old_cost, terrain_cost);
      master_grid.setCost(i, j, new_cost);
    }
  }
  current_ = true;
}

bool TerrainLayer::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transform) const
{
  if (target_frame == source_frame) {
    transform.header.frame_id = target_frame;
    transform.child_frame_id = source_frame;
    transform.transform.rotation.w = 1.0;
    return true;
  }
  try {
    const auto timeout = tf2::durationFromSec(std::max(0.0, transform_tolerance_));
    transform = tf_->lookupTransform(target_frame, source_frame, tf2::TimePointZero, timeout);
    return true;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Cannot transform terrain grid from '%s' to '%s': %s",
      source_frame.c_str(), target_frame.c_str(), exception.what());
    return false;
  }
}

void TerrainLayer::transformPoint(
  const geometry_msgs::msg::TransformStamped & transform,
  const double source_x,
  const double source_y,
  double & target_x,
  double & target_y) const
{
  const auto & q = transform.transform.rotation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  target_x = transform.transform.translation.x + cos_yaw * source_x - sin_yaw * source_y;
  target_y = transform.transform.translation.y + sin_yaw * source_x + cos_yaw * source_y;
}

unsigned char TerrainLayer::terrainCostToCostmap(const int value) const
{
  if (value < 0) {
    if (unknown_cost_ < 0 || unknown_cost_ > 254) {
      return nav2_costmap_2d::NO_INFORMATION;
    }
    return static_cast<unsigned char>(unknown_cost_);
  }
  if (value >= lethal_threshold_) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  const int scaled = static_cast<int>(std::round(static_cast<double>(value) * cost_scale_));
  return static_cast<unsigned char>(
    std::clamp(scaled, 0, static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)));
}

void TerrainLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_grid_.reset();
  current_ = false;
}

bool TerrainLayer::isClearable()
{
  return false;
}

}  // namespace lunar_rover_navigation

PLUGINLIB_EXPORT_CLASS(lunar_rover_navigation::TerrainLayer, nav2_costmap_2d::Layer)
