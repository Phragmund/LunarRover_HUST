#ifndef LUNAR_ROVER_NAVIGATION__TERRAIN_LAYER_HPP_
#define LUNAR_ROVER_NAVIGATION__TERRAIN_LAYER_HPP_

#include <mutex>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lunar_rover_navigation
{

class TerrainLayer : public nav2_costmap_2d::Layer
{
public:
  TerrainLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override;

private:
  void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  bool lookupTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transform) const;
  void transformPoint(
    const geometry_msgs::msg::TransformStamped & transform,
    double source_x,
    double source_y,
    double & target_x,
    double & target_y) const;
  unsigned char terrainCostToCostmap(int value) const;

  std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  std::string topic_{"/terrain/traversability"};
  int lethal_threshold_{100};
  int unknown_cost_{255};
  double cost_scale_{2.52};
  double transform_tolerance_{0.20};
  bool enabled_{true};
};

}  // namespace lunar_rover_navigation

#endif  // LUNAR_ROVER_NAVIGATION__TERRAIN_LAYER_HPP_
