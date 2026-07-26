#ifndef LUNAR_ROVER_NAVIGATION__TERRAIN_LAYER_HPP_
#define LUNAR_ROVER_NAVIGATION__TERRAIN_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

#include "lunar_rover_navigation/terrain_cost.hpp"

namespace lunar_rover_navigation
{

class TerrainLayer final : public nav2_costmap_2d::Layer
{
public:
  TerrainLayer() = default;
  ~TerrainLayer() override = default;

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
  void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr message);
  bool mapIsFresh() const;

  mutable std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Time last_received_{0, 0, RCL_ROS_TIME};
  TerrainCostParameters cost_parameters_;
  std::string topic_;
  std::string global_frame_;
  double transform_tolerance_{0.10};
  double maximum_map_age_{0.60};
};

}  // namespace lunar_rover_navigation

#endif  // LUNAR_ROVER_NAVIGATION__TERRAIN_LAYER_HPP_
