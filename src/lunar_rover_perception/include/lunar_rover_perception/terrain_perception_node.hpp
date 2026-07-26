#ifndef LUNAR_ROVER_PERCEPTION__TERRAIN_PERCEPTION_NODE_HPP_
#define LUNAR_ROVER_PERCEPTION__TERRAIN_PERCEPTION_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "lunar_rover_perception/terrain_analyzer.hpp"

namespace lunar_rover_perception
{

class TerrainPerceptionNode final : public rclcpp::Node
{
public:
  explicit TerrainPerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message,
    PointSource source, double source_weight);
  void publishTerrain();
  void publishPointCloud(
    const TerrainSnapshot & snapshot, bool obstacles,
    const rclcpp::Time & stamp,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher);

  TerrainAnalyzer analyzer_;
  std::mutex analyzer_mutex_;
  std::string output_frame_;
  std::string robot_frame_;
  double minimum_z_;
  double maximum_z_;
  double minimum_range_;
  double maximum_range_;
  double transform_timeout_;
  std::size_t maximum_points_per_cloud_;
  double center_x_{0.0};
  double center_y_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_subscription_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace lunar_rover_perception

#endif  // LUNAR_ROVER_PERCEPTION__TERRAIN_PERCEPTION_NODE_HPP_
