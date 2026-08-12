#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace lunar_rover_mapping
{
namespace
{

struct Cell
{
  int count{0};
  double min_z{std::numeric_limits<double>::infinity()};
  double max_z{-std::numeric_limits<double>::infinity()};
  double sum_z{0.0};
};

struct VoxelIndex
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelIndexHash
{
  std::size_t operator()(const VoxelIndex & index) const
  {
    std::size_t seed = 0;
    auto combine = [&seed](const int value) {
      seed ^= std::hash<int>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    };
    combine(index.x);
    combine(index.y);
    combine(index.z);
    return seed;
  }
};

void validate_positive(const char * name, const double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
}

}  // namespace

class ElevationGridNode : public rclcpp::Node
{
public:
  ElevationGridNode()
  : Node("elevation_grid"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/points");
    elevation_topic_ = declare_parameter<std::string>(
      "elevation_topic", "/terrain/elevation_cloud");
    heightmap_topic_ = declare_parameter<std::string>(
      "heightmap_topic", "/terrain/elevation_map");
    traversability_topic_ = declare_parameter<std::string>(
      "traversability_topic", "/terrain/traversability");
    current_voxel_topic_ = declare_parameter<std::string>(
      "current_voxel_topic", "/terrain/current_voxels");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    cell_size_ = declare_parameter<double>("cell_size", 0.10);
    width_cells_ = declare_parameter<int>("width_cells", 160);
    height_cells_ = declare_parameter<int>("height_cells", 160);
    rolling_window_ = declare_parameter<bool>("rolling_window", false);
    accumulate_ = declare_parameter<bool>("accumulate", false);
    origin_x_ = declare_parameter<double>("origin_x", -8.0);
    origin_y_ = declare_parameter<double>("origin_y", -8.0);
    min_points_per_cell_ = declare_parameter<int>("min_points_per_cell", 2);
    min_z_ = declare_parameter<double>("min_z", -0.60);
    max_z_ = declare_parameter<double>("max_z", 1.80);
    heightmap_min_z_ = declare_parameter<double>("heightmap_min_z", -0.60);
    heightmap_max_z_ = declare_parameter<double>("heightmap_max_z", 1.80);
    obstacle_height_ = declare_parameter<double>("obstacle_height", 0.28);
    roughness_height_ = declare_parameter<double>("roughness_height", 0.08);
    publish_current_voxels_ = declare_parameter<bool>("publish_current_voxels", true);
    current_voxel_size_ = declare_parameter<double>("current_voxel_size", 0.10);
    transform_timeout_ = declare_parameter<double>("transform_timeout", 0.10);

    validate_parameters();

    cells_.resize(static_cast<std::size_t>(width_cells_ * height_cells_));
    elevation_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      elevation_topic_, rclcpp::QoS(2).reliable());
    heightmap_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      heightmap_topic_, rclcpp::QoS(1).reliable().transient_local());
    traversability_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      traversability_topic_, rclcpp::QoS(2).reliable());
    current_voxel_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      current_voxel_topic_, rclcpp::QoS(2).reliable());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
        point_cloud_callback(*message);
      });
  }

private:
  void validate_parameters() const
  {
    if (input_topic_.empty() || elevation_topic_.empty() || heightmap_topic_.empty() ||
      traversability_topic_.empty() || current_voxel_topic_.empty() ||
      map_frame_.empty() || base_frame_.empty())
    {
      throw std::invalid_argument("Topics and frame names must not be empty");
    }
    validate_positive("cell_size", cell_size_);
    validate_positive("obstacle_height", obstacle_height_);
    validate_positive("roughness_height", roughness_height_);
    validate_positive("current_voxel_size", current_voxel_size_);
    validate_positive("transform_timeout", transform_timeout_);
    if (width_cells_ <= 0 || height_cells_ <= 0) {
      throw std::invalid_argument("Grid dimensions must be positive");
    }
    if (min_points_per_cell_ <= 0) {
      throw std::invalid_argument("min_points_per_cell must be positive");
    }
    if (!std::isfinite(min_z_) || !std::isfinite(max_z_) || min_z_ >= max_z_) {
      throw std::invalid_argument("min_z and max_z must be finite and ordered");
    }
    if (!std::isfinite(heightmap_min_z_) || !std::isfinite(heightmap_max_z_) ||
      heightmap_min_z_ >= heightmap_max_z_)
    {
      throw std::invalid_argument("heightmap_min_z and heightmap_max_z must be finite and ordered");
    }
  }

  void point_cloud_callback(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    sensor_msgs::msg::PointCloud2 cloud_map;
    try {
      const auto transform = tf_buffer_->lookupTransform(
        map_frame_,
        cloud.header.frame_id,
        cloud.header.stamp,
        tf2::durationFromSec(transform_timeout_));
      tf2::doTransform(cloud, cloud_map, transform);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform cloud from '%s' to '%s': %s",
        cloud.header.frame_id.c_str(), map_frame_.c_str(), error.what());
      return;
    }

    double origin_x = origin_x_;
    double origin_y = origin_y_;
    if (rolling_window_) {
      double center_x = 0.0;
      double center_y = 0.0;
      try {
        const auto robot_transform = tf_buffer_->lookupTransform(
          map_frame_, base_frame_, cloud.header.stamp, tf2::durationFromSec(transform_timeout_));
        center_x = robot_transform.transform.translation.x;
        center_y = robot_transform.transform.translation.y;
      } catch (const tf2::TransformException &) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Cannot locate '%s' in '%s'; using map origin for terrain grid",
          base_frame_.c_str(), map_frame_.c_str());
      }
      origin_x = center_x - 0.5 * static_cast<double>(width_cells_) * cell_size_;
      origin_y = center_y - 0.5 * static_cast<double>(height_cells_) * cell_size_;
    }

    if (!accumulate_ || rolling_window_) {
      reset_cells();
    }
    rasterize_cloud(cloud_map, origin_x, origin_y);
    if (publish_current_voxels_) {
      publish_current_voxel_cloud(cloud.header.stamp, cloud_map);
    }
    publish_elevation_cloud(cloud.header.stamp, origin_x, origin_y);
    publish_heightmap(cloud.header.stamp, origin_x, origin_y);
    publish_traversability(cloud.header.stamp, origin_x, origin_y);
  }

  void reset_cells()
  {
    for (auto & cell : cells_) {
      cell = Cell{};
    }
  }

  void rasterize_cloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const double origin_x,
    const double origin_y)
  {
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }
      if (*z < min_z_ || *z > max_z_) {
        continue;
      }

      const int column = static_cast<int>(std::floor((*x - origin_x) / cell_size_));
      const int row = static_cast<int>(std::floor((*y - origin_y) / cell_size_));
      if (column < 0 || column >= width_cells_ || row < 0 || row >= height_cells_) {
        continue;
      }

      auto & cell = cells_[static_cast<std::size_t>(row * width_cells_ + column)];
      cell.count += 1;
      cell.min_z = std::min(cell.min_z, static_cast<double>(*z));
      cell.max_z = std::max(cell.max_z, static_cast<double>(*z));
      cell.sum_z += *z;
    }
  }

  int traversability_cost(const Cell & cell) const
  {
    if (cell.count < min_points_per_cell_) {
      return -1;
    }
    const double height_range = cell.max_z - cell.min_z;
    if (height_range >= obstacle_height_) {
      return 100;
    }
    const double normalized = std::clamp(height_range / roughness_height_, 0.0, 1.0);
    return static_cast<int>(std::round(100.0 * normalized));
  }

  void publish_elevation_cloud(
    const rclcpp::Time & stamp,
    const double origin_x,
    const double origin_y)
  {
    std::size_t valid_cells = 0;
    for (const auto & cell : cells_) {
      if (cell.count >= min_points_per_cell_) {
        ++valid_cells;
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = map_frame_;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(valid_cells);
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(valid_cells);

    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> out_intensity(cloud, "intensity");
    for (int row = 0; row < height_cells_; ++row) {
      for (int column = 0; column < width_cells_; ++column) {
        const auto & cell = cells_[static_cast<std::size_t>(row * width_cells_ + column)];
        if (cell.count < min_points_per_cell_) {
          continue;
        }
        *out_x = static_cast<float>(origin_x + (static_cast<double>(column) + 0.5) * cell_size_);
        *out_y = static_cast<float>(origin_y + (static_cast<double>(row) + 0.5) * cell_size_);
        *out_z = static_cast<float>(cell.sum_z / static_cast<double>(cell.count));
        *out_intensity = static_cast<float>(std::max(0, traversability_cost(cell)));
        ++out_x;
        ++out_y;
        ++out_z;
        ++out_intensity;
      }
    }

    elevation_publisher_->publish(cloud);
  }

  void publish_current_voxel_cloud(
    const rclcpp::Time & stamp,
    const sensor_msgs::msg::PointCloud2 & cloud_map)
  {
    std::unordered_set<VoxelIndex, VoxelIndexHash> occupied;
    occupied.reserve(static_cast<std::size_t>(cloud_map.width * cloud_map.height));

    sensor_msgs::PointCloud2ConstIterator<float> x(cloud_map, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud_map, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud_map, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }
      if (*z < min_z_ || *z > max_z_) {
        continue;
      }
      occupied.insert(
        VoxelIndex{
          static_cast<int>(std::floor(*x / current_voxel_size_)),
          static_cast<int>(std::floor(*y / current_voxel_size_)),
          static_cast<int>(std::floor(*z / current_voxel_size_))});
    }

    sensor_msgs::msg::PointCloud2 voxels;
    voxels.header.stamp = stamp;
    voxels.header.frame_id = map_frame_;
    voxels.height = 1;
    voxels.width = static_cast<std::uint32_t>(occupied.size());
    voxels.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(voxels);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(occupied.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(voxels, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(voxels, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(voxels, "z");
    sensor_msgs::PointCloud2Iterator<float> out_intensity(voxels, "intensity");
    for (const auto & index : occupied) {
      *out_x = static_cast<float>((static_cast<double>(index.x) + 0.5) * current_voxel_size_);
      *out_y = static_cast<float>((static_cast<double>(index.y) + 0.5) * current_voxel_size_);
      *out_z = static_cast<float>((static_cast<double>(index.z) + 0.5) * current_voxel_size_);
      *out_intensity = *out_z;
      ++out_x;
      ++out_y;
      ++out_z;
      ++out_intensity;
    }

    current_voxel_publisher_->publish(voxels);
  }

  void publish_heightmap(
    const rclcpp::Time & stamp,
    const double origin_x,
    const double origin_y)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = map_frame_;
    grid.info.resolution = static_cast<float>(cell_size_);
    grid.info.width = static_cast<std::uint32_t>(width_cells_);
    grid.info.height = static_cast<std::uint32_t>(height_cells_);
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(static_cast<std::size_t>(width_cells_ * height_cells_), -1);

    const double height_range = heightmap_max_z_ - heightmap_min_z_;
    for (int row = 0; row < height_cells_; ++row) {
      for (int column = 0; column < width_cells_; ++column) {
        const auto index = static_cast<std::size_t>(row * width_cells_ + column);
        const auto & cell = cells_[index];
        if (cell.count < min_points_per_cell_) {
          continue;
        }
        const double mean_z = cell.sum_z / static_cast<double>(cell.count);
        const double normalized = std::clamp(
          (mean_z - heightmap_min_z_) / height_range, 0.0, 1.0);
        grid.data[index] = static_cast<int8_t>(std::lround(normalized * 100.0));
      }
    }
    heightmap_publisher_->publish(grid);
  }

  void publish_traversability(
    const rclcpp::Time & stamp,
    const double origin_x,
    const double origin_y)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = map_frame_;
    grid.info.resolution = static_cast<float>(cell_size_);
    grid.info.width = static_cast<std::uint32_t>(width_cells_);
    grid.info.height = static_cast<std::uint32_t>(height_cells_);
    grid.info.origin.position.x = origin_x;
    grid.info.origin.position.y = origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(static_cast<std::size_t>(width_cells_ * height_cells_), -1);

    for (int row = 0; row < height_cells_; ++row) {
      for (int column = 0; column < width_cells_; ++column) {
        const auto index = static_cast<std::size_t>(row * width_cells_ + column);
        grid.data[index] = static_cast<int8_t>(traversability_cost(cells_[index]));
      }
    }

    traversability_publisher_->publish(grid);
  }

  std::string input_topic_;
  std::string elevation_topic_;
  std::string heightmap_topic_;
  std::string traversability_topic_;
  std::string current_voxel_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double cell_size_{0.10};
  int width_cells_{160};
  int height_cells_{160};
  bool rolling_window_{false};
  bool accumulate_{false};
  double origin_x_{-8.0};
  double origin_y_{-8.0};
  int min_points_per_cell_{2};
  double min_z_{-0.60};
  double max_z_{1.80};
  double heightmap_min_z_{-0.60};
  double heightmap_max_z_{1.80};
  double obstacle_height_{0.28};
  double roughness_height_{0.08};
  bool publish_current_voxels_{true};
  double current_voxel_size_{0.10};
  double transform_timeout_{0.10};
  std::vector<Cell> cells_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr elevation_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr heightmap_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_voxel_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_publisher_;
};

}  // namespace lunar_rover_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lunar_rover_mapping::ElevationGridNode>());
  rclcpp::shutdown();
  return 0;
}
