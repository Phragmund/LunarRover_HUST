#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
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
  std::vector<std::uint32_t> z_histogram;

  void reset()
  {
    count = 0;
    min_z = std::numeric_limits<double>::infinity();
    max_z = -std::numeric_limits<double>::infinity();
    sum_z = 0.0;
    z_histogram.clear();
  }
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
    wall_topic_ = declare_parameter<std::string>(
      "wall_topic", "/terrain/wall_map");
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
    adaptive_heightmap_ = declare_parameter<bool>("adaptive_heightmap", false);
    heightmap_low_percentile_ = declare_parameter<double>("heightmap_low_percentile", 0.02);
    heightmap_high_percentile_ = declare_parameter<double>("heightmap_high_percentile", 0.98);
    heightmap_padding_ = declare_parameter<double>("heightmap_padding", 0.05);
    heightmap_min_span_ = declare_parameter<double>("heightmap_min_span", 0.25);
    ground_quantile_ = declare_parameter<double>("ground_quantile", 0.20);
    z_histogram_bins_ = declare_parameter<int>("z_histogram_bins", 96);
    ceiling_filter_enabled_ = declare_parameter<bool>("ceiling_filter_enabled", true);
    ceiling_filter_min_gap_ = declare_parameter<double>("ceiling_filter_min_gap", 0.40);
    ceiling_filter_min_cluster_points_ = declare_parameter<int>(
      "ceiling_filter_min_cluster_points", 2);
    ceiling_filter_ground_cluster_max_height_ = declare_parameter<double>(
      "ceiling_filter_ground_cluster_max_height", 0.45);
    ceiling_filter_valley_ratio_ = declare_parameter<double>(
      "ceiling_filter_valley_ratio", 0.25);
    wall_height_threshold_ = declare_parameter<double>("wall_height_threshold", 0.35);
    wall_min_points_ = declare_parameter<int>("wall_min_points", 2);
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
    wall_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      wall_topic_, rclcpp::QoS(1).reliable().transient_local());
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
      wall_topic_.empty() || traversability_topic_.empty() || current_voxel_topic_.empty() ||
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
    if (heightmap_low_percentile_ < 0.0 || heightmap_low_percentile_ > 1.0 ||
      heightmap_high_percentile_ < 0.0 || heightmap_high_percentile_ > 1.0 ||
      heightmap_low_percentile_ >= heightmap_high_percentile_)
    {
      throw std::invalid_argument("heightmap percentiles must be ordered in [0, 1]");
    }
    if (!std::isfinite(heightmap_padding_) || heightmap_padding_ < 0.0) {
      throw std::invalid_argument("heightmap_padding must be finite and non-negative");
    }
    validate_positive("heightmap_min_span", heightmap_min_span_);
    if (ground_quantile_ < 0.0 || ground_quantile_ > 1.0) {
      throw std::invalid_argument("ground_quantile must be in [0, 1]");
    }
    if (z_histogram_bins_ <= 1) {
      throw std::invalid_argument("z_histogram_bins must be greater than 1");
    }
    if (!std::isfinite(ceiling_filter_min_gap_) || ceiling_filter_min_gap_ < 0.0) {
      throw std::invalid_argument("ceiling_filter_min_gap must be finite and non-negative");
    }
    if (ceiling_filter_min_cluster_points_ <= 0) {
      throw std::invalid_argument("ceiling_filter_min_cluster_points must be positive");
    }
    if (!std::isfinite(ceiling_filter_ground_cluster_max_height_) ||
      ceiling_filter_ground_cluster_max_height_ < 0.0)
    {
      throw std::invalid_argument(
        "ceiling_filter_ground_cluster_max_height must be finite and non-negative");
    }
    if (!std::isfinite(ceiling_filter_valley_ratio_) ||
      ceiling_filter_valley_ratio_ < 0.0 || ceiling_filter_valley_ratio_ > 1.0)
    {
      throw std::invalid_argument("ceiling_filter_valley_ratio must be in [0, 1]");
    }
    validate_positive("wall_height_threshold", wall_height_threshold_);
    if (wall_min_points_ <= 0) {
      throw std::invalid_argument("wall_min_points must be positive");
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
    publish_wall_map(cloud.header.stamp, origin_x, origin_y);
    publish_traversability(cloud.header.stamp, origin_x, origin_y);
  }

  void reset_cells()
  {
    for (auto & cell : cells_) {
      cell.reset();
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
      if (cell.z_histogram.empty()) {
        cell.z_histogram.assign(static_cast<std::size_t>(z_histogram_bins_), 0U);
      }
      cell.count += 1;
      cell.min_z = std::min(cell.min_z, static_cast<double>(*z));
      cell.max_z = std::max(cell.max_z, static_cast<double>(*z));
      cell.sum_z += *z;
      const double normalized = (*z - min_z_) / (max_z_ - min_z_);
      const int bin = std::clamp(
        static_cast<int>(std::floor(normalized * static_cast<double>(z_histogram_bins_))),
        0, z_histogram_bins_ - 1);
      cell.z_histogram[static_cast<std::size_t>(bin)] += 1U;
    }
  }

  double histogram_bin_center(const int bin) const
  {
    const double bin_size = (max_z_ - min_z_) / static_cast<double>(z_histogram_bins_);
    return min_z_ + (static_cast<double>(bin) + 0.5) * bin_size;
  }

  std::uint32_t histogram_count(const Cell & cell) const
  {
    if (cell.z_histogram.empty()) {
      return 0U;
    }
    return static_cast<std::uint32_t>(
      std::accumulate(cell.z_histogram.begin(), cell.z_histogram.end(), 0U));
  }

  double histogram_quantile(const Cell & cell, const double ratio, const int max_bin) const
  {
    if (cell.z_histogram.empty()) {
      return cell.min_z;
    }

    const int last_bin = std::clamp(max_bin, 0, z_histogram_bins_ - 1);
    std::uint32_t total = 0U;
    for (int bin = 0; bin <= last_bin; ++bin) {
      total += cell.z_histogram[static_cast<std::size_t>(bin)];
    }
    if (total == 0U) {
      return cell.min_z;
    }

    const auto target = static_cast<std::uint32_t>(
      std::clamp(
        std::ceil(static_cast<double>(total) * ratio),
        1.0, static_cast<double>(total)));
    std::uint32_t cumulative = 0U;
    for (int bin = 0; bin <= last_bin; ++bin) {
      cumulative += cell.z_histogram[static_cast<std::size_t>(bin)];
      if (cumulative >= target) {
        return histogram_bin_center(bin);
      }
    }
    return histogram_bin_center(last_bin);
  }

  std::uint32_t histogram_range_count(const Cell & cell, const int first_bin, const int last_bin)
    const
  {
    if (cell.z_histogram.empty()) {
      return 0U;
    }
    const int first = std::clamp(first_bin, 0, z_histogram_bins_ - 1);
    const int last = std::clamp(last_bin, 0, z_histogram_bins_ - 1);
    if (first > last) {
      return 0U;
    }
    std::uint32_t total = 0U;
    for (int bin = first; bin <= last; ++bin) {
      total += cell.z_histogram[static_cast<std::size_t>(bin)];
    }
    return total;
  }

  bool ceiling_filtered_ground(const Cell & cell, double & height) const
  {
    if (!ceiling_filter_enabled_ || cell.count < min_points_per_cell_ ||
      cell.z_histogram.empty())
    {
      return false;
    }

    const auto total = histogram_count(cell);
    if (total == 0U) {
      return false;
    }

    std::vector<int> occupied_bins;
    occupied_bins.reserve(static_cast<std::size_t>(z_histogram_bins_));
    for (int bin = 0; bin < z_histogram_bins_; ++bin) {
      if (cell.z_histogram[static_cast<std::size_t>(bin)] > 0U) {
        occupied_bins.push_back(bin);
      }
    }
    if (occupied_bins.size() < 2U) {
      return false;
    }

    const auto min_cluster_points =
      static_cast<std::uint32_t>(std::max(1, ceiling_filter_min_cluster_points_));
    const int lower_min_bin = occupied_bins.front();
    int best_lower_max_bin = -1;
    double best_score = std::numeric_limits<double>::infinity();
    double best_gap = 0.0;

    for (int lower_max_bin = lower_min_bin; lower_max_bin < z_histogram_bins_ - 1;
      ++lower_max_bin)
    {
      const auto lower_count = histogram_range_count(cell, lower_min_bin, lower_max_bin);
      if (lower_count < min_cluster_points) {
        continue;
      }

      const double lower_height =
        histogram_bin_center(lower_max_bin) - histogram_bin_center(lower_min_bin);
      if (lower_height > ceiling_filter_ground_cluster_max_height_) {
        break;
      }

      for (int upper_min_bin = lower_max_bin + 1; upper_min_bin < z_histogram_bins_;
        ++upper_min_bin)
      {
        const double vertical_gap =
          histogram_bin_center(upper_min_bin) - histogram_bin_center(lower_max_bin);
        if (vertical_gap < ceiling_filter_min_gap_) {
          continue;
        }

        const auto upper_count = histogram_range_count(cell, upper_min_bin, z_histogram_bins_ - 1);
        if (upper_count < min_cluster_points) {
          continue;
        }

        const auto valley_count = histogram_range_count(cell, lower_max_bin + 1, upper_min_bin - 1);
        const auto side_support = std::min(lower_count, upper_count);
        const double valley_ratio = side_support > 0U ?
          static_cast<double>(valley_count) / static_cast<double>(side_support) : 1.0;
        if (valley_ratio > ceiling_filter_valley_ratio_) {
          continue;
        }

        const double score = valley_ratio - 0.01 * vertical_gap;
        if (score < best_score) {
          best_score = score;
          best_gap = vertical_gap;
          best_lower_max_bin = lower_max_bin;
        }
      }
    }

    if (best_lower_max_bin < 0 || best_gap < ceiling_filter_min_gap_) {
      return false;
    }

    height = histogram_quantile(cell, ground_quantile_, best_lower_max_bin);
    return std::isfinite(height);
  }

  bool raw_wall_cell(const Cell & cell) const
  {
    if (cell.count < min_points_per_cell_) {
      return false;
    }
    if ((cell.max_z - cell.min_z) < wall_height_threshold_) {
      return false;
    }
    return cell.count >= wall_min_points_;
  }

  bool wall_cell(const Cell & cell) const
  {
    double ignored_ground_z = 0.0;
    if (ceiling_filtered_ground(cell, ignored_ground_z)) {
      return false;
    }
    return raw_wall_cell(cell);
  }

  bool ground_height(const Cell & cell, double & height) const
  {
    if (cell.count < min_points_per_cell_) {
      return false;
    }
    if (ceiling_filtered_ground(cell, height)) {
      return true;
    }
    if (raw_wall_cell(cell)) {
      return false;
    }

    height = histogram_quantile(cell, ground_quantile_, z_histogram_bins_ - 1);
    return std::isfinite(height);
  }

  static double percentile(std::vector<double> values, const double ratio)
  {
    if (values.empty()) {
      return 0.0;
    }
    const auto index = static_cast<std::size_t>(
      std::clamp(
        std::round((static_cast<double>(values.size() - 1U)) * ratio),
        0.0, static_cast<double>(values.size() - 1U)));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
    return values[index];
  }

  void heightmap_bounds(double & min_height, double & max_height) const
  {
    min_height = heightmap_min_z_;
    max_height = heightmap_max_z_;
    if (!adaptive_heightmap_) {
      return;
    }

    std::vector<double> ground_heights;
    ground_heights.reserve(cells_.size());
    for (const auto & cell : cells_) {
      double ground_z = 0.0;
      if (!ground_height(cell, ground_z)) {
        continue;
      }
      ground_heights.push_back(ground_z);
    }
    if (ground_heights.empty()) {
      return;
    }

    min_height = percentile(ground_heights, heightmap_low_percentile_) - heightmap_padding_;
    max_height = percentile(ground_heights, heightmap_high_percentile_) + heightmap_padding_;
    if ((max_height - min_height) < heightmap_min_span_) {
      const double center = 0.5 * (min_height + max_height);
      min_height = center - 0.5 * heightmap_min_span_;
      max_height = center + 0.5 * heightmap_min_span_;
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
      double ground_z = 0.0;
      if (ground_height(cell, ground_z)) {
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
        double ground_z = 0.0;
        if (!ground_height(cell, ground_z)) {
          continue;
        }
        *out_x = static_cast<float>(origin_x + (static_cast<double>(column) + 0.5) * cell_size_);
        *out_y = static_cast<float>(origin_y + (static_cast<double>(row) + 0.5) * cell_size_);
        *out_z = static_cast<float>(ground_z);
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

    double min_height = 0.0;
    double max_height = 0.0;
    heightmap_bounds(min_height, max_height);
    const double height_range = max_height - min_height;
    for (int row = 0; row < height_cells_; ++row) {
      for (int column = 0; column < width_cells_; ++column) {
        const auto index = static_cast<std::size_t>(row * width_cells_ + column);
        const auto & cell = cells_[index];
        double ground_z = 0.0;
        if (!ground_height(cell, ground_z)) {
          continue;
        }
        const double normalized = std::clamp(
          (ground_z - min_height) / height_range, 0.0, 1.0);
        grid.data[index] = static_cast<int8_t>(std::lround(normalized * 100.0));
      }
    }
    heightmap_publisher_->publish(grid);
  }

  void publish_wall_map(
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
        const auto & cell = cells_[index];
        if (wall_cell(cell)) {
          grid.data[index] = 100;
        }
      }
    }

    wall_publisher_->publish(grid);
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
  std::string wall_topic_;
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
  bool adaptive_heightmap_{false};
  double heightmap_low_percentile_{0.02};
  double heightmap_high_percentile_{0.98};
  double heightmap_padding_{0.05};
  double heightmap_min_span_{0.25};
  double ground_quantile_{0.20};
  int z_histogram_bins_{96};
  bool ceiling_filter_enabled_{true};
  double ceiling_filter_min_gap_{0.40};
  int ceiling_filter_min_cluster_points_{2};
  double ceiling_filter_ground_cluster_max_height_{0.45};
  double ceiling_filter_valley_ratio_{0.25};
  double wall_height_threshold_{0.35};
  int wall_min_points_{2};
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
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr wall_publisher_;
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
