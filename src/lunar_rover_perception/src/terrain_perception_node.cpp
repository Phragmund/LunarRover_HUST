#include "lunar_rover_perception/terrain_perception_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"

namespace lunar_rover_perception
{
namespace
{

bool hasFloatField(
  const sensor_msgs::msg::PointCloud2 & message, const std::string & name)
{
  for (const auto & field : message.fields) {
    if (
      field.name == name &&
      field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      field.count == 1U)
    {
      return true;
    }
  }
  return false;
}

TerrainParameters declareTerrainParameters(rclcpp::Node & node)
{
  TerrainParameters parameters;
  parameters.resolution = node.declare_parameter<double>("map.resolution", 0.10);
  parameters.length_x = node.declare_parameter<double>("map.length_x", 10.0);
  parameters.length_y = node.declare_parameter<double>("map.length_y", 10.0);
  parameters.max_point_age = node.declare_parameter<double>("map.max_point_age", 1.5);
  parameters.required_point_weight =
    node.declare_parameter<double>("map.required_point_weight", 3.0);
  parameters.maximum_stored_cells = static_cast<std::size_t>(std::max<std::int64_t>(
      1, node.declare_parameter<std::int64_t>("map.maximum_stored_cells", 200000)));

  parameters.caution_step_height =
    node.declare_parameter<double>("thresholds.caution_step_height", 0.05);
  parameters.lethal_step_height =
    node.declare_parameter<double>("thresholds.lethal_step_height", 0.07);
  const double caution_slope_degrees =
    node.declare_parameter<double>("thresholds.caution_slope_degrees", 15.0);
  const double lethal_slope_degrees =
    node.declare_parameter<double>("thresholds.lethal_slope_degrees", 22.0);
  constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
  parameters.caution_slope_radians = caution_slope_degrees * degrees_to_radians;
  parameters.lethal_slope_radians = lethal_slope_degrees * degrees_to_radians;
  parameters.caution_roughness =
    node.declare_parameter<double>("thresholds.caution_roughness", 0.025);
  parameters.lethal_roughness =
    node.declare_parameter<double>("thresholds.lethal_roughness", 0.060);
  parameters.minimum_confidence =
    node.declare_parameter<double>("thresholds.minimum_confidence", 0.35);
  return parameters;
}

}  // namespace

TerrainPerceptionNode::TerrainPerceptionNode(const rclcpp::NodeOptions & options)
: Node("terrain_perception", options),
  analyzer_(declareTerrainParameters(*this)),
  output_frame_(declare_parameter<std::string>("frames.output", "odom")),
  robot_frame_(declare_parameter<std::string>("frames.robot", "base_footprint")),
  minimum_z_(declare_parameter<double>("filters.minimum_z", -1.0)),
  maximum_z_(declare_parameter<double>("filters.maximum_z", 1.5)),
  minimum_range_(declare_parameter<double>("filters.minimum_range", 0.20)),
  maximum_range_(declare_parameter<double>("filters.maximum_range", 20.0)),
  transform_timeout_(declare_parameter<double>("filters.transform_timeout", 0.10)),
  maximum_points_per_cloud_(static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("filters.maximum_points_per_cloud", 60000)))),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  if (
    minimum_z_ >= maximum_z_ || minimum_range_ < 0.0 ||
    minimum_range_ >= maximum_range_ || transform_timeout_ < 0.0)
  {
    throw std::invalid_argument("terrain perception filter parameters are invalid");
  }

  const auto lidar_topic =
    declare_parameter<std::string>("topics.lidar_points", "/livox/points");
  const auto depth_topic = declare_parameter<std::string>(
    "topics.depth_points", "/camera/depth_registered/points");
  const auto grid_map_topic =
    declare_parameter<std::string>("topics.grid_map", "/terrain/grid_map");
  const auto traversability_topic = declare_parameter<std::string>(
    "topics.traversability", "/terrain/traversability");
  const auto ground_topic =
    declare_parameter<std::string>("topics.ground", "/terrain/ground");
  const auto obstacle_topic =
    declare_parameter<std::string>("topics.obstacles", "/terrain/obstacles");
  const auto lidar_weight = declare_parameter<double>("sensors.lidar_weight", 1.0);
  const auto depth_weight = declare_parameter<double>("sensors.depth_weight", 0.70);
  const auto publish_frequency =
    declare_parameter<double>("publish_frequency", 5.0);
  if (lidar_weight <= 0.0 || depth_weight <= 0.0 || publish_frequency <= 0.0) {
    throw std::invalid_argument("sensor weights and publish frequency must be positive");
  }

  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  map_qos.reliable().transient_local();
  grid_map_publisher_ = create_publisher<grid_map_msgs::msg::GridMap>(
    grid_map_topic, map_qos);
  occupancy_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    traversability_topic, map_qos);
  ground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    ground_topic, rclcpp::SensorDataQoS());
  obstacle_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    obstacle_topic, rclcpp::SensorDataQoS());

  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);
  lidar_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic, sensor_qos,
    [this, lidar_weight](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
      cloudCallback(message, PointSource::kLidar, lidar_weight);
    });
  depth_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    depth_topic, sensor_qos,
    [this, depth_weight](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
      cloudCallback(message, PointSource::kDepth, depth_weight);
    });

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / publish_frequency));
  publish_timer_ = create_wall_timer(
    period, std::bind(&TerrainPerceptionNode::publishTerrain, this));

  RCLCPP_INFO(
    get_logger(),
    "Terrain perception uses %s, centered on %s, at %.2f m resolution",
    output_frame_.c_str(), robot_frame_.c_str(), analyzer_.parameters().resolution);
}

void TerrainPerceptionNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message,
  PointSource source, double source_weight)
{
  if (
    message->point_step == 0U || message->data.empty() ||
    !hasFloatField(*message, "x") || !hasFloatField(*message, "y") ||
    !hasFloatField(*message, "z"))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring PointCloud2 without FLOAT32 x/y/z fields");
    return;
  }
  const std::size_t point_count =
    static_cast<std::size_t>(message->width) * static_cast<std::size_t>(message->height);
  if (
    point_count == 0U ||
    point_count > message->data.size() / static_cast<std::size_t>(message->point_step))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring PointCloud2 whose dimensions exceed its data buffer");
    return;
  }
  if (message->header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring PointCloud2 with an empty frame_id");
    return;
  }

  geometry_msgs::msg::TransformStamped transform_message;
  try {
    transform_message = tf_buffer_.lookupTransform(
      output_frame_, message->header.frame_id, message->header.stamp,
      rclcpp::Duration::from_seconds(transform_timeout_));
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Cannot transform terrain cloud from %s to %s: %s",
      message->header.frame_id.c_str(), output_frame_.c_str(), exception.what());
    return;
  }

  const auto & translation = transform_message.transform.translation;
  const auto & rotation = transform_message.transform.rotation;
  const tf2::Transform transform(
    tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
    tf2::Vector3(translation.x, translation.y, translation.z));

  const std::size_t stride = std::max<std::size_t>(
    1U, (point_count + maximum_points_per_cloud_ - 1U) / maximum_points_per_cloud_);
  const double minimum_range_squared = minimum_range_ * minimum_range_;
  const double maximum_range_squared = maximum_range_ * maximum_range_;
  std::vector<TerrainPoint> points;
  points.reserve(std::min(point_count, maximum_points_per_cloud_));

  sensor_msgs::PointCloud2ConstIterator<float> x_iterator(*message, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_iterator(*message, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_iterator(*message, "z");
  for (std::size_t index = 0U; index < point_count;
    ++index, ++x_iterator, ++y_iterator, ++z_iterator)
  {
    if (index % stride != 0U) {
      continue;
    }
    const double x = static_cast<double>(*x_iterator);
    const double y = static_cast<double>(*y_iterator);
    const double z = static_cast<double>(*z_iterator);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    const double range_squared = x * x + y * y + z * z;
    if (range_squared < minimum_range_squared || range_squared > maximum_range_squared) {
      continue;
    }

    const tf2::Vector3 transformed = transform * tf2::Vector3(x, y, z);
    if (transformed.z() < minimum_z_ || transformed.z() > maximum_z_) {
      continue;
    }
    points.push_back(TerrainPoint{
        transformed.x(), transformed.y(), transformed.z(), source_weight, source});
  }

  rclcpp::Time message_stamp(message->header.stamp);
  if (message_stamp.nanoseconds() == 0) {
    message_stamp = now();
  }
  std::lock_guard<std::mutex> lock(analyzer_mutex_);
  analyzer_.addPoints(points, message_stamp.seconds());
}

void TerrainPerceptionNode::publishTerrain()
{
  try {
    const auto transform = tf_buffer_.lookupTransform(
      output_frame_, robot_frame_, tf2::TimePointZero,
      tf2::durationFromSec(transform_timeout_));
    center_x_ = transform.transform.translation.x;
    center_y_ = transform.transform.translation.y;
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Cannot center terrain map on %s: %s; retaining the previous center",
      robot_frame_.c_str(), exception.what());
  }

  const auto stamp = now();
  TerrainSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(analyzer_mutex_);
    snapshot = analyzer_.makeSnapshot(center_x_, center_y_, stamp.seconds());
  }

  const std::vector<std::string> layers{
    "elevation", "variance", "slope", "roughness", "step",
    "confidence", "traversability"};
  grid_map::GridMap map(layers);
  map.setFrameId(output_frame_);
  map.setBasicLayers({"elevation"});
  map.setTimestamp(static_cast<std::uint64_t>(stamp.nanoseconds()));
  map.setGeometry(
    grid_map::Length(
      static_cast<double>(snapshot.width) * snapshot.resolution,
      static_cast<double>(snapshot.height) * snapshot.resolution),
    snapshot.resolution,
    grid_map::Position(
      snapshot.origin_x +
      0.5 * static_cast<double>(snapshot.width) * snapshot.resolution,
      snapshot.origin_y +
      0.5 * static_cast<double>(snapshot.height) * snapshot.resolution));

  for (const auto & cell : snapshot.cells) {
    if (!cell.observed) {
      continue;
    }
    grid_map::Index index;
    if (!map.getIndex(grid_map::Position(cell.x, cell.y), index)) {
      continue;
    }
    map.at("elevation", index) = static_cast<float>(cell.elevation);
    map.at("variance", index) = static_cast<float>(cell.variance);
    map.at("slope", index) = static_cast<float>(cell.slope);
    map.at("roughness", index) = static_cast<float>(cell.roughness);
    map.at("step", index) = static_cast<float>(cell.step);
    map.at("confidence", index) = static_cast<float>(cell.confidence);
    if (cell.traversability >= 0) {
      map.at("traversability", index) =
        static_cast<float>(cell.traversability);
    }
  }
  grid_map_publisher_->publish(*grid_map::GridMapRosConverter::toMessage(map));

  nav_msgs::msg::OccupancyGrid occupancy;
  occupancy.header.stamp = stamp;
  occupancy.header.frame_id = output_frame_;
  occupancy.info.map_load_time = stamp;
  occupancy.info.resolution = static_cast<float>(snapshot.resolution);
  occupancy.info.width = static_cast<std::uint32_t>(snapshot.width);
  occupancy.info.height = static_cast<std::uint32_t>(snapshot.height);
  occupancy.info.origin.position.x = snapshot.origin_x;
  occupancy.info.origin.position.y = snapshot.origin_y;
  occupancy.info.origin.orientation.w = 1.0;
  occupancy.data.reserve(snapshot.cells.size());
  for (const auto & cell : snapshot.cells) {
    occupancy.data.push_back(cell.traversability);
  }
  occupancy_publisher_->publish(occupancy);
  publishPointCloud(snapshot, false, stamp, ground_publisher_);
  publishPointCloud(snapshot, true, stamp, obstacle_publisher_);
}

void TerrainPerceptionNode::publishPointCloud(
  const TerrainSnapshot & snapshot, bool obstacles,
  const rclcpp::Time & stamp,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  std::size_t output_size = 0U;
  for (const auto & cell : snapshot.cells) {
    const bool selected = obstacles ?
      cell.lethal :
      (cell.observed && cell.traversability >= 0 && !cell.lethal);
    if (selected) {
      ++output_size;
    }
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = output_frame_;
  cloud.height = 1U;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(output_size);
  sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");
  for (const auto & cell : snapshot.cells) {
    const bool selected = obstacles ?
      cell.lethal :
      (cell.observed && cell.traversability >= 0 && !cell.lethal);
    if (!selected) {
      continue;
    }
    *x_iterator = static_cast<float>(cell.x);
    *y_iterator = static_cast<float>(cell.y);
    *z_iterator = static_cast<float>(cell.elevation);
    ++x_iterator;
    ++y_iterator;
    ++z_iterator;
  }
  publisher->publish(cloud);
}

}  // namespace lunar_rover_perception
