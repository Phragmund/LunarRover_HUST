#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{
int field_offset(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return static_cast<int>(field.offset);
    }
  }
  return -1;
}

float read_float(const uint8_t * point, int offset, float fallback = 0.0F)
{
  if (offset < 0) {
    return fallback;
  }
  float value = fallback;
  std::memcpy(&value, point + offset, sizeof(float));
  return value;
}
}  // namespace

class Mid360SimAdapter : public rclcpp::Node
{
public:
  Mid360SimAdapter()
  : Node("mid360_sim_adapter"),
    rng_(declare_parameter<int>("random_seed", 360)),
    unit_normal_(0.0, 1.0),
    unit_uniform_(0.0, 1.0)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/sim/livox/points");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/livox/points");
    custom_topic_ = declare_parameter<std::string>("custom_topic", "/livox/lidar");
    output_frame_ = declare_parameter<std::string>("output_frame", "livox_frame");
    noise_enabled_ = declare_parameter<bool>("noise_enabled", true);
    scan_period_ = declare_parameter<double>("scan_period", 0.1);
    noise_stddev_ = declare_parameter<double>("noise_stddev", 0.02);
    noise_slope_ = declare_parameter<double>("noise_slope", 0.0005);
    dropout_probability_ = declare_parameter<double>("dropout_probability", 0.01);
    min_range_ = declare_parameter<double>("min_range", 0.1);
    max_range_ = declare_parameter<double>("max_range", 40.0);
    timestamp_jitter_stddev_ =
      declare_parameter<double>("timestamp_jitter_stddev", 0.0002);
    if (
      scan_period_ <= 0.0 || noise_stddev_ < 0.0 || noise_slope_ < 0.0 ||
      timestamp_jitter_stddev_ < 0.0 || dropout_probability_ < 0.0 ||
      dropout_probability_ > 1.0 || min_range_ < 0.0 || max_range_ <= min_range_)
    {
      throw std::invalid_argument("Invalid Mid-360 simulation noise parameters");
    }

    auto sensor_qos = rclcpp::SensorDataQoS();
    auto bridge_input_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto livox_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, sensor_qos);
    custom_pub_ =
      create_publisher<livox_ros_driver2::msg::CustomMsg>(custom_topic_, livox_qos);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, bridge_input_qos,
      std::bind(&Mid360SimAdapter::cloud_callback, this, std::placeholders::_1));
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const int x_offset = field_offset(*msg, "x");
    const int y_offset = field_offset(*msg, "y");
    const int z_offset = field_offset(*msg, "z");
    const int intensity_offset = field_offset(*msg, "intensity");
    if (
      x_offset < 0 || y_offset < 0 || z_offset < 0 ||
      msg->width == 0U || msg->height == 0U || msg->point_step == 0U ||
      static_cast<size_t>(x_offset) + sizeof(float) > msg->point_step ||
      static_cast<size_t>(y_offset) + sizeof(float) > msg->point_step ||
      static_cast<size_t>(z_offset) + sizeof(float) > msg->point_step ||
      static_cast<size_t>(msg->row_step) * msg->height > msg->data.size())
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Input PointCloud2 is malformed");
      return;
    }

    const size_t input_count =
      static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    sensor_msgs::msg::PointCloud2 filtered;
    filtered.header = msg->header;
    filtered.header.frame_id = output_frame_;
    filtered.height = 1;
    filtered.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(filtered);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(input_count);
    sensor_msgs::PointCloud2Iterator<float> out_x(filtered, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(filtered, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(filtered, "z");
    sensor_msgs::PointCloud2Iterator<float> out_intensity(filtered, "intensity");

    livox_ros_driver2::msg::CustomMsg custom;
    custom.header = msg->header;
    custom.header.frame_id = output_frame_;
    custom.lidar_id = 1;
    custom.rsvd = {0, 0, 0};
    custom.points.reserve(input_count);

    const double jitter = noise_enabled_ ?
      timestamp_jitter_stddev_ * unit_normal_(rng_) : 0.0;
    rclcpp::Time stamp(msg->header.stamp);
    stamp = stamp + rclcpp::Duration::from_seconds(jitter);
    if (stamp.nanoseconds() < 0) {
      stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    const int64_t stamp_ns = stamp.nanoseconds();
    custom.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    custom.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    filtered.header.stamp = custom.header.stamp;
    custom.timebase = static_cast<uint64_t>(stamp_ns);

    size_t accepted = 0;
    for (size_t index = 0; index < input_count; ++index) {
      const size_t row = index / msg->width;
      const size_t col = index % msg->width;
      const uint8_t * raw =
        msg->data.data() + row * msg->row_step + col * msg->point_step;
      double x = read_float(raw, x_offset);
      double y = read_float(raw, y_offset);
      double z = read_float(raw, z_offset);
      const float intensity = read_float(raw, intensity_offset);
      const double range = std::sqrt(x * x + y * y + z * z);
      if (!std::isfinite(range) || range < min_range_ || range > max_range_ ||
        (noise_enabled_ && unit_uniform_(rng_) < dropout_probability_))
      {
        continue;
      }

      const double sigma = noise_enabled_ ?
        noise_stddev_ + noise_slope_ * range : 0.0;
      const double radial_noise = sigma * unit_normal_(rng_);
      const double scale = (range + radial_noise) / range;
      x *= scale;
      y *= scale;
      z *= scale;

      *out_x = static_cast<float>(x);
      *out_y = static_cast<float>(y);
      *out_z = static_cast<float>(z);
      *out_intensity = intensity;
      ++out_x;
      ++out_y;
      ++out_z;
      ++out_intensity;

      livox_ros_driver2::msg::CustomPoint point;
      point.x = static_cast<float>(x);
      point.y = static_cast<float>(y);
      point.z = static_cast<float>(z);
      point.reflectivity = static_cast<uint8_t>(
        std::clamp(std::isfinite(intensity) ? intensity : 0.0F, 0.0F, 255.0F));
      point.tag = 0;
      point.line = static_cast<uint8_t>(index % 4U);
      const double phase = input_count > 1 ?
        static_cast<double>(index) / static_cast<double>(input_count - 1) : 0.0;
      point.offset_time = static_cast<uint32_t>(
        std::clamp(phase * scan_period_ * 1e9, 0.0,
        static_cast<double>(std::numeric_limits<uint32_t>::max())));
      custom.points.push_back(point);
      ++accepted;
    }

    modifier.resize(accepted);
    filtered.width = static_cast<uint32_t>(accepted);
    custom.point_num = static_cast<uint32_t>(custom.points.size());
    cloud_pub_->publish(filtered);
    custom_pub_->publish(custom);
  }

  std::string input_topic_;
  std::string cloud_topic_;
  std::string custom_topic_;
  std::string output_frame_;
  bool noise_enabled_{true};
  double scan_period_{0.1};
  double noise_stddev_{0.02};
  double noise_slope_{0.0005};
  double dropout_probability_{0.01};
  double min_range_{0.1};
  double max_range_{40.0};
  double timestamp_jitter_stddev_{0.0002};
  std::mt19937 rng_;
  std::normal_distribution<double> unit_normal_;
  std::uniform_real_distribution<double> unit_uniform_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mid360SimAdapter>());
  rclcpp::shutdown();
  return 0;
}
