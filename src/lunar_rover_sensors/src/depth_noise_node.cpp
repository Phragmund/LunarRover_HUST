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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

class DepthNoiseNode : public rclcpp::Node
{
public:
  DepthNoiseNode()
  : Node("gemini_depth_noise"),
    rng_(declare_parameter<int>("random_seed", 335)),
    normal_(0.0, 1.0),
    uniform_(0.0, 1.0)
  {
    const auto input = declare_parameter<std::string>(
      "input_topic", "/sim/camera/depth/image_raw");
    const auto output = declare_parameter<std::string>(
      "output_topic", "/camera/depth/image_raw");
    noise_enabled_ = declare_parameter<bool>("noise_enabled", true);
    sigma_base_ = declare_parameter<double>("sigma_base", 0.002);
    sigma_quadratic_ = declare_parameter<double>("sigma_quadratic", 0.0015);
    dropout_probability_ = declare_parameter<double>("dropout_probability", 0.01);
    edge_dropout_probability_ =
      declare_parameter<double>("edge_dropout_probability", 0.15);
    edge_threshold_ = declare_parameter<double>("edge_threshold", 0.05);
    min_depth_ = declare_parameter<double>("min_depth", 0.13);
    max_depth_ = declare_parameter<double>("max_depth", 10.0);
    if (
      sigma_base_ < 0.0 || sigma_quadratic_ < 0.0 ||
      dropout_probability_ < 0.0 || dropout_probability_ > 1.0 ||
      edge_dropout_probability_ < 0.0 || edge_dropout_probability_ > 1.0 ||
      edge_threshold_ < 0.0 || min_depth_ < 0.0 || max_depth_ <= min_depth_)
    {
      throw std::invalid_argument("Invalid Gemini 335 simulation noise parameters");
    }

    auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::Image>(output, qos);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input, qos, std::bind(&DepthNoiseNode::callback, this, std::placeholders::_1));
  }

private:
  static float read_depth(const sensor_msgs::msg::Image & image, size_t index)
  {
    const size_t row = index / image.width;
    const size_t column = index % image.width;
    if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      float value;
      const size_t offset = row * image.step + column * sizeof(float);
      std::memcpy(&value, image.data.data() + offset, sizeof(float));
      return value;
    }
    uint16_t value;
    const size_t offset = row * image.step + column * sizeof(uint16_t);
    std::memcpy(&value, image.data.data() + offset, sizeof(uint16_t));
    return static_cast<float>(value) * 0.001F;
  }

  static void write_depth(sensor_msgs::msg::Image & image, size_t index, float metres)
  {
    const size_t row = index / image.width;
    const size_t column = index % image.width;
    if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      const size_t offset = row * image.step + column * sizeof(float);
      std::memcpy(image.data.data() + offset, &metres, sizeof(float));
      return;
    }
    const uint16_t millimetres = std::isfinite(metres) ?
      static_cast<uint16_t>(std::clamp(metres * 1000.0F, 0.0F, 65535.0F)) : 0U;
    const size_t offset = row * image.step + column * sizeof(uint16_t);
    std::memcpy(image.data.data() + offset, &millimetres, sizeof(uint16_t));
  }

  void callback(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1 &&
      msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Expected 32FC1 or 16UC1 depth image");
      return;
    }
    const size_t bytes_per_pixel =
      msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ?
      sizeof(float) : sizeof(uint16_t);
    if (
      msg->width == 0U || msg->height == 0U ||
      msg->step < static_cast<size_t>(msg->width) * bytes_per_pixel ||
      static_cast<size_t>(msg->step) * msg->height > msg->data.size())
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Malformed depth image");
      return;
    }
    auto output = *msg;
    const size_t count = static_cast<size_t>(msg->width) * msg->height;
    std::vector<float> source(count);
    for (size_t i = 0; i < count; ++i) {
      source[i] = read_depth(*msg, i);
    }

    for (uint32_t row = 0; row < msg->height; ++row) {
      for (uint32_t col = 0; col < msg->width; ++col) {
        const size_t index = static_cast<size_t>(row) * msg->width + col;
        float z = source[index];
        bool edge = false;
        if (col + 1 < msg->width) {
          edge |= std::abs(z - source[index + 1]) > edge_threshold_;
        }
        if (row + 1 < msg->height) {
          edge |= std::abs(z - source[index + msg->width]) > edge_threshold_;
        }
        const double dropout = noise_enabled_ ?
          std::min(
          1.0, dropout_probability_ +
          (edge ? edge_dropout_probability_ : 0.0)) : 0.0;
        if (!std::isfinite(z) || z < min_depth_ || z > max_depth_ ||
          uniform_(rng_) < dropout)
        {
          write_depth(output, index, std::numeric_limits<float>::quiet_NaN());
          continue;
        }
        const double sigma = noise_enabled_ ?
          sigma_base_ + sigma_quadratic_ * z * z : 0.0;
        z = static_cast<float>(z + sigma * normal_(rng_));
        write_depth(output, index, z);
      }
    }
    publisher_->publish(output);
  }

  double sigma_base_{0.002};
  bool noise_enabled_{true};
  double sigma_quadratic_{0.0015};
  double dropout_probability_{0.01};
  double edge_dropout_probability_{0.15};
  double edge_threshold_{0.05};
  double min_depth_{0.13};
  double max_depth_{10.0};
  std::mt19937 rng_;
  std::normal_distribution<double> normal_;
  std::uniform_real_distribution<double> uniform_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthNoiseNode>());
  rclcpp::shutdown();
  return 0;
}
