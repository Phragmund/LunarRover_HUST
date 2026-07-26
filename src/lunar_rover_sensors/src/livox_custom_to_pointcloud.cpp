#include <memory>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class LivoxCustomToPointCloud : public rclcpp::Node
{
public:
  LivoxCustomToPointCloud()
  : Node("livox_custom_to_pointcloud")
  {
    const auto input = declare_parameter<std::string>("input_topic", "/livox/lidar");
    const auto output = declare_parameter<std::string>("output_topic", "/livox/points");
    const auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output, qos);
    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input, qos, [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = msg->header;
        cloud.height = 1;
        cloud.width = msg->point_num;
        cloud.is_dense = true;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(
          7,
          "x", 1, sensor_msgs::msg::PointField::FLOAT32,
          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
          "z", 1, sensor_msgs::msg::PointField::FLOAT32,
          "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
          "time", 1, sensor_msgs::msg::PointField::FLOAT32,
          "ring", 1, sensor_msgs::msg::PointField::UINT16,
          "tag", 1, sensor_msgs::msg::PointField::UINT8);
        modifier.resize(msg->points.size());
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
        sensor_msgs::PointCloud2Iterator<float> time(cloud, "time");
        sensor_msgs::PointCloud2Iterator<uint16_t> ring(cloud, "ring");
        sensor_msgs::PointCloud2Iterator<uint8_t> tag(cloud, "tag");
        for (const auto & point : msg->points) {
          *x = point.x;
          *y = point.y;
          *z = point.z;
          *intensity = static_cast<float>(point.reflectivity);
          *time = static_cast<float>(point.offset_time) * 1e-9F;
          *ring = point.line;
          *tag = point.tag;
          ++x;
          ++y;
          ++z;
          ++intensity;
          ++time;
          ++ring;
          ++tag;
        }
        cloud.width = static_cast<uint32_t>(msg->points.size());
        publisher_->publish(cloud);
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxCustomToPointCloud>());
  rclcpp::shutdown();
  return 0;
}

