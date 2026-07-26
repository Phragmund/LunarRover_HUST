#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "lunar_rover_perception/terrain_perception_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<lunar_rover_perception::TerrainPerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
