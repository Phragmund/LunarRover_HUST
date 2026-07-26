cmake_minimum_required(VERSION 3.8)
project(lunar_rover_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(mid360_sim_adapter src/mid360_sim_adapter.cpp)
ament_target_dependencies(mid360_sim_adapter rclcpp sensor_msgs livox_ros_driver2)

add_executable(livox_custom_to_pointcloud src/livox_custom_to_pointcloud.cpp)
ament_target_dependencies(livox_custom_to_pointcloud rclcpp sensor_msgs livox_ros_driver2)

add_executable(depth_noise_node src/depth_noise_node.cpp)
ament_target_dependencies(depth_noise_node rclcpp sensor_msgs)

install(TARGETS
  mid360_sim_adapter
  livox_custom_to_pointcloud
  depth_noise_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/depth_colormap_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
