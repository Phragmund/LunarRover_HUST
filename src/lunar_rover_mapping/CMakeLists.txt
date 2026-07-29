cmake_minimum_required(VERSION 3.8)
project(lunar_rover_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(elevation_grid_node
  src/elevation_grid_node.cpp
)
ament_target_dependencies(elevation_grid_node
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/composite_wheel_imu_odometry_node.py
    scripts/imu_ekf_preprocessor.py
    scripts/imu_dead_reckoning_node.py
    scripts/odometry_path_compare.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS elevation_grid_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
