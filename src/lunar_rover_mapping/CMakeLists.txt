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
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

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

add_library(lunar_rover_rviz_plugins SHARED
  src/rainbow_occupancy_grid_display.cpp
)
ament_target_dependencies(lunar_rover_rviz_plugins
  nav_msgs
  pluginlib
  rclcpp
  rviz_common
  rviz_rendering
)
target_link_libraries(lunar_rover_rviz_plugins
  Qt5::Widgets
)
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/composite_wheel_imu_odometry_node.py
    scripts/imu_ekf_preprocessor.py
    scripts/imu_dead_reckoning_node.py
    scripts/multisensor_cloud_fusion_node.py
    scripts/odometry_path_compare.py
    scripts/terrain_mapper_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS elevation_grid_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS lunar_rover_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  FILES rviz_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_cloud_fusion
    test/test_cloud_fusion.py
    TIMEOUT 60
  )
endif()

ament_package()
