cmake_minimum_required(VERSION 3.8)
project(lunar_rover_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

add_executable(twist_stamper_node
  src/twist_stamper_node.cpp
)
ament_target_dependencies(twist_stamper_node
  geometry_msgs
  nav_msgs
  rclcpp
)

add_library(lunar_rover_navigation_terrain_layer SHARED
  src/terrain_layer.cpp
)
target_include_directories(lunar_rover_navigation_terrain_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lunar_rover_navigation_terrain_layer
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  tf2
)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins/terrain_layer.xml)
ament_export_include_directories(include)
ament_export_libraries(lunar_rover_navigation_terrain_layer)
ament_export_dependencies(nav2_costmap_2d nav_msgs pluginlib rclcpp tf2)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY routes
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/frontier_exploration_test.py
    scripts/generate_image_cave_centerline_route.py
    scripts/image_cave_waypoint_follower.py
    scripts/chassis_cmd_mux_node.py
    scripts/keyboard_rover_teleop.py
    scripts/nav2_lifecycle_bringup_node.py
    scripts/navigation_smoke_test.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS twist_stamper_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS lunar_rover_navigation_terrain_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
