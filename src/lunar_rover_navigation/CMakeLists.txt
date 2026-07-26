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
find_package(tf2_ros REQUIRED)

add_library(terrain_layer SHARED src/terrain_layer.cpp)
target_include_directories(terrain_layer
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(terrain_layer
  geometry_msgs
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  tf2
  tf2_ros)

pluginlib_export_plugin_description_file(
  nav2_costmap_2d terrain_layer_plugin.xml)

install(
  TARGETS terrain_layer
  EXPORT export_lunar_rover_navigation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES terrain_layer_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_terrain_cost test/test_terrain_cost.cpp)
  target_include_directories(test_terrain_cost PRIVATE include)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_lunar_rover_navigation HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs nav2_costmap_2d nav_msgs pluginlib rclcpp tf2 tf2_ros)
ament_package()
