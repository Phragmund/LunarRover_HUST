cmake_minimum_required(VERSION 3.8)
project(lunar_rover_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(terrain_analyzer src/terrain_analyzer.cpp)
target_include_directories(terrain_analyzer
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

add_executable(terrain_perception_node
  src/terrain_perception_main.cpp
  src/terrain_perception_node.cpp)
target_include_directories(terrain_perception_node
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(terrain_perception_node terrain_analyzer)
ament_target_dependencies(terrain_perception_node
  geometry_msgs
  grid_map_core
  grid_map_msgs
  grid_map_ros
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros)

install(
  TARGETS terrain_analyzer terrain_perception_node
  EXPORT export_lunar_rover_perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/assembled_3d_mapper.py
    scripts/simple_2d_mapper.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_terrain_analyzer test/test_terrain_analyzer.cpp)
  target_link_libraries(test_terrain_analyzer terrain_analyzer)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_lunar_rover_perception HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
