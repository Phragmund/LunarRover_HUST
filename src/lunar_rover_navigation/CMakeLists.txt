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
find_package(rclcpp REQUIRED)

add_executable(twist_stamper_node
  src/twist_stamper_node.cpp
)
ament_target_dependencies(twist_stamper_node
  geometry_msgs
  rclcpp
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/frontier_exploration_test.py
    scripts/keyboard_rover_teleop.py
    scripts/navigation_smoke_test.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS twist_stamper_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
