cmake_minimum_required(VERSION 3.8)
project(lunar_rover_exploration)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/frontier_explorer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
