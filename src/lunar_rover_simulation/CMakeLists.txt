cmake_minimum_required(VERSION 3.8)
project(lunar_rover_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch models worlds scripts
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/mid360_livox_sim_driver.py
    scripts/mid360_livox_imu_driver.py
    scripts/world_model_marker_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
