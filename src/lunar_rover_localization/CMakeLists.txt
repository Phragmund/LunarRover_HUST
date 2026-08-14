cmake_minimum_required(VERSION 3.8)
project(lunar_rover_localization)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/fast_lio2_map_frame_node.py
    scripts/fast_lio2_input_normalizer.py
    scripts/map_odom_tf_bridge.py
    scripts/sim_fast_lio_imu_adapter.py
    scripts/truth_localization_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
