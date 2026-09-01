cmake_minimum_required(VERSION 3.8)
project(lunar_rover_manipulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/active_search_node.py
    scripts/charuco_handeye_sampler_node.py
    scripts/foundationpose_input_adapter_node.py
    scripts/foundationpose_result_bridge_node.py
    scripts/grasp_coordinator_node.py
    scripts/grasp_state_machine_node.py
    scripts/lidar_gate_node.py
    scripts/target_detector_node.py
    scripts/target_fusion_detector_node.py
    scripts/target_localizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
