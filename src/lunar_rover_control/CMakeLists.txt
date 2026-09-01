cmake_minimum_required(VERSION 3.8)
project(lunar_rover_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lunar_rover_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_kinematics SHARED
  src/kinematics.cpp
)
target_include_directories(${PROJECT_NAME}_kinematics PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_library(${PROJECT_NAME}_controllers SHARED
  src/rover_controller.cpp
  src/rocker_differential_controller.cpp
)
target_include_directories(${PROJECT_NAME}_controllers PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME}_controllers
  ${PROJECT_NAME}_kinematics
)
ament_target_dependencies(${PROJECT_NAME}_controllers
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

add_executable(sim_chassis_state_node
  src/sim_chassis_state_node.cpp
)
ament_target_dependencies(sim_chassis_state_node
  geometry_msgs
  lunar_rover_msgs
  rclcpp
  std_msgs
)

add_executable(joint_feedback_odometry_node
  src/joint_feedback_odometry_node.cpp
)
target_link_libraries(joint_feedback_odometry_node
  ${PROJECT_NAME}_kinematics
)
ament_target_dependencies(joint_feedback_odometry_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(
  controller_interface
  lunar_rover_control_plugins.xml
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(
  FILES lunar_rover_control_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS
    ${PROJECT_NAME}_kinematics
    ${PROJECT_NAME}_controllers
    joint_feedback_odometry_node
    sim_chassis_state_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)
install(
  PROGRAMS
    scripts/keyboard_arm_joint_controller.py
    scripts/keyboard_arm_ik_controller.py
    scripts/m3508_can_api.py
    scripts/steering_motor.py
    scripts/usb_cdc_steering.py
    scripts/h7_serial_client.py
    scripts/chassis_driver_node.py
    scripts/keyboard_h7_direct.py
    scripts/keyboard_controller.py
    scripts/motion_solver.py
    scripts/ros_web_console_server.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_test REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  ament_add_test(
    test_real_chassis_python
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND /usr/bin/python3
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_real_chassis_python.py
  )
  ament_add_test(
    test_arm_h7_mapping
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND /usr/bin/python3
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_arm_h7_mapping.py
  )
  target_link_libraries(
    test_kinematics
    ${PROJECT_NAME}_kinematics
    ${PROJECT_NAME}_controllers
  )
  target_include_directories(test_kinematics PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_kinematics)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  lunar_rover_msgs
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  tf2_ros
)
ament_package()
