"""Start the first simulation mapping stack without Gazebo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mapping_share = get_package_share_directory("lunar_rover_mapping")
    control_share = get_package_share_directory("lunar_rover_control")

    odometry_config = os.path.join(control_share, "config", "odometry.yaml")
    slam_config = os.path.join(mapping_share, "config", "slam_toolbox.yaml")
    octomap_config = os.path.join(mapping_share, "config", "octomap.yaml")
    elevation_config = os.path.join(mapping_share, "config", "elevation_grid.yaml")
    composite_odometry_config = os.path.join(
        mapping_share, "config", "composite_odometry.yaml"
    )
    ekf_config = os.path.join(mapping_share, "config", "wheel_imu_ekf.yaml")
    rviz_config = os.path.join(mapping_share, "rviz", "mapping.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start the mapping RViz2 view.",
            ),
            DeclareLaunchArgument(
                "imu_dead_reckoning",
                default_value="false",
                description="Start diagnostic pure IMU double-integration odometry.",
            ),
            DeclareLaunchArgument(
                "ekf",
                default_value="false",
                description="Start the old wheel+IMU robot_localization EKF diagnostic.",
            ),
            Node(
                package="lunar_rover_control",
                executable="joint_feedback_odometry_node",
                name="joint_feedback_odometry",
                # Keep raw wheel odometry available for diagnostics, but do not
                # let it own odom->base_link in mapping. SLAM should use the
                # composite wheel-distance + IMU-heading TF instead.
                parameters=[odometry_config, {"publish_tf": False}],
                output="screen",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[slam_config],
                output="screen",
            ),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server",
                parameters=[octomap_config],
                remappings=[("cloud_in", "/livox/points")],
                output="screen",
            ),
            Node(
                package="lunar_rover_mapping",
                executable="elevation_grid_node",
                name="elevation_grid",
                parameters=[elevation_config],
                output="screen",
            ),
            Node(
                package="lunar_rover_mapping",
                executable="imu_dead_reckoning_node.py",
                name="imu_dead_reckoning",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("imu_dead_reckoning")),
                output="screen",
            ),
            Node(
                package="lunar_rover_mapping",
                executable="imu_ekf_preprocessor.py",
                name="imu_ekf_preprocessor",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("ekf")),
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="wheel_imu_ekf",
                parameters=[ekf_config],
                condition=IfCondition(LaunchConfiguration("ekf")),
                output="screen",
            ),
            Node(
                package="lunar_rover_mapping",
                executable="composite_wheel_imu_odometry_node.py",
                name="composite_wheel_imu_odometry",
                parameters=[composite_odometry_config],
                output="screen",
            ),
            Node(
                package="lunar_rover_mapping",
                executable="odometry_path_compare.py",
                name="odometry_path_compare",
                parameters=[{"use_sim_time": True, "max_points": 0}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="mapping_rviz",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
