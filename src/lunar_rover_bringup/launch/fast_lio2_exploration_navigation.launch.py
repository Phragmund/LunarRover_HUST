"""Start the FAST-LIO2 terrain-aware exploration and navigation stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, launch_file, arguments=None):
    path = Path(get_package_share_directory(package)) / "launch" / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    mapping_share = Path(get_package_share_directory("lunar_rover_mapping"))
    navigation_share = Path(get_package_share_directory("lunar_rover_navigation"))

    composite_config = mapping_share / "config" / "composite_odometry.yaml"
    terrain_config = mapping_share / "config" / "terrain_mapper.yaml"
    nav2_params = navigation_share / "config" / "nav2_fast_lio2_params.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sim"),
            DeclareLaunchArgument("world", default_value="reachable_maze"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("mapping_rviz", default_value="true"),
            DeclareLaunchArgument("sensor_noise", default_value="false"),
            DeclareLaunchArgument("spawn_x", default_value="-3.92"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.20"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("start_exploration", default_value="true"),
            DeclareLaunchArgument("run_fast_lio2", default_value="true"),
            DeclareLaunchArgument("fast_lio_package", default_value="auto"),
            DeclareLaunchArgument("fast_lio_launch_file", default_value="auto"),
            DeclareLaunchArgument("direct_fast_lio_node", default_value="true"),
            _include(
                "lunar_rover_bringup",
                "bringup.launch.py",
                {
                    "mode": LaunchConfiguration("mode"),
                    "world": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "headless": LaunchConfiguration("headless"),
                    "rviz": "false",
                    "sensor_noise": LaunchConfiguration("sensor_noise"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                },
            ),
            Node(
                package="lunar_rover_mapping",
                executable="composite_wheel_imu_odometry_node.py",
                name="composite_wheel_imu_odometry",
                parameters=[str(composite_config)],
                output="screen",
            ),
            TimerAction(
                period=10.0,
                actions=[
                    _include(
                        "lunar_rover_localization",
                        "fast_lio2_localization.launch.py",
                        {
                            "run_fast_lio2": LaunchConfiguration("run_fast_lio2"),
                            "fast_lio_package": LaunchConfiguration("fast_lio_package"),
                            "fast_lio_launch_file": LaunchConfiguration("fast_lio_launch_file"),
                            "direct_fast_lio_node": LaunchConfiguration("direct_fast_lio_node"),
                        },
                    )
                ],
            ),
            TimerAction(
                period=14.0,
                actions=[
                    _include(
                        "lunar_rover_mapping",
                        "terrain_mapping.launch.py",
                        {"params_file": str(terrain_config)},
                    ),
                ],
            ),
            TimerAction(
                period=18.0,
                actions=[
                    _include(
                        "lunar_rover_navigation",
                        "navigation.launch.py",
                        {"params_file": str(nav2_params)},
                    )
                ],
            ),
            TimerAction(
                period=24.0,
                actions=[
                    _include(
                        "lunar_rover_exploration",
                        "frontier_exploration.launch.py",
                    )
                ],
                condition=IfCondition(LaunchConfiguration("start_exploration")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(mapping_share / "rviz" / "mapping.rviz")],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("mapping_rviz")),
                output="screen",
            ),
        ]
    )
