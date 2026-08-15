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
    exploration_share = Path(get_package_share_directory("lunar_rover_exploration"))

    composite_config = mapping_share / "config" / "composite_odometry.yaml"
    terrain_config = mapping_share / "config" / "terrain_mapper.yaml"
    nav2_params = navigation_share / "config" / "nav2_fast_lio2_params.yaml"
    exploration_params = exploration_share / "config" / "frontier_explorer.yaml"

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
            DeclareLaunchArgument("spawn_z", default_value="0.45"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "map_origin_z",
                default_value=LaunchConfiguration("spawn_z"),
                description=(
                    "Gazebo world Z coordinate that maps to RViz map z=0 for "
                    "static world/collision markers. Defaults to spawn_z so "
                    "marker alignment follows the initial robot pose."
                ),
            ),
            DeclareLaunchArgument(
                "collision_marker_z_offset",
                default_value="0.20",
                description=(
                    "Extra RViz-only Z offset for /simulation/world_collision_markers."
                ),
            ),
            DeclareLaunchArgument("start_exploration", default_value="true"),
            DeclareLaunchArgument("run_fast_lio2", default_value="true"),
            DeclareLaunchArgument("fast_lio_package", default_value="auto"),
            DeclareLaunchArgument("fast_lio_launch_file", default_value="auto"),
            DeclareLaunchArgument("direct_fast_lio_node", default_value="true"),
            DeclareLaunchArgument("imu_mount_pitch_source", default_value="fixed"),
            DeclareLaunchArgument("imu_mount_pitch_deg", default_value="35.273"),
            DeclareLaunchArgument("imu_mount_pitch_samples", default_value="80"),
            DeclareLaunchArgument("imu_mount_pitch_max_gyro_radps", default_value="0.02"),
            DeclareLaunchArgument("level_correction_source", default_value="fixed"),
            DeclareLaunchArgument("level_correction_samples", default_value="40"),
            DeclareLaunchArgument("level_correction_start_delay_sec", default_value="8.0"),
            DeclareLaunchArgument("level_correction_preserve_yaw", default_value="true"),
            DeclareLaunchArgument("level_correction_anchor_position", default_value="true"),
            DeclareLaunchArgument("level_correction_roll_deg", default_value="-0.58"),
            DeclareLaunchArgument("level_correction_pitch_deg", default_value="-0.67"),
            DeclareLaunchArgument("level_correction_yaw_deg", default_value="0.007"),
            DeclareLaunchArgument("fast_lio_start_delay_sec", default_value="6.0"),
            DeclareLaunchArgument("terrain_start_delay_sec", default_value="9.0"),
            DeclareLaunchArgument("navigation_start_delay_sec", default_value="12.0"),
            DeclareLaunchArgument("exploration_start_delay_sec", default_value="32.0"),
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
                period=LaunchConfiguration("fast_lio_start_delay_sec"),
                actions=[
                    _include(
                        "lunar_rover_localization",
                        "fast_lio2_localization.launch.py",
                        {
                            "run_fast_lio2": LaunchConfiguration("run_fast_lio2"),
                            "fast_lio_package": LaunchConfiguration("fast_lio_package"),
                            "fast_lio_launch_file": LaunchConfiguration("fast_lio_launch_file"),
                            "direct_fast_lio_node": LaunchConfiguration("direct_fast_lio_node"),
                            "imu_mount_pitch_source": LaunchConfiguration(
                                "imu_mount_pitch_source"
                            ),
                            "imu_mount_pitch_deg": LaunchConfiguration(
                                "imu_mount_pitch_deg"
                            ),
                            "imu_mount_pitch_samples": LaunchConfiguration(
                                "imu_mount_pitch_samples"
                            ),
                            "imu_mount_pitch_max_gyro_radps": LaunchConfiguration(
                                "imu_mount_pitch_max_gyro_radps"
                            ),
                            "level_correction_source": LaunchConfiguration(
                                "level_correction_source"
                            ),
                            "level_correction_samples": LaunchConfiguration(
                                "level_correction_samples"
                            ),
                            "level_correction_start_delay_sec": LaunchConfiguration(
                                "level_correction_start_delay_sec"
                            ),
                            "level_correction_preserve_yaw": LaunchConfiguration(
                                "level_correction_preserve_yaw"
                            ),
                            "level_correction_anchor_position": LaunchConfiguration(
                                "level_correction_anchor_position"
                            ),
                            "level_correction_roll_deg": LaunchConfiguration(
                                "level_correction_roll_deg"
                            ),
                            "level_correction_pitch_deg": LaunchConfiguration(
                                "level_correction_pitch_deg"
                            ),
                            "level_correction_yaw_deg": LaunchConfiguration(
                                "level_correction_yaw_deg"
                            ),
                        },
                    )
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("terrain_start_delay_sec"),
                actions=[
                    _include(
                        "lunar_rover_mapping",
                        "terrain_mapping.launch.py",
                        {"params_file": str(terrain_config)},
                    ),
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("navigation_start_delay_sec"),
                actions=[
                    _include(
                        "lunar_rover_navigation",
                        "navigation.launch.py",
                        {"params_file": str(nav2_params)},
                    )
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("exploration_start_delay_sec"),
                actions=[
                    _include(
                        "lunar_rover_exploration",
                        "frontier_exploration.launch.py",
                        {"params_file": str(exploration_params)},
                    )
                ],
                condition=IfCondition(LaunchConfiguration("start_exploration")),
            ),
            Node(
                package="lunar_rover_simulation",
                executable="world_model_marker_node.py",
                name="world_model_marker",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_name": LaunchConfiguration("world"),
                        "frame_id": "map",
                        "marker_topic": "/simulation/world_model_markers",
                        "alpha": 0.30,
                        "outline_alpha": 0.30,
                        "outline_width": 0.025,
                        "map_origin_world_x": LaunchConfiguration("spawn_x"),
                        "map_origin_world_y": LaunchConfiguration("spawn_y"),
                        "map_origin_world_z": LaunchConfiguration("map_origin_z"),
                        "map_origin_world_yaw": LaunchConfiguration("spawn_yaw"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="lunar_rover_simulation",
                executable="world_model_marker_node.py",
                name="world_collision_marker",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_name": LaunchConfiguration("world"),
                        "frame_id": "map",
                        "marker_topic": "/simulation/world_collision_markers",
                        "include_visual": False,
                        "include_collision": True,
                        "marker_namespace": "world_collision",
                        "outline_namespace": "world_collision_outline",
                        "alpha": 0.16,
                        "outline_alpha": 0.85,
                        "outline_width": 0.010,
                        "pit_marker_alpha": 0.10,
                        "color_r": 0.05,
                        "color_g": 0.85,
                        "color_b": 1.0,
                        "outline_color_r": 0.05,
                        "outline_color_g": 0.95,
                        "outline_color_b": 1.0,
                        "marker_z_offset": LaunchConfiguration(
                            "collision_marker_z_offset"
                        ),
                        "map_origin_world_x": LaunchConfiguration("spawn_x"),
                        "map_origin_world_y": LaunchConfiguration("spawn_y"),
                        "map_origin_world_z": LaunchConfiguration("map_origin_z"),
                        "map_origin_world_yaw": LaunchConfiguration("spawn_yaw"),
                    }
                ],
                output="screen",
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
