"""Start the physical Livox FAST-LIO2 mapping and manual-control chain."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, launch_file, arguments=None, condition=None):
    path = Path(get_package_share_directory(package)) / "launch" / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    repo_root = Path(os.environ.get("LUNAR_ROVER_ROOT", "/home/phragmund/LunarRover"))
    mapping_share = Path(get_package_share_directory("lunar_rover_mapping"))
    mapping_rviz = mapping_share / "rviz" / "mapping.rviz"
    real_fast_lio_params = (
        repo_root
        / "src"
        / "lunar_rover_localization"
        / "config"
        / "fast_lio2_mid360_real.yaml"
    )
    real_chassis_launch = repo_root / "src" / "lunar_rover_bringup" / "launch" / "real_chassis.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_chassis", default_value="true"),
            DeclareLaunchArgument("run_rviz", default_value="true"),
            DeclareLaunchArgument("run_terrain_mapping", default_value="true"),
            DeclareLaunchArgument("run_costmap", default_value="true"),
            DeclareLaunchArgument("run_grasping", default_value="true"),
            DeclareLaunchArgument("detector_backend", default_value="auto"),
            DeclareLaunchArgument("segmentation_model", default_value=""),
            DeclareLaunchArgument("active_search", default_value="true"),
            DeclareLaunchArgument("auto_trigger_grasp", default_value="false"),
            DeclareLaunchArgument("periodic_sweep", default_value="false"),
            DeclareLaunchArgument("run_foundationpose_bridge", default_value="false"),
            DeclareLaunchArgument("run_keyboard", default_value="true"),
            DeclareLaunchArgument("enable_cloud_fusion", default_value="false"),
            DeclareLaunchArgument("enable_elevation_grid", default_value="false"),
            DeclareLaunchArgument("lidar_frame_id", default_value="base_link"),
            DeclareLaunchArgument("lidar_output_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("lio_body_frame", default_value="base_link"),
            DeclareLaunchArgument("web_port", default_value="8000"),
            DeclareLaunchArgument("web_access_token", default_value=""),
            DeclareLaunchArgument(
                "handoff_dir",
                default_value=str(repo_root / "tmp" / "3508_can_handoff"),
            ),
            DeclareLaunchArgument("steering_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument(
                "h7_port",
                default_value=LaunchConfiguration("steering_port"),
            ),
            DeclareLaunchArgument("imu_mount_pitch_samples", default_value="200"),
            DeclareLaunchArgument("imu_mount_pitch_max_gyro_radps", default_value="0.08"),
            DeclareLaunchArgument("level_correction_samples", default_value="40"),
            DeclareLaunchArgument("level_correction_start_delay_sec", default_value="5.0"),
            DeclareLaunchArgument("level_correction_max_abs_tilt_deg", default_value="35.0"),
            DeclareLaunchArgument("arm_camera_xyz", default_value="0.04 0 0.065"),
            DeclareLaunchArgument(
                "arm_camera_rpy", default_value="0 -1.57079632679 0"
            ),
            _include(
                "lunar_rover_bringup",
                "bringup.launch.py",
                {
                    "mode": "real",
                    "rviz": "false",
                    "real_run_lidar": "true",
                    "real_run_camera": LaunchConfiguration("run_grasping"),
                    "real_lidar_frame_id": LaunchConfiguration("lidar_frame_id"),
                    "real_lidar_output_topic": "/livox/lidar_raw",
                    "real_imu_output_topic": "/livox/imu_raw",
                    "arm_camera_xyz": LaunchConfiguration("arm_camera_xyz"),
                    "arm_camera_rpy": LaunchConfiguration("arm_camera_rpy"),
                },
            ),
            Node(
                package="lunar_rover_manipulation",
                executable="lidar_gate_node.py",
                name="grasp_lidar_gate",
                parameters=[
                    {
                        "cloud_input_topic": "/livox/lidar_raw",
                        "cloud_output_topic": LaunchConfiguration("lidar_output_topic"),
                        "imu_input_topic": "/livox/imu_raw",
                        "imu_output_topic": "/livox/imu",
                        "hold_imu_during_freeze": True,
                    }
                ],
                output="screen",
            ),
            # Keep the same forward-only cloud contract as the phone-started
            # real Nav2 chain.  FAST-LIO consumes /livox/lidar_front190 from
            # fast_lio2_mid360_real.yaml.
            Node(
                package="lunar_rover_localization",
                executable="rear_sector_cloud_filter.py",
                name="rear_sector_cloud_filter",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("lidar_output_topic"),
                        "output_topic": "/livox/lidar_front190",
                        "base_frame": "base_link",
                        "front_half_angle_deg": 95.0,
                    }
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(real_chassis_launch)),
                launch_arguments={
                    "handoff_dir": LaunchConfiguration("handoff_dir"),
                    "steering_port": LaunchConfiguration("steering_port"),
                    "h7_port": LaunchConfiguration("h7_port"),
                    "web_port": LaunchConfiguration("web_port"),
                    "web_access_token": LaunchConfiguration("web_access_token"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("run_chassis")),
            ),
            _include(
                "lunar_rover_localization",
                "fast_lio2_localization.launch.py",
                {
                    "use_sim_time": "false",
                    "params_file": str(real_fast_lio_params),
                    "run_fast_lio2": "true",
                    "direct_fast_lio_node": "true",
                    "run_input_normalizer": "false",
                    "run_sim_imu_adapter": "false",
                    "run_tf_bridge": "false",
                    "lio_body_frame": LaunchConfiguration("lio_body_frame"),
                    "imu_mount_pitch_source": "startup_imu_gravity",
                    "imu_mount_pitch_samples": LaunchConfiguration(
                        "imu_mount_pitch_samples"
                    ),
                    "imu_mount_pitch_max_gyro_radps": LaunchConfiguration(
                        "imu_mount_pitch_max_gyro_radps"
                    ),
                    "level_correction_source": "startup_lio_pose",
                    "level_correction_samples": LaunchConfiguration(
                        "level_correction_samples"
                    ),
                    "level_correction_start_delay_sec": LaunchConfiguration(
                        "level_correction_start_delay_sec"
                    ),
                    "level_correction_max_abs_tilt_deg": LaunchConfiguration(
                        "level_correction_max_abs_tilt_deg"
                    ),
                    "level_correction_preserve_yaw": "true",
                    "level_correction_anchor_position": "true",
                },
            ),
            Node(
                package="lunar_rover_localization",
                executable="lio_odometry_tf_bridge.py",
                name="lio_odometry_tf_bridge",
                parameters=[
                    {
                        "use_sim_time": False,
                        "odom_topic": "/localization/lio_odometry",
                        "map_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "stamp_mode": "now",
                    }
                ],
                output="screen",
            ),
            _include(
                "lunar_rover_mapping",
                "terrain_mapping.launch.py",
                {
                    "use_sim_time": "false",
                    "enable_cloud_fusion": LaunchConfiguration("enable_cloud_fusion"),
                    "enable_elevation_grid": LaunchConfiguration("enable_elevation_grid"),
                },
                condition=IfCondition(LaunchConfiguration("run_terrain_mapping")),
            ),
            _include(
                "lunar_rover_navigation",
                "navigation.launch.py",
                {
                    "use_sim_time": "false",
                    "params_file": str(
                        repo_root
                        / "src"
                        / "lunar_rover_navigation"
                        / "config"
                        / "nav2_fast_lio2_params.yaml"
                    ),
                    "run_command_adapters": "false",
                },
                condition=IfCondition(LaunchConfiguration("run_costmap")),
            ),
            _include(
                "lunar_rover_manipulation",
                "real_grasping.launch.py",
                {
                    "arm_camera_xyz": LaunchConfiguration("arm_camera_xyz"),
                    "arm_camera_rpy": LaunchConfiguration("arm_camera_rpy"),
                    "detector_backend": LaunchConfiguration("detector_backend"),
                    "segmentation_model": LaunchConfiguration("segmentation_model"),
                    "active_search": LaunchConfiguration("active_search"),
                    "auto_trigger_grasp": LaunchConfiguration("auto_trigger_grasp"),
                    "periodic_sweep": LaunchConfiguration("periodic_sweep"),
                    "run_foundationpose_bridge": LaunchConfiguration(
                        "run_foundationpose_bridge"
                    ),
                },
                condition=IfCondition(LaunchConfiguration("run_grasping")),
            ),
            Node(
                package="lunar_rover_navigation",
                executable="keyboard_rover_teleop.py",
                name="keyboard_rover_teleop",
                parameters=[
                    {
                        "use_sim_time": False,
                        "command_topic": "/keyboard/chassis_cmd_vel",
                        "grasp_trigger_topic": "/grasp/trigger",
                    }
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("run_keyboard")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(mapping_rviz)],
                output="screen",
                condition=IfCondition(LaunchConfiguration("run_rviz")),
            ),
        ]
    )
