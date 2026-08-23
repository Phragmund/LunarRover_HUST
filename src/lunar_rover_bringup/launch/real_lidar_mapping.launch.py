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
            _include(
                "lunar_rover_bringup",
                "bringup.launch.py",
                {
                    "mode": "real",
                    "rviz": "false",
                    "real_run_lidar": "true",
                    "real_run_camera": "false",
                    "real_lidar_frame_id": LaunchConfiguration("lidar_frame_id"),
                    "real_lidar_output_topic": LaunchConfiguration("lidar_output_topic"),
                },
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
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(mapping_rviz)],
                output="screen",
                condition=IfCondition(LaunchConfiguration("run_rviz")),
            ),
        ]
    )
