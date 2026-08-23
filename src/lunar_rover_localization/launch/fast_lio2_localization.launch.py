"""Start FAST-LIO2 and publish Nav2-compatible map->odom correction."""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_fast_lio_share(package_name):
    candidates = [package_name, "fast_lio", "fast_lio_ros2", "FAST_LIO", "FAST_LIO_ROS2"]
    tried = []
    for candidate in dict.fromkeys(value for value in candidates if value):
        tried.append(candidate)
        try:
            return Path(get_package_share_directory(candidate)), candidate
        except PackageNotFoundError:
            continue
    raise RuntimeError(
        "FAST-LIO2 package not found. Import dependencies/fast_lio2.repos "
        f"or set fast_lio_package. Tried: {', '.join(tried)}"
    )


def _resolve_launch_path(fast_lio_share, requested_launch_file, resolved_package):
    launch_dir = fast_lio_share / "launch"
    if requested_launch_file and requested_launch_file.lower() != "auto":
        launch_path = launch_dir / requested_launch_file
        if launch_path.exists():
            return launch_path
    else:
        for launch_file in (
            "mapping.launch.py",
            "fast_lio.launch.py",
            "fastlio.launch.py",
            "run.launch.py",
        ):
            launch_path = launch_dir / launch_file
            if launch_path.exists():
                return launch_path

    launch_files = sorted(path.name for path in launch_dir.glob("*.launch.py"))
    if requested_launch_file and requested_launch_file.lower() == "auto" and launch_files:
        return launch_dir / launch_files[0]
    raise RuntimeError(
        f"FAST-LIO2 launch file '{requested_launch_file}' not found in {resolved_package}. "
        f"Available launch files: {', '.join(launch_files) or 'none'}"
    )


def _launch_setup(context):
    localization_share = Path(get_package_share_directory("lunar_rover_localization"))
    params_file = LaunchConfiguration("params_file").perform(context)
    bridge_params = LaunchConfiguration("bridge_params_file").perform(context)
    map_frame_params = LaunchConfiguration("map_frame_params_file").perform(context)
    fast_lio_package = LaunchConfiguration("fast_lio_package").perform(context)
    fast_lio_launch_file = LaunchConfiguration("fast_lio_launch_file").perform(context)
    run_fast_lio2 = LaunchConfiguration("run_fast_lio2").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    map_frame_runtime_params = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "lio_body_frame": LaunchConfiguration("lio_body_frame"),
        "imu_mount_pitch_source": LaunchConfiguration("imu_mount_pitch_source"),
        "imu_mount_pitch_deg": LaunchConfiguration("imu_mount_pitch_deg"),
        "imu_mount_pitch_samples": LaunchConfiguration("imu_mount_pitch_samples"),
        "imu_mount_pitch_max_gyro_radps": LaunchConfiguration(
            "imu_mount_pitch_max_gyro_radps"
        ),
        "imu_mount_pitch_correction_deg": LaunchConfiguration(
            "imu_mount_pitch_correction_deg"
        ),
        "level_correction_source": LaunchConfiguration("level_correction_source"),
        "level_correction_samples": LaunchConfiguration("level_correction_samples"),
        "level_correction_start_delay_sec": LaunchConfiguration(
            "level_correction_start_delay_sec"
        ),
        "level_correction_max_abs_tilt_deg": LaunchConfiguration(
            "level_correction_max_abs_tilt_deg"
        ),
        "level_correction_preserve_yaw": LaunchConfiguration(
            "level_correction_preserve_yaw"
        ),
        "level_correction_anchor_position": LaunchConfiguration(
            "level_correction_anchor_position"
        ),
        "level_correction_roll_deg": LaunchConfiguration("level_correction_roll_deg"),
        "level_correction_pitch_deg": LaunchConfiguration("level_correction_pitch_deg"),
        "level_correction_yaw_deg": LaunchConfiguration("level_correction_yaw_deg"),
    }

    actions = [
        Node(
            package="lunar_rover_localization",
            executable="fast_lio2_input_normalizer.py",
            name="fast_lio2_input_normalizer",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_input_normalizer")),
        ),
        Node(
            package="lunar_rover_localization",
            executable="sim_fast_lio_imu_adapter.py",
            name="sim_fast_lio_imu_adapter",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_sim_imu_adapter")),
        ),
        Node(
            package="lunar_rover_localization",
            executable="fast_lio2_map_frame_node.py",
            name="fast_lio2_map_frame",
            parameters=[
                map_frame_params,
                map_frame_runtime_params,
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_map_frame_node")),
        ),
        Node(
            package="lunar_rover_localization",
            executable="fast_lio2_map_frame_node.py",
            name="fast_lio2_map_frame_visualization",
            parameters=[
                map_frame_params,
                map_frame_runtime_params,
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_map_frame_visualization_node")),
        ),
        Node(
            package="lunar_rover_localization",
            executable="map_odom_tf_bridge.py",
            name="map_odom_tf_bridge",
            parameters=[bridge_params],
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_tf_bridge")),
        ),
    ]
    if not run_fast_lio2:
        return actions

    fast_lio_share, resolved_package = _find_fast_lio_share(fast_lio_package)
    launch_path = _resolve_launch_path(
        fast_lio_share, fast_lio_launch_file, resolved_package
    )
    direct_node = LaunchConfiguration("direct_fast_lio_node").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )

    if direct_node:
        actions.insert(
            0,
            Node(
                package=resolved_package,
                executable="fastlio_mapping",
                name="fastlio_mapping",
                parameters=[params_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=[
                    ("/Odometry", "/localization/lio_odometry_raw"),
                    ("/path", "/localization/lio_path_raw"),
                    ("/cloud_registered", "/localization/cloud_registered_raw"),
                    ("/cloud_registered_body", "/localization/cloud_registered_body"),
                    ("/cloud_effected", "/localization/cloud_effected"),
                    ("/Laser_map", "/localization/lio_map_cloud_raw"),
                ],
                output="screen",
            ),
        )
        return actions

    actions.insert(
        0,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_path)),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "config_path": str(Path(params_file).parent),
                "config_file": Path(params_file).name,
                "rviz": "false",
            }.items(),
        ),
    )
    return actions


def generate_launch_description():
    localization_share = Path(get_package_share_directory("lunar_rover_localization"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("run_fast_lio2", default_value="true"),
            DeclareLaunchArgument("run_input_normalizer", default_value="false"),
            DeclareLaunchArgument("run_tf_bridge", default_value="true"),
            DeclareLaunchArgument("run_map_frame_node", default_value="true"),
            DeclareLaunchArgument("run_map_frame_visualization_node", default_value="true"),
            DeclareLaunchArgument("run_sim_imu_adapter", default_value="false"),
            DeclareLaunchArgument("fast_lio_package", default_value="auto"),
            DeclareLaunchArgument("fast_lio_launch_file", default_value="auto"),
            DeclareLaunchArgument("direct_fast_lio_node", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(localization_share / "config" / "fast_lio2_mid360.yaml"),
            ),
            DeclareLaunchArgument(
                "bridge_params_file",
                default_value=str(localization_share / "config" / "map_odom_tf_bridge.yaml"),
            ),
            DeclareLaunchArgument(
                "map_frame_params_file",
                default_value=str(localization_share / "config" / "fast_lio2_map_frame.yaml"),
            ),
            DeclareLaunchArgument("lio_body_frame", default_value="livox_imu_frame"),
            DeclareLaunchArgument("imu_mount_pitch_source", default_value="fixed"),
            DeclareLaunchArgument("imu_mount_pitch_deg", default_value="35.273"),
            DeclareLaunchArgument("imu_mount_pitch_samples", default_value="80"),
            DeclareLaunchArgument("imu_mount_pitch_max_gyro_radps", default_value="0.02"),
            DeclareLaunchArgument("imu_mount_pitch_correction_deg", default_value="0.0"),
            DeclareLaunchArgument("level_correction_source", default_value="fixed"),
            DeclareLaunchArgument("level_correction_samples", default_value="40"),
            DeclareLaunchArgument("level_correction_start_delay_sec", default_value="20.0"),
            DeclareLaunchArgument("level_correction_max_abs_tilt_deg", default_value="10.0"),
            DeclareLaunchArgument("level_correction_preserve_yaw", default_value="true"),
            DeclareLaunchArgument("level_correction_anchor_position", default_value="true"),
            DeclareLaunchArgument("level_correction_roll_deg", default_value="-0.58"),
            DeclareLaunchArgument("level_correction_pitch_deg", default_value="-0.67"),
            DeclareLaunchArgument("level_correction_yaw_deg", default_value="0.007"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
