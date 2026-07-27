"""Start Gazebo + wheel/IMU EKF odometry + RTAB-Map 3D mapping."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, launch_file, arguments):
    path = Path(get_package_share_directory(package)) / "launch" / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    control_share = Path(get_package_share_directory("lunar_rover_control"))
    localization_share = Path(get_package_share_directory("lunar_rover_localization"))
    navigation_share = Path(get_package_share_directory("lunar_rover_navigation"))

    ekf_params = str(localization_share / "config" / "ekf_wheel_imu.yaml")
    rtabmap_params = str(localization_share / "config" / "rtabmap_wheel_imu_3d.yaml")
    rviz_config = str(navigation_share / "rviz" / "mapping_3d_slam.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="stairs_maze"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("sensor_noise", default_value="false"),
            DeclareLaunchArgument("spawn_x", default_value="-3.5"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.22"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            _include(
                "lunar_rover_simulation",
                "simulation.launch.py",
                {
                    "world": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "headless": LaunchConfiguration("headless"),
                    "rviz": "false",
                    "sensor_noise": LaunchConfiguration("sensor_noise"),
                    "controller_config": str(
                        control_share / "config" / "controllers.yaml"
                    ),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                },
            ),
            _include(
                "lunar_rover_control",
                "control.launch.py",
                {
                    "use_sim_time": "true",
                    "spawn_joint_state_broadcaster": "false",
                    "controllers_file": str(
                        control_share / "config" / "controllers.yaml"
                    ),
                    "odometry_file": str(control_share / "config" / "odometry.yaml"),
                },
            ),
            _include(
                "lunar_rover_sensors",
                "sim_sensors.launch.py",
                {"sensor_noise": LaunchConfiguration("sensor_noise")},
            ),
            Node(
                package="lunar_rover_sensors",
                executable="imu_covariance_node.py",
                name="imu_covariance_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_params, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "/odometry/filtered")],
            ),
            Node(
                package="lunar_rover_localization",
                executable="pose_tf_splitter",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                arguments=["--delete_db_on_start"],
                parameters=[rtabmap_params, {"use_sim_time": True}],
                remappings=[
                    ("scan_cloud", "/livox/points"),
                    ("odom", "/odometry/filtered"),
                    ("grid_map", "/map"),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
