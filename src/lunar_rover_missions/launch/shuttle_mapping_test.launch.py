"""Start IMU-wheel mapping with a fixed out-and-back motion test."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_launch = (
        Path(get_package_share_directory("lunar_rover_bringup"))
        / "launch"
        / "wheel_imu_slam_3d_test.launch.py"
    )
    rviz_config = (
        Path(get_package_share_directory("lunar_rover_navigation"))
        / "rviz"
        / "mapping_3d_slam.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("sensor_noise", default_value="false"),
            DeclareLaunchArgument("world", default_value="reachable_maze"),
            DeclareLaunchArgument("spawn_x", default_value="-3.92"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.22"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("travel_distance", default_value="2.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup_launch)),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "headless": "false",
                    "rviz": "false",
                    "sensor_noise": LaunchConfiguration("sensor_noise"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                }.items(),
            ),
            Node(
                package="lunar_rover_missions",
                executable="shuttle_mapping_mission",
                name="shuttle_mapping_mission",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "odometry_topic": "/simulation/ground_truth/odometry",
                        "command_topic": "/chassis/cmd_vel",
                        "travel_distance": ParameterValue(
                            LaunchConfiguration("travel_distance"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_mapping",
                output="screen",
                arguments=["-d", str(rviz_config)],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
