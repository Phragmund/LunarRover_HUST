"""Start the reachable maze SLAM visualization with a simple wall-following task."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_launch = (
        Path(get_package_share_directory("lunar_rover_bringup"))
        / "launch"
        / "wheel_imu_slam_3d_test.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("sensor_noise", default_value="false"),
            DeclareLaunchArgument("spawn_x", default_value="-3.92"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.22"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup_launch)),
                launch_arguments={
                    "world": "reachable_maze",
                    "gui": LaunchConfiguration("gui"),
                    "headless": "false",
                    "rviz": LaunchConfiguration("rviz"),
                    "sensor_noise": LaunchConfiguration("sensor_noise"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                }.items(),
            ),
            Node(
                package="lunar_rover_missions",
                executable="maze_mapping_mission",
                name="maze_mapping_mission",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "command_topic": "/chassis/cmd_vel",
                        "odometry_topic": "/simulation/ground_truth/odometry",
                        "cloud_topic": "/livox/points",
                        "angular_command_sign": 1.0,
                    }
                ],
            ),
        ]
    )
