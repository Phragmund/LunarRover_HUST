"""Start the complete physical H7 manual-control chain."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory("lunar_rover_control")
    control_launch = os.path.join(control_share, "launch", "real_chassis.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "handoff_dir",
                default_value="/home/phragmund/LunarRover/tmp/3508_can_handoff",
                description="Deprecated; kept for old scripts.",
            ),
            DeclareLaunchArgument(
                "steering_port",
                default_value="/dev/ttyACM0",
                description="Deprecated alias for h7_port.",
            ),
            DeclareLaunchArgument(
                "h7_port",
                default_value=LaunchConfiguration("steering_port"),
                description="H7 USB CDC serial port.",
            ),
            DeclareLaunchArgument("web_port", default_value="8000"),
            DeclareLaunchArgument("web_access_token", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(control_launch),
                launch_arguments={
                    "steering_port": LaunchConfiguration("steering_port"),
                    "h7_port": LaunchConfiguration("h7_port"),
                }.items(),
            ),
            Node(
                package="lunar_rover_navigation",
                executable="web_joystick_server.py",
                name="web_joystick_server",
                parameters=[
                    {
                        "use_sim_time": False,
                        "port": LaunchConfiguration("web_port"),
                        "access_token": LaunchConfiguration("web_access_token"),
                    }
                ],
                output="screen",
            ),
        ]
    )
