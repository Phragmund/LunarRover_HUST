"""Start the physical chassis driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory("lunar_rover_control")
    config_file = os.path.join(control_share, "config", "real_chassis.yaml")

    handoff_dir = LaunchConfiguration("handoff_dir")
    steering_port = LaunchConfiguration("steering_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "handoff_dir",
                default_value="/home/phragmund/LunarRover/tmp/3508_can_handoff",
            ),
            DeclareLaunchArgument("steering_port", default_value="/dev/ttyACM0"),
            SetEnvironmentVariable(
                "LD_LIBRARY_PATH",
                [
                    handoff_dir,
                    "/dlls:",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                ],
            ),
            Node(
                package="lunar_rover_control",
                executable="chassis_driver_node.py",
                name="chassis_driver_node",
                parameters=[
                    config_file,
                    {
                        "handoff_dir": handoff_dir,
                        "steering_port": steering_port,
                    },
                ],
                output="screen",
            ),
        ]
    )
