"""Start the physical H7 motion-solver chassis and arm chain."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("lunar_rover_control"))
    return LaunchDescription(
        [
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
            Node(
                package="lunar_rover_control",
                executable="motion_solver.py",
                name="motion_solver",
                parameters=[
                    str(share / "config" / "arm_hardware.yaml"),
                    {
                        "use_sim_time": False,
                        "h7_port": LaunchConfiguration("h7_port"),
                    },
                ],
                output="screen",
            ),
        ]
    )
