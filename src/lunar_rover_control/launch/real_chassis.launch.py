"""Start the physical H7 motion-solver chassis chain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
                    {
                        "use_sim_time": False,
                        "h7_port": LaunchConfiguration("h7_port"),
                    },
                ],
                output="screen",
            ),
        ]
    )
