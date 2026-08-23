"""Launch the chassis command mux together with the phone web joystick server."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="8000",
                description="Web server port for the phone joystick page.",
            ),
            DeclareLaunchArgument(
                "command_topic",
                default_value="/web/chassis_cmd_vel",
                description="TwistStamped topic the web joystick publishes to.",
            ),
            DeclareLaunchArgument(
                "access_token",
                default_value="",
                description="Required token in the phone URL query string.",
            ),
            Node(
                package="lunar_rover_navigation",
                executable="chassis_cmd_mux_node.py",
                name="chassis_cmd_mux",
                output="screen",
            ),
            Node(
                package="lunar_rover_navigation",
                executable="web_joystick_server.py",
                name="web_joystick_server",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "command_topic": LaunchConfiguration("command_topic"),
                        "access_token": LaunchConfiguration("access_token"),
                    }
                ],
                output="screen",
            ),
        ]
    )
