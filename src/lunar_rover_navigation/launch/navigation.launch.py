"""Start Nav2 for online mapping mode and adapt commands to the chassis."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory("lunar_rover_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")

    nav2_launch = os.path.join(nav2_share, "launch", "navigation_launch.py")
    nav2_params = os.path.join(navigation_share, "config", "nav2_mapping_params.yaml")
    stamper_config = os.path.join(navigation_share, "config", "twist_stamper.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=nav2_params,
                description="Nav2 parameters for online mapping validation.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "use_sim_time": "true",
                    "autostart": "false",
                    "params_file": LaunchConfiguration("params_file"),
                    "use_composition": "False",
                    "use_respawn": "False",
                    "log_level": "info",
                }.items(),
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="lunar_rover_navigation",
                        executable="nav2_lifecycle_bringup_node.py",
                        name="nav2_lifecycle_bringup",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "wait_timeout": 90.0,
                                "call_timeout": 20.0,
                            }
                        ],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="lunar_rover_navigation",
                executable="twist_stamper_node",
                name="twist_stamper",
                parameters=[stamper_config],
                output="screen",
            ),
            Node(
                package="lunar_rover_navigation",
                executable="chassis_cmd_mux_node.py",
                name="chassis_cmd_mux",
                parameters=[stamper_config],
                output="screen",
            ),
        ]
    )
