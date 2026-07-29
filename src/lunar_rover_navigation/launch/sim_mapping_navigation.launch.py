"""Start simulation, online mapping, 2.5D terrain, Nav2, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mapping_share = get_package_share_directory("lunar_rover_mapping")
    navigation_share = get_package_share_directory("lunar_rover_navigation")

    sim_mapping_launch = os.path.join(mapping_share, "launch", "sim_mapping.launch.py")
    navigation_launch = os.path.join(navigation_share, "launch", "navigation.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="reachable_maze"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("mapping_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_mapping_launch),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "mapping_rviz": LaunchConfiguration("mapping_rviz"),
                }.items(),
            ),
            TimerAction(
                period=14.0,
                actions=[
                    IncludeLaunchDescription(PythonLaunchDescriptionSource(navigation_launch))
                ],
            ),
        ]
    )
