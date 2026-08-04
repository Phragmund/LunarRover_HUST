"""Start terrain-aware frontier exploration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("lunar_rover_exploration"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(share / "config" / "frontier_explorer.yaml"),
            ),
            Node(
                package="lunar_rover_exploration",
                executable="frontier_explorer_node.py",
                name="frontier_explorer",
                parameters=[LaunchConfiguration("params_file")],
                output="screen",
            ),
        ]
    )
