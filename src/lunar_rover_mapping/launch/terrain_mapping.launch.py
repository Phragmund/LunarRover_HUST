"""Start the FAST-LIO2-backed 2.5D terrain mapper."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mapping_share = Path(get_package_share_directory("lunar_rover_mapping"))
    default_params = mapping_share / "config" / "terrain_mapper.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=str(default_params)),
            Node(
                package="lunar_rover_mapping",
                executable="terrain_mapper_node.py",
                name="terrain_mapper",
                parameters=[LaunchConfiguration("params_file")],
                output="screen",
            ),
        ]
    )
