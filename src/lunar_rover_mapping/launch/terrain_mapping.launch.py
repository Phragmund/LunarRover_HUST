"""Start the FAST-LIO2-backed 2.5D terrain mapper."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    mapping_share = Path(get_package_share_directory("lunar_rover_mapping"))
    default_params = mapping_share / "config" / "terrain_mapper.yaml"
    default_elevation_params = mapping_share / "config" / "elevation_grid.yaml"
    default_fusion_params = mapping_share / "config" / "cloud_fusion.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=str(default_params)),
            DeclareLaunchArgument(
                "elevation_params_file", default_value=str(default_elevation_params)
            ),
            DeclareLaunchArgument(
                "fusion_params_file", default_value=str(default_fusion_params)
            ),
            DeclareLaunchArgument("enable_cloud_fusion", default_value="false"),
            DeclareLaunchArgument("enable_elevation_grid", default_value="false"),
            Node(
                package="lunar_rover_mapping",
                executable="multisensor_cloud_fusion_node.py",
                name="multisensor_cloud_fusion",
                parameters=[LaunchConfiguration("fusion_params_file")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_cloud_fusion")),
            ),
            Node(
                package="lunar_rover_mapping",
                executable="terrain_mapper_node.py",
                name="terrain_mapper",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "input_topic": PythonExpression(
                            [
                                "'/localization/cloud_fused' if '",
                                LaunchConfiguration("enable_cloud_fusion"),
                                "'.lower() in ('1', 'true', 'yes', 'on') "
                                "else '/localization/cloud_registered'",
                            ]
                        )
                    },
                ],
                output="screen",
            ),
            Node(
                package="lunar_rover_mapping",
                executable="elevation_grid_node",
                name="elevation_grid",
                parameters=[
                    LaunchConfiguration("elevation_params_file"),
                    {
                        "input_topic": PythonExpression(
                            [
                                "'/localization/cloud_fused' if '",
                                LaunchConfiguration("enable_cloud_fusion"),
                                "'.lower() in ('1', 'true', 'yes', 'on') "
                                "else '/localization/cloud_registered'",
                            ]
                        )
                    },
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_elevation_grid")),
            ),
        ]
    )
