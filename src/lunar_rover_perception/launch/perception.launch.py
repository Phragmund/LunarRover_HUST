from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("lunar_rover_perception")
    )
    default_params = str(package_share / "config" / "terrain_perception.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    lidar_points = LaunchConfiguration("lidar_points")
    depth_points = LaunchConfiguration("depth_points")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Terrain perception parameter file.",
            ),
            DeclareLaunchArgument(
                "lidar_points",
                default_value="/livox/points",
                description="Standard PointCloud2 produced by the Mid-360 adapter.",
            ),
            DeclareLaunchArgument(
                "depth_points",
                default_value="/camera/depth_registered/points",
                description="Registered Gemini 335 depth point cloud.",
            ),
            Node(
                package="lunar_rover_perception",
                executable="terrain_perception_node",
                name="terrain_perception",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "topics.lidar_points": lidar_points,
                        "topics.depth_points": depth_points,
                    },
                ],
            ),
        ]
    )
