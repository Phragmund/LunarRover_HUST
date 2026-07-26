from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("lunar_rover_sensors"))
    sensor_noise = ParameterValue(
        LaunchConfiguration("sensor_noise"), value_type=bool
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("sensor_noise", default_value="true"),
            Node(
                package="lunar_rover_sensors",
                executable="mid360_sim_adapter",
                output="screen",
                parameters=[
                    str(share / "config" / "mid360_sim.yaml"),
                    {"noise_enabled": sensor_noise},
                ],
            ),
            Node(
                package="lunar_rover_sensors",
                executable="depth_noise_node",
                output="screen",
                parameters=[
                    str(share / "config" / "gemini335_sim.yaml"),
                    {"noise_enabled": sensor_noise},
                ],
            ),
            Node(
                package="depth_image_proc",
                executable="point_cloud_xyz_node",
                name="depth_to_points",
                remappings=[
                    ("image_rect", "/camera/depth/image_raw"),
                    ("camera_info", "/camera/depth/camera_info"),
                    ("points", "/camera/depth_registered/points"),
                ],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
