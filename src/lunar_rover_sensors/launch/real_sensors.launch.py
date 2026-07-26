from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    share = Path(get_package_share_directory("lunar_rover_sensors"))
    orbbec_share = Path(get_package_share_directory("orbbec_camera"))
    return LaunchDescription(
        [
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                output="screen",
                parameters=[
                    {
                        "xfer_format": 1,
                        "multi_topic": 0,
                        "data_src": 0,
                        "publish_freq": 10.0,
                        "output_data_type": 0,
                        "frame_id": "livox_frame",
                        "user_config_path": str(
                            share / "config" / "livox_mid360.json"
                        ),
                    }
                ],
            ),
            Node(
                package="lunar_rover_sensors",
                executable="livox_custom_to_pointcloud",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/livox/lidar",
                        "output_topic": "/livox/points",
                    }
                ],
            ),
            GroupAction(
                [
                    SetRemap(
                        src="/camera/depth/points",
                        dst="/camera/depth_registered/points",
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            str(
                                orbbec_share
                                / "launch"
                                / "gemini_330_series.launch.py"
                            )
                        ),
                        launch_arguments={
                            "camera_name": "camera",
                            "config_file_path": str(
                                share / "config" / "gemini335_real.yaml"
                            ),
                        }.items(),
                    ),
                ]
            ),
        ]
    )
