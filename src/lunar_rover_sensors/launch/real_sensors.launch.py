# Copyright 2026 Lunar Rover Maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    share = Path(get_package_share_directory("lunar_rover_sensors"))
    orbbec_share = Path(get_package_share_directory("orbbec_camera"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("run_lidar", default_value="true"),
            DeclareLaunchArgument("run_camera", default_value="true"),
            DeclareLaunchArgument("lidar_frame_id", default_value="livox_frame"),
            DeclareLaunchArgument("lidar_output_topic", default_value="/livox/points"),
            DeclareLaunchArgument("imu_output_topic", default_value="/livox/imu"),
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                output="screen",
                condition=IfCondition(LaunchConfiguration("run_lidar")),
                remappings=[
                    ("/livox/lidar", LaunchConfiguration("lidar_output_topic")),
                    ("/livox/imu", LaunchConfiguration("imu_output_topic")),
                ],
                parameters=[
                    {
                        "xfer_format": 0,
                        "multi_topic": 0,
                        "data_src": 0,
                        "publish_freq": 10.0,
                        "output_data_type": 0,
                        "frame_id": LaunchConfiguration("lidar_frame_id"),
                        "user_config_path": str(
                            share / "config" / "livox_mid360.json"
                        ),
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
                ],
                condition=IfCondition(LaunchConfiguration("run_camera")),
            ),
        ]
    )
