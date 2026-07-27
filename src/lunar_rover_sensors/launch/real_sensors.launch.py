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
                remappings=[("/livox/lidar", "/livox/points")],
                parameters=[
                    {
                        "xfer_format": 0,
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
