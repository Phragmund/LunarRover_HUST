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

"""Standalone Gemini 335L camera bring-up for unboxing / functional test.

This launch starts ONLY the Orbbec Gemini 335 camera (no Livox lidar, no
chassis), publishes the raw color/depth/point-cloud topics, and optionally
opens RViz with a ready-made view showing:

  - RGB color image      (/camera/color/image_raw)
  - depth grayscale      (/camera/depth/image_raw)
  - depth point cloud    (/camera/depth/points)

Usage:
    ros2 launch lunar_rover_sensors gemini335_test.launch.py
    ros2 launch lunar_rover_sensors gemini335_test.launch.py start_rviz:=false
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("lunar_rover_sensors"))
    orbbec_share = Path(get_package_share_directory("orbbec_camera"))

    camera_name = LaunchConfiguration("camera_name")
    config_file_path = LaunchConfiguration("config_file_path")
    start_rviz = LaunchConfiguration("start_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_name",
                default_value="camera",
                description="Camera node namespace; topics become /<name>/color/image_raw etc.",
            ),
            DeclareLaunchArgument(
                "config_file_path",
                default_value=str(share / "config" / "gemini335_test.yaml"),
                description="Orbbec Gemini 335 driver parameter file.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Open RViz with the RGB/depth/point-cloud test view.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(orbbec_share / "launch" / "gemini_330_series.launch.py")
                ),
                launch_arguments={
                    "camera_name": camera_name,
                    "config_file_path": config_file_path,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(share / "rviz" / "gemini335_test.rviz")],
                condition=IfCondition(start_rviz),
            ),
        ]
    )
