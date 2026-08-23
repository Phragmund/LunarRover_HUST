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

"""Gemini 335L depth-camera RGB-D point-cloud mapping via RTAB-Map.

Brings up ONLY the Orbbec Gemini 335 camera (no LiDAR/chassis) plus the full
RTAB-Map RGB-D pipeline:

    camera (publish_tf)  ->  rgbd_sync  ->  rgbd_odometry  ->  rtabmap  ->  rtabmap_viz

The map is a 3D point-cloud map built from the depth camera alone, with
RTAB-Map's internal RGB-D visual odometry (no external wheel/IMU odometry).

Usage:
    ros2 launch lunar_rover_sensors gemini335_rtabmap.launch.py
    ros2 launch lunar_rover_sensors gemini335_rtabmap.launch.py args:="-d --Vis/CorType 0"
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("lunar_rover_sensors"))
    orbbec_share = Path(get_package_share_directory("orbbec_camera"))
    rtabmap_share = Path(get_package_share_directory("rtabmap_launch"))

    rtabmap_args = LaunchConfiguration("rtabmap_args")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rtabmap_args",
                default_value="-d",
                description="RTAB-Map CLI flags/params (default: -d = delete DB on start).",
            ),
            # --- Orbbec Gemini 335 camera (publish TF, no driver-side point cloud) ---
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(orbbec_share / "launch" / "gemini_330_series.launch.py")
                ),
                launch_arguments={
                    "camera_name": "camera",
                    "config_file_path": str(
                        share / "config" / "gemini335_mapping.yaml"
                    ),
                }.items(),
            ),
            # --- RTAB-Map RGB-D mapping ---
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(rtabmap_share / "launch" / "rtabmap.launch.py")
                ),
                launch_arguments={
                    "rgb_topic": "/camera/color/image_raw",
                    "depth_topic": "/camera/depth/image_raw",
                    "camera_info_topic": "/camera/color/camera_info",
                    "frame_id": "camera_link",
                    "rgbd_sync": "true",
                    "depth_scale": "0.001",
                    "visual_odometry": "true",
                    "qos": "1",
                    "args": rtabmap_args,
                    "rtabmap_viz": "true",
                    "rviz": "false",
                }.items(),
            ),
        ]
    )
