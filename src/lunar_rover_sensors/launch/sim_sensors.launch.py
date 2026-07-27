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

"""Expose only raw simulated sensor topics and a basic depth point-cloud view."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
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
