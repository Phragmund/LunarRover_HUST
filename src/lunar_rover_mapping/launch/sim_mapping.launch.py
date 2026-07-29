"""Start Gazebo, simulated sensors, mapping, and the mapping RViz view."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    simulation_share = get_package_share_directory("lunar_rover_simulation")
    sensors_share = get_package_share_directory("lunar_rover_sensors")
    mapping_share = get_package_share_directory("lunar_rover_mapping")

    simulation_launch = os.path.join(simulation_share, "launch", "simulation.launch.py")
    sensors_launch = os.path.join(sensors_share, "launch", "sim_sensors.launch.py")
    mapping_launch = os.path.join(mapping_share, "launch", "mapping.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="reachable_maze",
                description="Simulation world used for mapping validation.",
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "mapping_rviz",
                default_value="true",
                description="Start the mapping RViz2 view.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "headless": "false",
                    "rviz": "false",
                    "sensor_noise": "false",
                    "spawn_x": "-3.92",
                    "spawn_y": "0.0",
                    "spawn_z": "0.20",
                    "spawn_yaw": "0.0",
                }.items(),
            ),
            TimerAction(
                period=4.0,
                actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(sensors_launch))],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(mapping_launch),
                        launch_arguments={
                            "rviz": LaunchConfiguration("mapping_rviz")
                        }.items(),
                    )
                ],
            ),
        ]
    )
