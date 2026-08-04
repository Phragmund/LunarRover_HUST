"""Start only the rover model, low-level control, raw sensors, and visualization."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def _include(package, launch_file, arguments):
    path = Path(get_package_share_directory(package)) / "launch" / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=arguments.items(),
    )


def _launch_setup(context):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode not in ("sim", "real"):
        raise RuntimeError("mode must be either 'sim' or 'real'")

    if mode == "sim":
        control_share = Path(get_package_share_directory("lunar_rover_control"))
        return [
            _include(
                "lunar_rover_simulation",
                "simulation.launch.py",
                {
                    "world": LaunchConfiguration("world"),
                    "gui": LaunchConfiguration("gui"),
                    "headless": LaunchConfiguration("headless"),
                    "rviz": LaunchConfiguration("rviz"),
                    "sensor_noise": LaunchConfiguration("sensor_noise"),
                    "controller_config": str(
                        control_share / "config" / "controllers.yaml"
                    ),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                },
            ),
            _include("lunar_rover_sensors", "sim_sensors.launch.py", {}),
        ]

    description_share = Path(get_package_share_directory("lunar_rover_description"))
    xacro_file = description_share / "urdf" / "lunar_rover.urdf.xacro"
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": Command(
                        [
                            FindExecutable(name="xacro"),
                            " ",
                            str(xacro_file),
                            " use_sim:=false",
                        ]
                    ),
                    "use_sim_time": False,
                }
            ],
        ),
        _include("lunar_rover_sensors", "real_sensors.launch.py", {}),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sim"),
            DeclareLaunchArgument("world", default_value="image_cave"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("spawn_x", default_value="-3.92"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="1.45"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "sensor_noise",
                default_value="false",
                description="Enable only Gazebo's sensor noise model.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
