"""Start the complete lunar rover stack in simulation or on the real vehicle."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


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

    use_sim_time = "true" if mode == "sim" else "false"
    control_share = Path(get_package_share_directory("lunar_rover_control"))
    bringup_share = Path(get_package_share_directory("lunar_rover_bringup"))
    localization_share = Path(get_package_share_directory("lunar_rover_localization"))
    navigation_share = Path(get_package_share_directory("lunar_rover_navigation"))
    perception_share = Path(get_package_share_directory("lunar_rover_perception"))

    actions = []
    if mode == "sim":
        actions.extend(
            [
                _include(
                    "lunar_rover_simulation",
                    "simulation.launch.py",
                    {
                        "world": LaunchConfiguration("world"),
                        "gui": LaunchConfiguration("gui"),
                        "headless": LaunchConfiguration("headless"),
                        "rviz": "false",
                        "sensor_noise": LaunchConfiguration("sensor_noise"),
                        "controller_config": str(
                            control_share / "config" / "controllers.yaml"
                        ),
                    },
                ),
                _include(
                    "lunar_rover_control",
                    "control.launch.py",
                    {
                        "use_sim_time": use_sim_time,
                        "spawn_joint_state_broadcaster": "false",
                        "controllers_file": str(
                            control_share / "config" / "controllers.yaml"
                        ),
                        "odometry_file": str(
                            control_share / "config" / "odometry.yaml"
                        ),
                    },
                ),
                _include(
                    "lunar_rover_sensors",
                    "sim_sensors.launch.py",
                    {"sensor_noise": LaunchConfiguration("sensor_noise")},
                ),
            ]
        )
    else:
        description_share = Path(
            get_package_share_directory("lunar_rover_description")
        )
        xacro_file = description_share / "urdf" / "lunar_rover.urdf.xacro"
        actions.extend(
            [
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
                Node(
                    package="lunar_rover_control",
                    executable="wheel_odometry_node",
                    output="screen",
                    parameters=[
                        str(control_share / "config" / "odometry.yaml"),
                        {"use_sim_time": False},
                    ],
                ),
            ]
        )

    actions.extend(
        [
            _include(
                "lunar_rover_localization",
                "localization_mapping.launch.py",
                {
                    "use_sim_time": use_sim_time,
                    "ekf_params_file": str(
                        localization_share
                        / "config"
                        / ("ekf_sim.yaml" if mode == "sim" else "ekf.yaml")
                    ),
                    "rtabmap_params_file": LaunchConfiguration("rtabmap_params_file"),
                },
            ),
            _include(
                "lunar_rover_perception",
                "perception.launch.py",
                {
                    "use_sim_time": use_sim_time,
                    "params_file": str(
                        perception_share / "config" / "terrain_perception.yaml"
                    ),
                },
            ),
            _include(
                "lunar_rover_navigation",
                "navigation.launch.py",
                {
                    "use_sim_time": use_sim_time,
                    "autostart": LaunchConfiguration("autostart"),
                    "rviz": LaunchConfiguration("rviz"),
                    "params_file": str(
                        navigation_share / "config" / "nav2_params.yaml"
                    ),
                },
            ),
            Node(
                package="lunar_rover_bringup",
                executable="safety_supervisor",
                output="screen",
                parameters=[
                    str(bringup_share / "config" / "safety.yaml"),
                    {"use_sim_time": mode == "sim"},
                ],
            ),
        ]
    )

    mission_file = LaunchConfiguration("mission_file").perform(context).strip()
    if mission_file:
        actions.append(
            Node(
                package="lunar_rover_missions",
                executable="mission_runner",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": mode == "sim",
                        "mission_file": mission_file,
                        "auto_start": True,
                    }
                ],
            )
        )

    if mode == "sim" and _as_bool(
        LaunchConfiguration("record_metrics").perform(context)
    ):
        actions.append(
            Node(
                package="lunar_rover_missions",
                executable="metrics_recorder",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "output_file": LaunchConfiguration("metrics_file").perform(
                            context
                        ),
                    }
                ],
            )
        )

    return actions


def generate_launch_description():
    localization_share = Path(get_package_share_directory("lunar_rover_localization"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="sim",
                description="Runtime target: sim or real.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="mixed",
                description="Simulation world: flat, steps, rocks, mixed, or stairs_maze.",
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("sensor_noise", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "mission_file",
                default_value="",
                description="Optional absolute waypoint mission YAML path.",
            ),
            DeclareLaunchArgument(
                "rtabmap_params_file",
                default_value=str(localization_share / "config" / "rtabmap.yaml"),
                description="RTAB-Map parameter YAML.",
            ),
            DeclareLaunchArgument(
                "record_metrics",
                default_value="false",
                description="Record simulation metrics on shutdown.",
            ),
            DeclareLaunchArgument(
                "metrics_file",
                default_value="~/.ros/lunar_rover_metrics.json",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
