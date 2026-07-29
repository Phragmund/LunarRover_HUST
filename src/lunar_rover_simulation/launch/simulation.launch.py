"""Start Gazebo Fortress, spawn the rover, and bridge simulated sensors."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


SUPPORTED_WORLDS = (
    "flat",
    "steps",
    "rocks",
    "mixed",
    "stairs_maze",
    "reachable_maze",
    "lava_tube",
    "image_cave",
)


def _as_bool(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _launch_setup(context):
    world_name = LaunchConfiguration("world").perform(context)
    if world_name not in SUPPORTED_WORLDS:
        supported = ", ".join(SUPPORTED_WORLDS)
        raise RuntimeError(f"Unsupported world '{world_name}'. Choose one of: {supported}")

    gui = _as_bool(LaunchConfiguration("gui").perform(context))
    headless = _as_bool(LaunchConfiguration("headless").perform(context))
    simulation_share = get_package_share_directory("lunar_rover_simulation")
    world_path = os.path.join(simulation_share, "worlds", f"{world_name}.sdf")
    controller_config = LaunchConfiguration("controller_config").perform(context)
    bridge_config = os.path.join(simulation_share, "config", "bridge.yaml")

    gz_args = [
        "-r",
        "-v",
        LaunchConfiguration("verbosity").perform(context),
    ]
    if headless:
        gz_args.extend(["-s", "--headless-rendering"])
    elif not gui:
        gz_args.append("-s")
    gz_args.append(world_path)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": " ".join(gz_args)}.items(),
    )

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("lunar_rover_description"),
            "urdf",
            "lunar_rover.urdf.xacro",
        ]
    )
    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                xacro_file,
                " use_sim:=true",
                " sensor_noise:=",
                LaunchConfiguration("sensor_noise"),
                " controller_config:=",
                controller_config,
            ]
        ),
        "use_sim_time": True,
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    spawn_rover = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "lunar_rover",
            "-topic",
            "robot_description",
            "-allow_renaming",
            "false",
            "-x",
            LaunchConfiguration("spawn_x"),
            "-y",
            LaunchConfiguration("spawn_y"),
            "-z",
            LaunchConfiguration("spawn_z"),
            "-Y",
            LaunchConfiguration("spawn_yaw"),
        ],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-type",
            "joint_state_broadcaster/JointStateBroadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--param-file",
            controller_config,
        ],
        output="screen",
    )
    rocker_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rocker_differential_controller",
            "--controller-type",
            "lunar_rover_control/RockerDifferentialController",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--param-file",
            controller_config,
        ],
        output="screen",
    )
    rover_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rover_controller",
            "--controller-type",
            "lunar_rover_control/RoverController",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--param-file",
            controller_config,
        ],
        output="screen",
    )
    sim_chassis_state = Node(
        package="lunar_rover_control",
        executable="sim_chassis_state_node",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    spawn_controllers_after_robot = RegisterEventHandler(
        OnProcessExit(target_action=spawn_rover, on_exit=[joint_state_spawner])
    )
    spawn_rocker_after_joint_states = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_spawner, on_exit=[rocker_spawner])
    )
    spawn_rover_after_rocker = RegisterEventHandler(
        OnProcessExit(target_action=rocker_spawner, on_exit=[rover_spawner])
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="simulation_bridge",
        parameters=[
            {
                "config_file": bridge_config,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("lunar_rover_description"),
            "rviz",
            "lunar_rover.rviz",
        ]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return [
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_rover,
        spawn_controllers_after_robot,
        spawn_rocker_after_joint_states,
        spawn_rover_after_rocker,
        sim_chassis_state,
        rviz,
    ]


def generate_launch_description():
    simulation_share = get_package_share_directory("lunar_rover_simulation")
    model_path = os.path.join(simulation_share, "models")
    existing_resource_paths = (
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    )
    resource_path = os.pathsep.join(
        dict.fromkeys(
            path
            for path in (model_path, *existing_resource_paths)
            if path
        )
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
            DeclareLaunchArgument(
                "world",
                default_value="mixed",
                description=(
                    "One of: flat, steps, rocks, mixed, stairs_maze, reachable_maze, lava_tube, image_cave."
                ),
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Use server-only EGL rendering for camera and lidar sensors.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2.",
            ),
            DeclareLaunchArgument(
                "sensor_noise",
                default_value="false",
                description="Enable only Gazebo's built-in sensor noise model.",
            ),
            DeclareLaunchArgument(
                "controller_config",
                default_value=os.path.join(
                    simulation_share, "config", "controllers.yaml"
                ),
                description="Controller-manager parameter file loaded by gz_ros2_control.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="-3.92"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.20"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "seed",
                default_value="360335",
                description="Deterministic Gazebo random seed.",
            ),
            DeclareLaunchArgument(
                "verbosity",
                default_value="2",
                description="Gazebo console verbosity from 0 to 4.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
