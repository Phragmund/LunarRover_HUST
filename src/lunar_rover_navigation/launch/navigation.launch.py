from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    package_share = Path(
        get_package_share_directory("lunar_rover_navigation")
    )

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    rviz = LaunchConfiguration("rviz")
    log_level = LaunchConfiguration("log_level")

    params_path = params_file.perform(context)
    nav2_params = [params_path, {"use_sim_time": use_sim_time}]
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_params,
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
    )
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_params,
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_params,
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_params,
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_params,
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_params,
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_params,
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings + [
            ("cmd_vel", "cmd_vel_nav"),
            ("cmd_vel_smoothed", "cmd_vel"),
        ],
    )
    navigation_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": lifecycle_nodes,
            }
        ],
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        namespace=namespace,
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=nav2_params,
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )
    collision_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision_monitor",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["collision_monitor"],
            }
        ],
    )
    rviz_node = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        output="screen",
        arguments=[
            "-d",
            str(package_share / "rviz" / "navigation.rviz"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    return [
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        navigation_lifecycle,
        collision_monitor,
        collision_lifecycle,
        rviz_node,
    ]


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("lunar_rover_navigation")
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_respawn", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(package_share / "config" / "nav2_params.yaml"),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
