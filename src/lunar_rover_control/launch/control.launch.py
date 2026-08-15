"""Start the simulated chassis and rocker controllers without odometry."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    controller_manager = LaunchConfiguration("controller_manager")
    spawn_joint_state_broadcaster = LaunchConfiguration(
        "spawn_joint_state_broadcaster"
    )
    controllers_file = LaunchConfiguration("controllers_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "controller_manager", default_value="/controller_manager"
            ),
            DeclareLaunchArgument(
                "spawn_joint_state_broadcaster",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("lunar_rover_control"),
                        "config",
                        "controllers.yaml",
                    ]
                ),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                condition=IfCondition(spawn_joint_state_broadcaster),
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-type",
                    "joint_state_broadcaster/JointStateBroadcaster",
                    "--controller-manager",
                    controller_manager,
                    "--controller-manager-timeout",
                    "60",
                    "--param-file",
                    controllers_file,
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "rocker_differential_controller",
                    "--controller-type",
                    "lunar_rover_control/RockerDifferentialController",
                    "--controller-manager",
                    controller_manager,
                    "--controller-manager-timeout",
                    "60",
                    "--param-file",
                    controllers_file,
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "rover_controller",
                    "--controller-type",
                    "lunar_rover_control/RoverController",
                    "--controller-manager",
                    controller_manager,
                    "--controller-manager-timeout",
                    "60",
                    "--param-file",
                    controllers_file,
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "--controller-type",
                    "joint_trajectory_controller/JointTrajectoryController",
                    "--controller-manager",
                    controller_manager,
                    "--controller-manager-timeout",
                    "60",
                    "--param-file",
                    controllers_file,
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "gripper_controller",
                    "--controller-type",
                    "position_controllers/JointGroupPositionController",
                    "--controller-manager",
                    controller_manager,
                    "--controller-manager-timeout",
                    "60",
                    "--param-file",
                    controllers_file,
                ],
                output="screen",
            ),
            Node(
                package="lunar_rover_control",
                executable="sim_chassis_state_node",
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
