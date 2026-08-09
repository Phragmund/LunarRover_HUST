"""Display the rover model without starting Gazebo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("rviz")
    model = PathJoinSubstitution(
        [FindPackageShare("lunar_rover_description"), "urdf", "lunar_rover.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("lunar_rover_description"), "rviz", "lunar_rover.rviz"]
    )
    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                model,
                " use_sim:=false",
            ]
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
