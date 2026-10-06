"""Start the real-rover autonomous grasping nodes.

Notes:
  - The MoveIt/URDF path is retained only for the legacy ``grasp_backend:=moveit``
    flow. As of 2026-08-30, real-arm RViz/TF verification should rely on the
    hand-eye preview chain instead of the old Rover_0 camera mounting model.
  - ``grasp_backend:=state_machine`` intentionally avoids starting ``move_group``
    so the temporary real-arm hand-eye TF is the only active camera/tool chain.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = Path(get_package_share_directory("lunar_rover_manipulation"))
    repo_root = Path(os.environ.get("LUNAR_ROVER_ROOT", Path.cwd()))
    description_share = Path(get_package_share_directory("lunar_rover_description"))
    xacro_file = description_share / "urdf" / "lunar_rover.urdf.xacro"
    camera_xyz = LaunchConfiguration("arm_camera_xyz")
    camera_rpy = LaunchConfiguration("arm_camera_rpy")
    detector_backend = LaunchConfiguration("detector_backend")
    segmentation_model = LaunchConfiguration("segmentation_model")
    detector_mode = LaunchConfiguration("detector_mode")
    grasp_backend = LaunchConfiguration("grasp_backend")

    moveit_config = (
        MoveItConfigsBuilder(
            "lunar_rover", package_name="lunar_rover_manipulation"
        )
        .robot_description(
            file_path=str(xacro_file),
            mappings={
                "use_sim": "false",
                "arm_camera_xyz": camera_xyz,
                "arm_camera_rpy": camera_rpy,
            },
        )
        .robot_description_semantic(file_path="srdf/lunar_rover.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(
            default_planning_pipeline="ompl", pipelines=["ompl"]
        )
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_camera_xyz", default_value="0.04 0 0.065"),
            DeclareLaunchArgument(
                "arm_camera_rpy", default_value="0 -1.57079632679 0"
            ),
            DeclareLaunchArgument("detector_backend", default_value="ultralytics"),
            DeclareLaunchArgument(
                "detector_mode",
                default_value="fusion",
                description="fusion is the production HSV-proposal + self-trained best.pt confirmation path; legacy directly runs target_detector_node.py.",
            ),
            DeclareLaunchArgument(
                "segmentation_model", default_value=str(repo_root / "weights" / "best.pt")
            ),
            DeclareLaunchArgument("open_vocab_enabled", default_value="false"),
            DeclareLaunchArgument("sam_enabled", default_value="false"),
            DeclareLaunchArgument("active_search", default_value="true"),
            DeclareLaunchArgument("auto_trigger_grasp", default_value="false"),
            DeclareLaunchArgument("periodic_sweep", default_value="false"),
            DeclareLaunchArgument(
                "pose_average_pose_count",
                default_value="1",
                description="Current grasp gate uses one centered mask-distance pose; legacy four-corner sampling is bypassed.",
            ),
            DeclareLaunchArgument("pose_average_samples_per_pose", default_value="5"),
            DeclareLaunchArgument("mission_executive", default_value="true"),
            DeclareLaunchArgument(
                "run_foundationpose_bridge", default_value="false"
            ),
            DeclareLaunchArgument(
                "grasp_backend",
                default_value="moveit",
                description="moveit uses grasp_coordinator (MoveIt path); state_machine uses grasp_state_machine_node (real-arm skeleton).",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                parameters=[moveit_config.to_dict()],
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", grasp_backend, "' == 'moveit'"])
                ),
            ),
            Node(
                package="lunar_rover_manipulation",
                executable="target_detector_node.py",
                name="target_detector",
                parameters=[
                    str(share / "config" / "grasping.yaml"),
                    {
                        "backend": ParameterValue(
                            detector_backend, value_type=str
                        ),
                        "model_path": ParameterValue(
                            segmentation_model, value_type=str
                        ),
                    },
                ],
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", detector_mode, "' == 'legacy'"])
                ),
            ),
            Node(
                package="lunar_rover_manipulation",
                executable="target_fusion_detector_node.py",
                name="target_fusion_detector",
                parameters=[
                    str(share / "config" / "grasping.yaml"),
                    {
                        "yolo_confirm_model": ParameterValue(
                            segmentation_model, value_type=str
                        ),
                        "open_vocab_enabled": ParameterValue(
                            LaunchConfiguration("open_vocab_enabled"),
                            value_type=bool,
                        ),
                        "sam_enabled": ParameterValue(
                            LaunchConfiguration("sam_enabled"),
                            value_type=bool,
                        ),
                    },
                ],
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", detector_mode, "' == 'fusion'"])
                ),
            ),
            Node(
                package="lunar_rover_manipulation",
                executable="target_localizer_node.py",
                name="grasp_target_localizer",
                parameters=[str(share / "config" / "grasping.yaml")],
                output="screen",
            ),
            Node(
                package="lunar_rover_manipulation",
                executable="active_search_node.py",
                name="active_search",
                parameters=[
                    str(share / "config" / "grasping.yaml"),
                    {
                        "enabled": ParameterValue(
                            LaunchConfiguration("active_search"), value_type=bool
                        ),
                        "auto_trigger_grasp": ParameterValue(
                            LaunchConfiguration("auto_trigger_grasp"), value_type=bool
                        ),
                        "periodic_sweep_enabled": ParameterValue(
                            LaunchConfiguration("periodic_sweep"), value_type=bool
                        ),
                    },
                ],
                output="screen",
            ),
            Node(
                package="lunar_rover_manipulation",
                executable="foundationpose_input_adapter_node.py",
                name="foundationpose_input_adapter",
                parameters=[str(share / "config" / "grasping.yaml")],
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("run_foundationpose_bridge")
                ),
            ),
            Node(
                package="lunar_rover_manipulation",
                executable="foundationpose_result_bridge_node.py",
                name="foundationpose_result_bridge",
                parameters=[str(share / "config" / "grasping.yaml")],
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("run_foundationpose_bridge")
                ),
            ),
            Node(
                package="lunar_rover_manipulation",
                executable="grasp_coordinator_node.py",
                name="grasp_coordinator",
                parameters=[str(share / "config" / "grasping.yaml")],
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", grasp_backend, "' == 'moveit'"])
                ),
            ),
            Node(
                package="lunar_rover_manipulation",
                executable="grasp_state_machine_node.py",
                name="grasp_state_machine",
                parameters=[
                    str(share / "config" / "grasping.yaml"),
                    {
                        "pose_average_pose_count": ParameterValue(
                            LaunchConfiguration("pose_average_pose_count"),
                            value_type=int,
                        ),
                        "pose_average_samples_per_pose": ParameterValue(
                            LaunchConfiguration("pose_average_samples_per_pose"),
                            value_type=int,
                        ),
                    },
                ],
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", grasp_backend, "' == 'state_machine'"])
                ),
            ),
            Node(
                package="lunar_rover_manipulation",
                executable="mission_executive_node.py",
                name="mission_executive",
                parameters=[str(share / "config" / "grasping.yaml")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("mission_executive")),
            ),
        ]
    )
