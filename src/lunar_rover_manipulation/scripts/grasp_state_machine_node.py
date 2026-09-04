#!/usr/bin/env python3
"""Visual grasp state machine (阶段 A/B partial skeleton).

Implements the first half of the real-rover visual grasp pipeline:

    ABORT -> WIDE_SEARCH -> TARGET_LOCKED -> POSE_AVERAGING
          -> WORKSPACE_CHECK -> APPROACH_WITH_CHASSIS (placeholder only)
          -> MOVE_TO_GRASP (motion entry point)
         -> ABORT

It still does NOT command the chassis or the arm. It only:

  - publishes the current state on /grasp/status,
  - publishes /grasp/active and /perception/grasp_mode,
  - watches /perception/target_detection for a YOLO-confirmed candidate
    (class_id left the hsv_candidate placeholder) that stays put for a few frames,
    then locks onto it,
  - after HSV lock, switches to the low-frequency HSV+SAM perception mode and
    averages target pose samples only while the camera depth is in the reliable
    far-enough range,
  - evaluates whether the averaged target pose is inside the arm workspace; if
    yes, enters MOVE_TO_GRASP, otherwise enters APPROACH_WITH_CHASSIS as a
    placeholder reminder that chassis approach control still needs to be
    implemented.

Locking normally requires YOLO confirmation.  The operator can explicitly send
``skip_yolo_confirm`` to accept the latest HSV proposal for commissioning; this
is deliberately a manual, visible override rather than an automatic fallback.

Triggers (either one starts a search):

  - /grasp/trigger (std_msgs/Empty)
  - /grasp/command (std_msgs/String): "start" / "start_search"

The unified web console (rover_web_controller.html) already publishes both, so its
"Start Search" / "Abort" buttons drive this node directly.
"""

from __future__ import annotations

import math
import time
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from lunar_rover_msgs.msg import TargetObservation
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection2D


IDLE = "IDLE"
WIDE_SEARCH = "WIDE_SEARCH"
TARGET_LOCKED = "TARGET_LOCKED"
POSE_AVERAGING = "POSE_AVERAGING"
WORKSPACE_CHECK = "WORKSPACE_CHECK"
APPROACH_WITH_CHASSIS = "APPROACH_WITH_CHASSIS"
DEPLOY_GRASP = "DEPLOY_GRASP"
MOVE_TO_GRASP = "MOVE_TO_GRASP"
ABORT = "ABORT"


def _rotation_matrix(quaternion):
    q = np.array(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=np.float64
    )
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        raise ValueError("invalid zero-length TF quaternion")
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def clamp(value, low, high):
    return min(max(float(value), float(low)), float(high))


def normalize_deg(angle):
    value = float(angle) % 360.0
    if value < 0.0:
        value += 360.0
    return value


@dataclass
class IkTarget:
    x: float
    y: float
    z: float
    pitch_deg: float
    grip_deg: float


class GraspStateMachine(Node):
    def __init__(self):
        super().__init__("grasp_state_machine")

        # Unconfirmed HSV proposals keep this class_id prefix until YOLO
        # confirms them. The detector may append "+sam" (hsv_candidate+sam), so
        # we treat it as a prefix.
        self.hsv_class_id = str(
            self.declare_parameter("hsv_class_id", "hsv_candidate").value
        )
        self.lock_confirmation_frames = int(
            self.declare_parameter("lock_confirmation_frames", 3).value
        )
        self.skip_yolo_candidate_timeout_sec = max(
            0.1,
            float(
                self.declare_parameter("skip_yolo_candidate_timeout_sec", 1.0).value
            ),
        )
        self.lock_center_spread_frac = float(
            self.declare_parameter("lock_center_spread_frac", 0.5).value
        )
        self.lock_window_sec = float(
            self.declare_parameter("lock_window_sec", 3.0).value
        )
        self.target_timeout_sec = float(
            self.declare_parameter("target_timeout_sec", 1.5).value
        )
        self.observation_max_age_sec = float(
            self.declare_parameter("observation_max_age_sec", 0.8).value
        )
        self.camera_frame = str(
            self.declare_parameter("camera_frame", "camera_color_optical_frame").value
        )
        self.pose_average_pose_count = int(
            self.declare_parameter("pose_average_pose_count", 5).value
        )
        self.pose_average_samples_per_pose = int(
            self.declare_parameter("pose_average_samples_per_pose", 15).value
        )
        self.pose_average_pose_change_deg = float(
            self.declare_parameter("pose_average_pose_change_deg", 5.0).value
        )
        self.pose_average_settle_sec = float(
            self.declare_parameter("pose_average_settle_sec", 0.8).value
        )
        self.pose_average_joint_stable_deg = float(
            self.declare_parameter("pose_average_joint_stable_deg", 0.6).value
        )
        self.pose_average_min_camera_depth_m = float(
            self.declare_parameter("pose_average_min_camera_depth_m", 0.20).value
        )
        self.pose_average_max_camera_depth_m = float(
            self.declare_parameter("pose_average_max_camera_depth_m", 1.20).value
        )
        self.pose_average_max_spread_m = float(
            self.declare_parameter("pose_average_max_spread_m", 0.05).value
        )
        self.chassis_approach_enabled = bool(
            self.declare_parameter("chassis_approach_enabled", True).value
        )
        self.chassis_approach_topic = str(
            self.declare_parameter("chassis_approach_topic", "/grasp/chassis_cmd_vel").value
        )
        self.chassis_approach_step_m = max(
            0.01, float(self.declare_parameter("chassis_approach_step_m", 0.10).value)
        )
        self.chassis_approach_speed_mps = max(
            0.01, float(self.declare_parameter("chassis_approach_speed_mps", 0.04).value)
        )
        self.chassis_approach_settle_sec = max(
            0.1, float(self.declare_parameter("chassis_approach_settle_sec", 1.0).value)
        )
        self.chassis_approach_deadband_px = max(
            0.0, float(self.declare_parameter("chassis_approach_deadband_px", 20.0).value)
        )
        self.chassis_approach_steer_gain = float(
            self.declare_parameter("chassis_approach_steer_gain", 0.0015).value
        )
        self.chassis_approach_max_angular_z = max(
            0.01, float(self.declare_parameter("chassis_approach_max_angular_z", 0.12).value)
        )
        # Automated four-corner parallax sampling.  The center is frozen at
        # the instant YOLO confirmation locks a target; only J1/J4 move while
        # J2/J3/J5 remain at that captured pose.
        self.auto_sampling_enabled = bool(
            self.declare_parameter("auto_sampling_enabled", True).value
        )
        self.auto_sampling_j1_offset_deg = abs(
            float(self.declare_parameter("auto_sampling_j1_offset_deg", 8.0).value)
        )
        self.auto_sampling_j4_offset_deg = abs(
            float(self.declare_parameter("auto_sampling_j4_offset_deg", 8.0).value)
        )
        self.auto_sampling_motion_duration_sec = max(
            0.1,
            float(
                self.declare_parameter(
                    "auto_sampling_motion_duration_sec", 1.0
                ).value
            ),
        )
        self.auto_sampling_motion_margin_sec = max(
            0.0,
            float(
                self.declare_parameter("auto_sampling_motion_margin_sec", 0.15).value
            ),
        )
        self.auto_execute_increment_enabled = bool(
            self.declare_parameter("auto_execute_increment_enabled", True).value
        )
        self.tf_timeout_sec = float(
            self.declare_parameter("tf_timeout_sec", 0.12).value
        )
        self.workspace_center_x_m = float(
            self.declare_parameter("workspace_center_x_m", 0.13).value
        )
        self.workspace_center_y_m = float(
            self.declare_parameter("workspace_center_y_m", 0.0).value
        )
        self.workspace_center_z_m = float(
            self.declare_parameter("workspace_center_z_m", 0.056).value
        )
        self.workspace_radius_m = float(
            self.declare_parameter("workspace_radius_m", 0.45537).value
        )
        self.workspace_x_min_m = float(
            self.declare_parameter("workspace_x_min_m", 0.18).value
        )
        self.workspace_x_max_m = float(
            self.declare_parameter("workspace_x_max_m", 0.30).value
        )
        self.workspace_abs_y_max_m = float(
            self.declare_parameter("workspace_abs_y_max_m", 0.08).value
        )
        self.workspace_z_min_m = float(
            self.declare_parameter("workspace_z_min_m", -0.15).value
        )
        self.workspace_z_max_m = float(
            self.declare_parameter("workspace_z_max_m", 0.35).value
        )
        self.arm_joint_names = list(
            self.declare_parameter(
                "arm_joint_names",
                ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"],
            ).value
        )
        self.arm_zero_degrees = [
            float(x)
            for x in self.declare_parameter(
                "arm_zero_degrees", [90.0, 180.0, 135.0, 135.0, 0.0]
            ).value
        ]
        self.arm_ros_directions = [
            float(x)
            for x in self.declare_parameter(
                "arm_ros_directions", [1.0, -1.0, 1.0, 1.0, 1.0]
            ).value
        ]
        self.base_to_shoulder_m = float(
            self.declare_parameter("base_to_shoulder_m", 0.07545).value
        )
        self.upper_arm_m = float(self.declare_parameter("upper_arm_m", 0.11391).value)
        self.forearm_m = float(self.declare_parameter("forearm_m", 0.11000).value)
        self.tool_m = float(self.declare_parameter("tool_m", 0.15601).value)
        self.gripper_stroke_m = float(
            self.declare_parameter("gripper_stroke_m", 0.045).value
        )
        self.increment_step_m = float(
            self.declare_parameter("increment_step_m", 0.05).value
        )
        self.increment_duration_sec = float(
            self.declare_parameter("increment_duration_sec", 1.5).value
        )
        self.increment_goal_tolerance_m = float(
            self.declare_parameter("increment_goal_tolerance_m", 0.025).value
        )
        self.tf_direct_target_enabled = bool(
            self.declare_parameter("tf_direct_target_enabled", False).value
        )
        self.tf_direct_target_duration_sec = float(
            self.declare_parameter("tf_direct_target_duration_sec", 2.0).value
        )
        self.tf_direct_pre_gripper_reset_enabled = bool(
            self.declare_parameter("tf_direct_pre_gripper_reset_enabled", False).value
        )
        self.tf_direct_pre_gripper_deg = clamp(
            self.declare_parameter("tf_direct_pre_gripper_deg", 45.0).value,
            0.0,
            90.0,
        )
        self.tf_direct_pre_gripper_duration_sec = max(
            0.1,
            float(
                self.declare_parameter(
                    "tf_direct_pre_gripper_duration_sec", 1.0
                ).value
            ),
        )
        self.tf_direct_overshoot_m = max(
            0.0,
            float(self.declare_parameter("tf_direct_overshoot_m", 0.05).value),
        )
        self.increment_sample_log_path = Path(
            str(
                self.declare_parameter(
                    "increment_sample_log_path",
                    "generated/logs/grasp_increment_samples.jsonl",
                ).value
            )
        ).expanduser()
        # Experimental compensation for the long, pure TF-direct reach.  It
        # is deliberately not applied to keyboard IK, normal increments, or
        # visual/gimbal servo corrections.
        self.tf_direct_j1_offset_deg = float(
            self.declare_parameter("tf_direct_j1_offset_deg", 0.0).value
        )
        self.tf_direct_j1_completion_margin_sec = max(
            0.0,
            float(
                self.declare_parameter(
                    "tf_direct_j1_completion_margin_sec", 0.15
                ).value
            ),
        )
        # Positive is defined in the displayed, inverted-camera image: move
        # the end-effector view upward.  The calibrated J4 image-up direction
        # is negative joint angle (the same convention as gimbal_j4_sign).
        self.tf_direct_j4_image_up_offset_deg = float(
            self.declare_parameter("tf_direct_j4_image_up_offset_deg", 0.0).value
        )
        # Optional staged test sequence after a pure TF-direct reach.  Kept
        # off by default so production/normal IK motion never closes J5 or
        # rehomes J2 as an unintended side effect.
        self.tf_direct_post_sequence_enabled = bool(
            self.declare_parameter("tf_direct_post_sequence_enabled", False).value
        )
        self.tf_direct_post_tf_settle_sec = max(
            0.0,
            float(
                self.declare_parameter("tf_direct_post_tf_settle_sec", 1.0).value
            ),
        )
        self.tf_direct_post_gripper_deg = clamp(
            self.declare_parameter("tf_direct_post_gripper_deg", 27.0).value,
            0.0,
            90.0,
        )
        self.tf_direct_post_gripper_duration_sec = max(
            0.1,
            float(
                self.declare_parameter(
                    "tf_direct_post_gripper_duration_sec", 1.0
                ).value
            ),
        )
        self.tf_direct_post_j2_deg = clamp(
            self.declare_parameter("tf_direct_post_j2_deg", 180.0).value,
            0.0,
            270.0,
        )
        self.tf_direct_post_j2_duration_sec = max(
            0.1,
            float(
                self.declare_parameter("tf_direct_post_j2_duration_sec", 1.0).value
            ),
        )
        self.grasp_verify_enabled = bool(
            self.declare_parameter("grasp_verify_enabled", True).value
        )
        self.grasp_verify_roi = [
            float(value)
            for value in self.declare_parameter(
                "grasp_verify_roi", [0.0, 0.55, 0.45, 0.45]
            ).value
        ]
        self.grasp_verify_min_roi_coverage = max(
            0.0,
            min(1.0, float(self.declare_parameter("grasp_verify_min_roi_coverage", 0.01).value)),
        )
        self.grasp_verify_required_samples = max(
            1, int(self.declare_parameter("grasp_verify_required_samples", 3).value)
        )
        self.grasp_verify_timeout_sec = max(
            0.1, float(self.declare_parameter("grasp_verify_timeout_sec", 2.0).value)
        )
        # H7 does not acknowledge a MULTI command at the ROS trajectory
        # boundary.  Repeat only the long, manual TF-direct command so a
        # transient UART/H7 parse loss cannot leave one joint (notably J4) at
        # its old target.  Normal IK streaming deliberately remains single
        # shot and is unaffected.
        self.tf_direct_target_repeat_count = max(
            1,
            int(self.declare_parameter("tf_direct_target_repeat_count", 3).value),
        )
        self.tf_direct_target_repeat_interval_sec = max(
            0.02,
            float(
                self.declare_parameter("tf_direct_target_repeat_interval_sec", 0.12).value
            ),
        )
        self.increment_min_xyz = tuple(
            float(x)
            for x in self.declare_parameter(
                "increment_min_xyz", [-0.32, -0.32, -0.18]
            ).value
        )
        self.increment_max_xyz = tuple(
            float(x)
            for x in self.declare_parameter(
                "increment_max_xyz", [0.42, 0.32, 0.42]
            ).value
        )
        self.visual_servo_enabled = bool(
            self.declare_parameter("visual_servo_enabled", True).value
        )
        self.visual_servo_tool_pixel = np.asarray(
            self.declare_parameter("visual_servo_tool_pixel", [454.0, 0.0]).value,
            dtype=np.float64,
        )
        self.visual_servo_depth_weight = float(
            self.declare_parameter("visual_servo_depth_weight", 0.70).value
        )
        self.visual_servo_max_pixel_error = float(
            self.declare_parameter("visual_servo_max_pixel_error", 600.0).value
        )
        self.visual_servo_max_age_sec = float(
            self.declare_parameter("visual_servo_max_age_sec", 0.8).value
        )
        self.tf_plane_correction_gain = float(
            self.declare_parameter("tf_plane_correction_gain", 0.12).value
        )
        self.tf_plane_correction_limit = float(
            self.declare_parameter("tf_plane_correction_limit", 0.75).value
        )
        self.gimbal_servo_enabled = bool(
            self.declare_parameter("gimbal_servo_enabled", False).value
        )
        self.gimbal_tf_approach_enabled = bool(
            self.declare_parameter("gimbal_tf_approach_enabled", True).value
        )
        self.gimbal_j1_gain_deg_per_px = float(
            self.declare_parameter("gimbal_j1_gain_deg_per_px", 0.02).value
        )
        self.gimbal_j4_gain_deg_per_px = float(
            self.declare_parameter("gimbal_j4_gain_deg_per_px", 0.02).value
        )
        self.gimbal_j1_sign = float(
            self.declare_parameter("gimbal_j1_sign", 1.0).value
        )
        self.gimbal_j4_sign = float(
            self.declare_parameter("gimbal_j4_sign", 1.0).value
        )
        self.gimbal_pixel_deadband = float(
            self.declare_parameter("gimbal_pixel_deadband", 8.0).value
        )
        self.gimbal_phase_delay_sec = float(
            self.declare_parameter("gimbal_phase_delay_sec", 0.5).value
        )
        self.gimbal_correction_duration_sec = float(
            self.declare_parameter("gimbal_correction_duration_sec", 0.5).value
        )
        # Before four-corner parallax sampling, use the proven J1/J4 image
        # gimbal mapping to bring the locked target to the RGB image center.
        self.auto_center_before_sampling_enabled = bool(
            self.declare_parameter("auto_center_before_sampling_enabled", True).value
        )
        self.auto_center_pixel = np.asarray(
            self.declare_parameter("auto_center_pixel", [320.0, 200.0]).value,
            dtype=np.float64,
        )
        self.auto_center_deadband_px = max(
            1.0,
            float(self.declare_parameter("auto_center_deadband_px", 12.0).value),
        )
        self.auto_center_motion_duration_sec = max(
            0.1,
            float(
                self.declare_parameter("auto_center_motion_duration_sec", 0.5).value
            ),
        )
        self.auto_center_motion_margin_sec = max(
            0.0,
            float(
                self.declare_parameter("auto_center_motion_margin_sec", 0.15).value
            ),
        )
        self.auto_center_max_steps = max(
            1, int(self.declare_parameter("auto_center_max_steps", 12).value)
        )
        self.auto_center_timeout_sec = max(
            1.0,
            float(self.declare_parameter("auto_center_timeout_sec", 15.0).value),
        )
        self.gimbal_gain_reference_depth_m = float(
            self.declare_parameter("gimbal_gain_reference_depth_m", 0.338).value
        )
        self.gimbal_gain_zero_depth_m = float(
            self.declare_parameter("gimbal_gain_zero_depth_m", 0.05).value
        )
        self.tf_target_pixel_topic = str(
            self.declare_parameter(
                "tf_target_pixel_topic", "/grasp/tf_target_pixel"
            ).value
        )
        # Depot release is deliberately a parameterised, low-speed sequence:
        # deployment teams tune the pose for the physical collection box
        # without changing the mission executive.
        self.unload_arm_degrees = list(
            self.declare_parameter("unload_arm_degrees", [90.0, 150.0, 120.0, 135.0]).value
        )
        self.unload_arm_duration_sec = float(
            self.declare_parameter("unload_arm_duration_sec", 2.0).value
        )
        self.unload_release_deg = float(
            self.declare_parameter("unload_release_deg", 0.0).value
        )
        self.unload_release_sec = float(
            self.declare_parameter("unload_release_sec", 1.0).value
        )

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.active_publisher = self.create_publisher(Bool, "/grasp/active", state_qos)
        self.status_publisher = self.create_publisher(
            String, "/grasp/status", state_qos
        )
        self.result_publisher = self.create_publisher(String, "/grasp/result", state_qos)
        self.mode_publisher = self.create_publisher(
            String, "/perception/grasp_mode", state_qos
        )
        self.workspace_publisher = self.create_publisher(
            String, "/grasp/workspace_status", state_qos
        )
        self.pose_average_publisher = self.create_publisher(
            String, "/grasp/pose_average_status", state_qos
        )
        self.arm_trajectory_publisher = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.gripper_command_publisher = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )
        self.tf_target_pixel_publisher = self.create_publisher(
            PointStamped, self.tf_target_pixel_topic, 10
        )
        self.chassis_approach_publisher = self.create_publisher(
            TwistStamped, self.chassis_approach_topic, 10
        )

        self.create_subscription(Empty, "/grasp/trigger", self.on_trigger, 10)
        self.create_subscription(String, "/grasp/command", self.on_command, 10)
        self.create_subscription(
            Detection2D, "/perception/target_detection", self.on_detection, 20
        )
        self.create_subscription(Image, "/perception/target_mask", self.on_target_mask, 20)
        self.create_subscription(
            TargetObservation,
            "/perception/target_observation",
            self.on_observation,
            20,
        )
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.on_camera_info, 10
        )

        self.active = False
        self.state = ABORT
        self.state_entered = time.monotonic()
        self.pending = []
        self.pose_samples = []
        self.pose_batch_means = []
        self.pose_batch_depths = []
        self.pose_batch_joint_reference = None
        self.pose_batch_changed_at = 0.0
        self.pose_batch_waiting_for_motion = False
        self.pose_batch_motion_reference = None
        self.pose_batch_last_joint_readback = None
        self.pose_batch_stable_since = None
        self.auto_sampling_center_degrees = None
        self.auto_sampling_targets = []
        self.auto_sampling_index = 0
        self.auto_sampling_release_at = 0.0
        self.auto_sampling_start_pending = False
        self.auto_increment_dispatched = False
        self.last_detection_at = 0.0
        self.visual_target_lost = False
        self.latest_detection_center = None
        self.latest_detection_bbox = None
        self.latest_hsv_candidate = None
        self.camera_info = None
        self.tf_plane_correction_camera = np.zeros(2, dtype=np.float64)
        self.locked_class = None
        self.locked_confidence = 0.0
        self.locked_center = None
        self.averaged_point = None
        self.averaged_depth_m = None
        self.latest_observation = None
        self.latest_observation_at = 0.0
        self.current_mode = "idle"
        self.current_workspace_status = "NO_OBSERVATION"
        self.current_pose_average_status = "NO_SAMPLES"
        self.current_status_text = ""
        self.current_status_detail = ""
        self.current_joint_ros = [0.0] * 5
        self.current_joints_ready = False
        self.gimbal_phase_timer = None
        self.gimbal_phase_pending = False
        self.auto_center_active = False
        self.auto_center_started_at = 0.0
        self.auto_center_next_command_at = 0.0
        self.auto_center_steps = 0
        self.arm_trajectory_retry_timers = []
        self.gripper_command_retry_timers = []
        self.tf_direct_j1_sequence_timer = None
        self.tf_direct_j1_sequence_pending = None
        self.tf_direct_pre_sequence_timer = None
        self.tf_direct_pre_sequence_pending = False
        self.tf_direct_post_sequence_timer = None
        self.tf_direct_post_sequence_pending = None
        self.grasp_verify_samples = []
        self.grasp_verify_active = False
        self.chassis_approach_phase = "idle"
        self.chassis_approach_release_at = 0.0
        self.chassis_approach_step_count = 0
        self.last_branch_sign = None
        self.grasp_finish_timer = None
        self.unload_timer = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publish_active(False)
        self.publish_mode("idle")
        self.publish_workspace_status("NO_OBSERVATION")
        self.publish_pose_average_status("NO_SAMPLES")
        self.set_state(ABORT, "未启动，等待开始搜索")
        self.create_timer(0.2, self.on_timer)
        self.get_logger().info(
            "grasp state machine ready (阶段 A skeleton): "
            f"lock requires YOLO confirmation unless manually overridden, "
            f"hsv_class_id={self.hsv_class_id!r}"
        )

    # ---- arm IK ---------------------------------------------------------

    def on_joint_state(self, message) -> None:
        found = 0
        for name, position in zip(message.name, message.position):
            if name in self.arm_joint_names:
                self.current_joint_ros[self.arm_joint_names.index(name)] = float(position)
                found += 1
        if found >= 4:
            self.current_joints_ready = True

    def on_camera_info(self, message) -> None:
        """Keep the camera model used to turn image-space error into a ray."""
        self.camera_info = message

    def _display_target_point(self):
        """Return the point currently used as the task's TF target."""
        if self.averaged_point is not None:
            return np.asarray(self.averaged_point, dtype=np.float64)
        if self.latest_observation is None:
            return None
        point = self.latest_observation.pose.position
        return np.array([point.x, point.y, point.z], dtype=np.float64)

    def publish_tf_target_projection(self) -> None:
        """Publish the locked target projected into the current RGB image."""
        if self.state in (ABORT, WIDE_SEARCH) or self.camera_info is None:
            return
        point_base = self._display_target_point()
        if point_base is None:
            return
        fx, fy = float(self.camera_info.k[0]), float(self.camera_info.k[4])
        cx, cy = float(self.camera_info.k[2]), float(self.camera_info.k[5])
        if fx <= 1e-6 or fy <= 1e-6:
            return
        camera_frame = self.camera_info.header.frame_id or self.camera_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                "base_link",
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException:
            return
        rotation = _rotation_matrix(transform.transform.rotation)
        translation = transform.transform.translation
        point_camera = rotation @ point_base + np.array(
            [translation.x, translation.y, translation.z], dtype=np.float64
        )
        depth = float(point_camera[2])
        if depth <= 0.03:
            return
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = camera_frame
        message.point.x = fx * float(point_camera[0]) / depth + cx
        message.point.y = fy * float(point_camera[1]) / depth + cy
        message.point.z = depth
        self.tf_target_pixel_publisher.publish(message)

    def reset_pose_averaging(self) -> None:
        """Reset only when a new target lock/search begins, never by age."""
        self.pose_samples.clear()
        self.pose_batch_means.clear()
        self.pose_batch_depths.clear()
        self.pose_batch_joint_reference = None
        self.pose_batch_changed_at = time.monotonic()
        self.pose_batch_waiting_for_motion = False
        self.pose_batch_motion_reference = None
        self.pose_batch_last_joint_readback = None
        self.pose_batch_stable_since = None
        self.auto_sampling_center_degrees = None
        self.auto_sampling_targets = []
        self.auto_sampling_index = 0
        self.auto_sampling_release_at = 0.0
        self.auto_sampling_start_pending = False
        self.auto_increment_dispatched = False

    def _start_auto_sampling(self) -> bool:
        if not self.auto_sampling_enabled or self.state != POSE_AVERAGING:
            return False
        if not self.current_joints_ready:
            self.auto_sampling_start_pending = True
            self.publish_pose_average_status("AUTO_WAITING_JOINTS")
            return False
        if self.auto_sampling_targets:
            return True

        center = self.ros_to_degrees(self.current_joint_ros)
        self.auto_sampling_center_degrees = list(center)
        j1, j4 = center[0], center[3]
        j1_offset = self.auto_sampling_j1_offset_deg
        j4_offset = self.auto_sampling_j4_offset_deg
        # Traverse adjacent corners where possible, rather than diagonally
        # crossing the square between every batch.
        corners = [
            (-j1_offset, -j4_offset),
            (-j1_offset, +j4_offset),
            (+j1_offset, +j4_offset),
            (+j1_offset, -j4_offset),
        ]
        self.auto_sampling_targets = []
        for j1_delta, j4_delta in corners:
            target = list(center[:4])
            target[0] = clamp(j1 + j1_delta, 0.0, 270.0)
            target[3] = clamp(j4 + j4_delta, 0.0, 270.0)
            self.auto_sampling_targets.append(target)
        self.auto_sampling_index = 0
        self.auto_sampling_start_pending = False
        self._move_to_auto_sampling_corner()
        return True

    def _move_to_auto_sampling_corner(self) -> None:
        if self.auto_sampling_index >= len(self.auto_sampling_targets):
            return
        target = self.auto_sampling_targets[self.auto_sampling_index]
        self.pose_samples.clear()
        self.pose_batch_waiting_for_motion = False
        self.pose_batch_joint_reference = None
        self.pose_batch_motion_reference = None
        self.pose_batch_last_joint_readback = None
        self.pose_batch_stable_since = None
        self.pose_batch_changed_at = time.monotonic()
        self.auto_sampling_release_at = (
            time.monotonic()
            + self.auto_sampling_motion_duration_sec
            + self.auto_sampling_motion_margin_sec
        )
        self._publish_arm_trajectory(
            self.degrees_to_ros(target),
            duration_sec=self.auto_sampling_motion_duration_sec,
        )
        self.publish_pose_average_status(
            "AUTO_MOVE "
            f"pose={self.auto_sampling_index + 1}/4 "
            f"j1={target[0]:.1f}deg j4={target[3]:.1f}deg "
            f"wait={self.auto_sampling_motion_duration_sec + self.auto_sampling_motion_margin_sec:.2f}s"
        )
        self.get_logger().info(
            "自动采样角点 %d/4：J1=%.1fdeg J4=%.1fdeg"
            % (self.auto_sampling_index + 1, target[0], target[3])
        )

    def ros_to_degrees(self, positions):
        angles = []
        for index, position in enumerate(positions):
            if index == 4:
                degrees = position * 90.0 / max(self.gripper_stroke_m, 1e-9)
            else:
                degrees = self.arm_zero_degrees[index] + (
                    self.arm_ros_directions[index] * math.degrees(position)
                )
            angles.append(clamp(degrees, 0.0, 270.0 if index < 4 else 90.0))
        return angles

    def degrees_to_ros(self, angles_deg):
        positions = []
        for index, degrees in enumerate(angles_deg):
            if index == 4:
                positions.append(
                    clamp(degrees, 0.0, 90.0) * self.gripper_stroke_m / 90.0
                )
            else:
                positions.append(
                    math.radians(
                        (degrees - self.arm_zero_degrees[index])
                        / self.arm_ros_directions[index]
                    )
                )
        return positions

    def fk_from_degrees(self, angles_deg):
        j1, j2, j3, j4, j5 = angles_deg
        t2 = math.radians(j2)
        t3 = t2 + math.radians(135.0 - j3)
        t4 = t3 + math.radians(135.0 - j4)

        radius = self.base_to_shoulder_m
        z = 0.0
        radius += self.upper_arm_m * math.sin(t2)
        z -= self.upper_arm_m * math.cos(t2)
        radius += self.forearm_m * math.sin(t3)
        z -= self.forearm_m * math.cos(t3)
        radius += self.tool_m * math.sin(t4)
        z -= self.tool_m * math.cos(t4)

        yaw = math.radians(j1)
        x = radius * math.sin(yaw)
        y = -radius * math.cos(yaw)
        pitch = normalize_deg(math.degrees(t4))
        return IkTarget(x=x, y=y, z=z, pitch_deg=pitch, grip_deg=clamp(j5, 0.0, 90.0))

    def solve_ik(self, target):
        radius = math.hypot(target.x, target.y)
        j1 = normalize_deg(math.degrees(math.atan2(target.x, -target.y)))

        pitch = normalize_deg(target.pitch_deg)
        pitch_rad = math.radians(pitch)
        planar_r = max(0.0, radius - self.base_to_shoulder_m)
        wrist_r = planar_r - self.tool_m * math.sin(pitch_rad)
        wrist_z = target.z + self.tool_m * math.cos(pitch_rad)

        min_reach = abs(self.upper_arm_m - self.forearm_m)
        max_reach = self.upper_arm_m + self.forearm_m
        wrist_distance = math.hypot(wrist_r, wrist_z)
        clamped = False
        if wrist_distance > max_reach:
            scale = max_reach / max(wrist_distance, 1e-9)
            wrist_r *= scale
            wrist_z *= scale
            clamped = True
        elif wrist_distance < min_reach:
            scale = min_reach / max(wrist_distance, 1e-9)
            wrist_r *= scale
            wrist_z *= scale
            clamped = True

        candidates = []
        for sign in (-1.0, 1.0):
            solution = self.solve_arm_branch(wrist_r, wrist_z, pitch, sign)
            if solution is not None:
                candidates.append((sign, [j1, *solution, target.grip_deg]))

        legal = [
            (sign, candidate)
            for sign, candidate in candidates
            if all(-1e-6 <= angle <= 270.0 + 1e-6 for angle in candidate[:4])
        ]
        chosen_sign = None
        chosen = None
        for sign, candidate in legal:
            if self.last_branch_sign is not None and sign == self.last_branch_sign:
                chosen_sign = sign
                chosen = candidate
                break
        if chosen is None and legal:
            chosen_sign, chosen = legal[0]
        if chosen is None and candidates:
            chosen_sign, candidate = candidates[0]
            chosen = [
                clamp(candidate[0], 0.0, 270.0),
                clamp(candidate[1], 0.0, 270.0),
                clamp(candidate[2], 0.0, 270.0),
                clamp(candidate[3], 0.0, 270.0),
                target.grip_deg,
            ]
            clamped = True
        if chosen is None:
            return None, None, True

        chosen[1] = self.compensate_j2_angle(chosen[1])
        chosen[4] = clamp(chosen[4], 0.0, 90.0)
        return chosen, chosen_sign, clamped

    def solve_arm_branch(self, wrist_r, wrist_z, pitch, sign):
        distance = math.hypot(wrist_r, wrist_z)
        min_reach = abs(self.upper_arm_m - self.forearm_m)
        max_reach = self.upper_arm_m + self.forearm_m
        if distance < min_reach - 1e-9 or distance > max_reach + 1e-9:
            return None
        cos_delta = clamp(
            (
                distance * distance
                - self.upper_arm_m * self.upper_arm_m
                - self.forearm_m * self.forearm_m
            )
            / (2.0 * self.upper_arm_m * self.forearm_m),
            -1.0,
            1.0,
        )
        delta = sign * math.acos(cos_delta)
        gamma = math.atan2(wrist_z, wrist_r)
        theta2 = gamma + math.atan2(
            self.upper_arm_m + self.forearm_m * math.cos(delta),
            self.forearm_m * math.sin(delta),
        )
        theta3 = theta2 + delta
        return [
            normalize_deg(math.degrees(theta2)),
            normalize_deg(135.0 - math.degrees(delta)),
            normalize_deg(math.degrees(theta3) + 135.0 - pitch),
        ]

    @staticmethod
    def compensate_j2_angle(j2):
        if j2 > 182.2:
            return j2 - 2.0
        if j2 < 177.8:
            return j2 + 2.0
        return j2

    def _target_in_arm_base(self):
        point = self._latest_point()
        if point is None:
            return None
        arm_base = np.array(
            [
                self.workspace_center_x_m,
                self.workspace_center_y_m,
                self.workspace_center_z_m,
            ],
            dtype=np.float64,
        )
        return point - arm_base

    def _arm_from_camera_rotation(self):
        """Return R_arm_camera for vectors; translation is deliberately unused."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "arm_base_link",
                self.camera_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
            return _rotation_matrix(transform.transform.rotation)
        except TransformException as exc:
            self.get_logger().warning(
                f"视觉伺服无法读取 arm_base_link<-{self.camera_frame} TF: {exc}"
            )
            return None

    @staticmethod
    def _unit(vector):
        length = float(np.linalg.norm(vector))
        if length < 1e-9:
            return None
        return np.asarray(vector, dtype=np.float64) / length

    def _visual_servo_direction(self, tf_direction):
        """Return a visual-led arm-frame direction, or None when vision is unsafe.

        The image ray is built only from target/tool pixels and camera intrinsics.
        The 3-D target TF is used here solely to learn its camera-plane bias for
        a later vision-loss fallback; it does not steer a valid visual step.
        """
        if not self.visual_servo_enabled:
            return None, "DISABLED"
        if self.visual_target_lost or self.latest_detection_center is None:
            return None, "TARGET_LOST"
        if time.monotonic() - self.last_detection_at > self.visual_servo_max_age_sec:
            return None, "STALE_DETECTION"
        if self.camera_info is None or len(self.camera_info.k) < 6:
            return None, "NO_CAMERA_INFO"

        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        if fx <= 1e-6 or fy <= 1e-6:
            return None, "INVALID_CAMERA_INFO"

        pixel_error = self.latest_detection_center - self.visual_servo_tool_pixel
        if float(np.linalg.norm(pixel_error)) > self.visual_servo_max_pixel_error:
            return None, "PIXEL_ERROR_OUTLIER"

        arm_from_camera = self._arm_from_camera_rotation()
        if arm_from_camera is None:
            return None, "NO_CAMERA_TF"

        # +x/+y in optical frame correspond to right/down in the RGB image.
        # A positive depth component advances along the camera optical axis.
        image_plane = np.array([pixel_error[0] / fx, pixel_error[1] / fy])
        visual_camera = self._unit(
            [image_plane[0], image_plane[1], self.visual_servo_depth_weight]
        )
        if visual_camera is None:
            return None, "INVALID_VISUAL_DIRECTION"

        # Learn only the image-plane disagreement.  The correction is bounded
        # and is intentionally updated only for a user-requested step.
        tf_camera = arm_from_camera.T @ tf_direction
        tf_plane = self._unit(tf_camera[:2])
        visual_plane = self._unit(image_plane)
        if tf_plane is not None and visual_plane is not None:
            self.tf_plane_correction_camera += self.tf_plane_correction_gain * (
                visual_plane - tf_plane
            )
            magnitude = float(np.linalg.norm(self.tf_plane_correction_camera))
            if magnitude > self.tf_plane_correction_limit:
                self.tf_plane_correction_camera *= (
                    self.tf_plane_correction_limit / magnitude
                )

        direction = self._unit(arm_from_camera @ visual_camera)
        if direction is None:
            return None, "INVALID_ARM_DIRECTION"
        return direction, (
            f"VISION px=({pixel_error[0]:+.0f},{pixel_error[1]:+.0f}) "
            f"tf_bias=({self.tf_plane_correction_camera[0]:+.3f},"
            f"{self.tf_plane_correction_camera[1]:+.3f})"
        )

    def _tf_fallback_direction(self, tf_direction):
        """Use the accumulated camera-plane correction only when vision fails."""
        arm_from_camera = self._arm_from_camera_rotation()
        if arm_from_camera is None:
            return tf_direction, "TF_FALLBACK raw_tf(no_camera_tf)"
        camera_direction = arm_from_camera.T @ tf_direction
        camera_direction[:2] += self.tf_plane_correction_camera
        corrected = self._unit(arm_from_camera @ camera_direction)
        if corrected is None:
            return tf_direction, "TF_FALLBACK raw_tf(invalid_correction)"
        return corrected, (
            f"TF_FALLBACK bias=({self.tf_plane_correction_camera[0]:+.3f},"
            f"{self.tf_plane_correction_camera[1]:+.3f})"
        )

    def _gimbal_correction(self):
        """Map one current image error to small J1/J4 angle corrections."""
        if not self.gimbal_servo_enabled:
            return (0.0, 0.0, "GIMBAL_OFF")
        if self.visual_target_lost or self.latest_detection_center is None:
            return (0.0, 0.0, "GIMBAL_NO_VISION")
        if time.monotonic() - self.last_detection_at > self.visual_servo_max_age_sec:
            return (0.0, 0.0, "GIMBAL_STALE")

        error = self.latest_detection_center - self.visual_servo_tool_pixel
        depth = self._current_target_camera_depth()
        if depth is None:
            gain_scale = 1.0
            depth_status = "depth=unknown scale=1.00"
        else:
            span = max(
                self.gimbal_gain_reference_depth_m - self.gimbal_gain_zero_depth_m,
                1e-6,
            )
            gain_scale = clamp(
                (depth - self.gimbal_gain_zero_depth_m) / span, 0.0, 1.0
            )
            depth_status = f"depth={depth:.3f} scale={gain_scale:.2f}"
        j1 = 0.0 if abs(float(error[0])) <= self.gimbal_pixel_deadband else (
            self.gimbal_j1_sign
            * self.gimbal_j1_gain_deg_per_px
            * gain_scale
            * float(error[0])
        )
        j4 = 0.0 if abs(float(error[1])) <= self.gimbal_pixel_deadband else (
            self.gimbal_j4_sign
            * self.gimbal_j4_gain_deg_per_px
            * gain_scale
            * float(error[1])
        )
        return (
            j1,
            j4,
            f"GIMBAL px=({error[0]:+.0f},{error[1]:+.0f}) "
            f"ddeg=({j1:+.2f},{j4:+.2f}) {depth_status}",
        )

    def _auto_center_correction(self):
        """Reuse J1/J4 image gimbal calibration for pre-sampling centering."""
        if self.visual_target_lost or self.latest_detection_center is None:
            return None, "NO_LOCKED_VISION"
        if time.monotonic() - self.last_detection_at > self.visual_servo_max_age_sec:
            return None, "STALE_DETECTION"
        error = self.latest_detection_center - self.auto_center_pixel
        if float(np.max(np.abs(error))) <= self.auto_center_deadband_px:
            return (0.0, 0.0, error), "CENTERED"

        depth = self._current_target_camera_depth()
        if depth is None:
            gain_scale = 1.0
        else:
            span = max(
                self.gimbal_gain_reference_depth_m - self.gimbal_gain_zero_depth_m,
                1e-6,
            )
            gain_scale = clamp(
                (depth - self.gimbal_gain_zero_depth_m) / span, 0.0, 1.0
            )
        j1_delta = self.gimbal_j1_sign * self.gimbal_j1_gain_deg_per_px * gain_scale * float(error[0])
        j4_delta = self.gimbal_j4_sign * self.gimbal_j4_gain_deg_per_px * gain_scale * float(error[1])
        return (j1_delta, j4_delta, error), "CORRECTING"

    def _begin_auto_centering(self) -> None:
        if not self.auto_center_before_sampling_enabled:
            self._start_auto_sampling()
            return
        self.auto_center_active = True
        self.auto_center_started_at = time.monotonic()
        self.auto_center_next_command_at = self.auto_center_started_at
        self.auto_center_steps = 0
        self.publish_pose_average_status("AUTO_CENTERING：锁定 HSV 目标移动到画面中心")

    def _advance_auto_centering(self) -> None:
        if not self.auto_center_active or self.state != POSE_AVERAGING:
            return
        now = time.monotonic()
        correction, status = self._auto_center_correction()
        if correction is not None and status == "CENTERED":
            _, _, error = correction
            self.auto_center_active = False
            self.publish_pose_average_status(
                "AUTO_CENTERED "
                f"px=({error[0]:+.0f},{error[1]:+.0f})，开始四角采样"
            )
            self.get_logger().info("预采样视觉居中完成，开始自动四角采样")
            self._start_auto_sampling()
            return
        if now - self.auto_center_started_at >= self.auto_center_timeout_sec:
            self.auto_center_active = False
            self.publish_pose_average_status(
                "AUTO_CENTER_TIMEOUT：目标未居中，已停止自动采样；请中止后重试"
            )
            self.set_state(POSE_AVERAGING, "预采样视觉居中超时，未启动四角采样")
            return
        if correction is None:
            self.publish_pose_average_status(f"AUTO_CENTER_WAITING {status}")
            return
        if now < self.auto_center_next_command_at:
            return
        if self.auto_center_steps >= self.auto_center_max_steps:
            self.auto_center_active = False
            self.publish_pose_average_status(
                "AUTO_CENTER_MAX_STEPS：目标未居中，已停止自动采样；请中止后重试"
            )
            self.set_state(POSE_AVERAGING, "预采样视觉居中达到最大步数，未启动四角采样")
            return
        if not self.current_joints_ready:
            self.publish_pose_average_status("AUTO_CENTER_WAITING_JOINTS")
            return

        j1_delta, j4_delta, error = correction
        current_degrees = self.ros_to_degrees(self.current_joint_ros[:4])
        target_degrees = list(current_degrees)
        target_degrees[0] = clamp(target_degrees[0] + j1_delta, 0.0, 270.0)
        target_degrees[3] = clamp(target_degrees[3] + j4_delta, 0.0, 270.0)
        self._publish_arm_trajectory(
            self.degrees_to_ros(target_degrees),
            duration_sec=self.auto_center_motion_duration_sec,
        )
        self.auto_center_steps += 1
        self.auto_center_next_command_at = (
            now + self.auto_center_motion_duration_sec + self.auto_center_motion_margin_sec
        )
        self.publish_pose_average_status(
            "AUTO_CENTERING "
            f"step={self.auto_center_steps}/{self.auto_center_max_steps} "
            f"px=({error[0]:+.0f},{error[1]:+.0f}) "
            f"ddeg=({j1_delta:+.2f},{j4_delta:+.2f})"
        )

    def _current_target_camera_depth(self):
        if self.latest_observation is not None:
            point = self.latest_observation.pose.position
            depth = self._camera_depth_from_base_point(
                np.array([point.x, point.y, point.z], dtype=np.float64),
                self.latest_observation.header.stamp,
            )
            if depth is not None:
                return depth
        return self.averaged_depth_m

    def _cancel_arm_trajectory_retries(self) -> None:
        for timer in self.arm_trajectory_retry_timers:
            timer.cancel()
        self.arm_trajectory_retry_timers.clear()

    def _cancel_gripper_command_retries(self) -> None:
        for timer in self.gripper_command_retry_timers:
            timer.cancel()
        self.gripper_command_retry_timers.clear()

    def _publish_gripper_command(
        self,
        degrees: float,
        duration_sec: float,
        repeat_count: int = 1,
        repeat_interval_sec: float = 0.12,
    ) -> None:
        self._cancel_gripper_command_retries()
        message = Float64MultiArray()
        message.data = [
            clamp(degrees, 0.0, 90.0) * self.gripper_stroke_m / 90.0,
            float(duration_sec),
        ]
        self.gripper_command_publisher.publish(message)
        for attempt in range(1, max(1, int(repeat_count))):
            timer_holder = {}

            def publish_retry(holder=timer_holder, retry_attempt=attempt):
                timer = holder.get("timer")
                if timer is not None:
                    timer.cancel()
                    if timer in self.gripper_command_retry_timers:
                        self.gripper_command_retry_timers.remove(timer)
                if self.active and self.state == MOVE_TO_GRASP:
                    self.gripper_command_publisher.publish(message)
                    self.get_logger().info(
                        "TF 直达 J5 指令补发 %d/%d"
                        % (retry_attempt + 1, repeat_count)
                    )

            timer = self.create_timer(repeat_interval_sec * attempt, publish_retry)
            timer_holder["timer"] = timer
            self.gripper_command_retry_timers.append(timer)

    def _cancel_tf_direct_j1_sequence(self) -> None:
        if self.tf_direct_j1_sequence_timer is not None:
            self.tf_direct_j1_sequence_timer.cancel()
            self.tf_direct_j1_sequence_timer = None
        self.tf_direct_j1_sequence_pending = None

    def _cancel_tf_direct_pre_sequence(self) -> None:
        if self.tf_direct_pre_sequence_timer is not None:
            self.tf_direct_pre_sequence_timer.cancel()
            self.tf_direct_pre_sequence_timer = None
        self.tf_direct_pre_sequence_pending = False

    def _begin_tf_direct_pre_sequence(self) -> None:
        self._cancel_tf_direct_pre_sequence()
        self._publish_gripper_command(
            self.tf_direct_pre_gripper_deg,
            self.tf_direct_pre_gripper_duration_sec,
            repeat_count=self.tf_direct_target_repeat_count,
            repeat_interval_sec=self.tf_direct_target_repeat_interval_sec,
        )
        self.tf_direct_pre_sequence_pending = True
        self.set_state(
            MOVE_TO_GRASP,
            "单步增量预备：J5 已复位至 %.1fdeg，等待 %.1fs 后执行 J1"
            % (
                self.tf_direct_pre_gripper_deg,
                self.tf_direct_pre_gripper_duration_sec
                + self.tf_direct_target_repeat_interval_sec
                * (self.tf_direct_target_repeat_count - 1),
            ),
        )
        self.tf_direct_pre_sequence_timer = self.create_timer(
            self.tf_direct_pre_gripper_duration_sec
            + self.tf_direct_target_repeat_interval_sec
            * (self.tf_direct_target_repeat_count - 1),
            self._complete_tf_direct_pre_sequence,
        )

    def _complete_tf_direct_pre_sequence(self) -> None:
        self._cancel_tf_direct_pre_sequence()
        if self.active and self.state == MOVE_TO_GRASP:
            self.execute_increment_step(skip_pre_gripper_reset=True)

    def _log_increment_target(self, target, current_tip) -> None:
        """Append one target sample per user-triggered increment attempt."""
        record = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "frame_id": "base_link",
            "target_m": {
                "x": round(float(target[0]), 6),
                "y": round(float(target[1]), 6),
                "z": round(float(target[2]), 6),
            },
            "tool_tip_m": {
                "x": round(float(current_tip.x), 6),
                "y": round(float(current_tip.y), 6),
                "z": round(float(current_tip.z), 6),
            },
            "tf_direct": bool(self.tf_direct_target_enabled),
            "j1_left_offset_deg": round(self.tf_direct_j1_offset_deg, 3),
            "j4_image_up_offset_deg": round(
                self.tf_direct_j4_image_up_offset_deg, 3
            ),
            "post_target_overshoot_m": round(self.tf_direct_overshoot_m, 4),
        }
        try:
            self.increment_sample_log_path.parent.mkdir(parents=True, exist_ok=True)
            with self.increment_sample_log_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, ensure_ascii=False) + "\n")
            self.get_logger().info(
                "抓取样本已写入 %s" % self.increment_sample_log_path
            )
        except OSError as exc:
            self.get_logger().error("抓取样本日志写入失败: %s" % exc)

    def _cancel_tf_direct_post_sequence(self) -> None:
        if self.tf_direct_post_sequence_timer is not None:
            self.tf_direct_post_sequence_timer.cancel()
            self.tf_direct_post_sequence_timer = None
        self.tf_direct_post_sequence_pending = None
        self.grasp_verify_active = False
        self.grasp_verify_samples.clear()

    def _send_post_grasp_j2(self) -> None:
        if not self.current_joints_ready:
            self.set_state(MOVE_TO_GRASP, "抓稳确认后等待 J2：缺少关节状态")
            return
        target_degrees = self.ros_to_degrees(self.current_joint_ros[:4])
        target_degrees[1] = self.tf_direct_post_j2_deg
        self._publish_arm_trajectory(
            self.degrees_to_ros(target_degrees),
            duration_sec=self.tf_direct_post_j2_duration_sec,
            repeat_count=self.tf_direct_target_repeat_count,
            repeat_interval_sec=self.tf_direct_target_repeat_interval_sec,
        )
        self._cancel_tf_direct_post_sequence()
        self.set_state(
            MOVE_TO_GRASP,
            "抓稳确认通过：J2 已发送至 %.1fdeg duration=%.1fs"
            % (self.tf_direct_post_j2_deg, self.tf_direct_post_j2_duration_sec),
        )
        # The H7 exposes command-shadowed joint readback rather than an action
        # result.  After its configured motion window, this is the task-level
        # completion point consumed by the mission executive.
        self._schedule_grasp_success(
            self.tf_direct_post_j2_duration_sec
            + self.tf_direct_target_repeat_interval_sec * (self.tf_direct_target_repeat_count - 1)
        )

    def _publish_result(self, value: str) -> None:
        message = String()
        message.data = value
        self.result_publisher.publish(message)

    def _schedule_grasp_success(self, delay_sec: float) -> None:
        if self.grasp_finish_timer is not None:
            self.grasp_finish_timer.cancel()
        self.grasp_finish_timer = self.create_timer(
            max(0.05, float(delay_sec)), self._finish_grasp_success
        )

    def _finish_grasp_success(self) -> None:
        if self.grasp_finish_timer is not None:
            self.grasp_finish_timer.cancel()
            self.grasp_finish_timer = None
        if not self.active:
            return
        self.active = False
        self.publish_active(False)
        self.publish_mode("idle")
        self.set_state(ABORT, "GRASP_SUCCESS：已完成回收，等待任务调度继续")
        self._publish_result("SUCCESS")

    def start_unload(self) -> None:
        if self.active:
            self.get_logger().warning("机械臂抓取流程仍在执行，拒绝卸货指令")
            return
        if len(self.unload_arm_degrees) != 4:
            self.get_logger().error("unload_arm_degrees 必须是四个关节角")
            self._publish_result("FAILED")
            return
        self._publish_arm_trajectory(
            self.degrees_to_ros(self.unload_arm_degrees),
            duration_sec=self.unload_arm_duration_sec,
            repeat_count=self.tf_direct_target_repeat_count,
            repeat_interval_sec=self.tf_direct_target_repeat_interval_sec,
        )
        self.set_state("UNLOADING", "已到投放区，机械臂移动至卸货姿态")
        self._schedule_unload_phase(self.unload_arm_duration_sec, self._open_for_unload)

    def _schedule_unload_phase(self, delay_sec: float, callback) -> None:
        if self.unload_timer is not None:
            self.unload_timer.cancel()
        self.unload_timer = self.create_timer(max(0.05, delay_sec), callback)

    def _open_for_unload(self) -> None:
        if self.unload_timer is not None:
            self.unload_timer.cancel()
            self.unload_timer = None
        self._publish_gripper_command(self.unload_release_deg, self.unload_release_sec)
        self.set_state("UNLOADING", "卸货姿态到位，已打开夹爪释放物块")
        self._schedule_unload_phase(self.unload_release_sec, self._finish_unload)

    def _finish_unload(self) -> None:
        if self.unload_timer is not None:
            self.unload_timer.cancel()
            self.unload_timer = None
        self.set_state(ABORT, "UNLOAD_COMPLETE：投放完成")
        self._publish_result("UNLOAD_COMPLETE")

    def _begin_tf_direct_post_sequence(self, tf_duration_sec: float) -> None:
        if not self.tf_direct_post_sequence_enabled:
            return
        self._cancel_tf_direct_post_sequence()
        now = time.monotonic()
        self.tf_direct_post_sequence_pending = {
            "phase": "wait_tf",
            "release_at": now
            + float(tf_duration_sec)
            + self.tf_direct_j1_completion_margin_sec
            + self.tf_direct_post_tf_settle_sec,
        }
        self.get_logger().info(
            "TF 后续序列已排队：等待 TF 完成后 %.1fs，再令 J5=%.1fdeg"
            % (self.tf_direct_post_tf_settle_sec, self.tf_direct_post_gripper_deg)
        )
        self.tf_direct_post_sequence_timer = self.create_timer(
            0.05, self._advance_tf_direct_post_sequence
        )

    def _advance_tf_direct_post_sequence(self) -> None:
        pending = self.tf_direct_post_sequence_pending
        if pending is None:
            self._cancel_tf_direct_post_sequence()
            return
        if not self.active or self.state != MOVE_TO_GRASP:
            self._cancel_tf_direct_post_sequence()
            return
        now = time.monotonic()
        if now < pending["release_at"]:
            return

        if pending["phase"] == "wait_tf":
            self._publish_gripper_command(
                self.tf_direct_post_gripper_deg,
                self.tf_direct_post_gripper_duration_sec,
                repeat_count=self.tf_direct_target_repeat_count,
                repeat_interval_sec=self.tf_direct_target_repeat_interval_sec,
            )
            pending["phase"] = "wait_gripper"
            gripper_window_sec = (
                self.tf_direct_post_gripper_duration_sec
                + self.tf_direct_target_repeat_interval_sec
                * (self.tf_direct_target_repeat_count - 1)
            )
            pending["release_at"] = now + gripper_window_sec
            self.set_state(
                MOVE_TO_GRASP,
                "TF 到位后等待完成：J5 已发送至 %.1fdeg，等待 %.1fs 后回 J2"
                % (
                    self.tf_direct_post_gripper_deg,
                    gripper_window_sec,
                ),
            )
            return

        if pending["phase"] == "wait_gripper":
            if not self.grasp_verify_enabled:
                self._send_post_grasp_j2()
                return
            self.grasp_verify_samples.clear()
            self.grasp_verify_active = True
            pending["phase"] = "wait_grasp_verify"
            pending["release_at"] = now + self.grasp_verify_timeout_sec
            self.set_state(
                MOVE_TO_GRASP,
                "J5 已闭合：等待左下角锁定 HSV 抓稳确认 "
                f"samples=0/{self.grasp_verify_required_samples}",
            )
            return

        if pending["phase"] == "wait_grasp_verify":
            count = len(self.grasp_verify_samples)
            if count >= self.grasp_verify_required_samples:
                self._send_post_grasp_j2()
                return
            if now >= pending["release_at"]:
                self._cancel_tf_direct_post_sequence()
                self.set_state(
                    MOVE_TO_GRASP,
                    "GRASP_UNCONFIRMED：左下角锁定 HSV 未满足抓稳占比，停止回收 J2",
                )
                self._publish_result("UNCONFIRMED")
            else:
                self.set_state(
                    MOVE_TO_GRASP,
                    "等待抓稳确认：左下角锁定 HSV "
                    f"samples={count}/{self.grasp_verify_required_samples}",
                )

    def _begin_tf_direct_j1_sequence(
        self, target_positions, duration_sec: float, tf_target
    ) -> None:
        """Move J1 first, then release the remaining direct-TF joint target."""
        self._cancel_tf_direct_j1_sequence()
        hold_positions = list(self.current_joint_ros[:4])
        hold_positions[0] = float(target_positions[0])
        self.tf_direct_j1_sequence_pending = {
            "target_positions": [float(value) for value in target_positions[:4]],
            "tf_target": [float(value) for value in tf_target],
            "duration_sec": float(duration_sec),
            "started_at": time.monotonic(),
            "release_at": time.monotonic()
            + float(duration_sec)
            + self.tf_direct_j1_completion_margin_sec,
        }
        hold_degrees = self.ros_to_degrees(hold_positions)
        target_degrees = self.ros_to_degrees(target_positions)
        self.get_logger().info(
            "TF 直达第一段：仅 J1 发送 "
            f"j1={target_degrees[0]:.2f}deg；"
            "J2/J3/J4 保持 "
            f"({hold_degrees[1]:.2f},{hold_degrees[2]:.2f},{hold_degrees[3]:.2f})deg；"
            f"release_after={duration_sec + self.tf_direct_j1_completion_margin_sec:.2f}s"
        )
        self._publish_arm_trajectory(
            hold_positions,
            duration_sec=duration_sec,
            repeat_count=self.tf_direct_target_repeat_count,
            repeat_interval_sec=self.tf_direct_target_repeat_interval_sec,
        )
        self.tf_direct_j1_sequence_timer = self.create_timer(
            0.05, self._complete_tf_direct_j1_sequence
        )

    def _complete_tf_direct_j1_sequence(self) -> None:
        pending = self.tf_direct_j1_sequence_pending
        if pending is None:
            self._cancel_tf_direct_j1_sequence()
            return
        if not self.active or self.state != MOVE_TO_GRASP:
            self._cancel_tf_direct_j1_sequence()
            return
        now = time.monotonic()
        if now < pending["release_at"]:
            return

        target = np.asarray(pending["tf_target"], dtype=np.float64)
        duration_sec = pending["duration_sec"]
        waited_sec = now - pending["started_at"]
        if not self.current_joints_ready:
            self.set_state(MOVE_TO_GRASP, "J1 完成后等待其余关节：缺少关节状态")
            return
        current_degrees = self.ros_to_degrees(self.current_joint_ros)
        current_tip = self.fk_from_degrees(current_degrees)
        current = np.array(
            [current_tip.x, current_tip.y, current_tip.z], dtype=np.float64
        )
        direction = self._unit(target - current)
        if direction is None:
            self.set_state(MOVE_TO_GRASP, "J1 完成后目标与工具点重合，取消后续直达")
            self._cancel_tf_direct_j1_sequence()
            return
        final_tip = target + direction * self.tf_direct_overshoot_m
        final_tip = np.array(
            [
                clamp(final_tip[0], self.increment_min_xyz[0], self.increment_max_xyz[0]),
                clamp(final_tip[1], self.increment_min_xyz[1], self.increment_max_xyz[1]),
                clamp(final_tip[2], self.increment_min_xyz[2], self.increment_max_xyz[2]),
            ],
            dtype=np.float64,
        )
        solution, branch_sign, clamped = self.solve_ik(
            IkTarget(
                x=float(final_tip[0]),
                y=float(final_tip[1]),
                z=float(final_tip[2]),
                pitch_deg=current_tip.pitch_deg,
                grip_deg=current_tip.grip_deg,
            )
        )
        if solution is None:
            self.set_state(MOVE_TO_GRASP, "J1 完成后 5cm 超前目标 IK 无解，取消后续直达")
            self._cancel_tf_direct_j1_sequence()
            return
        if branch_sign is not None:
            self.last_branch_sign = branch_sign
        # Preserve the completed J1 target; only the remaining arm geometry is
        # recomputed from the actual post-J1 tool pose.
        solution[0] = self.ros_to_degrees(pending["target_positions"])[0]
        if self.tf_direct_j4_image_up_offset_deg:
            solution[3] = clamp(
                solution[3] - self.tf_direct_j4_image_up_offset_deg, 0.0, 270.0
            )
        target_positions = self.degrees_to_ros(solution)
        self._cancel_tf_direct_j1_sequence()
        self._publish_arm_trajectory(
            target_positions,
            duration_sec=duration_sec,
            repeat_count=self.tf_direct_target_repeat_count,
            repeat_interval_sec=self.tf_direct_target_repeat_interval_sec,
        )
        if self.gimbal_servo_enabled:
            self.gimbal_phase_pending = True
            self.gimbal_phase_timer = self.create_timer(
                self.gimbal_phase_delay_sec, self._complete_gimbal_phase
            )
        self._begin_tf_direct_post_sequence(duration_sec)
        self.set_state(
            MOVE_TO_GRASP,
            "J1 动作窗口完成，已按新方向重算其余关节直达目标 "
            f"waited={waited_sec:.2f}s overshoot={self.tf_direct_overshoot_m:.3f}m "
            f"final=({final_tip[0]:.3f},{final_tip[1]:.3f},{final_tip[2]:.3f}) "
            f"duration={duration_sec:.1f}s{' IK_CLAMPED' if clamped else ''}",
        )

    def _publish_arm_trajectory(
        self,
        positions,
        duration_sec=None,
        repeat_count: int = 1,
        repeat_interval_sec: float = 0.12,
    ):
        self._cancel_arm_trajectory_retries()
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(self.arm_joint_names[:4])
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions[:4]]
        duration = self.increment_duration_sec if duration_sec is None else duration_sec
        point.time_from_start = Duration(seconds=duration).to_msg()
        message.points = [point]
        self.arm_trajectory_publisher.publish(message)

        for attempt in range(1, max(1, int(repeat_count))):
            timer_holder = {}

            def publish_retry(holder=timer_holder, retry_attempt=attempt):
                timer = holder.get("timer")
                if timer is not None:
                    timer.cancel()
                    if timer in self.arm_trajectory_retry_timers:
                        self.arm_trajectory_retry_timers.remove(timer)
                if self.active and self.state == MOVE_TO_GRASP:
                    self.arm_trajectory_publisher.publish(message)
                    self.get_logger().info(
                        "TF 直达机械臂指令补发 %d/%d"
                        % (retry_attempt + 1, repeat_count)
                    )

            timer = self.create_timer(
                repeat_interval_sec * attempt,
                publish_retry,
            )
            timer_holder["timer"] = timer
            self.arm_trajectory_retry_timers.append(timer)

    def _complete_gimbal_phase(self) -> None:
        if self.gimbal_phase_timer is not None:
            self.gimbal_phase_timer.cancel()
            self.gimbal_phase_timer = None
        self.gimbal_phase_pending = False
        if self.state != MOVE_TO_GRASP:
            return
        self._send_gimbal_correction()

    def _send_gimbal_correction(self) -> None:
        if not self.current_joints_ready:
            self.set_state(MOVE_TO_GRASP, "J1/J4 校准阶段跳过：没有最新关节反馈")
            return

        j1_delta, j4_delta, gimbal_status = self._gimbal_correction()
        if not j1_delta and not j4_delta:
            self.set_state(MOVE_TO_GRASP, f"J1/J4 校准阶段无有效视觉修正 {gimbal_status}")
            return

        current_degrees = self.ros_to_degrees(self.current_joint_ros)
        corrected = list(current_degrees)
        # Deliberately no per-step angle clamp here. The normal servo-angle
        # conversion still applies the hardware's configured legal range.
        corrected[0] = clamp(corrected[0] + j1_delta, 0.0, 270.0)
        corrected[3] = clamp(corrected[3] + j4_delta, 0.0, 270.0)
        positions = self.degrees_to_ros(corrected)
        self._publish_arm_trajectory(
            positions, duration_sec=self.gimbal_correction_duration_sec
        )
        self.set_state(
            MOVE_TO_GRASP,
            f"J1/J4 校准阶段已发送 {gimbal_status} "
            f"duration={self.gimbal_correction_duration_sec:.1f}s",
        )

    def execute_increment_step(self, skip_pre_gripper_reset: bool = False) -> None:
        if self.state != MOVE_TO_GRASP:
            self.get_logger().warning("忽略单步增量：当前状态不是 MOVE_TO_GRASP")
            return
        if self.gimbal_phase_pending:
            self.get_logger().warning("忽略单步增量：正在等待 J1/J4 校准阶段")
            return
        if (
            self.tf_direct_pre_sequence_pending
            or self.tf_direct_j1_sequence_pending
            or self.tf_direct_post_sequence_pending
        ):
            self.get_logger().warning("忽略单步增量：TF 分阶段动作尚未完成")
            return
        if not self.current_joints_ready:
            self.get_logger().warning("忽略单步增量：尚未收到完整 /joint_states")
            return
        if self.gimbal_servo_enabled and not self.gimbal_tf_approach_enabled:
            self._send_gimbal_correction()
            return
        target = self._target_in_arm_base()
        if target is None:
            self.get_logger().warning("忽略单步增量：没有平均目标位姿")
            return
        current_degrees = self.ros_to_degrees(self.current_joint_ros)
        current_tip = self.fk_from_degrees(current_degrees)
        if not skip_pre_gripper_reset:
            self._log_increment_target(target, current_tip)
            if (
                self.tf_direct_target_enabled
                and self.tf_direct_pre_gripper_reset_enabled
            ):
                self._begin_tf_direct_pre_sequence()
                return
        current = np.array([current_tip.x, current_tip.y, current_tip.z], dtype=np.float64)
        delta = target - current
        distance = float(np.linalg.norm(delta))
        tf_direction = self._unit(delta)
        if tf_direction is None:
            self.get_logger().warning("忽略单步增量：工具点与 TF 目标重合，无法生成方向")
            return

        # Pure-TF testing intentionally bypasses both image steering and the
        # learned camera-plane correction accumulated by visual servo.
        direction = tf_direction
        steering_status = "TF_ONLY"
        using_vision = False

        # A valid image ray is primary and may still be useful even when the
        # stale averaged TF point says the tool is already close.  TF fallback
        # retains the old proximity guard.
        if not using_vision and distance <= self.increment_goal_tolerance_m:
            self.set_state(
                MOVE_TO_GRASP,
                "TF 回退目标已接近工具点 "
                f"target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}) "
                f"tip=({current[0]:.3f},{current[1]:.3f},{current[2]:.3f}) "
                f"remain={distance:.3f} {steering_status}",
            )
            return

        step = (
            distance
            if self.tf_direct_target_enabled
            else min(
                self.increment_step_m,
                max(0.0, distance - self.increment_goal_tolerance_m),
            )
        )
        next_tip = current + direction * step
        next_tip = np.array(
            [
                clamp(next_tip[0], self.increment_min_xyz[0], self.increment_max_xyz[0]),
                clamp(next_tip[1], self.increment_min_xyz[1], self.increment_max_xyz[1]),
                clamp(next_tip[2], self.increment_min_xyz[2], self.increment_max_xyz[2]),
            ],
            dtype=np.float64,
        )
        ik_target = IkTarget(
            x=float(next_tip[0]),
            y=float(next_tip[1]),
            z=float(next_tip[2]),
            pitch_deg=current_tip.pitch_deg,
            grip_deg=current_tip.grip_deg,
        )
        solution, branch_sign, clamped = self.solve_ik(ik_target)
        if solution is None:
            self.get_logger().warning("忽略单步增量：IK 无解")
            return
        if branch_sign is not None:
            self.last_branch_sign = branch_sign
        if self.tf_direct_target_enabled and self.tf_direct_j1_offset_deg:
            # J1 increases from rover right toward rover front/left in the
            # calibrated arm convention.  A positive offset is therefore a
            # leftward correction.
            solution[0] = clamp(
                solution[0] + self.tf_direct_j1_offset_deg, 0.0, 270.0
            )
        if self.tf_direct_target_enabled and self.tf_direct_j4_image_up_offset_deg:
            solution[3] = clamp(
                solution[3] - self.tf_direct_j4_image_up_offset_deg, 0.0, 270.0
            )
        positions = self.degrees_to_ros(solution)
        duration = (
            self.tf_direct_target_duration_sec
            if self.tf_direct_target_enabled
            else self.increment_duration_sec
        )
        if self.tf_direct_target_enabled:
            self._begin_tf_direct_j1_sequence(positions, duration, target)
        else:
            self._publish_arm_trajectory(positions, duration_sec=duration)
        if self.gimbal_servo_enabled and not self.tf_direct_target_enabled:
            self.gimbal_phase_pending = True
            self.gimbal_phase_timer = self.create_timer(
                self.gimbal_phase_delay_sec, self._complete_gimbal_phase
            )
        suffix = " IK_CLAMPED" if clamped else ""
        phase_text = (
            "TF 直达：J1 已先行，等待到位后释放其余关节"
            if self.tf_direct_target_enabled
            else "TF 阶段已发送"
        )
        gimbal_text = (
            f"等待 J1/J4 校准 delay={self.gimbal_phase_delay_sec:.1f}s"
            if self.gimbal_servo_enabled
            else "无视觉伺服"
        )
        self.set_state(
            MOVE_TO_GRASP,
            f"{phase_text} {gimbal_text} "
            f"step={step:.3f}m duration={duration:.1f}s "
            f"tx={self.tf_direct_target_repeat_count if self.tf_direct_target_enabled else 1} "
            f"j1_offset={self.tf_direct_j1_offset_deg:+.1f}deg "
            f"j4_image_up={self.tf_direct_j4_image_up_offset_deg:+.1f}deg "
            f"tip=({current[0]:.3f},{current[1]:.3f},{current[2]:.3f}) -> "
            f"({next_tip[0]:.3f},{next_tip[1]:.3f},{next_tip[2]:.3f}) "
            f"target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}) "
            f"tf_remain={max(0.0, distance - step):.3f} "
            f"{steering_status}{suffix}",
        )

    # ---- publishers -----------------------------------------------------

    def publish_active(self, active: bool) -> None:
        message = Bool()
        message.data = bool(active)
        self.active_publisher.publish(message)

    def publish_mode(self, mode: str) -> None:
        self.current_mode = str(mode)
        message = String()
        message.data = self.current_mode
        self.mode_publisher.publish(message)

    def publish_workspace_status(self, text: str) -> None:
        self.current_workspace_status = str(text)
        message = String()
        message.data = self.current_workspace_status
        self.workspace_publisher.publish(message)

    def publish_pose_average_status(self, text: str) -> None:
        self.current_pose_average_status = str(text)
        message = String()
        message.data = self.current_pose_average_status
        self.pose_average_publisher.publish(message)

    def _publish_chassis_approach(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = float(linear_x)
        message.twist.angular.z = float(angular_z)
        self.chassis_approach_publisher.publish(message)

    def _stop_chassis_approach(self) -> None:
        self._publish_chassis_approach()
        self.chassis_approach_phase = "idle"

    def _begin_chassis_approach(self) -> None:
        if not self.chassis_approach_enabled:
            self.set_state(APPROACH_WITH_CHASSIS, "底盘接近已禁用，等待人工靠近")
            return
        if self.latest_detection_center is None:
            self.set_state(APPROACH_WITH_CHASSIS, "等待锁定 HSV 目标重新进入画面后再靠近")
            return
        duration = self.chassis_approach_step_m / self.chassis_approach_speed_mps
        self.chassis_approach_phase = "moving"
        self.chassis_approach_release_at = time.monotonic() + duration
        self.chassis_approach_step_count += 1
        self.set_state(
            APPROACH_WITH_CHASSIS,
            "底盘步进 %d：前进 %.2fm，保持目标居中"
            % (self.chassis_approach_step_count, self.chassis_approach_step_m),
        )

    def _advance_chassis_approach(self) -> None:
        if self.chassis_approach_phase == "idle":
            self._begin_chassis_approach()
            return
        now = time.monotonic()
        if self.chassis_approach_phase == "moving":
            if self.latest_detection_center is None or now - self.last_detection_at > self.target_timeout_sec:
                self._stop_chassis_approach()
                self.set_state(APPROACH_WITH_CHASSIS, "目标视觉丢失，底盘已停止")
                return
            pixel_error = float(self.latest_detection_center[0] - self.auto_center_pixel[0])
            angular_z = 0.0
            if abs(pixel_error) > self.chassis_approach_deadband_px:
                angular_z = clamp(
                    -pixel_error * self.chassis_approach_steer_gain,
                    -self.chassis_approach_max_angular_z,
                    self.chassis_approach_max_angular_z,
                )
            self._publish_chassis_approach(self.chassis_approach_speed_mps, angular_z)
            if now < self.chassis_approach_release_at:
                return
            self._publish_chassis_approach()
            self.chassis_approach_phase = "settling"
            self.chassis_approach_release_at = now + self.chassis_approach_settle_sec
            self.set_state(APPROACH_WITH_CHASSIS, "已完成 0.10m 步进，停车等待相机重新判距")
            return
        if self.chassis_approach_phase == "settling" and now >= self.chassis_approach_release_at:
            if self.latest_observation is None or now - self.latest_observation_at > self.observation_max_age_sec:
                self.chassis_approach_phase = "waiting_observation"
                self.set_state(APPROACH_WITH_CHASSIS, "等待停车后的新 RGB-D 观测")
                return
            point = self.latest_observation.pose.position
            self._update_post_lock_state(
                np.array([point.x, point.y, point.z], dtype=np.float64),
                allow_approach=True,
            )

    def set_state(self, state: str, detail: str = "") -> None:
        self.state = state
        self.state_entered = time.monotonic()
        self.current_status_detail = detail
        text = state if not detail else f"{state}: {detail}"
        self.current_status_text = text
        message = String()
        message.data = text
        self.status_publisher.publish(message)
        self.get_logger().info("[state] " + text)

    def republish_status(self) -> None:
        status = String()
        status.data = self.current_status_text
        self.status_publisher.publish(status)

        mode = String()
        mode.data = self.current_mode
        self.mode_publisher.publish(mode)

        workspace = String()
        workspace.data = self.current_workspace_status
        self.workspace_publisher.publish(workspace)

        average = String()
        average.data = self.current_pose_average_status
        self.pose_average_publisher.publish(average)

    # ---- triggers / commands -------------------------------------------

    def on_trigger(self, _message) -> None:
        if self.active:
            self.get_logger().warning("抓取任务已在进行，忽略触发")
            return
        self.start_search()

    def on_command(self, message) -> None:
        command = (message.data or "").strip().lower()
        if command in ("start", "start_search"):
            self.on_trigger(message)
        elif command in ("abort", "stop", "cancel"):
            self.abort("用户中止")
        elif command in ("step_once", "step"):
            self.execute_increment_step()
        elif command in ("skip_yolo_confirm", "skip_yolo"):
            self.skip_yolo_confirmation()
        elif command in ("unload", "drop", "release"):
            self.start_unload()
        else:
            self.get_logger().warning(f"未知抓取指令: {message.data!r}")

    def start_search(self) -> None:
        self._stop_chassis_approach()
        self._cancel_arm_trajectory_retries()
        self._cancel_gripper_command_retries()
        self._cancel_tf_direct_pre_sequence()
        self._cancel_tf_direct_j1_sequence()
        self._cancel_tf_direct_post_sequence()
        if self.gimbal_phase_timer is not None:
            self.gimbal_phase_timer.cancel()
            self.gimbal_phase_timer = None
        self.gimbal_phase_pending = False
        self.auto_center_active = False
        self.active = True
        self.publish_active(True)
        self.pending.clear()
        self.reset_pose_averaging()
        self.last_detection_at = time.monotonic()
        self.visual_target_lost = False
        self.latest_detection_center = None
        self.latest_detection_bbox = None
        self.latest_hsv_candidate = None
        self.tf_plane_correction_camera[:] = 0.0
        self.locked_class = None
        self.locked_confidence = 0.0
        self.locked_center = None
        self.averaged_point = None
        self.averaged_depth_m = None
        self.latest_observation = None
        self.latest_observation_at = 0.0
        self.publish_workspace_status("NO_OBSERVATION")
        self.publish_pose_average_status("NO_SAMPLES")
        self.publish_mode("wide_search")
        self.set_state(WIDE_SEARCH, "搜索中，等待自训练 YOLO 确认")

    def abort(self, reason: str) -> None:
        was_active = self.active
        self._stop_chassis_approach()
        self._cancel_arm_trajectory_retries()
        self._cancel_gripper_command_retries()
        self._cancel_tf_direct_pre_sequence()
        self._cancel_tf_direct_j1_sequence()
        self._cancel_tf_direct_post_sequence()
        if self.grasp_finish_timer is not None:
            self.grasp_finish_timer.cancel()
            self.grasp_finish_timer = None
        if self.unload_timer is not None:
            self.unload_timer.cancel()
            self.unload_timer = None
        if self.gimbal_phase_timer is not None:
            self.gimbal_phase_timer.cancel()
            self.gimbal_phase_timer = None
        self.gimbal_phase_pending = False
        self.auto_center_active = False
        self.publish_mode("idle")
        self.publish_active(False)
        self.active = False
        self.pending.clear()
        self.reset_pose_averaging()
        self.visual_target_lost = False
        self.latest_detection_center = None
        self.latest_detection_bbox = None
        self.latest_hsv_candidate = None
        self.tf_plane_correction_camera[:] = 0.0
        self.locked_class = None
        self.locked_confidence = 0.0
        self.locked_center = None
        self.averaged_point = None
        self.averaged_depth_m = None
        self.latest_observation = None
        self.latest_observation_at = 0.0
        self.publish_workspace_status("NO_OBSERVATION")
        self.publish_pose_average_status("NO_SAMPLES")
        self.set_state(ABORT, reason)
        if was_active:
            self._publish_result("ABORTED")

    # ---- perception -----------------------------------------------------

    def on_observation(self, message) -> None:
        if message.header.frame_id != "base_link":
            return
        self.latest_observation = message
        self.latest_observation_at = time.monotonic()
        if self.state == POSE_AVERAGING:
            self._collect_pose_sample(message)
        elif (
            self.state == APPROACH_WITH_CHASSIS
            and self.chassis_approach_phase == "waiting_observation"
        ):
            point = message.pose.position
            self._update_post_lock_state(
                np.array([point.x, point.y, point.z], dtype=np.float64),
                allow_approach=True,
            )

    def on_detection(self, message) -> None:
        if not self.active:
            return

        class_id = ""
        confidence = 0.0
        if message.results:
            class_id = message.results[0].hypothesis.class_id or ""
            confidence = float(message.results[0].hypothesis.score)
        center = (
            float(message.bbox.center.position.x),
            float(message.bbox.center.position.y),
        )
        self.latest_detection_center = np.asarray(center, dtype=np.float64)
        self.latest_detection_bbox = (
            float(message.bbox.size_x),
            float(message.bbox.size_y),
        )
        diagonal = math.hypot(float(message.bbox.size_x), float(message.bbox.size_y))

        # In locked_hsv mode, every detection is already constrained by the
        # remembered HSV profile, so it is safe to treat it as reacquisition.
        self.last_detection_at = time.monotonic()
        if self.state == WIDE_SEARCH and class_id.startswith(self.hsv_class_id):
            # The fusion node publishes its best HSV proposal on the regular
            # detection topic. Keep only this latest proposal; a manual bypass
            # must never resurrect an old candidate after the target changed.
            self.latest_hsv_candidate = {
                "at": self.last_detection_at,
                "class_id": class_id,
                "confidence": confidence,
                "center": center,
            }
        if self.visual_target_lost and self.state != WIDE_SEARCH:
            self.visual_target_lost = False
            self.publish_mode("precise_hsv_sam")
            self.set_state(
                self.state,
                "VISION_REACQUIRED：已重新找到锁定 HSV 目标，恢复 HSV+低频 SAM",
            )

        if self.state == WIDE_SEARCH and self._is_confirmed(class_id):
            self.pending.append(
                {
                    "at": self.last_detection_at,
                    "class_id": class_id,
                    "confidence": confidence,
                    "center": center,
                    "diagonal": diagonal,
                }
            )
            self._prune_pending()
            if self._try_lock():
                self.lock_target(class_id, confidence, center)

    def on_target_mask(self, message) -> None:
        """Measure locked HSV/SAM mask occupancy in the post-grasp lower-left ROI."""
        if not self.grasp_verify_active:
            return
        image_width, image_height = int(message.width), int(message.height)
        if image_width <= 0 or image_height <= 0 or len(self.grasp_verify_roi) != 4:
            return
        if message.step < image_width or len(message.data) < message.step * image_height:
            return
        rx, ry, rw, rh = self.grasp_verify_roi
        roi_x0 = int(round(max(0.0, min(1.0, rx)) * image_width))
        roi_y0 = int(round(max(0.0, min(1.0, ry)) * image_height))
        roi_x1 = int(round(max(0.0, min(1.0, rx + rw)) * image_width))
        roi_y1 = int(round(max(0.0, min(1.0, ry + rh)) * image_height))
        roi_area = max(1.0, float((roi_x1 - roi_x0) * (roi_y1 - roi_y0)))
        raw = np.frombuffer(message.data, dtype=np.uint8).reshape(image_height, message.step)
        mask = raw[:, :image_width]
        coverage = float(np.count_nonzero(mask[roi_y0:roi_y1, roi_x0:roi_x1])) / roi_area
        if coverage < self.grasp_verify_min_roi_coverage:
            return
        self.grasp_verify_samples.append((time.monotonic(), coverage))
        self.publish_pose_average_status(
            "GRASP_VERIFY lower_left_coverage=%.3f samples=%d/%d"
            % (
                coverage,
                len(self.grasp_verify_samples),
                self.grasp_verify_required_samples,
            )
        )

    def skip_yolo_confirmation(self) -> None:
        """Manually accept the latest best HSV proposal during commissioning."""
        if not self.active or self.state != WIDE_SEARCH:
            self.get_logger().warning("跳过 YOLO 被忽略：请先处于 WIDE_SEARCH")
            return
        candidate = self.latest_hsv_candidate
        if candidate is None:
            self.get_logger().warning("跳过 YOLO 被忽略：尚无 HSV 候选")
            self.set_state(WIDE_SEARCH, "等待 HSV 候选后才能跳过 YOLO 确认")
            return
        age = time.monotonic() - candidate["at"]
        if age > self.skip_yolo_candidate_timeout_sec:
            self.get_logger().warning("跳过 YOLO 被忽略：HSV 候选已过期")
            self.set_state(WIDE_SEARCH, "HSV 候选已过期，请等待当前最优候选")
            return
        self.get_logger().warning(
            "人工跳过 YOLO 确认：接受当前 HSV 候选，后续仅用于联调"
        )
        self.lock_target(
            "manual_hsv_confirmed",
            candidate["confidence"],
            candidate["center"],
        )

    def _is_confirmed(self, class_id: str) -> bool:
        """True only after the configured YOLO model confirms the proposal.

        The default detector replaces the HSV placeholder with a label from the
        self-trained ``weights/best.pt`` model.  The optional fusion backend
        may instead retain ``hsv_candidate`` (or ``hsv_candidate+sam``) until
        its explicitly enabled confirmation stage succeeds. Pure HSV is not
        trustworthy and must never lock the state machine.
        """
        if not class_id:
            return False
        return not class_id.startswith(self.hsv_class_id)

    def _prune_pending(self) -> None:
        now = time.monotonic()
        self.pending = [
            sample for sample in self.pending if now - sample["at"] <= self.lock_window_sec
        ]

    def _try_lock(self) -> bool:
        if len(self.pending) < self.lock_confirmation_frames:
            return False
        recent = self.pending[-self.lock_confirmation_frames :]
        centers = [sample["center"] for sample in recent]
        spread = 0.0
        for a, b in zip(centers, centers[1:]):
            spread = max(spread, math.hypot(a[0] - b[0], a[1] - b[1]))
        median_diagonal = float(
            np.median([sample["diagonal"] for sample in recent])
        )
        if median_diagonal <= 1e-6:
            return False
        return spread <= self.lock_center_spread_frac * median_diagonal

    def lock_target(self, class_id: str, confidence: float, center) -> None:
        self.visual_target_lost = False
        self.tf_plane_correction_camera[:] = 0.0
        self.locked_class = class_id
        self.locked_confidence = confidence
        self.locked_center = center
        self.publish_mode("locked_hsv")
        self.reset_pose_averaging()
        self.averaged_point = None
        self.averaged_depth_m = None
        self.publish_pose_average_status("NO_SAMPLES")
        position = ""
        if self.latest_observation is not None:
            point = self.latest_observation.pose.position
            position = (
                f" 位置=({point.x:.3f},{point.y:.3f},{point.z:.3f})"
            )
        self.set_state(
            TARGET_LOCKED,
            f"类别={class_id} 置信度={confidence:.2f} "
            f"中心=({center[0]:.0f},{center[1]:.0f})px{position}",
        )
        self.publish_mode("precise_hsv_sam")
        self.set_state(
            POSE_AVERAGING,
            "HSV 已锁定，进入 HSV+低频 SAM，等待 20cm 以上可靠深度统计",
        )
        self._begin_auto_centering()

    def _camera_depth_from_base_point(self, point, stamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                "base_link",
                Time.from_msg(stamp),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    "base_link",
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec),
                )
            except TransformException:
                return None
        rotation = _rotation_matrix(transform.transform.rotation)
        translation = transform.transform.translation
        offset = np.array([translation.x, translation.y, translation.z], dtype=np.float64)
        camera_point = rotation @ point + offset
        return float(camera_point[2])

    def _collect_pose_sample(self, observation) -> None:
        now = time.monotonic()
        point = observation.pose.position
        point_array = np.array([point.x, point.y, point.z], dtype=np.float64)
        depth = self._camera_depth_from_base_point(point_array, observation.header.stamp)
        if depth is None:
            self.publish_pose_average_status("WAITING_TF")
            return
        if depth < self.pose_average_min_camera_depth_m:
            self.publish_pose_average_status(
                "WAITING_RELIABLE_DEPTH "
                f"camera_z={depth:.3f}m min={self.pose_average_min_camera_depth_m:.3f}m"
            )
            return
        if depth > self.pose_average_max_camera_depth_m:
            self.publish_pose_average_status(
                "WAITING_TARGET_RANGE "
                f"camera_z={depth:.3f}m max={self.pose_average_max_camera_depth_m:.3f}m"
            )
            return

        if not self.current_joints_ready:
            self.publish_pose_average_status("WAITING_JOINTS")
            return
        if self.auto_center_active:
            self.publish_pose_average_status("AUTO_CENTERING：等待目标收敛到画面中心")
            return
        if self.auto_sampling_enabled and self.auto_sampling_targets:
            if now < self.auto_sampling_release_at:
                self.publish_pose_average_status(
                    "AUTO_MOVING "
                    f"pose={self.auto_sampling_index + 1}/4; "
                    f"wait={self.auto_sampling_release_at - now:.2f}s"
                )
                return
        current_joints = np.asarray(self.current_joint_ros[:4], dtype=np.float64)
        change_threshold = math.radians(self.pose_average_pose_change_deg)
        stable_threshold = math.radians(self.pose_average_joint_stable_deg)

        # After each completed batch, static readback must never create the
        # next pose batch. Require a deliberate, sufficiently large arm move.
        if self.pose_batch_waiting_for_motion:
            movement = float(
                np.max(
                    np.abs(current_joints - self.pose_batch_motion_reference)
                )
            )
            if movement < change_threshold:
                self.publish_pose_average_status(
                    "WAITING_NEXT_POSE "
                    f"pose={len(self.pose_batch_means)}/{self.pose_average_pose_count} "
                    f"move={math.degrees(movement):.2f}/{self.pose_average_pose_change_deg:.1f}deg"
                )
                return
            self.pose_batch_waiting_for_motion = False
            self.pose_batch_joint_reference = current_joints.copy()
            self.pose_batch_last_joint_readback = current_joints.copy()
            self.pose_batch_stable_since = now
            self.pose_batch_changed_at = now
            self.pose_samples.clear()
            self.publish_pose_average_status(
                "POSE_MOVED "
                f"pose={len(self.pose_batch_means) + 1}/{self.pose_average_pose_count}; "
                "等待机械臂静止"
            )
            return

        if self.pose_batch_joint_reference is None:
            self.pose_batch_joint_reference = current_joints.copy()
            self.pose_batch_changed_at = now
            self.pose_batch_last_joint_readback = current_joints.copy()
            self.pose_batch_stable_since = now

        joint_step = float(
            np.max(np.abs(current_joints - self.pose_batch_last_joint_readback))
        )
        joint_drift = float(
            np.max(np.abs(current_joints - self.pose_batch_joint_reference))
        )
        self.pose_batch_last_joint_readback = current_joints.copy()
        if joint_step > stable_threshold or joint_drift > stable_threshold:
            self.pose_samples.clear()
            self.pose_batch_joint_reference = current_joints.copy()
            self.pose_batch_changed_at = now
            self.pose_batch_stable_since = now
            self.publish_pose_average_status(
                "SETTLING "
                f"pose={len(self.pose_batch_means) + 1}/{self.pose_average_pose_count} "
                f"samples=0/{self.pose_average_samples_per_pose}; "
                f"joint_delta={math.degrees(max(joint_step, joint_drift)):.2f}deg"
            )
            return
        if now - self.pose_batch_stable_since < self.pose_average_settle_sec:
            batch_size = len(self.pose_samples)
            self.publish_pose_average_status(
                "SETTLING "
                f"pose={len(self.pose_batch_means) + 1}/{self.pose_average_pose_count} "
                f"samples={batch_size}/{self.pose_average_samples_per_pose}"
            )
            return

        self.pose_samples.append({"at": now, "point": point_array, "depth": depth})
        points = np.array([sample["point"] for sample in self.pose_samples], dtype=np.float64)
        center = np.mean(points, axis=0)
        spread = float(np.max(np.linalg.norm(points - center, axis=1))) if len(points) > 1 else 0.0
        self.publish_pose_average_status(
            "COLLECTING "
            f"pose={len(self.pose_batch_means) + 1}/{self.pose_average_pose_count} "
            f"samples={len(self.pose_samples)}/{self.pose_average_samples_per_pose} "
            f"camera_z={depth:.3f}m spread={spread:.3f}m"
        )
        if len(self.pose_samples) < self.pose_average_samples_per_pose:
            return

        if spread > self.pose_average_max_spread_m:
            self.pose_samples.clear()
            self.pose_batch_joint_reference = current_joints.copy()
            self.pose_batch_last_joint_readback = current_joints.copy()
            self.pose_batch_stable_since = now
            self.publish_pose_average_status(
                "REJECTED_SPREAD "
                f"pose={len(self.pose_batch_means) + 1}/{self.pose_average_pose_count} "
                f"spread={spread:.3f}m max={self.pose_average_max_spread_m:.3f}m; "
                "清空本角样本并重新采集"
            )
            return

        batch_mean = np.mean(points, axis=0)
        batch_depth = float(np.mean([sample["depth"] for sample in self.pose_samples]))
        self.pose_batch_means.append(batch_mean)
        self.pose_batch_depths.append(batch_depth)
        self.pose_samples.clear()
        self.pose_batch_joint_reference = current_joints.copy()
        self.pose_batch_motion_reference = current_joints.copy()
        self.pose_batch_last_joint_readback = current_joints.copy()
        self.pose_batch_stable_since = None
        self.pose_batch_changed_at = now
        completed = len(self.pose_batch_means)
        if completed < self.pose_average_pose_count:
            if self.auto_sampling_enabled and self.auto_sampling_targets:
                self.auto_sampling_index += 1
                if self.auto_sampling_index < len(self.auto_sampling_targets):
                    self._move_to_auto_sampling_corner()
                    return
                self.publish_pose_average_status(
                    "AUTO_SAMPLING_ERROR：角点数量不足以完成配置的采样轮数"
                )
                return
            self.pose_batch_waiting_for_motion = True
            self.publish_pose_average_status(
                "POSE_COMPLETE "
                f"pose={completed}/{self.pose_average_pose_count} "
                f"samples={self.pose_average_samples_per_pose}/{self.pose_average_samples_per_pose} "
                f"mean=({batch_mean[0]:.3f},{batch_mean[1]:.3f},{batch_mean[2]:.3f}) "
                "请移动机械臂到下一个姿态"
            )
            return

        batch_points = np.asarray(self.pose_batch_means, dtype=np.float64)
        self.averaged_point = np.mean(batch_points, axis=0)
        self.averaged_depth_m = float(np.mean(self.pose_batch_depths))
        pose_spread = float(
            np.max(np.linalg.norm(batch_points - self.averaged_point, axis=1))
        )
        self.publish_pose_average_status(
            "AVERAGED "
            f"poses={completed}/{self.pose_average_pose_count} "
            f"samples={completed * self.pose_average_samples_per_pose} "
            f"point=({self.averaged_point[0]:.3f},{self.averaged_point[1]:.3f},{self.averaged_point[2]:.3f}) "
            f"pose_spread={pose_spread:.3f}m"
        )
        self.set_state(
            WORKSPACE_CHECK,
            f"{completed} 个机械臂姿态的远距离位姿统计完成，进行工作区判定",
        )
        self._update_post_lock_state()

    def _latest_point(self):
        if self.averaged_point is not None:
            return np.array(self.averaged_point, dtype=np.float64)
        return None

    def _workspace_check(self, point):
        center = np.array(
            [
                self.workspace_center_x_m,
                self.workspace_center_y_m,
                self.workspace_center_z_m,
            ],
            dtype=np.float64,
        )
        distance = float(np.linalg.norm(point - center))
        failures = []
        if distance > self.workspace_radius_m:
            failures.append(f"dist>{self.workspace_radius_m:.3f}")
        return (len(failures) == 0), failures, distance

    def _update_post_lock_state(self, point=None, allow_approach=False) -> None:
        if point is None:
            point = self._latest_point()
        if point is None:
            self.publish_workspace_status("NO_OBSERVATION")
            return
        in_workspace, failures, distance = self._workspace_check(point)
        point_text = f"point=({point[0]:.3f},{point[1]:.3f},{point[2]:.3f})"
        center_text = (
            f" center=({self.workspace_center_x_m:.3f},"
            f"{self.workspace_center_y_m:.3f},{self.workspace_center_z_m:.3f})"
            f" dist={distance:.3f}"
        )
        if in_workspace:
            self._stop_chassis_approach()
            self.publish_workspace_status(f"IN_WORKSPACE {point_text}{center_text}")
            self.publish_mode("precise_hsv_sam")
            if self.state != MOVE_TO_GRASP:
                self.set_state(
                    MOVE_TO_GRASP,
                    "目标已进入工作区 "
                    f"位置=({point[0]:.3f},{point[1]:.3f},{point[2]:.3f}) "
                    f"dist={distance:.3f}，等待 TF+J1/J4 云台校准增量移动",
                )
                if (
                    self.auto_sampling_enabled
                    and self.auto_execute_increment_enabled
                    and not self.auto_increment_dispatched
                ):
                    self.auto_increment_dispatched = True
                    self.get_logger().info("自动四角采样完成，自动触发一次单步增量")
                    self.execute_increment_step()
            return
        self.publish_mode("locked_hsv")
        detail = ",".join(failures)
        self.publish_workspace_status(
            f"OUT_OF_WORKSPACE {point_text}{center_text} constraints={detail}"
        )
        if self.state != APPROACH_WITH_CHASSIS:
            self.set_state(
                APPROACH_WITH_CHASSIS,
                "目标未进入工作区，等待后续底盘接近控制 "
                f"位置=({point[0]:.3f},{point[1]:.3f},{point[2]:.3f}) "
                f"dist={distance:.3f} 约束={detail}",
            )
        if allow_approach or self.chassis_approach_phase == "idle":
            self._begin_chassis_approach()

    # ---- heartbeat ------------------------------------------------------

    def on_timer(self) -> None:
        if self.state == WIDE_SEARCH:
            self._prune_pending()
            self.republish_status()
            return
        if self.state in (
            TARGET_LOCKED,
            POSE_AVERAGING,
            WORKSPACE_CHECK,
            APPROACH_WITH_CHASSIS,
            DEPLOY_GRASP,
            MOVE_TO_GRASP,
        ):
            now = time.monotonic()
            if self.state == APPROACH_WITH_CHASSIS:
                self._advance_chassis_approach()
            if self.state == POSE_AVERAGING and self.auto_center_active:
                self._advance_auto_centering()
            if self.state == POSE_AVERAGING and self.auto_sampling_start_pending:
                self._start_auto_sampling()
            self.publish_tf_target_projection()
            self.republish_status()
            if (
                not self.visual_target_lost
                and now - self.last_detection_at > self.target_timeout_sec
            ):
                # A momentary occlusion from arm motion must not erase the HSV
                # lock or the far-range pose estimate. Keep tracking only the
                # locked color until it reappears; MOVE_TO_GRASP can continue
                # using its averaged TF target while vision is absent.
                self.visual_target_lost = True
                self.publish_mode("locked_hsv")
                if self.averaged_point is not None:
                    point = self.averaged_point
                    self.set_state(
                        self.state,
                        "VISUAL_HOLD：视觉短暂丢失，保留锁定 HSV 与平均 TF 目标 "
                        f"位置=({point[0]:.3f},{point[1]:.3f},{point[2]:.3f})",
                    )
                else:
                    self.publish_pose_average_status(
                        "VISUAL_HOLD：保持锁定 HSV，等待目标重获；"
                        f"pose={len(self.pose_batch_means) + 1}/{self.pose_average_pose_count} "
                        f"samples={len(self.pose_samples)}/{self.pose_average_samples_per_pose}"
                    )
                    self.set_state(
                        POSE_AVERAGING,
                        "VISUAL_HOLD：视觉短暂丢失，保留锁定 HSV 和已累计采样，等待重获",
                    )


def main():
    rclpy.init()
    node = GraspStateMachine()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok() and node.active:
            node.publish_active(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
