#!/usr/bin/env python3
"""Coordinate navigation locking, staged visual servoing, and grasp execution."""

import math
import time
from collections import Counter

import numpy as np
import rclpy
from action_msgs.srv import CancelGoal
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from lunar_rover_msgs.msg import TargetObservation
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String
from trajectory_msgs.msg import JointTrajectoryPoint


class GraspCoordinator(Node):
    def __init__(self):
        super().__init__("grasp_coordinator")
        self.arm_group = self.declare_parameter("arm_group", "arm").value
        self.arm_joint_names = list(
            self.declare_parameter(
                "arm_joint_names",
                ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4"],
            ).value
        )
        self.target_samples_required = int(
            self.declare_parameter("target_samples", 5).value
        )
        self.servo_samples_required = int(
            self.declare_parameter("servo_samples", 3).value
        )
        self.chassis_settle_sec = float(
            self.declare_parameter("chassis_settle_sec", 1.0).value
        )
        self.target_timeout_sec = float(
            self.declare_parameter("target_timeout_sec", 8.0).value
        )
        self.action_timeout_sec = float(
            self.declare_parameter("action_timeout_sec", 45.0).value
        )
        self.visual_servo_enabled = bool(
            self.declare_parameter("visual_servo_enabled", True).value
        )
        self.servo_coarse_tolerance_m = float(
            self.declare_parameter("servo_coarse_tolerance_m", 0.018).value
        )
        self.servo_fine_tolerance_m = float(
            self.declare_parameter("servo_fine_tolerance_m", 0.008).value
        )
        self.servo_max_iterations = int(
            self.declare_parameter("servo_max_iterations", 2).value
        )
        self.precise_preference_sec = float(
            self.declare_parameter("precise_preference_sec", 0.75).value
        )
        self.pregrasp_height_m = float(
            self.declare_parameter("pregrasp_height_m", 0.08).value
        )
        self.fine_standoff_m = float(
            self.declare_parameter("fine_standoff_m", 0.04).value
        )
        self.grasp_z_offset_m = float(
            self.declare_parameter("grasp_z_offset_m", 0.015).value
        )
        self.lift_height_m = float(
            self.declare_parameter("lift_height_m", 0.12).value
        )
        self.wall_pregrasp_standoff_m = float(
            self.declare_parameter("wall_pregrasp_standoff_m", 0.08).value
        )
        self.wall_fine_standoff_m = float(
            self.declare_parameter("wall_fine_standoff_m", 0.04).value
        )
        self.wall_grasp_offset_m = float(
            self.declare_parameter("wall_grasp_offset_m", 0.005).value
        )
        self.wall_retreat_m = float(
            self.declare_parameter("wall_retreat_m", 0.10).value
        )
        self.max_wall_yaw_mismatch_deg = float(
            self.declare_parameter("max_wall_yaw_mismatch_deg", 35.0).value
        )
        self.gripper_open_m = float(
            self.declare_parameter("gripper_open_m", 0.040).value
        )
        self.gripper_closed_m = float(
            self.declare_parameter("gripper_closed_m", 0.004).value
        )
        self.planning_time_sec = float(
            self.declare_parameter("planning_time_sec", 5.0).value
        )
        self.velocity_scaling = float(
            self.declare_parameter("velocity_scaling", 0.20).value
        )
        self.acceleration_scaling = float(
            self.declare_parameter("acceleration_scaling", 0.15).value
        )
        self.shoulder_x_m = float(
            self.declare_parameter("shoulder_x_m", 0.13).value
        )
        self.shoulder_z_m = float(
            self.declare_parameter("shoulder_z_m", 0.13145).value
        )
        self.upper_arm_m = float(
            self.declare_parameter("upper_arm_m", 0.11391).value
        )
        self.forearm_m = float(self.declare_parameter("forearm_m", 0.11000).value)
        self.tool_m = float(self.declare_parameter("tool_m", 0.15601).value)

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.active_publisher = self.create_publisher(Bool, "/grasp/active", state_qos)
        self.status_publisher = self.create_publisher(String, "/grasp/status", 10)
        self.create_subscription(Empty, "/grasp/trigger", self.on_trigger, 10)
        self.create_subscription(
            TargetObservation,
            "/perception/target_observation",
            self.on_target_observation,
            20,
        )
        self.create_subscription(
            TargetObservation,
            "/perception/target_pose_precise",
            self.on_target_observation,
            20,
        )
        self.create_subscription(PoseStamped, "/grasp/target_pose", self.on_legacy_pose, 10)
        self.create_subscription(Bool, "/search/active", self.on_search_active, state_qos)
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )
        self.nav_cancel_clients = [
            self.create_client(CancelGoal, name)
            for name in (
                "/navigate_to_pose/_action/cancel_goal",
                "/navigate_through_poses/_action/cancel_goal",
                "/follow_waypoints/_action/cancel_goal",
            )
        ]
        self.active = False
        self.search_active = False
        self.state = "idle"
        self.state_started = time.monotonic()
        self.accept_targets_after = 0.0
        self.last_structured_observation = 0.0
        self.last_precise_observation = -math.inf
        self.target_samples = []
        self.reference_target = None
        self.sequence = []
        self.sequence_index = 0
        self.servo_iterations = {"coarse": 0, "fine": 0}
        self.action_started = 0.0
        self.create_timer(0.1, self.on_timer)
        self.publish_active(False)

    def publish_active(self, active):
        message = Bool()
        message.data = bool(active)
        self.active_publisher.publish(message)

    def set_status(self, state, detail=""):
        self.state = state
        self.state_started = time.monotonic()
        message = String()
        message.data = state if not detail else f"{state}: {detail}"
        self.status_publisher.publish(message)
        self.get_logger().info(message.data)

    def on_trigger(self, _message):
        if self.search_active:
            self.get_logger().warning(
                "grasp request ignored: active arm search has not finished"
            )
            return
        if self.active:
            self.get_logger().warning("grasp request ignored: a grasp is already active")
            return
        self.active = True
        self.reference_target = None
        self.target_samples.clear()
        self.publish_active(True)
        self.cancel_navigation()
        self.accept_targets_after = time.monotonic() + self.chassis_settle_sec
        self.set_status("waiting_for_target")

    def cancel_navigation(self):
        request = CancelGoal.Request()
        for client in self.nav_cancel_clients:
            if client.service_is_ready():
                client.call_async(request)

    @staticmethod
    def _observation_values(message):
        point = message.pose.position
        normal = message.surface_normal
        return {
            "point": np.array([point.x, point.y, point.z], dtype=np.float64),
            "normal": np.array([normal.x, normal.y, normal.z], dtype=np.float64),
            "surface_type": int(message.surface_type),
            "class_id": message.class_id or "unknown",
            "confidence": float(message.confidence),
            "precise": bool(message.pose_is_precise),
        }

    def on_target_observation(self, message):
        now = time.monotonic()
        self.last_structured_observation = now
        if message.header.frame_id != "base_link":
            return
        if message.pose_is_precise:
            if any(not sample["precise"] for sample in self.target_samples):
                self.target_samples.clear()
            self.last_precise_observation = now
        elif now - self.last_precise_observation <= self.precise_preference_sec:
            return
        self.accept_target(self._observation_values(message))

    def on_search_active(self, message):
        self.search_active = bool(message.data)

    def on_legacy_pose(self, message):
        if time.monotonic() - self.last_structured_observation < 0.25:
            return
        if message.header.frame_id != "base_link":
            return
        point = message.pose.position
        self.accept_target(
            {
                "point": np.array([point.x, point.y, point.z], dtype=np.float64),
                "normal": np.array([0.0, 0.0, 1.0]),
                "surface_type": TargetObservation.SURFACE_GROUND,
                "class_id": "legacy_target",
                "confidence": 1.0,
                "precise": False,
            }
        )

    def accept_target(self, sample):
        if not self.active or time.monotonic() < self.accept_targets_after:
            return
        if self.state not in (
            "waiting_for_target",
            "visual_servo_coarse",
            "visual_servo_fine",
        ):
            return
        if not np.all(np.isfinite(sample["point"])):
            return
        self.target_samples.append(sample)
        required = (
            self.target_samples_required
            if self.state == "waiting_for_target"
            else self.servo_samples_required
        )
        if len(self.target_samples) < required:
            return
        target = self.aggregate_target(self.target_samples[-required:])
        self.target_samples.clear()
        if self.state == "waiting_for_target":
            self.start_grasp_sequence(target)
        else:
            stage = "coarse" if self.state.endswith("coarse") else "fine"
            self.complete_servo_observation(stage, target)

    @staticmethod
    def aggregate_target(samples):
        points = np.stack([sample["point"] for sample in samples])
        normals = np.stack([sample["normal"] for sample in samples])
        surface_type = Counter(sample["surface_type"] for sample in samples).most_common(1)[0][0]
        class_id = Counter(sample["class_id"] for sample in samples).most_common(1)[0][0]
        normal = np.median(normals, axis=0)
        norm = np.linalg.norm(normal)
        if norm > 1e-6:
            normal /= norm
        return {
            "point": np.median(points, axis=0),
            "normal": normal,
            "surface_type": surface_type,
            "class_id": class_id,
            "confidence": float(np.median([sample["confidence"] for sample in samples])),
            "precise": any(sample["precise"] for sample in samples),
        }

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def solve_planar_ik(self, target, tool_pitch):
        dx = float(target[0]) - self.shoulder_x_m
        dy = float(target[1])
        radial = math.hypot(dx, dy)
        z = float(target[2]) - self.shoulder_z_m
        yaw = math.atan2(dy, dx)
        wrist_r = radial - self.tool_m * math.sin(tool_pitch)
        wrist_z = z - self.tool_m * math.cos(tool_pitch)
        numerator = (
            wrist_r * wrist_r
            + wrist_z * wrist_z
            - self.upper_arm_m * self.upper_arm_m
            - self.forearm_m * self.forearm_m
        )
        denominator = 2.0 * self.upper_arm_m * self.forearm_m
        cosine = numerator / denominator
        if cosine < -1.0 - 1e-6 or cosine > 1.0 + 1e-6:
            raise ValueError(
                f"target outside arm workspace (wrist distance {math.hypot(wrist_r, wrist_z):.3f} m)"
            )
        cosine = min(max(cosine, -1.0), 1.0)
        candidates = []
        limits = [
            (-math.pi / 2.0, math.pi),
            (-1.5, math.pi),
            (-3.0 * math.pi / 4.0, 3.0 * math.pi / 4.0),
            (-1.5, math.pi),
        ]
        for delta in (math.acos(cosine), -math.acos(cosine)):
            theta = math.atan2(wrist_z, wrist_r) - math.atan2(
                self.forearm_m * math.sin(delta),
                self.upper_arm_m + self.forearm_m * math.cos(delta),
            )
            shoulder = math.pi / 2.0 - theta
            elbow = -delta
            wrist = self._normalize_angle(tool_pitch - shoulder - elbow)
            joints = [yaw, shoulder, elbow, wrist]
            if all(low <= value <= high for value, (low, high) in zip(joints, limits)):
                candidates.append(joints)
        if not candidates:
            raise ValueError("target IK solutions violate physical joint limits")
        return min(candidates, key=lambda values: sum(abs(value) for value in values))

    def normalized_surface(self, target):
        surface_type = target["surface_type"]
        point = target["point"]
        normal = target["normal"].copy()
        if surface_type == TargetObservation.SURFACE_UNKNOWN:
            if point[2] <= 0.18:
                surface_type = TargetObservation.SURFACE_GROUND
                normal = np.array([0.0, 0.0, 1.0])
            else:
                raise ValueError("target support surface is unknown; refusing blind grasp")
        if surface_type == TargetObservation.SURFACE_GROUND:
            if np.linalg.norm(normal) < 1e-6:
                normal = np.array([0.0, 0.0, 1.0])
            if normal[2] < 0.0:
                normal = -normal
        elif surface_type == TargetObservation.SURFACE_WALL:
            if np.linalg.norm(normal) < 1e-6:
                raise ValueError("wall target has no valid support normal")
            shoulder = np.array([self.shoulder_x_m, 0.0, self.shoulder_z_m])
            if float(np.dot(normal, shoulder - point)) < 0.0:
                normal = -normal
            normal[2] = 0.0
            normal /= max(np.linalg.norm(normal), 1e-9)
            approach = -normal
            radial = point.copy()
            radial[0] -= self.shoulder_x_m
            radial[2] = 0.0
            radial /= max(np.linalg.norm(radial), 1e-9)
            alignment = float(np.clip(np.dot(approach, radial), -1.0, 1.0))
            mismatch = math.degrees(math.acos(alignment))
            if mismatch > self.max_wall_yaw_mismatch_deg:
                raise ValueError(
                    f"wall normal is {mismatch:.1f} deg outside the 4-DOF arm approach plane"
                )
        target = dict(target)
        target["surface_type"] = surface_type
        target["normal"] = normal / max(np.linalg.norm(normal), 1e-9)
        return target

    def stage_pose(self, stage):
        target = self.reference_target
        point = target["point"].copy()
        if target["surface_type"] == TargetObservation.SURFACE_WALL:
            normal = target["normal"]
            offsets = {
                "pregrasp": self.wall_pregrasp_standoff_m,
                "near": self.wall_fine_standoff_m,
                "grasp": self.wall_grasp_offset_m,
                "lift": self.wall_retreat_m,
            }
            pose = point + normal * offsets[stage]
            if stage == "lift":
                pose[2] += 0.03
            return pose, math.pi / 2.0
        heights = {
            "pregrasp": self.pregrasp_height_m,
            "near": self.fine_standoff_m,
            "grasp": self.grasp_z_offset_m,
            "lift": self.lift_height_m,
        }
        pose = point.copy()
        pose[2] += heights[stage]
        return pose, math.pi

    def start_grasp_sequence(self, target):
        try:
            self.reference_target = self.normalized_surface(target)
            for stage in ("pregrasp", "near", "grasp", "lift"):
                pose, pitch = self.stage_pose(stage)
                self.solve_planar_ik(pose, pitch)
        except ValueError as exc:
            self.finish(False, str(exc))
            return
        surface = (
            "wall"
            if self.reference_target["surface_type"] == TargetObservation.SURFACE_WALL
            else "ground"
        )
        self.get_logger().info(
            f"grasp strategy={surface}, class={self.reference_target['class_id']}"
        )
        self.servo_iterations = {"coarse": 0, "fine": 0}
        self.sequence = [
            ("gripper", self.gripper_open_m, "open_gripper"),
            ("arm", "pregrasp", "move_pregrasp"),
            ("observe", "coarse", "visual_servo_coarse"),
            ("arm", "near", "move_near"),
            ("observe", "fine", "visual_servo_fine"),
            ("arm", "grasp", "move_to_target"),
            ("gripper", self.gripper_closed_m, "close_gripper"),
            ("arm", "lift", "retreat_target"),
        ]
        self.sequence_index = 0
        self.send_next_action()

    def complete_servo_observation(self, stage, target):
        try:
            updated = self.normalized_surface(target)
        except ValueError as exc:
            self.finish(False, str(exc))
            return
        if updated["surface_type"] != self.reference_target["surface_type"]:
            self.finish(False, "support surface classification changed during servo")
            return
        error = float(np.linalg.norm(updated["point"] - self.reference_target["point"]))
        tolerance = (
            self.servo_coarse_tolerance_m
            if stage == "coarse"
            else self.servo_fine_tolerance_m
        )
        self.reference_target = updated
        if error > tolerance:
            if self.servo_iterations[stage] >= self.servo_max_iterations:
                self.finish(
                    False,
                    f"{stage} visual servo did not converge ({error:.3f} m)",
                )
                return
            self.servo_iterations[stage] += 1
            pose_stage = "pregrasp" if stage == "coarse" else "near"
            self.sequence[self.sequence_index : self.sequence_index + 1] = [
                ("arm", pose_stage, f"visual_servo_{stage}_correction"),
                ("observe", stage, f"visual_servo_{stage}"),
            ]
            self.send_next_action()
            return
        self.get_logger().info(
            f"{stage} visual servo converged: residual={error:.3f} m"
        )
        self.sequence_index += 1
        self.send_next_action()

    def send_next_action(self):
        if self.sequence_index >= len(self.sequence):
            self.finish(True, "target grasped and retreated")
            return
        kind, value, label = self.sequence[self.sequence_index]
        if kind == "observe":
            if not self.visual_servo_enabled:
                self.sequence_index += 1
                self.send_next_action()
                return
            self.target_samples.clear()
            self.accept_targets_after = time.monotonic() + 0.15
            self.set_status(label)
            return
        self.set_status(label)
        self.action_started = time.monotonic()
        if kind == "arm":
            try:
                pose, pitch = self.stage_pose(value)
                joints = self.solve_planar_ik(pose, pitch)
            except ValueError as exc:
                self.finish(False, str(exc))
                return
            self.send_move_group_goal(joints)
        else:
            self.send_gripper_goal(float(value))

    def send_move_group_goal(self, joint_positions):
        if not self.move_group_client.wait_for_server(timeout_sec=0.5):
            self.finish(False, "MoveIt /move_action is unavailable")
            return
        goal = MoveGroup.Goal()
        goal.request.group_name = self.arm_group
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnectkConfigDefault"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = self.planning_time_sec
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.acceleration_scaling
        constraints = Constraints()
        constraints.name = self.state
        for name, position in zip(self.arm_joint_names, joint_positions):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(position)
            constraint.tolerance_above = 0.025
            constraint.tolerance_below = 0.025
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self.on_action_goal_response)

    def send_gripper_goal(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=0.5):
            self.finish(False, "gripper trajectory action is unavailable")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["arm_joint5"]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = 1
        goal.trajectory.points.append(point)
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self.on_action_goal_response)

    def on_action_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.finish(False, f"action submission failed: {exc}")
            return
        if goal_handle is None or not goal_handle.accepted:
            self.finish(False, f"{self.state} goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_action_result)

    def on_action_result(self, future):
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self.finish(False, f"action failed: {exc}")
            return
        if hasattr(result, "error_code") and hasattr(result.error_code, "val"):
            success = result.error_code.val == MoveItErrorCodes.SUCCESS
            detail = f"MoveIt error {result.error_code.val}"
        else:
            success = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            detail = getattr(result, "error_string", "trajectory failed")
        if not success:
            self.finish(False, detail)
            return
        self.sequence_index += 1
        self.send_next_action()

    def on_timer(self):
        if not self.active:
            return
        now = time.monotonic()
        if self.state in (
            "waiting_for_target",
            "visual_servo_coarse",
            "visual_servo_fine",
        ):
            if now - self.state_started > self.target_timeout_sec:
                self.finish(False, f"{self.state} received no stable RGB-D target")
        elif now - self.action_started > self.action_timeout_sec:
            self.finish(False, f"{self.state} timed out")

    def finish(self, success, detail):
        self.set_status("succeeded" if success else "failed", detail)
        self.active = False
        self.publish_active(False)
        self.target_samples.clear()
        self.sequence.clear()


def main():
    rclpy.init()
    node = GraspCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.active and rclpy.ok():
            node.publish_active(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
