#!/usr/bin/env python3
"""Confirm targets during normal operation and optionally execute discrete arm sweeps."""

import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from lunar_rover_msgs.msg import ManualControl, TargetObservation
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformException, TransformListener


def _rotation_matrix(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class ActiveSearch(Node):
    def __init__(self):
        super().__init__("active_search")
        self.enabled = bool(self.declare_parameter("enabled", True).value)
        self.auto_trigger_grasp = bool(
            self.declare_parameter("auto_trigger_grasp", False).value
        )
        self.periodic_sweep_enabled = bool(
            self.declare_parameter("periodic_sweep_enabled", False).value
        )
        self.confirmation_frames = int(
            self.declare_parameter("confirmation_frames", 5).value
        )
        self.confirmation_window_sec = float(
            self.declare_parameter("confirmation_window_sec", 1.2).value
        )
        self.max_position_spread_m = float(
            self.declare_parameter("max_position_spread_m", 0.035).value
        )
        self.min_confidence = float(
            self.declare_parameter("min_confidence", 0.45).value
        )
        self.memory_merge_distance_m = float(
            self.declare_parameter("memory_merge_distance_m", 0.12).value
        )
        self.retrigger_cooldown_sec = float(
            self.declare_parameter("retrigger_cooldown_sec", 20.0).value
        )
        self.stationary_speed_mps = float(
            self.declare_parameter("stationary_speed_mps", 0.02).value
        )
        self.stationary_before_sweep_sec = float(
            self.declare_parameter("stationary_before_sweep_sec", 3.0).value
        )
        self.startup_delay_sec = float(
            self.declare_parameter("startup_delay_sec", 20.0).value
        )
        self.sweep_interval_sec = float(
            self.declare_parameter("sweep_interval_sec", 30.0).value
        )
        self.sweep_dwell_sec = float(
            self.declare_parameter("sweep_dwell_sec", 1.0).value
        )
        self.search_poses = [
            list(self.declare_parameter("search_left", [0.65, 0.20, -0.45, 0.25]).value),
            list(self.declare_parameter("search_center", [0.0, 0.20, -0.45, 0.25]).value),
            list(self.declare_parameter("search_right", [-0.65, 0.20, -0.45, 0.25]).value),
        ]
        if any(len(pose) != 4 for pose in self.search_poses):
            raise ValueError("each search pose must contain four arm joint values")

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.search_active_publisher = self.create_publisher(
            Bool, "/search/active", state_qos
        )
        self.confirmed_publisher = self.create_publisher(
            TargetObservation, "/search/target_confirmed", 10
        )
        self.status_publisher = self.create_publisher(String, "/search/status", 10)
        self.trigger_publisher = self.create_publisher(Empty, "/grasp/trigger", 10)
        self.create_subscription(
            TargetObservation,
            "/perception/target_observation",
            self.on_observation,
            20,
        )
        self.create_subscription(
            TwistStamped, "/chassis/cmd_vel", self.on_chassis_command, 10
        )
        self.create_subscription(
            ManualControl, "/manual_control", self.on_manual_control, 10
        )
        self.create_subscription(Bool, "/grasp/active", self.on_grasp_active, state_qos)
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.observations = deque()
        self.memory = []
        self.grasp_active = False
        self.search_active = False
        self.scan_index = 0
        self.action_pending = False
        self.target_found_during_scan = False
        self.pending_auto_trigger = False
        self.last_chassis_motion = time.monotonic()
        self.started_at = time.monotonic()
        self.last_sweep_finished = self.started_at
        self.dwell_until = 0.0
        self.publish_search_active(False)
        self.create_timer(0.1, self.on_timer)
        self.publish_status("continuous_search_ready")

    def publish_status(self, value):
        message = String()
        message.data = value
        self.status_publisher.publish(message)
        self.get_logger().info(value)

    def publish_search_active(self, active):
        self.search_active = bool(active)
        message = Bool()
        message.data = self.search_active
        self.search_active_publisher.publish(message)

    def on_chassis_command(self, message):
        linear = message.twist.linear
        angular = message.twist.angular
        speed = math.sqrt(linear.x * linear.x + linear.y * linear.y)
        speed += abs(angular.z) * 0.2
        if speed > self.stationary_speed_mps:
            self.last_chassis_motion = time.monotonic()

    def on_manual_control(self, message):
        if max(abs(message.lx), abs(message.ly), abs(message.rx), abs(message.ry)) > 4:
            self.last_chassis_motion = time.monotonic()

    def on_grasp_active(self, message):
        self.grasp_active = bool(message.data)

    def _map_point(self, observation):
        point = observation.pose.position
        source = np.array([point.x, point.y, point.z], dtype=np.float64)
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                observation.header.frame_id,
                Time.from_msg(observation.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None
        rotation = _rotation_matrix(transform.transform.rotation)
        translation = transform.transform.translation
        return rotation @ source + np.array(
            [translation.x, translation.y, translation.z]
        )

    def _already_known(self, class_id, map_point, now):
        if map_point is None:
            return False
        self.memory = [
            entry
            for entry in self.memory
            if now - entry[2] <= self.retrigger_cooldown_sec
        ]
        for entry in self.memory:
            same_class = entry[0] == class_id or "unknown" in (entry[0], class_id)
            if same_class and np.linalg.norm(entry[1] - map_point) <= self.memory_merge_distance_m:
                if now - entry[2] <= self.retrigger_cooldown_sec:
                    return True
        return False

    def on_observation(self, observation):
        if not self.enabled or self.grasp_active:
            return
        if observation.confidence < self.min_confidence:
            return
        now = time.monotonic()
        map_point = self._map_point(observation)
        point = map_point
        if point is None:
            position = observation.pose.position
            point = np.array([position.x, position.y, position.z], dtype=np.float64)
        self.observations.append((now, observation.class_id, point, observation, map_point))
        while self.observations and now - self.observations[0][0] > self.confirmation_window_sec:
            self.observations.popleft()
        same_class = [
            item for item in self.observations if item[1] == observation.class_id
        ]
        if len(same_class) < self.confirmation_frames:
            return
        points = np.stack([item[2] for item in same_class[-self.confirmation_frames :]])
        center = np.median(points, axis=0)
        spread = float(np.max(np.linalg.norm(points - center, axis=1)))
        if spread > self.max_position_spread_m:
            return
        if self._already_known(observation.class_id, map_point, now):
            return

        self.confirmed_publisher.publish(observation)
        if map_point is not None:
            self.memory.append((observation.class_id, map_point, now))
        self.observations.clear()
        surface = {
            TargetObservation.SURFACE_GROUND: "ground",
            TargetObservation.SURFACE_WALL: "wall",
        }.get(observation.surface_type, "unknown_surface")
        self.publish_status(
            f"target_confirmed class={observation.class_id} surface={surface} spread={spread:.3f}m"
        )
        if self.search_active:
            self.target_found_during_scan = True
        if self.auto_trigger_grasp:
            if self.search_active:
                self.pending_auto_trigger = True
            else:
                self.trigger_publisher.publish(Empty())

    def _move_group_goal(self, positions):
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.planner_id = "RRTConnectkConfigDefault"
        goal.request.num_planning_attempts = 2
        goal.request.allowed_planning_time = 4.0
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.12
        constraints = Constraints()
        for index, position in enumerate(positions, start=1):
            constraint = JointConstraint()
            constraint.joint_name = f"arm_joint{index}"
            constraint.position = float(position)
            constraint.tolerance_above = 0.03
            constraint.tolerance_below = 0.03
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 1
        return goal

    def start_sweep(self):
        if not self.move_group_client.wait_for_server(timeout_sec=0.2):
            self.publish_status("search_sweep_skipped_moveit_unavailable")
            self.last_sweep_finished = time.monotonic()
            return
        self.scan_index = 0
        self.target_found_during_scan = False
        self.publish_search_active(True)
        self.publish_status("active_search_sweep_started")
        self.send_scan_pose()

    def send_scan_pose(self):
        if self.scan_index >= len(self.search_poses) or self.target_found_during_scan:
            self.finish_sweep()
            return
        self.action_pending = True
        future = self.move_group_client.send_goal_async(
            self._move_group_goal(self.search_poses[self.scan_index])
        )
        future.add_done_callback(self.on_scan_goal_response)

    def on_scan_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.publish_status(f"active_search_goal_error: {exc}")
            self.finish_sweep()
            return
        if handle is None or not handle.accepted:
            self.publish_status("active_search_goal_rejected")
            self.finish_sweep()
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_scan_result)

    def on_scan_result(self, future):
        self.action_pending = False
        try:
            result = future.result().result
            success = result.error_code.val == MoveItErrorCodes.SUCCESS
        except Exception:  # noqa: BLE001
            success = False
        if not success:
            self.publish_status("active_search_motion_failed")
            self.finish_sweep()
            return
        self.scan_index += 1
        self.dwell_until = time.monotonic() + self.sweep_dwell_sec

    def finish_sweep(self):
        self.action_pending = False
        self.publish_search_active(False)
        self.last_sweep_finished = time.monotonic()
        self.publish_status(
            "active_search_sweep_target_found"
            if self.target_found_during_scan
            else "active_search_sweep_complete"
        )
        if self.pending_auto_trigger:
            self.pending_auto_trigger = False
            self.trigger_publisher.publish(Empty())

    def on_timer(self):
        if not self.enabled or self.grasp_active:
            return
        now = time.monotonic()
        if self.search_active:
            if not self.action_pending and now >= self.dwell_until:
                self.send_scan_pose()
            return
        if not self.periodic_sweep_enabled:
            return
        if now - self.started_at < self.startup_delay_sec:
            return
        if now - self.last_chassis_motion < self.stationary_before_sweep_sec:
            return
        if now - self.last_sweep_finished < self.sweep_interval_sec:
            return
        self.start_sweep()


def main():
    rclpy.init()
    node = ActiveSearch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok() and node.search_active:
            node.publish_search_active(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
