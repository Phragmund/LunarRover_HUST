#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运动解算节点：订阅手动控制消息(杆量+按键) -> 解算 -> 直接控制 H7。

本节点直接持有 H7 单串口，把统一的杆量消息（LX/LY/RX/RY + relax/rearm）解算成
$M3508:SPD 与 $ANG 下发。逻辑与之前的键盘直连模式一致，但杆量是连续的：

    LX：阿克曼转向半径（右正），|LX| 越大转向越急；±255 时前外侧轮舵角恰好 30°。
    LY：油门（前正），±255 时为四轮平均速度 max_speed。
    RX：原地转向，非零即覆盖其他输入；±255 时四轮平均速度 max_speed。
    RY：预留。

轮序与下位机一致：0=右后 1=左后 2=右前 3=左前。$ANG 正 = 顺时针。
"""

import math
import re
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from lunar_rover_msgs.msg import ManualControl
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory

from h7_serial_client import H7SerialClient, DummyH7Client


class MotionSolver(Node):
    def __init__(self):
        super().__init__("motion_solver")

        # ---- H7 串口 ----
        self.h7_backend = self.declare_parameter("h7_backend", "serial").value
        self.h7_port = self.declare_parameter("h7_port", "/dev/ttyACM0").value
        self.h7_baud = int(self.declare_parameter("h7_baud", 115200).value)

        # ---- 几何 ----
        self.front_axle_x = float(self.declare_parameter("front_axle_x", 0.22).value)
        self.rear_axle_x = float(self.declare_parameter("rear_axle_x", -0.20).value)
        self.front_track = float(self.declare_parameter("front_track", 0.415).value)
        self.rear_track = float(self.declare_parameter("rear_track", 0.350).value)

        # ---- 运动参数 ----
        self.max_speed = float(self.declare_parameter("max_speed", 1.5).value)
        self.max_steer_deg = float(self.declare_parameter("max_steer_deg", 30.0).value)
        self.spin_pose_deg = float(self.declare_parameter("spin_pose_deg", 45.0).value)
        self.spin_delay = float(self.declare_parameter("spin_delay", 0.5).value)
        # Steering must reach its new angle before the drive motors push, or a
        # short step is spent creeping straight.  The H7 reports the real steer
        # angles in its periodic $STD frame, so gate on that feedback rather
        # than on a fixed delay.
        self.turn_steer_settle_enabled = bool(
            self.declare_parameter("turn_steer_settle_enabled", True).value
        )
        self.turn_steer_tolerance_deg = float(
            self.declare_parameter("turn_steer_tolerance_deg", 2.0).value
        )
        # Fall back to driving after this long so a steering fault degrades to
        # the old behaviour instead of stalling the rover outright.  The H7's
        # own $STEER:STALL/RESUME pairs show a steering move completing in
        # ~60 ms, so this is a generous cap that also bounds how long a
        # continuously steered joystick can hold the drive.
        self.turn_steer_timeout_sec = float(
            self.declare_parameter("turn_steer_timeout_sec", 0.5).value
        )
        # $STD arrives at roughly 5 Hz.  If it stops arriving there is nothing
        # to confirm against, so drive immediately instead of waiting out the
        # timeout on every command.
        self.steer_feedback_timeout_sec = float(
            self.declare_parameter("steer_feedback_timeout_sec", 1.0).value
        )
        self.boost_factor = float(self.declare_parameter("boost_factor", 2.2).value)
        self.front_toe_in_comp_deg = float(
            self.declare_parameter("front_toe_in_comp_deg", 2.5).value
        )
        self.arm_joint_ids = [
            int(x) for x in self.declare_parameter("arm_joint_ids", [1, 2, 3, 4, 5]).value
        ]
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
        self.arm_min_degrees = [
            float(x)
            for x in self.declare_parameter(
                "arm_min_degrees", [0.0, 0.0, 0.0, 0.0, 0.0]
            ).value
        ]
        self.arm_max_degrees = [
            float(x)
            for x in self.declare_parameter(
                "arm_max_degrees", [270.0, 270.0, 270.0, 270.0, 90.0]
            ).value
        ]
        self.arm_min_pulses = [
            int(x)
            for x in self.declare_parameter(
                "arm_min_pulses", [500, 500, 500, 500, 500]
            ).value
        ]
        self.arm_max_pulses = [
            int(x)
            for x in self.declare_parameter(
                "arm_max_pulses", [2500, 2500, 2500, 2500, 2500]
            ).value
        ]
        self.arm_inverted = [
            bool(x)
            for x in self.declare_parameter(
                "arm_inverted", [False, True, False, True, False]
            ).value
        ]
        self.gripper_stroke_m = float(
            self.declare_parameter("gripper_stroke_m", 0.045).value
        )
        self.arm_read_period = float(
            self.declare_parameter("arm_read_period", 0.10).value
        )
        self.arm_min_segment_ms = int(
            self.declare_parameter("arm_min_segment_ms", 120).value
        )
        self.arm_max_segment_ms = int(
            self.declare_parameter("arm_max_segment_ms", 5000).value
        )
        self.arm_topic_segment_ms = int(
            self.declare_parameter("arm_topic_segment_ms", 50).value
        )
        self.arm_topic_min_segment_ms = int(
            self.declare_parameter("arm_topic_min_segment_ms", 30).value
        )
        self.enable_drive_on_start = bool(
            self.declare_parameter("enable_drive_on_start", True).value
        )
        self.log_arm_feedback = bool(
            self.declare_parameter("log_arm_feedback", False).value
        )
        self.input_topic = self.declare_parameter("input_topic", "/manual_control").value
        self.arm_joint_trajectory_topic = self.declare_parameter(
            "arm_joint_trajectory_topic", "/arm_controller/joint_trajectory"
        ).value
        self.gripper_command_topic = self.declare_parameter(
            "gripper_command_topic", "/gripper_controller/commands"
        ).value
        self.raw_command_topic = self.declare_parameter(
            "raw_command_topic", "/h7/raw_command"
        ).value
        self.raw_feedback_topic = self.declare_parameter(
            "raw_feedback_topic", "/h7/rx_frame"
        ).value
        # Narrow exception to the manipulation lock below.  /grasp/active is
        # raised for a whole grasp session, including the guarded chassis
        # approach that the grasp state machine itself drives; without this
        # channel the lock would drop the approach twist and deadlock the
        # chassis.  The flag is honoured only while it keeps arriving.
        self.chassis_approach_active_topic = self.declare_parameter(
            "chassis_approach_active_topic", "/grasp/chassis_approach_active"
        ).value
        self.chassis_approach_timeout = max(
            0.1, float(self.declare_parameter("chassis_approach_timeout", 0.5).value)
        )

        count = len(self.arm_joint_names)
        mapping_lengths = {
            len(self.arm_joint_ids), len(self.arm_zero_degrees),
            len(self.arm_ros_directions), len(self.arm_min_degrees),
            len(self.arm_max_degrees), len(self.arm_min_pulses),
            len(self.arm_max_pulses), len(self.arm_inverted), count,
        }
        if mapping_lengths != {count} or count != 5:
            raise ValueError("all arm mapping arrays must contain exactly five entries")

        self._precompute()

        # ---- H7 客户端（steer_auto_send=False -> ANG 单发）----
        self.h7 = self._create_h7()

        # ---- 状态 ----
        self.mode = "stop"
        self.turn_side = "left"
        self.spin_side = "left"
        self._turn_wheels_started = False
        self._last_ang = None
        # Steer-then-drive gate shared by straight and turn modes.
        self._turn_angles = None
        self._turn_drive_speeds = [0.0] * 4
        self._turn_wheels_started = False
        self._turn_steer_sent_at = 0.0
        self._turn_hold_deadline = 0.0
        self._steer_feedback_at = 0.0
        self._prev_relax = False
        self._prev_rearm = False
        self._prev_calibrate = False
        self._calibration_active = False
        self._last_calibration_time = -1.0e9
        self.boost_active = False
        self.grasp_active = False
        self.search_active = False
        # chassis-approach yield channel: last value plus arrival time, so a
        # publisher that dies mid-step cannot leave the chassis unlocked.
        self._chassis_approach_requested = False
        self._chassis_approach_stamp = 0.0
        self._manipulation_lock_state = False
        self._arm_lock = threading.Lock()
        self._trajectory_lock = threading.Lock()
        # Real-arm desk-facing home confirmed on 2026-08-29. The older coarse
        # [90, 180, 135, 135, 45] seed is obsolete but preserved in git history.
        home_degrees = [96.754752, 201.804458, 183.510522, 110.293936, 45.0]
        self._arm_positions = [
            self._degrees_to_ros(index, degrees)
            for index, degrees in enumerate(home_degrees)
        ]
        self._arm_feedback_time = [0.0] * count
        self._arm_read_index = 0
        self._arm_read_pending_index = None
        self._last_arm_read_sent = 0.0
        self.arm_read_timeout = float(
            self.declare_parameter("arm_read_timeout", 0.20).value
        )
        self.h7.register_frame_callback(self._on_h7_frame)

        callback_group = ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            ManualControl, self.input_topic, self.on_message, 1,
            callback_group=callback_group,
        )
        grasp_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.grasp_subscription = self.create_subscription(
            Bool,
            "/grasp/active",
            self.on_grasp_active,
            grasp_qos,
            callback_group=callback_group,
        )
        self.search_subscription = self.create_subscription(
            Bool,
            "/search/active",
            self.on_search_active,
            grasp_qos,
            callback_group=callback_group,
        )
        # Deliberately VOLATILE, unlike the two locks above: a late-joining
        # motion_solver must not receive a latched "true" from a finished
        # approach and unlock the chassis with no live publisher behind it.
        # The producer refreshes the flag at its approach tick rate instead.
        self.chassis_approach_subscription = self.create_subscription(
            Bool,
            self.chassis_approach_active_topic,
            self.on_chassis_approach_active,
            10,
            callback_group=callback_group,
        )
        self.arm_trajectory_subscription = self.create_subscription(
            JointTrajectory,
            self.arm_joint_trajectory_topic,
            self.on_arm_joint_trajectory,
            10,
            callback_group=callback_group,
        )
        self.gripper_command_subscription = self.create_subscription(
            Float64MultiArray,
            self.gripper_command_topic,
            self.on_gripper_command,
            10,
            callback_group=callback_group,
        )
        self.raw_command_subscription = self.create_subscription(
            String,
            self.raw_command_topic,
            self.on_raw_command,
            10,
            callback_group=callback_group,
        )
        self.raw_feedback_publisher = self.create_publisher(
            String, self.raw_feedback_topic, 50
        )
        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states", 20)
        self.arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_trajectory,
            goal_callback=self.validate_trajectory_goal,
            cancel_callback=self.cancel_trajectory,
            callback_group=callback_group,
        )
        self.gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            execute_callback=self.execute_trajectory,
            goal_callback=self.validate_trajectory_goal,
            cancel_callback=self.cancel_trajectory,
            callback_group=callback_group,
        )
        self.create_timer(0.02, self.tick, callback_group=callback_group)
        self.create_timer(0.05, self.publish_joint_states, callback_group=callback_group)
        self.create_timer(
            max(0.05, self.arm_read_period),
            self.request_next_arm_feedback,
            callback_group=callback_group,
        )

        self.get_logger().info(
            "运动解算节点已启动：max_speed %.2f、AKM前外侧满舵 %.1f°、原地转延迟 %.2f s"
            % (self.max_speed, self.max_steer_deg, self.spin_delay)
        )
        self.get_logger().info("订阅 %s" % self.input_topic)
        self.get_logger().info(
            "机械臂轨迹接口已就绪：/arm_controller 与 /gripper_controller"
        )
        self.get_logger().info(
            "机械臂手操 topic 接口已就绪：%s 与 %s"
            % (self.arm_joint_trajectory_topic, self.gripper_command_topic)
        )
        self.get_logger().info("H7 raw ROS 指令入口已就绪：%s" % self.raw_command_topic)
        self.get_logger().info("H7 raw ROS 回传出口已就绪：%s" % self.raw_feedback_topic)

    def _precompute(self):
        # 轮位顺序：右后、左后、右前、左前（x 前正，y 左正）
        self.positions = [
            (self.rear_axle_x, -self.rear_track * 0.5),    # 右后
            (self.rear_axle_x, +self.rear_track * 0.5),    # 左后
            (self.front_axle_x, -self.front_track * 0.5),  # 右前
            (self.front_axle_x, +self.front_track * 0.5),  # 左前
        ]
        self.zero_speeds = [0.0] * 4
        self.zero_angles = [0.0] * 4
        self.forward_comp_angles = [
            0.0,
            0.0,
            -self.front_toe_in_comp_deg,
            +self.front_toe_in_comp_deg,
        ]
        # 原地转向：四轮滚动方向切向指向车体中心旋转圆。
        # 轮序为右后、左后、右前、左前；H7 $ANG 正为顺时针。
        rear_spin_deg = math.degrees(
            math.atan2(abs(self.rear_axle_x), self.rear_track * 0.5)
        )
        front_spin_deg = math.degrees(
            math.atan2(abs(self.front_axle_x), self.front_track * 0.5)
        )
        self.spin_pose_angles = [
            +rear_spin_deg,
            -rear_spin_deg,
            -front_spin_deg,
            +front_spin_deg,
        ]
        # 原地转向轮速比例（按前后轮到中心的距离，平均=1）
        front_dist = math.hypot(self.front_axle_x, self.front_track * 0.5)
        rear_dist = math.hypot(abs(self.rear_axle_x), self.rear_track * 0.5)
        mean_dist = (front_dist + rear_dist) / 2.0
        self.front_ratio = front_dist / mean_dist
        self.rear_ratio = rear_dist / mean_dist

    def _create_h7(self):
        if self.h7_backend == "serial":
            return H7SerialClient(
                port=self.h7_port,
                baud=self.h7_baud,
                logger=self.get_logger(),
                steer_auto_send=False,
            )
        if self.h7_backend == "dummy":
            return DummyH7Client(logger=self.get_logger())
        raise ValueError("h7_backend must be 'serial' or 'dummy'")

    # ---------- 机械臂 ROS / H7 映射 ----------
    def _ros_to_degrees(self, index, position):
        if index == 4:
            if self.gripper_stroke_m <= 0.0:
                raise ValueError("gripper_stroke_m must be positive")
            degrees = position * 90.0 / self.gripper_stroke_m
        else:
            degrees = self.arm_zero_degrees[index] + self.arm_ros_directions[index] * math.degrees(position)
        return min(max(degrees, self.arm_min_degrees[index]), self.arm_max_degrees[index])

    def _degrees_to_ros(self, index, degrees):
        if index == 4:
            return degrees * self.gripper_stroke_m / 90.0
        return math.radians(
            (degrees - self.arm_zero_degrees[index]) / self.arm_ros_directions[index]
        )

    def _degrees_to_pulse(self, index, degrees):
        span_degrees = self.arm_max_degrees[index] - self.arm_min_degrees[index]
        ratio = (degrees - self.arm_min_degrees[index]) / span_degrees
        if self.arm_inverted[index]:
            ratio = 1.0 - ratio
        pulse = self.arm_min_pulses[index] + ratio * (
            self.arm_max_pulses[index] - self.arm_min_pulses[index]
        )
        return int(round(min(max(pulse, self.arm_min_pulses[index]), self.arm_max_pulses[index])))

    def _pulse_to_degrees(self, index, pulse):
        ratio = (pulse - self.arm_min_pulses[index]) / float(
            self.arm_max_pulses[index] - self.arm_min_pulses[index]
        )
        if self.arm_inverted[index]:
            ratio = 1.0 - ratio
        return self.arm_min_degrees[index] + ratio * (
            self.arm_max_degrees[index] - self.arm_min_degrees[index]
        )

    def _on_h7_frame(self, frame):
        feedback = String()
        feedback.data = frame
        self.raw_feedback_publisher.publish(feedback)
        if frame.startswith("$STD"):
            # $STD carries the measured steer angles; its arrival time is what
            # makes the steer-then-drive gate trustworthy.
            self._steer_feedback_at = time.monotonic()
        if frame.startswith("$STEER:"):
            self.get_logger().info("H7 回执: %s" % frame)
            if frame.startswith("$STEER:CALSTART,"):
                self._calibration_active = True
            elif frame.startswith("$STEER:CALDONE,"):
                self._calibration_active = False
                self._return_steering_to_zero_after_calibration()
            elif frame.startswith("$STEER:CALABORT!") or frame.startswith("$STEER:ERR,CAL_BUSY!"):
                self._calibration_active = False
            return
        match = re.search(r"\$ARM:RESP,#(\d{1,3})P(\d{1,4})!", frame)
        if match is None:
            return
        if self.log_arm_feedback:
            self.get_logger().info("[ARM] RX %s" % match.group(0))
        joint_id = int(match.group(1))
        try:
            index = self.arm_joint_ids.index(joint_id)
        except ValueError:
            return
        degrees = self._pulse_to_degrees(index, int(match.group(2)))
        with self._arm_lock:
            self._arm_positions[index] = self._degrees_to_ros(index, degrees)
            self._arm_feedback_time[index] = time.monotonic()
            if self._arm_read_pending_index == index:
                self._arm_read_pending_index = None

    def request_next_arm_feedback(self):
        now = time.monotonic()
        with self._arm_lock:
            if self._arm_read_pending_index is not None:
                if now - self._last_arm_read_sent < self.arm_read_timeout:
                    return
                self._arm_read_pending_index = None
            index = self._arm_read_index
            self._arm_read_index = (index + 1) % len(self.arm_joint_ids)
            self._arm_read_pending_index = index
            self._last_arm_read_sent = now
        if self.log_arm_feedback:
            self.get_logger().info("[ARM] TX $ARM:READ,%d!" % self.arm_joint_ids[index])
        self.h7.send_raw("$ARM:READ,%d!" % self.arm_joint_ids[index])

    def publish_joint_states(self):
        with self._arm_lock:
            positions = list(self._arm_positions)
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.arm_joint_names)
        message.position = positions
        self.joint_state_publisher.publish(message)

    def validate_trajectory_goal(self, goal_request):
        trajectory = goal_request.trajectory
        names = list(trajectory.joint_names)
        valid_sets = [self.arm_joint_names[:4], [self.arm_joint_names[4]]]
        if names not in valid_sets or not trajectory.points:
            self.get_logger().error("拒绝机械臂轨迹：关节集合或轨迹点无效")
            return GoalResponse.REJECT
        if any(len(point.positions) != len(names) for point in trajectory.points):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def cancel_trajectory(_goal_handle):
        return CancelResponse.ACCEPT

    @staticmethod
    def _duration_seconds(duration):
        return float(duration.sec) + float(duration.nanosec) * 1e-9

    def _send_arm_positions(self, names, positions, duration_ms):
        parts = []
        updates = []
        for name, position in zip(names, positions):
            index = self.arm_joint_names.index(name)
            degrees = self._ros_to_degrees(index, float(position))
            pulse = self._degrees_to_pulse(index, degrees)
            parts.extend([str(self.arm_joint_ids[index]), str(pulse)])
            updates.append((index, self._degrees_to_ros(index, degrees)))
        frame = "$ARM:MULTI,%s,%d!" % (",".join(parts), duration_ms)
        self.get_logger().info("[ARM] SEND %s" % frame)
        self.h7.send_raw(frame)
        with self._arm_lock:
            for index, position in updates:
                self._arm_positions[index] = position

    def on_arm_joint_trajectory(self, message):
        names = list(message.joint_names)
        if names != self.arm_joint_names[:4] or not message.points:
            self.get_logger().warning(
                "忽略机械臂 topic 轨迹：关节集合或轨迹点无效",
                throttle_duration_sec=2.0,
            )
            return
        point = message.points[-1]
        if len(point.positions) != len(names):
            self.get_logger().warning(
                "忽略机械臂 topic 轨迹：positions 数量不匹配",
                throttle_duration_sec=2.0,
            )
            return
        duration_ms = int(round(self._duration_seconds(point.time_from_start) * 1000.0))
        if duration_ms <= 0:
            duration_ms = self.arm_topic_segment_ms
        duration_ms = min(max(duration_ms, self.arm_topic_min_segment_ms), self.arm_max_segment_ms)
        try:
            self._send_arm_positions(names, point.positions, duration_ms)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("机械臂 topic 轨迹发送失败: %s" % exc)

    def on_gripper_command(self, message):
        if not message.data:
            return
        names = [self.arm_joint_names[4]]
        positions = [float(message.data[0])]
        duration_sec = (
            float(message.data[1]) if len(message.data) > 1 else None
        )
        requested_ms = (
            int(round(duration_sec * 1000.0))
            if duration_sec is not None
            else self.arm_topic_segment_ms
        )
        duration_ms = min(
            max(requested_ms, self.arm_topic_min_segment_ms),
            self.arm_max_segment_ms,
        )
        try:
            self._send_arm_positions(names, positions, duration_ms)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("夹爪 topic 命令发送失败: %s" % exc)

    def on_raw_command(self, message):
        frame = str(message.data).strip()
        if not frame:
            return
        if not frame.startswith("$") or not frame.endswith("!"):
            self.get_logger().warning(
                "忽略 H7 raw 指令：帧格式必须为 $...!"
            )
            return
        allowed_prefixes = (
            "$M3508:",
            "$ANG,",
            "$SPD,",
            "$STOP!",
            "$ZERO!",
            "$STEER:",
            "$ARM:",
        )
        if not any(frame.startswith(prefix) for prefix in allowed_prefixes):
            self.get_logger().warning("忽略 H7 raw 指令：不允许的前缀 %s" % frame[:16])
            return
        self.h7.send_raw(frame)

    def _stop_arm_joints(self, names):
        for name in names:
            index = self.arm_joint_names.index(name)
            self.h7.send_raw("$ARM:STOP,%d!" % self.arm_joint_ids[index])

    def execute_trajectory(self, goal_handle):
        result = FollowJointTrajectory.Result()
        if not self._trajectory_lock.acquire(blocking=False):
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "another arm trajectory is active"
            goal_handle.abort()
            return result
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)
        try:
            for name in names:
                index = self.arm_joint_names.index(name)
                self.h7.send_raw("$ARM:TORQUE,%d,1!" % self.arm_joint_ids[index])

            previous_time = 0.0
            for point in trajectory.points:
                if goal_handle.is_cancel_requested:
                    self._stop_arm_joints(names)
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "trajectory canceled"
                    return result
                point_time = self._duration_seconds(point.time_from_start)
                segment_seconds = max(0.0, point_time - previous_time)
                duration_ms = int(round(segment_seconds * 1000.0))
                duration_ms = min(
                    max(duration_ms, self.arm_min_segment_ms), self.arm_max_segment_ms
                )
                self._send_arm_positions(names, point.positions, duration_ms)

                deadline = time.monotonic() + duration_ms * 0.001
                while time.monotonic() < deadline:
                    if goal_handle.is_cancel_requested:
                        self._stop_arm_joints(names)
                        goal_handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "trajectory canceled"
                        return result
                    time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
                previous_time = point_time

                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = names
                feedback.desired = point
                with self._arm_lock:
                    feedback.actual.positions = [
                        self._arm_positions[self.arm_joint_names.index(name)]
                        for name in names
                    ]
                feedback.error.positions = [
                    desired - actual
                    for desired, actual in zip(
                        feedback.desired.positions, feedback.actual.positions
                    )
                ]
                goal_handle.publish_feedback(feedback)

            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "trajectory completed"
            return result
        except Exception as exc:  # noqa: BLE001
            self._stop_arm_joints(names)
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            self.get_logger().error("机械臂轨迹失败: %s" % exc)
            return result
        finally:
            self._trajectory_lock.release()

    # ---------- 消息入口 ----------
    def on_message(self, msg):
        if self.manipulation_locked():
            return
        if msg.relax and not self._prev_relax:
            self._relax_all()
        if msg.rearm and not self._prev_rearm:
            self._rearm_all()
        if msg.calibrate and not self._prev_calibrate:
            self._trigger_calibration()
        self._prev_relax = msg.relax
        self._prev_rearm = msg.rearm
        self._prev_calibrate = msg.calibrate
        self.boost_active = msg.boost

        lx, ly, rx = msg.lx, msg.ly, msg.rx

        # 优先级：RX 原地转 > LX 转向 > LY 直行 > 停止
        if rx != 0:
            self._enter_spin(rx)
        elif lx != 0:
            self._enter_turn(lx, ly)
        elif ly != 0:
            self._enter_straight(ly)
        else:
            self._enter_stop()

    def on_grasp_active(self, message):
        active = bool(message.data)
        if active == self.grasp_active:
            return
        self.grasp_active = active
        self.update_manipulation_lock("抓取会话")

    def on_search_active(self, message):
        active = bool(message.data)
        if active == self.search_active:
            return
        self.search_active = active
        self.update_manipulation_lock("探索扫描")

    def on_chassis_approach_active(self, message):
        self._chassis_approach_requested = bool(message.data)
        self._chassis_approach_stamp = time.monotonic()
        self.update_manipulation_lock(
            "底盘接近让行" if self._chassis_approach_requested else "底盘接近让行结束"
        )

    def chassis_approach_yield(self):
        """True only while a live guarded approach keeps refreshing the flag."""
        if not self._chassis_approach_requested:
            return False
        age = time.monotonic() - self._chassis_approach_stamp
        return 0.0 <= age <= self.chassis_approach_timeout

    def manipulation_locked(self):
        if self.chassis_approach_yield():
            # The grasp state machine is itself driving the chassis and its
            # twist reaches us through /manual_control.  Honouring the lock
            # here would drop the one command that is trying to move the rover,
            # so a live guarded approach owns the chassis instead.
            return False
        return self.grasp_active or self.search_active

    def update_manipulation_lock(self, source):
        # The approach flag is refreshed several times per second, so act only
        # on real transitions: re-issuing the stop on every refresh would
        # cancel the very step the yield was granted for.
        locked = self.manipulation_locked()
        if locked == self._manipulation_lock_state:
            return
        self._manipulation_lock_state = locked
        if locked:
            self.boost_active = False
            self._enter_stop()
            self.get_logger().info(
                f"{source}：底盘安全锁已启用，忽略遥控并保持3508零速"
            )
        else:
            self.get_logger().info(f"{source}：底盘安全锁已解除，恢复遥控")

    # ---------- 单发 ANG（变化超过阈值才发）----------
    def _send_ang_if_changed(self, angles):
        if self._last_ang is None or any(
            abs(a - b) > 0.5 for a, b in zip(angles, self._last_ang)
        ):
            self.h7.send_steering_once(angles)
            self._last_ang = list(angles)

    def _send_ang_force(self, angles):
        self.h7.send_steering_once(angles)
        self._last_ang = list(angles)

    # ---------- 下发轮速（boost 时整体放大）----------
    def _set_drive_speeds(self, speeds):
        if self.boost_active:
            speeds = [s * self.boost_factor for s in speeds]
        self.h7.set_drive_speeds(speeds)

    # ---------- 各模式 ----------
    # ---------- 先打舵，舵到位后再驱动 ----------
    def _steering_feedback_fresh(self):
        return (
            time.monotonic() - self._steer_feedback_at
        ) <= self.steer_feedback_timeout_sec

    def _steering_at_target(self):
        if self._turn_angles is None or not self._steering_feedback_fresh():
            return False
        actual = self.h7.get_steer_angles()
        if actual is None or len(actual) != len(self._turn_angles):
            return False
        return all(
            abs(float(a) - float(b)) <= self.turn_steer_tolerance_deg
            for a, b in zip(actual, self._turn_angles)
        )

    def _format_angles(self, values):
        return "[" + ",".join("%.1f" % v for v in values) + "]"

    def _command_steering(self, mode, angles, speeds, force_rearm=False, hold_sec=None):
        """Send $ANG first; tick() releases the drive once the wheels arrive.

        Issuing $ANG and $M3508:SPD in the same call let the drive motors push
        while the steered wheels were still travelling, so a short guarded step
        was spent creeping straight and the rover never actually turned.  The
        hold budget is per mode entry and is deliberately not reset when the
        target angle changes: a continuously steered joystick would otherwise
        re-arm the wait forever and hold the drive at zero.

        straight, turn and spin all share this gate, so all three are handled
        here.
        """
        entry = force_rearm or self.mode != mode
        if entry:
            budget = self.turn_steer_timeout_sec if hold_sec is None else hold_sec
            self._turn_hold_deadline = time.monotonic() + budget
            self._turn_wheels_started = False
            self._turn_steer_sent_at = time.monotonic()
        self.mode = mode
        self._send_ang_if_changed(angles)
        self._turn_angles = [float(a) for a in angles]
        self._turn_drive_speeds = [float(s) for s in speeds]
        if not self.turn_steer_settle_enabled:
            self._turn_wheels_started = True
        if self._turn_wheels_started:
            self._set_drive_speeds(speeds)
        else:
            self._set_drive_speeds(self.zero_speeds)

    def _advance_steer_settle(self):
        """Release the drive as soon as the steered wheels are in position."""
        if self._turn_wheels_started:
            return
        if not self._steering_feedback_fresh():
            # Nothing live to confirm against: drive straight away rather than
            # stalling every command behind an unverifiable wait.
            reason = "无舵角反馈"
        elif self._steering_at_target():
            reason = "舵角已到位"
        elif time.monotonic() >= self._turn_hold_deadline:
            reason = "等待超时"
        else:
            self._set_drive_speeds(self.zero_speeds)
            return
        self._turn_wheels_started = True
        waited = time.monotonic() - self._turn_steer_sent_at
        detail = "目标 %s / 反馈 %s" % (
            self._format_angles(self._turn_angles or []),
            self._format_angles(self.h7.get_steer_angles()),
        )
        if reason == "等待超时":
            self.get_logger().warning(
                "转向 %.2fs 未到位，按超时放行驱动：%s" % (waited, detail)
            )
        else:
            self.get_logger().info(
                "转向就绪（%s，%.2fs）：%s" % (reason, waited, detail)
            )
        self._set_drive_speeds(self._turn_drive_speeds)

    def _enter_straight(self, ly):
        speed = ly / 255.0 * self.max_speed
        angles = self.forward_comp_angles if ly > 0 else self.zero_angles
        self._command_steering("straight", angles, [speed, speed, speed, speed])

    def _enter_turn(self, lx, ly):
        side = "right" if lx > 0 else "left"
        max_steer = abs(lx) / 255.0 * math.radians(self.max_steer_deg)
        radius = self.front_axle_x / math.tan(max_steer) + self.front_track * 0.5
        angles_l = []
        dists = []
        for x, y in self.positions:
            kin = math.atan2(x, radius - y)      # 左转运动学舵角（逆时针正）
            angles_l.append(-math.degrees(kin))  # $ANG 正=顺时针，所以下发取反
            dists.append(math.hypot(x, radius - y))
        mean_d = sum(dists) / 4.0
        ratios_l = [d / mean_d for d in dists]
        if side == "left":
            angles = angles_l
            ratios = ratios_l
        else:
            angles = [-a for a in angles_l]
            ratios = [ratios_l[1], ratios_l[0], ratios_l[3], ratios_l[2]]
        if ly > 0:
            angles = [a + c for a, c in zip(angles, self.forward_comp_angles)]
        throttle = ly / 255.0 * self.max_speed
        speeds = [r * throttle for r in ratios]
        self.turn_side = side
        self._command_steering("turn", angles, speeds)

    def _enter_spin(self, rx):
        side = "right" if rx > 0 else "left"
        mean_speed = abs(rx) / 255.0 * self.max_speed
        rr = self.rear_ratio * mean_speed
        rf = self.front_ratio * mean_speed
        if side == "left":
            speeds = [rr, -rr, rf, -rf]    # 左自转：右进左退
        else:
            speeds = [-rr, rr, -rf, rf]    # 右自转：右退左进

        # Spin also needs the wheels at the spin pose before the motors turn:
        # that is exactly the "set the steering first, then rotate" order.  A
        # change of spin direction is a fresh manoeuvre and re-arms the gate.
        self._command_steering(
            "spin",
            self.spin_pose_angles,
            speeds,
            force_rearm=side != self.spin_side,
            hold_sec=self.spin_delay + self.turn_steer_timeout_sec,
        )
        self.spin_side = side

    def _enter_stop(self):
        previous_mode = self.mode
        self.mode = "stop"
        if previous_mode != "stop":
            self._send_ang_force(self.zero_angles)
        else:
            self._send_ang_if_changed(self.zero_angles)
        self._set_drive_speeds(self.zero_speeds)
        self._turn_wheels_started = False

    # ---------- R / T 单次动作 ----------
    def _relax_all(self):
        self.h7.send_raw("$M3508:STOP!")
        self._set_drive_speeds(self.zero_speeds)
        self.h7.send_raw("$STOP!")
        for joint_id in self.arm_joint_ids:
            self.h7.send_raw("$ARM:TORQUE,%d,0!" % joint_id)
        self.mode = "stop"
        self._turn_wheels_started = False
        self.get_logger().info("R：已断开所有转矩输出")

    def _rearm_all(self):
        self.h7.send_raw("$ZERO!")
        self.h7.send_raw("$ANG,0,0,0,0!")
        for joint_id in self.arm_joint_ids:
            self.h7.send_raw("$ARM:TORQUE,%d,1!" % joint_id)
        self.get_logger().info("T：已重启位置保持并设零")

    def _trigger_calibration(self):
        # 防抖：距上次标定发送不足 1000ms 则忽略本次触发
        now = time.monotonic()
        if now - self._last_calibration_time < 1.0:
            self.get_logger().info("标定防抖：距上次标定不足 1s，忽略本次触发")
            return
        self._last_calibration_time = now
        # 触发四轮机械限位标定（mask=15 全通道，单通道超时 8000ms），下位机自动跑
        self._set_drive_speeds(self.zero_speeds)
        self.mode = "stop"
        self._turn_wheels_started = False
        self._calibration_active = True
        self.h7.send_raw("$STEER:CAL,15,8000!")
        self.get_logger().info("已触发四轮机械限位标定: $STEER:CAL,15,8000")

    def _return_steering_to_zero_after_calibration(self):
        self._send_ang_force(self.zero_angles)
        self.get_logger().info("标定完成：已下发舵轮回零 $ANG,0,0,0,0!")

    # ---------- 原地转延迟 ----------
    def tick(self):
        if self._calibration_active:
            self._set_drive_speeds(self.zero_speeds)
            return
        # Re-evaluate here as well: an approach yield that stops being
        # refreshed must re-engage the lock and stop the wheels on its own,
        # without waiting for another /grasp/active or /search/active edge.
        self.update_manipulation_lock("底盘接近让行超时")
        if self.manipulation_locked():
            self._set_drive_speeds(self.zero_speeds)
            return
        if self.mode == "spin" and not self._turn_wheels_started:
            # Keep the historical minimum hold on top of the steer-arrival
            # gate: the spin pose is a 45 deg move, so give it at least
            # spin_delay before the wheels are driven against each other.
            if time.monotonic() - self._turn_steer_sent_at < self.spin_delay:
                self._set_drive_speeds(self.zero_speeds)
            else:
                self._advance_steer_settle()
        elif self.mode in ("turn", "straight"):
            # The drive stays at zero until the steered wheels have reached the
            # commanded angle.
            self._advance_steer_settle()

    def shutdown(self):
        try:
            self.h7.unregister_frame_callback(self._on_h7_frame)
            self.h7.set_enabled(False)
        finally:
            try:
                self.h7.stop()
            except Exception:  # noqa: BLE001
                pass


def main():
    rclpy.init()
    node = None
    try:
        node = MotionSolver()
        if node.enable_drive_on_start:
            node.h7.set_enabled(True)  # 使能 3508
            node.get_logger().info("已使能 3508，等待控制指令...")
        else:
            node.get_logger().info("未使能 3508，仅使用 H7 机械臂/反馈接口")
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.shutdown()
            except Exception:  # noqa: BLE001
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
