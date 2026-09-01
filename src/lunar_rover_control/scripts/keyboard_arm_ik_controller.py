#!/usr/bin/env python3
"""Keyboard Cartesian IK controller for the real rover arm.

This ports the IK jog workflow from the H7 unified web console into ROS:

  W/S:        move end effector forward/backward
  A/D:        move end effector left/right
  Space/Shift move end effector up/down
  Up/Down:   pitch wrist up/down
  Right/Left open/close gripper
  R:          return to IK home

The node commands ROS arm controllers only. It does not write to the H7 serial
port directly; the existing motion_solver remains responsible for translating
trajectory goals into lower-controller frames.
"""

from __future__ import annotations

import glob
import math
import os
import re
import select
import struct
import sys
import time
from dataclasses import dataclass

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


HELP = """\
Lunar rover arm IK keyboard controller

  W / S       : move target forward / backward
  A / D       : move target left / right
  Space/Shift : move target up / down
  Up / Down   : increase / decrease end-effector pitch
  Right/Left  : open / close gripper
  R           : return to IK home
  F           : synchronize IK target from current joint feedback
  Ctrl-C      : quit

The target pose is expressed in the arm-base frame:
  +X forward, +Y left, +Z up.
"""

EV_KEY = 0x01
INPUT_EVENT = struct.Struct("llHHi")

KEY_CODES = {
    17: "w",
    31: "s",
    30: "a",
    32: "d",
    57: "space",
    42: "shift",
    54: "shift",
    103: "up",
    108: "down",
    106: "right",
    105: "left",
    19: "r",
    33: "f",
}

MOTION_KEYS = {
    "w",
    "s",
    "a",
    "d",
    "space",
    "shift",
    "up",
    "down",
    "right",
    "left",
}


class DirectKeyboardError(RuntimeError):
    pass


def _dedupe_paths(paths):
    seen = set()
    result = []
    for path in paths:
        if not path:
            continue
        real = os.path.realpath(path)
        if real in seen or not os.path.exists(path):
            continue
        seen.add(real)
        result.append(path)
    return result


def discover_keyboard_devices():
    paths = []
    paths.extend(sorted(glob.glob("/dev/input/by-id/*event-kbd")))
    paths.extend(sorted(glob.glob("/dev/input/by-path/*event-kbd")))

    try:
        with open("/proc/bus/input/devices", "r", encoding="utf-8") as handle:
            blocks = handle.read().split("\n\n")
    except OSError:
        blocks = []

    for block in blocks:
        name_match = re.search(r'N: Name="([^"]+)"', block)
        handlers_match = re.search(r"H: Handlers=(.*)", block)
        if handlers_match is None:
            continue
        name = name_match.group(1) if name_match else ""
        handlers = handlers_match.group(1)
        if "kbd" not in handlers:
            continue
        lowered = name.lower()
        if any(skip in lowered for skip in ("power button", "sleep button", "video bus")):
            continue
        for event_name in re.findall(r"\bevent\d+\b", handlers):
            paths.append(f"/dev/input/{event_name}")

    return _dedupe_paths(paths)


class LinuxInputKeyboardReader:
    def __init__(self, explicit_devices=""):
        if explicit_devices:
            paths = [
                item.strip()
                for item in explicit_devices.replace(";", ",").split(",")
                if item.strip()
            ]
        else:
            paths = discover_keyboard_devices()

        if not paths:
            raise DirectKeyboardError("未找到 keyboard 类型的 /dev/input/event* 设备。")

        self.devices = []
        errors = []
        for path in paths:
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except PermissionError as exc:
                errors.append(f"{path}: 权限不足({exc.errno})")
                continue
            except OSError as exc:
                errors.append(f"{path}: {exc}")
                continue
            self.devices.append((fd, path))

        if not self.devices:
            details = "\n  ".join(errors[:12])
            raise DirectKeyboardError(
                "无法读取键盘事件设备。需要 root 权限，或把当前用户加入 input 组后重新登录。\n"
                f"候选设备:\n  {details}"
            )

    @property
    def fds(self):
        return [fd for fd, _ in self.devices]

    @property
    def paths(self):
        return [path for _, path in self.devices]

    def read_key_events(self):
        readable, _, _ = select.select(self.fds, [], [], 0.0)
        for fd in readable:
            while True:
                try:
                    data = os.read(fd, INPUT_EVENT.size * 32)
                except BlockingIOError:
                    break
                except OSError:
                    break
                if not data:
                    break
                usable = len(data) - (len(data) % INPUT_EVENT.size)
                for offset in range(0, usable, INPUT_EVENT.size):
                    _, _, event_type, code, value = INPUT_EVENT.unpack_from(data, offset)
                    if event_type != EV_KEY or code not in KEY_CODES:
                        continue
                    if value == 1:
                        yield KEY_CODES[code], True
                    elif value == 0:
                        yield KEY_CODES[code], False

    def close(self):
        for fd, _ in self.devices:
            try:
                os.close(fd)
            except OSError:
                pass
        self.devices.clear()


@dataclass
class IkTarget:
    x: float
    y: float
    z: float
    pitch_deg: float
    grip_deg: float


@dataclass
class IkSolution:
    angles_deg: list[float]
    clamped: bool
    branch_sign: float | None = None


def normalize_deg(angle):
    value = math.fmod(angle, 360.0)
    if value < 0.0:
        value += 360.0
    return value


def clamp(value, lower, upper):
    return min(max(value, lower), upper)


class KeyboardArmIkController(Node):
    def __init__(self):
        super().__init__("keyboard_arm_ik_controller")

        self.joints = list(
            self.declare_parameter(
                "joints",
                ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"],
            ).value
        )
        self.joint_state_topic = self.declare_parameter("joint_state_topic", "/joint_states").value
        self.command_mode = str(self.declare_parameter("command_mode", "topic").value).lower()
        self.arm_topic = self.declare_parameter(
            "arm_command_topic", "/arm_controller/joint_trajectory"
        ).value
        self.gripper_topic = self.declare_parameter(
            "gripper_command_topic", "/gripper_controller/commands"
        ).value
        self.arm_action = self.declare_parameter(
            "arm_action_name", "/arm_controller/follow_joint_trajectory"
        ).value
        self.gripper_action = self.declare_parameter(
            "gripper_action_name", "/gripper_controller/follow_joint_trajectory"
        ).value
        self.input_device = self.declare_parameter("input_device", "").value
        self.publish_rate = float(self.declare_parameter("publish_rate", 50.0).value)
        self.command_rate = float(self.declare_parameter("command_rate", 20.0).value)
        self.trajectory_duration = float(
            self.declare_parameter("trajectory_duration", 0.05).value
        )

        # Corrected measured geometry, in metres.
        self.base_to_shoulder = float(
            self.declare_parameter("base_to_shoulder_m", 0.07545).value
        )
        self.upper_arm = float(self.declare_parameter("upper_arm_m", 0.11391).value)
        self.forearm = float(self.declare_parameter("forearm_m", 0.11000).value)
        self.tool = float(self.declare_parameter("tool_m", 0.15601).value)

        self.linear_speed = float(self.declare_parameter("linear_speed_mps", 0.035).value)
        self.pitch_speed = float(self.declare_parameter("pitch_speed_degps", 18.0).value)
        self.grip_speed = float(self.declare_parameter("grip_speed_degps", 30.0).value)

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
        self.home_degrees = [
            float(x)
            for x in self.declare_parameter(
                "home_degrees",
                [96.754752, 201.804458, 183.510522, 110.293936, 45.0],
            ).value
        ]
        self.gripper_stroke_m = float(
            self.declare_parameter("gripper_stroke_m", 0.045).value
        )
        self.compensate_j2 = bool(self.declare_parameter("compensate_j2", True).value)

        self.min_xyz = tuple(
            float(x)
            for x in self.declare_parameter("min_xyz", [-0.32, -0.32, -0.18]).value
        )
        self.max_xyz = tuple(
            float(x)
            for x in self.declare_parameter("max_xyz", [0.42, 0.32, 0.42]).value
        )

        self.held_keys = set()
        self.current_ros = [0.0] * 5
        self.current_ready = False
        self.target = self.fk_from_degrees(self.home_degrees)
        self.last_command_time = 0.0
        self.last_print_time = 0.0
        self.last_action_warn_time = 0.0
        self.last_solution = None
        self.last_sent_gripper = None
        self.last_branch_sign = None

        self.joint_state_subscriber = self.create_subscription(
            JointState, self.joint_state_topic, self.on_joint_state, 10
        )
        self.arm_publisher = self.create_publisher(JointTrajectory, self.arm_topic, 10)
        self.gripper_publisher = self.create_publisher(
            Float64MultiArray, self.gripper_topic, 10
        )
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, self.arm_action
        )
        self.gripper_action_client = ActionClient(
            self, FollowJointTrajectory, self.gripper_action
        )

        if self.command_mode not in ("action", "topic"):
            self.get_logger().warn(
                f"unsupported command_mode={self.command_mode!r}; using action"
            )
            self.command_mode = "action"

    def on_joint_state(self, message):
        found = 0
        for name, position in zip(message.name, message.position):
            if name in self.joints:
                self.current_ros[self.joints.index(name)] = float(position)
                found += 1
        if found < 4:
            return
        if not self.current_ready:
            self.current_ready = True
            self.sync_target_from_current()
            self.get_logger().info(
                "IK target initialized from /joint_states: "
                f"x={self.target.x:.3f} y={self.target.y:.3f} "
                f"z={self.target.z:.3f} pitch={self.target.pitch_deg:.1f} "
                f"grip={self.target.grip_deg:.1f}"
            )

    def sync_target_from_current(self):
        self.target = self.fk_from_degrees(self.ros_to_degrees(self.current_ros))

    def ros_to_degrees(self, positions):
        result = []
        for index, position in enumerate(positions):
            if index == 4:
                degrees = position * 90.0 / max(self.gripper_stroke_m, 1e-9)
            else:
                degrees = self.arm_zero_degrees[index] + (
                    self.arm_ros_directions[index] * math.degrees(position)
                )
            result.append(clamp(degrees, 0.0, 270.0 if index < 4 else 90.0))
        return result

    def degrees_to_ros(self, angles_deg):
        result = []
        for index, degrees in enumerate(angles_deg):
            if index == 4:
                result.append(clamp(degrees, 0.0, 90.0) * self.gripper_stroke_m / 90.0)
            else:
                result.append(
                    math.radians(
                        (degrees - self.arm_zero_degrees[index])
                        / self.arm_ros_directions[index]
                    )
                )
        return result

    def fk_from_degrees(self, angles_deg):
        j1, j2, j3, j4, j5 = angles_deg
        t2 = math.radians(j2)
        t3 = t2 + math.radians(135.0 - j3)
        t4 = t3 + math.radians(135.0 - j4)

        radius = self.base_to_shoulder
        z = 0.0
        radius += self.upper_arm * math.sin(t2)
        z -= self.upper_arm * math.cos(t2)
        radius += self.forearm * math.sin(t3)
        z -= self.forearm * math.cos(t3)
        radius += self.tool * math.sin(t4)
        z -= self.tool * math.cos(t4)

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
        planar_r = max(0.0, radius - self.base_to_shoulder)
        wrist_r = planar_r - self.tool * math.sin(pitch_rad)
        wrist_z = target.z + self.tool * math.cos(pitch_rad)

        min_reach = abs(self.upper_arm - self.forearm)
        max_reach = self.upper_arm + self.forearm
        wrist_distance = math.hypot(wrist_r, wrist_z)
        clamped = False
        tolerance = 1e-7
        if wrist_distance > max_reach + tolerance:
            scale = max_reach / max(wrist_distance, 1e-9)
            wrist_r *= scale
            wrist_z *= scale
            clamped = True
        elif wrist_distance < min_reach - tolerance:
            scale = min_reach / max(wrist_distance, 1e-9)
            wrist_r *= scale
            wrist_z *= scale
            clamped = True

        candidates = []
        for sign in (-1.0, 1.0):
            solution = self.solve_arm_branch(wrist_r, wrist_z, pitch, sign)
            if solution is not None:
                candidates.append((sign, [j1, *solution, target.grip_deg]))

        chosen = None
        chosen_sign = None
        legal_candidates = [
            (sign, candidate)
            for sign, candidate in candidates
            if all(-1e-6 <= angle <= 270.0 + 1e-6 for angle in candidate[:4])
        ]
        for sign, candidate in legal_candidates:
            if self.last_branch_sign is not None and sign == self.last_branch_sign:
                chosen = candidate
                chosen_sign = sign
                break
        if chosen is None and legal_candidates:
            chosen_sign, chosen = legal_candidates[0]
        if chosen is None:
            if candidates:
                chosen_sign, first_candidate = candidates[0]
                chosen = [
                    clamp(first_candidate[0], 0.0, 270.0),
                    clamp(first_candidate[1], 0.0, 270.0),
                    clamp(first_candidate[2], 0.0, 270.0),
                    clamp(first_candidate[3], 0.0, 270.0),
                    target.grip_deg,
                ]
            else:
                chosen = [90.0, 180.0, 135.0, 135.0, target.grip_deg]
            clamped = True

        if self.compensate_j2:
            chosen[1] = self.compensate_j2_angle(chosen[1])
        chosen[4] = clamp(chosen[4], 0.0, 90.0)
        return IkSolution(chosen, clamped, chosen_sign)

    def solve_arm_branch(self, wrist_r, wrist_z, pitch, sign):
        distance = math.hypot(wrist_r, wrist_z)
        min_reach = abs(self.upper_arm - self.forearm)
        max_reach = self.upper_arm + self.forearm
        if distance < min_reach - 1e-9 or distance > max_reach + 1e-9:
            return None
        cos_delta = clamp(
            (distance * distance - self.upper_arm * self.upper_arm - self.forearm * self.forearm)
            / (2.0 * self.upper_arm * self.forearm),
            -1.0,
            1.0,
        )
        delta = sign * math.acos(cos_delta)
        gamma = math.atan2(wrist_z, wrist_r)
        theta2 = gamma + math.atan2(
            self.upper_arm + self.forearm * math.cos(delta),
            self.forearm * math.sin(delta),
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

    def set_key_state(self, key, pressed):
        if key == "r" and pressed:
            self.target = self.fk_from_degrees(self.home_degrees)
            self.publish_solution(force=True)
            return
        if key == "f" and pressed:
            if not self.current_ready:
                self.get_logger().warning("尚未收到完整 /joint_states，无法同步 IK")
                return
            self.held_keys.clear()
            self.sync_target_from_current()
            self.get_logger().info(
                "已按 F 同步 IK 目标：x=%.3f y=%.3f z=%.3f pitch=%.1f grip=%.1f"
                % (
                    self.target.x,
                    self.target.y,
                    self.target.z,
                    self.target.pitch_deg,
                    self.target.grip_deg,
                )
            )
            self.print_status(force=True)
            return
        if key in MOTION_KEYS:
            if pressed:
                self.held_keys.add(key)
            else:
                self.held_keys.discard(key)

    def update_target(self, elapsed):
        changed = False
        step = self.linear_speed * elapsed
        if "w" in self.held_keys and "s" not in self.held_keys:
            self.target.x += step
            changed = True
        elif "s" in self.held_keys and "w" not in self.held_keys:
            self.target.x -= step
            changed = True

        if "a" in self.held_keys and "d" not in self.held_keys:
            self.target.y += step
            changed = True
        elif "d" in self.held_keys and "a" not in self.held_keys:
            self.target.y -= step
            changed = True

        if "space" in self.held_keys and "shift" not in self.held_keys:
            self.target.z += step
            changed = True
        elif "shift" in self.held_keys and "space" not in self.held_keys:
            self.target.z -= step
            changed = True

        pitch_step = self.pitch_speed * elapsed
        if "up" in self.held_keys and "down" not in self.held_keys:
            self.target.pitch_deg = normalize_deg(self.target.pitch_deg + pitch_step)
            changed = True
        elif "down" in self.held_keys and "up" not in self.held_keys:
            self.target.pitch_deg = normalize_deg(self.target.pitch_deg - pitch_step)
            changed = True

        grip_step = self.grip_speed * elapsed
        if "right" in self.held_keys and "left" not in self.held_keys:
            self.target.grip_deg = clamp(self.target.grip_deg + grip_step, 0.0, 90.0)
            changed = True
        elif "left" in self.held_keys and "right" not in self.held_keys:
            self.target.grip_deg = clamp(self.target.grip_deg - grip_step, 0.0, 90.0)
            changed = True

        self.target.x = clamp(self.target.x, self.min_xyz[0], self.max_xyz[0])
        self.target.y = clamp(self.target.y, self.min_xyz[1], self.max_xyz[1])
        self.target.z = clamp(self.target.z, self.min_xyz[2], self.max_xyz[2])
        return changed

    def trajectory_msg(self, joint_names, positions):
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in positions]
        point.time_from_start = Duration(seconds=self.trajectory_duration).to_msg()
        message.points = [point]
        return message

    def send_action_goal(self, client, trajectory):
        if not client.server_is_ready():
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        client.send_goal_async(goal)
        return True

    def publish_solution(self, force=False):
        now = time.monotonic()
        min_interval = 1.0 / max(self.command_rate, 0.1)
        if not force and now - self.last_command_time < min_interval:
            return

        solution = self.solve_ik(self.target)
        if solution.branch_sign is not None:
            self.last_branch_sign = solution.branch_sign
        if solution.clamped:
            self.target = self.fk_from_degrees(solution.angles_deg)
        positions = self.degrees_to_ros(solution.angles_deg)
        arm_msg = self.trajectory_msg(self.joints[:4], positions[:4])
        gripper_msg = self.trajectory_msg([self.joints[4]], [positions[4]])

        if self.command_mode == "topic":
            self.arm_publisher.publish(arm_msg)
            if (
                force
                or self.last_sent_gripper is None
                or not math.isclose(positions[4], self.last_sent_gripper, abs_tol=1e-5)
            ):
                gripper_command = Float64MultiArray()
                gripper_command.data = [positions[4]]
                self.gripper_publisher.publish(gripper_command)
                self.last_sent_gripper = positions[4]
        else:
            arm_ok = self.send_action_goal(self.arm_action_client, arm_msg)
            grip_ok = self.send_action_goal(self.gripper_action_client, gripper_msg)
            if not arm_ok or not grip_ok:
                if now - self.last_action_warn_time > 2.0:
                    self.get_logger().warn(
                        "arm action server is not ready; waiting for motion_solver/controllers"
                    )
                    self.last_action_warn_time = now
                return

        self.last_command_time = now
        self.last_solution = solution
        self.print_status(force=force)

    def print_status(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_print_time < 0.5:
            return
        suffix = ""
        if self.last_solution is not None:
            angles = self.last_solution.angles_deg
            suffix = (
                f" | J={angles[0]:5.1f},{angles[1]:5.1f},"
                f"{angles[2]:5.1f},{angles[3]:5.1f} G={angles[4]:4.1f}"
            )
            if self.last_solution.clamped:
                suffix += " CLAMPED"
        line = (
            f"x={self.target.x:+.3f} y={self.target.y:+.3f} "
            f"z={self.target.z:+.3f} pitch={self.target.pitch_deg:6.1f} "
            f"grip={self.target.grip_deg:4.1f}{suffix}"
        )
        sys.stdout.write(f"\r{line:120s}")
        sys.stdout.flush()
        self.last_print_time = now


def run_direct_backend(node):
    reader = LinuxInputKeyboardReader(node.input_device)
    period = 1.0 / max(node.publish_rate, 0.001)
    print(HELP)
    print("input backend: direct /dev/input")
    print("devices:")
    for path in reader.paths:
        print(f"  {path}")

    try:
        last_time = time.monotonic()
        while rclpy.ok():
            start = time.monotonic()
            elapsed = start - last_time
            last_time = start

            for key, pressed in reader.read_key_events():
                node.set_key_state(key, pressed)

            if node.update_target(elapsed):
                node.publish_solution()
            else:
                node.print_status()

            rclpy.spin_once(node, timeout_sec=0.0)

            sleep_left = period - (time.monotonic() - start)
            if sleep_left > 0.0:
                time.sleep(sleep_left)
    finally:
        reader.close()


def main():
    rclpy.init()
    node = KeyboardArmIkController()
    try:
        run_direct_backend(node)
    except KeyboardInterrupt:
        pass
    except DirectKeyboardError as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(2) from exc
    finally:
        node.held_keys.clear()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\narm IK keyboard controller stopped")


if __name__ == "__main__":
    main()
