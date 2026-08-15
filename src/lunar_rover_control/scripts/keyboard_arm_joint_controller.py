#!/usr/bin/env python3
"""Keyboard arm joint controller for the 5-DOF rover manipulator.

This node extends the direct Linux /dev/input/event* keyboard backend used by
``lunar_rover_navigation/keyboard_rover_teleop.py``.

Controls
--------
Numpad 1-5 (or top-row 1-5): select arm_joint1 .. arm_joint5
Up / Down arrows:             move the selected joint in the positive/negative
                              direction while held
Left / Right arrows:          switch to the previous / next joint

J1-J4 are commanded through the ``joint_trajectory_controller`` topic
(``/arm_controller/joint_trajectory`` by default) and J5 is commanded through
the gripper ``JointGroupPositionController`` topic
(``/gripper_controller/commands`` by default).
"""

import glob
import math
import os
import re
import select
import struct
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


HELP = """\
Lunar rover arm joint keyboard controller

  Numpad 1-5 / top-row 1-5 : select arm_joint1 .. arm_joint5
  Up / Down                : positive / negative joint motion while held
  Left / Right             : previous / next joint
  Ctrl-C                   : quit

Direct input backend listens to physical keyboard press/release events.
Keep NumLock on when using the numpad keys.
"""

EV_KEY = 0x01

# Selected joint keys: top-row digits 1-5 and numpad 1-5 (NumLock on).
KEY_CODES = {
    2: "joint1",
    3: "joint2",
    4: "joint3",
    5: "joint4",
    6: "joint5",
    79: "joint1",  # KEY_KP1
    80: "joint2",  # KEY_KP2
    81: "joint3",  # KEY_KP3
    75: "joint4",  # KEY_KP4
    76: "joint5",  # KEY_KP5
    103: "up",     # KEY_UP
    108: "down",   # KEY_DOWN
    105: "left",   # KEY_LEFT
    106: "right",  # KEY_RIGHT
}

INPUT_EVENT = struct.Struct("llHHi")
JOINT_KEYS = {"joint1", "joint2", "joint3", "joint4", "joint5"}
ARROW_KEYS = {"up", "down", "left", "right"}


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
    """Return likely Linux keyboard event devices, most specific paths first."""
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
            raise DirectKeyboardError(
                "未找到 keyboard 类型的 /dev/input/event* 设备。"
            )

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
        """Yield (key, pressed) events for relevant keys."""
        if not self.devices:
            return
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


class KeyboardArmJointController(Node):
    def __init__(self):
        super().__init__("keyboard_arm_joint_controller")

        self.joints = list(
            self.declare_parameter(
                "joints",
                [
                    "arm_joint1",
                    "arm_joint2",
                    "arm_joint3",
                    "arm_joint4",
                    "arm_joint5",
                ],
            ).value
        )
        if len(self.joints) != 5:
            raise ValueError(f"joints must contain exactly 5 names, got {self.joints!r}")

        self.arm_command_topic = self.declare_parameter(
            "arm_command_topic", "/arm_controller/joint_trajectory"
        ).value
        self.gripper_command_topic = self.declare_parameter(
            "gripper_command_topic", "/gripper_controller/commands"
        ).value
        self.joint_state_topic = self.declare_parameter(
            "joint_state_topic", "/joint_states"
        ).value
        self.publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)
        if not math.isfinite(self.publish_rate) or self.publish_rate <= 0.0:
            self.get_logger().warn(
                f"invalid publish_rate={self.publish_rate!r}; using 20.0"
            )
            self.publish_rate = 20.0
        self.input_backend = self.declare_parameter("input_backend", "direct").value
        self.input_device = self.declare_parameter("input_device", "").value
        self.trajectory_duration = float(
            self.declare_parameter("trajectory_duration", 0.05).value
        )
        if not math.isfinite(self.trajectory_duration) or self.trajectory_duration <= 0.0:
            self.get_logger().warn(
                f"invalid trajectory_duration={self.trajectory_duration!r}; using 0.05"
            )
            self.trajectory_duration = 0.05

        # Joint limits and keyboard step rates. Limits are intentionally kept in
        # sync with lunar_rover.urdf.xacro and the configured controllers.
        self.joint_limits = [
            list(row)
            for row in self.declare_parameter(
                "joint_limits",
                [
                    [-math.pi / 2.0, 1.0],
                    [-1.5, 1.5],
                    [-2.0, 2.0],
                    [-1.5, 1.5],
                    [0.0, 0.045],
                ],
            ).value
        ]
        if len(self.joint_limits) != 5:
            raise ValueError(
                f"joint_limits must contain exactly 5 [lower, upper] rows, got "
                f"{self.joint_limits!r}"
            )
        for row in self.joint_limits:
            if len(row) != 2 or not row[0] <= row[1]:
                raise ValueError(f"invalid joint limit row {row!r}")

        # rad/s for revolute joints, m/s for the gripper prismatic joint.
        self.step_rates = [
            float(value)
            for value in self.declare_parameter(
                "step_rates", [0.30, 0.30, 0.30, 0.30, 0.010]
            ).value
        ]
        if len(self.step_rates) != 5:
            raise ValueError(
                f"step_rates must contain exactly 5 values, got {self.step_rates!r}"
            )

        self.selected = 0
        self.held_keys = set()
        self.target_positions = [0.0] * 5
        self.positions_initialized = False
        self.target_changed = False
        self.last_print = 0.0

        self.arm_publisher = self.create_publisher(
            JointTrajectory, self.arm_command_topic, 10
        )
        self.gripper_publisher = self.create_publisher(
            Float64MultiArray, self.gripper_command_topic, 10
        )
        self.joint_state_subscriber = self.create_subscription(
            JointState, self.joint_state_topic, self.on_joint_state, 10
        )

    def on_joint_state(self, message):
        if self.positions_initialized:
            return
        if not message.name:
            return
        found = 0
        for name, position in zip(message.name, message.position):
            if name in self.joints:
                index = self.joints.index(name)
                self.target_positions[index] = float(position)
                found += 1
        if found == 5:
            self.positions_initialized = True
            self.get_logger().info(
                "Arm joint positions initialized from "
                f"{self.joint_state_topic!r}: {self.status_line()}"
            )
            self.print_status(force=True)

    def set_key_state(self, key, pressed):
        key = str(key).lower()
        if key in JOINT_KEYS:
            if pressed:
                self.selected = {
                    "joint1": 0,
                    "joint2": 1,
                    "joint3": 2,
                    "joint4": 3,
                    "joint5": 4,
                }[key]
                self.get_logger().info(f"selected {self.joints[self.selected]}")
                self.print_status(force=True)
            return

        if key in ARROW_KEYS:
            if pressed:
                if key == "left":
                    self.selected = (self.selected - 1) % 5
                    self.get_logger().info(
                        f"selected {self.joints[self.selected]}"
                    )
                    self.print_status(force=True)
                elif key == "right":
                    self.selected = (self.selected + 1) % 5
                    self.get_logger().info(
                        f"selected {self.joints[self.selected]}"
                    )
                    self.print_status(force=True)
                elif key in ("up", "down"):
                    self.held_keys.add(key)
            else:
                self.held_keys.discard(key)

    def status_line(self):
        parts = []
        for index, name in enumerate(self.joints):
            marker = "*" if index == self.selected else " "
            parts.append(f"{marker}{name}={self.target_positions[index]:+.4f}")
        return " ".join(parts)

    def print_status(self, force=False):
        now = time.monotonic()
        if force or now - self.last_print > 1.0:
            sys.stdout.write(f"\r{self.status_line():70s}")
            sys.stdout.flush()
            self.last_print = now

    def update_targets(self, elapsed):
        """Advance held-key joint targets. Return True when a target changed."""
        if not self.positions_initialized:
            return False

        keys = self.held_keys
        if "up" not in keys and "down" not in keys:
            return False
        if "up" in keys and "down" in keys:
            return False

        direction = 1.0 if "up" in keys else -1.0
        step = direction * self.step_rates[self.selected] * elapsed
        if step == 0.0:
            return False

        lower, upper = self.joint_limits[self.selected]
        old_value = self.target_positions[self.selected]
        new_value = min(max(old_value + step, lower), upper)
        if math.isclose(new_value, old_value, abs_tol=1e-9):
            return False

        self.target_positions[self.selected] = new_value
        return True

    def arm_trajectory_msg(self):
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = self.joints[:4]
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in self.target_positions[:4]]
        point.time_from_start = Duration(seconds=self.trajectory_duration).to_msg()
        message.points = [point]
        return message

    def gripper_command_msg(self):
        message = Float64MultiArray()
        message.data = [float(self.target_positions[4])]
        return message

    def publish_targets(self):
        self.arm_publisher.publish(self.arm_trajectory_msg())
        self.gripper_publisher.publish(self.gripper_command_msg())
        self.target_changed = False
        self.print_status(force=True)


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

            if node.update_targets(elapsed):
                node.target_changed = True

            if node.target_changed and node.positions_initialized:
                node.publish_targets()

            rclpy.spin_once(node, timeout_sec=0.0)

            sleep_left = period - (time.monotonic() - start)
            if sleep_left > 0.0:
                time.sleep(sleep_left)
    finally:
        reader.close()


def main():
    rclpy.init()
    node = KeyboardArmJointController()

    backend = str(node.input_backend).strip().lower()
    if backend not in ("auto", "event", "direct"):
        node.get_logger().warn(
            f"unsupported input_backend={node.input_backend!r}; using direct"
        )
    elif backend == "auto":
        node.get_logger().info("input_backend=auto resolves to direct /dev/input")

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
        print("\narm keyboard controller stopped")


if __name__ == "__main__":
    main()
