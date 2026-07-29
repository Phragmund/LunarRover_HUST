#!/usr/bin/env python3
"""Keyboard teleop for the rover chassis TwistStamped interface.

The default backend reads from the focused terminal.  A Linux /dev/input/event*
backend is kept as an optional diagnostic mode, but it needs elevated input
device permissions on this machine and is therefore not used by default.
"""

import glob
import os
import re
import select
import struct
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


HELP = """\
Lunar rover keyboard teleop

  W/S : forward / reverse
  A/D : in-place left / right rotation
  W/S + A/D : Ackermann-style turning while driving
  SPACE : stop
  Q or Ctrl-C : quit in terminal backend
  Ctrl-C : quit in direct backend by default

Direct input backend listens to physical keyboard press/release events.
Terminal backend requires this terminal to stay focused.
"""

EV_KEY = 0x01
KEY_CODES = {
    16: "q",
    17: "w",
    30: "a",
    31: "s",
    32: "d",
    57: " ",
}
INPUT_EVENT = struct.Struct("llHHi")


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


class KeyboardRoverTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_rover_teleop")
        self.command_topic = self.declare_parameter(
            "command_topic", "/chassis/cmd_vel"
        ).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.linear_speed = float(self.declare_parameter("linear_speed", 0.16).value)
        self.reverse_speed = float(self.declare_parameter("reverse_speed", 0.10).value)
        self.spin_angular_speed = float(
            self.declare_parameter("spin_angular_speed", 0.36).value
        )
        self.publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)
        self.key_timeout = float(self.declare_parameter("key_timeout", 0.45).value)
        self.input_backend = self.declare_parameter("input_backend", "terminal").value
        self.input_device = self.declare_parameter("input_device", "").value
        self.direct_quit_key_enabled = bool(
            self.declare_parameter("direct_quit_key_enabled", False).value
        )

        self.publisher = self.create_publisher(TwistStamped, self.command_topic, 1)
        self.last_seen = {}
        self.held_keys = set()
        self.use_direct_keys = False
        self.last_mode = ""
        self.last_print = 0.0

    def set_direct_mode(self, enabled):
        self.use_direct_keys = enabled
        if enabled:
            self.last_seen.clear()
        else:
            self.held_keys.clear()

    def note_key(self, key):
        key = key.lower()
        now = time.monotonic()
        if key in ("w", "a", "s", "d"):
            self.last_seen[key] = now
        elif key == " ":
            self.last_seen.clear()

    def set_key_state(self, key, pressed):
        key = key.lower()
        if key in ("w", "a", "s", "d"):
            if pressed:
                self.held_keys.add(key)
            else:
                self.held_keys.discard(key)
        elif key == " " and pressed:
            self.held_keys.clear()

    def active_keys(self):
        if self.use_direct_keys:
            return set(self.held_keys)

        now = time.monotonic()
        return {
            key
            for key, stamp in self.last_seen.items()
            if now - stamp <= self.key_timeout
        }

    def compute_command(self):
        keys = self.active_keys()
        forward = "w" in keys and "s" not in keys
        reverse = "s" in keys and "w" not in keys
        left = "a" in keys and "d" not in keys
        right = "d" in keys and "a" not in keys

        linear_x = 0.0
        angular_z = 0.0
        mode = "stop"

        if forward:
            linear_x = self.linear_speed
            mode = "forward"
        elif reverse:
            linear_x = -self.reverse_speed
            mode = "reverse"

        if left:
            angular_z = self.spin_angular_speed
            mode = f"{mode} left" if linear_x else "spin left"
        elif right:
            angular_z = -self.spin_angular_speed
            mode = f"{mode} right" if linear_x else "spin right"

        return linear_x, angular_z, mode

    def publish_command(self):
        linear_x, angular_z, mode = self.compute_command()
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.twist.linear.x = linear_x
        message.twist.angular.z = angular_z
        self.publisher.publish(message)

        now = time.monotonic()
        if mode != self.last_mode or now - self.last_print > 1.0:
            sys.stdout.write(
                f"\rmode={mode:16s} v={linear_x:+.3f} m/s "
                f"wz={angular_z:+.3f} rad/s     "
            )
            sys.stdout.flush()
            self.last_mode = mode
            self.last_print = now


def read_available_key():
    readable, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not readable:
        return None
    return sys.stdin.read(1)


def run_direct_backend(node):
    reader = LinuxInputKeyboardReader(node.input_device)
    node.set_direct_mode(True)
    period = 1.0 / node.publish_rate
    print(HELP)
    print("input backend: direct /dev/input")
    print("devices:")
    for path in reader.paths:
        print(f"  {path}")

    quit_requested = False
    try:
        while rclpy.ok() and not quit_requested:
            start = time.monotonic()
            for key, pressed in reader.read_key_events():
                if key == "q" and pressed and node.direct_quit_key_enabled:
                    quit_requested = True
                    break
                node.set_key_state(key, pressed)
                node.publish_command()

            node.publish_command()
            rclpy.spin_once(node, timeout_sec=0.0)
            elapsed = time.monotonic() - start
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        reader.close()


def run_terminal_backend(node):
    node.set_direct_mode(False)
    old_settings = termios.tcgetattr(sys.stdin)
    period = 1.0 / node.publish_rate

    print(HELP)
    print("input backend: terminal fallback")
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            start = time.monotonic()
            key = read_available_key()
            if key in ("q", "Q", "\x03"):
                break
            if key is not None:
                node.note_key(key)

            node.publish_command()
            rclpy.spin_once(node, timeout_sec=0.0)
            elapsed = time.monotonic() - start
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    node = KeyboardRoverTeleop()

    backend = str(node.input_backend).strip().lower()
    if backend not in ("auto", "event", "direct", "terminal"):
        node.get_logger().warn(
            f"unknown input_backend={node.input_backend!r}; falling back to auto"
        )
        backend = "auto"

    try:
        if backend in ("event", "direct", "auto"):
            try:
                run_direct_backend(node)
            except DirectKeyboardError as exc:
                if backend in ("event", "direct"):
                    raise
                node.get_logger().warn(str(exc))
                run_terminal_backend(node)
        else:
            run_terminal_backend(node)
    except KeyboardInterrupt:
        pass
    except DirectKeyboardError as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(2) from exc
    finally:
        node.last_seen.clear()
        node.held_keys.clear()
        if rclpy.ok():
            for _ in range(5):
                node.publish_command()
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\nteleop stopped")


if __name__ == "__main__":
    main()
