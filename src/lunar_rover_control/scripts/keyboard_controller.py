#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘控制器节点：读键盘 -> 映射成杆量/按键 -> 发布 ManualControl 消息。

本节点不再直接控制 H7，只负责把键盘输入翻译成统一的手动控制消息，由
motion_solver 节点订阅后统一解算下发。

键位映射（杆量直接取最大 ±255）：
    W -> LY = +255（前进）      S -> LY = -255（后退）
    A -> LX = -255（左转）      D -> LX = +255（右转）
    Q -> RX = -255（左自转）    E -> RX = +255（右自转）
    R -> relax（断开转矩）      T -> rearm（重启+设零）
    Space -> 清空所有杆量（停止+回正）
"""

import glob
import os
import re
import select
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from lunar_rover_msgs.msg import ManualControl


EV_KEY = 0x01
KEY_CODES = {
    16: "q",
    17: "w",
    18: "e",
    19: "r",
    20: "t",
    30: "a",
    31: "s",
    32: "d",
    42: "left_shift",
    57: " ",
}
INPUT_EVENT = struct.Struct("llHHi")
DRIVE_KEYS = {"w", "s", "a", "d", "q", "e"}
NEWLINE = chr(10)


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
            blocks = handle.read().split(NEWLINE * 2)
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
        for event_name in re.findall(r"event[0-9]+", handlers):
            paths.append("/dev/input/" + event_name)
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
                errors.append("%s: 权限不足(%s)" % (path, exc.errno))
                continue
            except OSError as exc:
                errors.append("%s: %s" % (path, exc))
                continue
            self.devices.append((fd, path))
        if not self.devices:
            details = "  ".join(errors[:12])
            raise DirectKeyboardError(
                "无法读取键盘事件设备。需要 root 权限，或把当前用户加入 input 组后重新登录。"
                "候选设备: " + details
            )

    @property
    def fds(self):
        return [fd for fd, _ in self.devices]

    @property
    def paths(self):
        return [path for _, path in self.devices]

    def read_key_events(self):
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


class KeyboardController(Node):
    def __init__(self):
        super().__init__("keyboard_controller")
        self.publish_topic = self.declare_parameter("publish_topic", "/manual_control").value
        self.publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)
        self.input_device = self.declare_parameter("input_device", "").value

        self.reader = LinuxInputKeyboardReader(self.input_device)
        self.held = set()
        self.publisher = self.create_publisher(ManualControl, self.publish_topic, 1)

        self.get_logger().info("键盘控制器已启动，发布到 %s" % self.publish_topic)
        self.get_logger().info("设备: %s" % ", ".join(self.reader.paths))

    def _sticks_from_held(self):
        ly = 255 if "w" in self.held else (-255 if "s" in self.held else 0)
        lx = -255 if "a" in self.held else (255 if "d" in self.held else 0)
        rx = -255 if "q" in self.held else (255 if "e" in self.held else 0)
        return lx, ly, rx

    def on_key(self, key, pressed):
        if key == " ":
            if pressed:
                self.held.difference_update(DRIVE_KEYS)
        elif key in DRIVE_KEYS or key in ("r", "t", "left_shift"):
            if pressed:
                self.held.add(key)
            else:
                self.held.discard(key)
        else:
            return

    def publish_state(self):
        lx, ly, rx = self._sticks_from_held()
        msg = ManualControl()
        msg.lx = lx
        msg.ly = ly
        msg.rx = rx
        msg.ry = 0
        msg.relax = "r" in self.held
        msg.rearm = "t" in self.held
        msg.boost = "left_shift" in self.held
        self.publisher.publish(msg)

    def run(self):
        period = 1.0 / self.publish_rate
        last_publish = 0.0
        while rclpy.ok():
            changed = False
            for key, pressed in self.reader.read_key_events():
                self.on_key(key, pressed)
                changed = True
            now = time.monotonic()
            if changed or now - last_publish >= period:
                self.publish_state()
                last_publish = now
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.005)


def main():
    rclpy.init()
    node = None
    try:
        node = KeyboardController()
        node.run()
    except KeyboardInterrupt:
        pass
    except DirectKeyboardError as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(str(exc), file=sys.stderr)
        raise SystemExit(2) from exc
    finally:
        if node is not None:
            node.reader.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
