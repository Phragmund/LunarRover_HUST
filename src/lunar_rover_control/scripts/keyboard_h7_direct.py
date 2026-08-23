#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""H7 直连键盘控制节点（死命令模式）。

直接读键盘，按“死命令”方式给 H7 发 SPD（轮速）与 ANG（舵角），
不再走 TwistStamped -> 阿克曼逆解 -> mux -> driver 链路。

按键语义（优先级从高到低）：

  Q/E  原地转向（最高优先级，无视 WASD）
       - 按下先发一条固定转向姿态 ANG（前轮内八、后轮外八，单发）
       - 延迟 spin_start_delay(默认 0.5s) 后启动轮电机（<= spin_speed）
       - Q 向左自转：左轮后退、右轮前进；E 反之

  A/D  阿克曼转向（固定转弯半径 turn_radius，最大舵角 <= 30°）
       - 按下单发一条 ANG 位置指令（一次即可，不反复发）
       - 松开瞬间发一条 ANG 归零指令
       - 若同时按 W/S，则用阿克曼轮速结果覆盖四轮同速
       - 只用 ANG 位置模式，全程不发转向速度指令

  W/S  直行
       - W 四轮同速前进 SPD straight_speed(默认 0.5)
       - S 四轮同速后退 SPD -straight_speed

  Space / 无驱动键  停止（轮速 0 + 舵角归零）

  R    断开所有转矩输出（3508 停止、舵轮/机械臂取消位置保持）
  T    重启舵轮/机械臂位置保持，舵轮当前位置设零

注意：SPD 走 H7SerialClient 的周期发送（20Hz，满足下位机 500ms 超时）；
ANG 走 send_steering_once 单发（满足“一次即可”）。轮序与下位机一致：
0=右后 1=左后 2=右前 3=左前。
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
from rclpy.node import Node

from h7_serial_client import H7SerialClient, DummyH7Client


# ===== 键盘事件读取（直接读 /dev/input/event*）=====
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
    """返回可能的键盘事件设备路径，最具体的路径在前。"""
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
        """yield (key, pressed)。"""
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


class KeyboardH7Direct(Node):
    def __init__(self):
        super().__init__("keyboard_h7_direct")

        # ---- H7 串口 ----
        self.h7_backend = self.declare_parameter("h7_backend", "serial").value
        self.h7_port = self.declare_parameter("h7_port", "/dev/ttyACM0").value
        self.h7_baud = int(self.declare_parameter("h7_baud", 115200).value)

        # ---- 几何（用于预计算阿克曼/原地转死命令）----
        self.front_axle_x = float(self.declare_parameter("front_axle_x", 0.22).value)
        self.rear_axle_x = float(self.declare_parameter("rear_axle_x", -0.20).value)
        self.front_track = float(self.declare_parameter("front_track", 0.415).value)
        self.rear_track = float(self.declare_parameter("rear_track", 0.350).value)

        # ---- 死命令参数 ----
        self.straight_speed = float(self.declare_parameter("straight_speed", 0.5).value)
        self.turn_radius = float(self.declare_parameter("turn_radius", 0.6).value)
        self.turn_speed_max = float(self.declare_parameter("turn_speed_max", 0.5).value)
        self.spin_pose_deg = float(self.declare_parameter("spin_pose_deg", 45.0).value)
        self.spin_speed = float(self.declare_parameter("spin_speed", 0.5).value)
        self.spin_start_delay = float(self.declare_parameter("spin_start_delay", 0.5).value)
        self.input_device = self.declare_parameter("input_device", "").value
        self.arm_joint_ids = [
            int(x) for x in self.declare_parameter("arm_joint_ids", [1, 2, 3, 4, 5]).value
        ]

        self._precompute()

        # ---- H7 客户端（steer_auto_send=False -> ANG 单发）----
        self.h7 = self._create_h7()

        # ---- 键盘状态 ----
        self.reader = LinuxInputKeyboardReader(self.input_device)
        self.held = set()
        self.mode = "stop"
        self.turn_side = "left"
        self.spin_side = "left"
        self.spin_start_time = 0.0
        self.spin_wheels_started = False
        self._spin_drive_speeds = [0.0] * 4

        self.get_logger().info(
            "键盘直连控制已启动：直行 %.2f、转向半径 %.2f m、原地转 %.1f°(延迟 %.2f s)"
            % (self.straight_speed, self.turn_radius, self.spin_pose_deg, self.spin_start_delay)
        )
        self.get_logger().info("设备: %s" % ", ".join(self.reader.paths))

    def _precompute(self):
        # 轮位顺序：右后、左后、右前、左前（x 前正，y 左正）
        self.positions = [
            (self.rear_axle_x, -self.rear_track * 0.5),    # 右后
            (self.rear_axle_x, +self.rear_track * 0.5),    # 左后
            (self.front_axle_x, -self.front_track * 0.5),  # 右前
            (self.front_axle_x, +self.front_track * 0.5),  # 左前
        ]
        self.fwd_speeds = [self.straight_speed] * 4
        self.rev_speeds = [-self.straight_speed] * 4
        self.zero_speeds = [0.0] * 4
        self.zero_angles = [0.0] * 4
        (
            self.left_turn_angles, self.left_turn_speeds,
            self.right_turn_angles, self.right_turn_speeds,
        ) = self._compute_turn()
        a = self.spin_pose_deg
        v = self.spin_speed
        # 原地转向：左转/右转用同一套转向姿态（前轮内八、后轮外八），
        # 只有轮速方向相反（左转右进左退、右转右退左进）。
        self.spin_left_angles = [+a, -a, -a, +a]
        self.spin_right_angles = [+a, -a, -a, +a]
        self.spin_left_speeds = [+v, -v, +v, -v]
        self.spin_right_speeds = [-v, +v, -v, +v]

        self.get_logger().info(
            "左转舵角(右后/左后/右前/左前)=[%.1f, %.1f, %.1f, %.1f]°"
            % tuple(self.left_turn_angles)
        )
        self.get_logger().info(
            "左转轮速=[%.3f, %.3f, %.3f, %.3f] r/s"
            % tuple(self.left_turn_speeds)
        )

    def _compute_turn(self):
        R = self.turn_radius  # 左转 ICR 在 +y
        angles = []
        dists = []
        for x, y in self.positions:
            ang_kin = math.atan2(x, R - y)          # 运动学舵角（逆时针为正）
            angles.append(math.degrees(-ang_kin))   # $ANG（顺时针为正）取反
            dists.append(math.hypot(x, R - y))
        scale = self.turn_speed_max / max(dists)
        speeds = [d * scale for d in dists]
        left_angles = angles
        left_speeds = speeds
        right_angles = [-a for a in angles]
        right_speeds = [speeds[1], speeds[0], speeds[3], speeds[2]]
        return left_angles, left_speeds, right_angles, right_speeds

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

    # ---------- 键盘事件 ----------
    def on_key(self, key, pressed):
        if key == "r" and pressed:
            self._relax_all()
            return
        if key == "t" and pressed:
            self._rearm_all()
            return
        if key == " ":
            if pressed:
                self.held.difference_update(DRIVE_KEYS)
        elif key in DRIVE_KEYS:
            if pressed:
                self.held.add(key)
            else:
                self.held.discard(key)
        else:
            return
        self._recompute()

    def _recompute(self):
        # 优先级：Q/E 原地转 > A/D 转向 > W/S 直行 > 停止
        if "q" in self.held or "e" in self.held:
            self._enter_spin("left" if "q" in self.held else "right")
        elif "a" in self.held or "d" in self.held:
            self._enter_turn("left" if "a" in self.held else "right")
        elif "w" in self.held or "s" in self.held:
            self._enter_straight("w" in self.held)
        else:
            self._enter_stop()

    def _enter_straight(self, forward):
        if self.mode != "straight":
            self.mode = "straight"
            self.h7.send_steering_once(self.zero_angles)  # 回正舵
        self.h7.set_drive_speeds(self.fwd_speeds if forward else self.rev_speeds)

    def _enter_turn(self, side):
        angles = self.left_turn_angles if side == "left" else self.right_turn_angles
        speeds = self.left_turn_speeds if side == "left" else self.right_turn_speeds
        if self.mode != "turn" or side != self.turn_side:
            self.h7.send_steering_once(angles)  # 单发一次 ANG
            self.mode = "turn"
            self.turn_side = side
        if "w" in self.held:
            self.h7.set_drive_speeds(speeds)
        elif "s" in self.held:
            self.h7.set_drive_speeds([-v for v in speeds])
        else:
            self.h7.set_drive_speeds(self.zero_speeds)

    def _enter_spin(self, side):
        angles = self.spin_left_angles if side == "left" else self.spin_right_angles
        speeds = self.spin_left_speeds if side == "left" else self.spin_right_speeds
        if self.mode != "spin" or side != self.spin_side:
            self.h7.send_steering_once(angles)  # 单发姿态 ANG
            self.h7.set_drive_speeds(self.zero_speeds)  # 延迟期间轮子不动
            self.mode = "spin"
            self.spin_side = side
            self.spin_start_time = time.monotonic()
            self.spin_wheels_started = False
        self._spin_drive_speeds = speeds

    def _enter_stop(self):
        if self.mode == "spin" or self.mode == "turn":
            self.h7.send_steering_once(self.zero_angles)  # 回正舵
        self.mode = "stop"
        self.h7.set_drive_speeds(self.zero_speeds)
        self.spin_wheels_started = False

    def _relax_all(self):
        """R 键：断开所有转矩输出。"""
        self.h7.send_raw("$M3508:STOP!")            # 3508 停止
        self.h7.set_drive_speeds(self.zero_speeds)   # 驱动目标清零（周期 SPD 发 0）
        self.h7.send_raw("$STOP!")                   # 舵轮取消位置保持
        for joint_id in self.arm_joint_ids:
            self.h7.send_raw("$ARM:TORQUE,%d,0!" % joint_id)  # 机械臂扭矩关
        self.held.difference_update(DRIVE_KEYS)      # 清空驱动/转向键
        self.mode = "stop"
        self.spin_wheels_started = False
        self.get_logger().info("R：已断开所有转矩输出")

    def _rearm_all(self):
        """T 键：重启舵轮/机械臂位置保持，舵轮当前位置设零。"""
        self.h7.send_raw("$ZERO!")                   # 舵轮当前位置设零
        self.h7.send_raw("$ANG,0,0,0,0!")            # 舵轮重启位置保持（保持在新零点）
        for joint_id in self.arm_joint_ids:
            self.h7.send_raw("$ARM:TORQUE,%d,1!" % joint_id)  # 机械臂扭矩开
        self.get_logger().info("T：已重启位置保持并设零")

    def tick(self):
        # 原地转：延迟 spin_start_delay 后启动轮电机
        if self.mode == "spin" and not self.spin_wheels_started:
            if time.monotonic() - self.spin_start_time >= self.spin_start_delay:
                self.spin_wheels_started = True
                self.h7.set_drive_speeds(self._spin_drive_speeds)

    def shutdown(self):
        try:
            self.h7.set_enabled(False)
        finally:
            try:
                self.h7.stop()
            except Exception:  # noqa: BLE001
                pass
        self.reader.close()


def main():
    rclpy.init()
    node = None
    try:
        node = KeyboardH7Direct()
        node.h7.set_enabled(True)  # 使能 3508（舵轮无需使能）
        node.get_logger().info("已使能 3508，等待键盘输入...")
        while rclpy.ok():
            for key, pressed in node.reader.read_key_events():
                node.on_key(key, pressed)
            node.tick()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.005)
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
            try:
                node.shutdown()
            except Exception:  # noqa: BLE001
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
