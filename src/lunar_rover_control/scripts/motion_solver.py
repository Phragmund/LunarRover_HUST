#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运动解算节点：订阅手动控制消息(杆量+按键) -> 解算 -> 直接控制 H7。

本节点直接持有 H7 单串口，把统一的杆量消息（LX/LY/RX/RY + relax/rearm）解算成
$M3508:SPD 与 $ANG 下发。逻辑与之前的键盘直连模式一致，但杆量是连续的：

    LX：阿克曼转向半径（右正），|LX| 越大转向越急；±255 时最急轮的舵角恰好 60°。
    LY：油门（前正），±255 时为四轮速度 0.8；转向时取四轮平均速度。
    RX：原地转向，非零即覆盖其他输入；±255 时四轮平均 0.8。
    RY：预留。

轮序与下位机一致：0=右后 1=左后 2=右前 3=左前。$ANG 正 = 顺时针。
"""

import math
import time

import rclpy
from rclpy.node import Node
from lunar_rover_msgs.msg import ManualControl

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
        self.max_speed = float(self.declare_parameter("max_speed", 2.5).value)
        self.max_steer_deg = float(self.declare_parameter("max_steer_deg", 60.0).value)
        self.spin_pose_deg = float(self.declare_parameter("spin_pose_deg", 45.0).value)
        self.spin_delay = float(self.declare_parameter("spin_delay", 0.5).value)
        self.boost_factor = float(self.declare_parameter("boost_factor", 2.2).value)
        self.arm_joint_ids = [
            int(x) for x in self.declare_parameter("arm_joint_ids", [1, 2, 3, 4, 5]).value
        ]
        self.input_topic = self.declare_parameter("input_topic", "/manual_control").value

        self._precompute()

        # ---- H7 客户端（steer_auto_send=False -> ANG 单发）----
        self.h7 = self._create_h7()

        # ---- 状态 ----
        self.mode = "stop"
        self.turn_side = "left"
        self.spin_side = "left"
        self.spin_start_time = 0.0
        self.spin_wheels_started = False
        self._spin_drive_speeds = [0.0] * 4
        self._last_ang = None
        self._prev_relax = False
        self._prev_rearm = False
        self.boost_active = False

        self.subscription = self.create_subscription(
            ManualControl, self.input_topic, self.on_message, 1
        )
        self.create_timer(0.02, self.tick)

        self.get_logger().info(
            "运动解算节点已启动：max_speed %.2f、max_steer %.1f°、原地转 %.1f°(延迟 %.2f s)"
            % (self.max_speed, self.max_steer_deg, self.spin_pose_deg, self.spin_delay)
        )
        self.get_logger().info("订阅 %s" % self.input_topic)

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
        # 原地转向：固定姿态（前轮内八、后轮外八），左/右转共用
        a = self.spin_pose_deg
        self.spin_pose_angles = [+a, -a, -a, +a]
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

    # ---------- 消息入口 ----------
    def on_message(self, msg):
        if msg.relax and not self._prev_relax:
            self._relax_all()
        if msg.rearm and not self._prev_rearm:
            self._rearm_all()
        self._prev_relax = msg.relax
        self._prev_rearm = msg.rearm
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

    # ---------- 单发 ANG（变化超过阈值才发）----------
    def _send_ang_if_changed(self, angles):
        if self._last_ang is None or any(
            abs(a - b) > 0.5 for a, b in zip(angles, self._last_ang)
        ):
            self.h7.send_steering_once(angles)
            self._last_ang = list(angles)

    # ---------- 下发轮速（boost 时整体放大）----------
    def _set_drive_speeds(self, speeds):
        if self.boost_active:
            speeds = [s * self.boost_factor for s in speeds]
        self.h7.set_drive_speeds(speeds)

    # ---------- 各模式 ----------
    def _enter_straight(self, ly):
        speed = ly / 255.0 * self.max_speed
        self.mode = "straight"
        self._send_ang_if_changed(self.zero_angles)
        self._set_drive_speeds([speed, speed, speed, speed])

    def _enter_turn(self, lx, ly):
        side = "right" if lx > 0 else "left"
        # ===== 临时更改：LX 映射为左右轮速度差值比例（差速/滑移转向，不转舵）=====
        throttle = ly / 255.0 * self.max_speed
        factor = lx / 255.0                       # -1..1
        left_speed = throttle * (1.0 + factor)    # LX>0 右转：左轮快
        right_speed = throttle * (1.0 - factor)
        # 轮序 [右后, 左后, 右前, 左前]：右轮=右后/右前，左轮=左后/左前
        speeds = [right_speed, left_speed, right_speed, left_speed]
        self.mode = "turn"
        self.turn_side = side
        self._send_ang_if_changed(self.zero_angles)   # 差速转向不转舵，保持直行
        self._set_drive_speeds(speeds)
        # ===== 原始阿克曼解算。恢复时删除上方临时块、取消下方注释 =====
        # max_steer = abs(lx) / 255.0 * math.radians(self.max_steer_deg)
        # radius = self.front_axle_x / math.tan(max_steer) + self.front_track * 0.5
        # angles_l = []
        # dists = []
        # for x, y in self.positions:
        #     kin = math.atan2(x, radius - y)          # 左转运动学舵角（逆时针正）
        #     angles_l.append(-math.degrees(kin))       # $ANG（顺时针正）取反
        #     dists.append(math.hypot(x, radius - y))
        # mean_d = sum(dists) / 4.0
        # ratios_l = [d / mean_d for d in dists]
        # if side == "left":
        #     angles = angles_l
        #     ratios = ratios_l
        # else:
        #     angles = [-a for a in angles_l]
        #     ratios = [ratios_l[1], ratios_l[0], ratios_l[3], ratios_l[2]]
        # throttle = ly / 255.0 * self.max_speed
        # speeds = [r * throttle for r in ratios]
        # self.mode = "turn"
        # self.turn_side = side
        # self._send_ang_if_changed(angles)
        # self._set_drive_speeds(speeds)
        # ===== 临时更改结束 =====

    def _enter_spin(self, rx):
        side = "right" if rx > 0 else "left"
        # ===== 临时更改：原地转向四轮同速（统一 2.0），不使用距离比例解算值 =====
        speed = abs(rx) / 255.0 * 2.0
        if side == "left":
            speeds = [speed, -speed, speed, -speed]    # 左自转：右进左退（四轮同速）
        else:
            speeds = [-speed, speed, -speed, speed]    # 右自转：右退左进（四轮同速）
        # ===== 原始解算值（距离比例，前快后慢）。恢复时删除上方同速块、取消下方注释 =====
        # mean_speed = abs(rx) / 255.0 * self.max_speed
        # rr = self.rear_ratio * mean_speed
        # rf = self.front_ratio * mean_speed
        # if side == "left":
        #     speeds = [rr, -rr, rf, -rf]    # 左自转：右进左退
        # else:
        #     speeds = [-rr, rr, -rf, rf]    # 右自转：右退左进
        # ===== 临时更改结束 =====

        self._send_ang_if_changed(self.spin_pose_angles)
        if self.mode != "spin" or side != self.spin_side:
            # 进入/切换原地转：先停轮，延迟后再启动
            self._set_drive_speeds(self.zero_speeds)
            self.mode = "spin"
            self.spin_side = side
            self.spin_start_time = time.monotonic()
            self.spin_wheels_started = False
        self._spin_drive_speeds = speeds
        if self.spin_wheels_started:
            self._set_drive_speeds(speeds)

    def _enter_stop(self):
        self.mode = "stop"
        self._send_ang_if_changed(self.zero_angles)
        self._set_drive_speeds(self.zero_speeds)
        self.spin_wheels_started = False

    # ---------- R / T 单次动作 ----------
    def _relax_all(self):
        self.h7.send_raw("$M3508:STOP!")
        self._set_drive_speeds(self.zero_speeds)
        self.h7.send_raw("$STOP!")
        for joint_id in self.arm_joint_ids:
            self.h7.send_raw("$ARM:TORQUE,%d,0!" % joint_id)
        self.mode = "stop"
        self.spin_wheels_started = False
        self.get_logger().info("R：已断开所有转矩输出")

    def _rearm_all(self):
        self.h7.send_raw("$ZERO!")
        self.h7.send_raw("$ANG,0,0,0,0!")
        for joint_id in self.arm_joint_ids:
            self.h7.send_raw("$ARM:TORQUE,%d,1!" % joint_id)
        self.get_logger().info("T：已重启位置保持并设零")

    # ---------- 原地转延迟 ----------
    def tick(self):
        if self.mode == "spin" and not self.spin_wheels_started:
            if time.monotonic() - self.spin_start_time >= self.spin_delay:
                self.spin_wheels_started = True
                self._set_drive_speeds(self._spin_drive_speeds)

    def shutdown(self):
        try:
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
        node.h7.set_enabled(True)  # 使能 3508
        node.get_logger().info("已使能 3508，等待控制指令...")
        rclpy.spin(node)
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
