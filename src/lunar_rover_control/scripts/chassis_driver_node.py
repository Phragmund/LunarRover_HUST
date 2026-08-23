#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真车底盘解算节点（ROS2，H7 单串口上位机）。

订阅 /chassis/cmd_vel（TwistStamped，车体前向速度 vx 与绕 z 轴角速度 wz），
按阿克曼几何逆解算出 4 个动力轮输出轴转速与 4 个舵轮角度，再通过 H7 的
单 USB CDC 串口统一下发：

    - 动力轮：$M3508:SPD,s0,s1,s2,s3!（输出轴 r/s，正 = 前进；方向由下位机处理）
    - 舵轮  ：$ANG,a0,a1,a2,a3!（deg，顺时针为正，限幅 ±60°）

轮序（与下位机一致）：0=右后 1=左后 2=右前 3=左前。
中间一对轮无动力、无舵，为从动轮（原地转向时会被拖动，属正常现象）。

车辆几何（实车标定值，2026-08-20）：
    - 前轮距 415 mm（半轮距 207.5 mm）
    - 中轮距 415 mm（从动，不参与角点轮逆解）
    - 后轮距 350 mm（半轮距 175 mm）
    - 前轮轴在中轮轴前方 220 mm（x = +0.22 m）
    - 后轮轴在中轮轴后方 200 mm（x = -0.20 m）
"""

import math
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from lunar_rover_msgs.msg import ChassisState
from std_msgs.msg import Bool, UInt32

# 允许直接 import 同目录的 H7 串口客户端
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from h7_serial_client import H7SerialClient, DummyH7Client  # noqa: E402


def normalize_angle(angle):
    """把角度规约到 [-pi, pi]。"""
    return math.remainder(angle, 2.0 * math.pi)


def minimal_steering(heading, speed):
    """把舵角限制在 ±90° 内，超过则反转轮速（减少舵机行程）。"""
    heading = normalize_angle(heading)
    if heading > math.pi * 0.5:
        heading -= math.pi
        speed = -speed
    elif heading < -math.pi * 0.5:
        heading += math.pi
        speed = -speed
    return heading, speed


def ackermann_inverse(linear_x, angular_z, geometry):
    """阿克曼逆解算，返回 4 个转向轮的 (steering_deg, wheel_speed_rps)。

    坐标系：x 前为正，y 左为正，z 上为正（ROS 车体系）。
    linear_x: 车体前进速度 (m/s)
    angular_z: 车体绕 z 轴角速度 (rad/s)，左转为正
    geometry: dict，键为
        wheel_radius      车轮半径 (m)
        front_axle_x      前轮轴到中轮轴距离 (m，正=前)
        rear_axle_x       后轮轴到中轮轴距离 (m，负=后)
        front_track       前轮距 (m)
        rear_track        后轮距 (m)
        max_steering_angle  最大舵角 (rad)

    返回顺序与下位机一致：右后、左后、右前、左前。
    每个元素为 (steering_deg, wheel_speed_rps)，wheel_speed_rps 为输出轴转速
    （r/s，正 = 该轮前进）。

    原理：对每个角点轮，其接地点瞬时速度 v = (vx - wz*y, wz*x)，舵角取该速度
    方向 atan2(vy, vx)，轮速取 |v| / wheel_radius。原地转向时 linear_x=0，
    各轮自动指向以车体中心为圆心的切线方向，即零半径原地转。
    """
    wheel_radius = geometry['wheel_radius']
    front_axle_x = geometry['front_axle_x']
    rear_axle_x = geometry['rear_axle_x']
    front_track = geometry['front_track']
    rear_track = geometry['rear_track']
    max_steering = geometry['max_steering_angle']

    # 轮位顺序：右后、左后、右前、左前（x 前正，y 左正）
    locations = [
        (rear_axle_x, -rear_track * 0.5),    # 右后
        (rear_axle_x, +rear_track * 0.5),    # 左后
        (front_axle_x, -front_track * 0.5),  # 右前
        (front_axle_x, +front_track * 0.5),  # 左前
    ]

    result = []
    saturated = False
    for x, y in locations:
        velocity_x = linear_x - angular_z * y
        velocity_y = angular_z * x

        steering = 0.0
        wheel_speed = velocity_x / wheel_radius

        contact_speed = math.hypot(velocity_x, velocity_y)
        if contact_speed > 1e-12:
            steering, wheel_speed = minimal_steering(
                math.atan2(velocity_y, velocity_x),
                contact_speed / wheel_radius,
            )

        limited = max(-max_steering, min(max_steering, steering))
        if limited != steering:
            # 舵角受限时，把轮速投影到可达舵角方向
            wheel_speed = (
                velocity_x * math.cos(limited) + velocity_y * math.sin(limited)
            ) / wheel_radius
            steering = limited
            saturated = True

        # 输出轴转速：rad/s -> r/s
        result.append((math.degrees(steering), wheel_speed / (2.0 * math.pi)))

    return result, saturated


class ChassisDriverNode(Node):
    FAULT_SERIAL = 1 << 28
    FAULT_DRIVE_FEEDBACK = 1 << 29
    FAULT_STEERING = 1 << 30

    def __init__(self):
        super().__init__('chassis_driver_node')

        # ---- H7 串口后端 ----
        self.h7_backend = self.declare_parameter('h7_backend', 'serial').value
        self.h7_port = self.declare_parameter('h7_port', '/dev/ttyACM0').value
        self.h7_baud = int(self.declare_parameter('h7_baud', 115200).value)

        # ---- 几何（实车标定值） ----
        self.wheel_radius = float(self.declare_parameter('wheel_radius', 0.125).value)
        self.front_axle_x = float(self.declare_parameter('front_axle_x', 0.22).value)
        self.rear_axle_x = float(self.declare_parameter('rear_axle_x', -0.20).value)
        self.front_track = float(self.declare_parameter('front_track', 0.415).value)
        self.rear_track = float(self.declare_parameter('rear_track', 0.350).value)
        self.max_steering_angle = float(
            self.declare_parameter('max_steering_angle', math.radians(60.0)).value
        )
        # 3508 速度非归一化：正常行进约 0.5 r/s，可超过 1；2.0 为软安全上限
        self.max_wheel_speed_rps = float(
            self.declare_parameter('max_wheel_speed_rps', 2.0).value
        )

        # ---- 舵角符号 ----
        # 下位机约定：$ANG 正 = 顺时针（向右）、负 = 逆时针（向左）。
        # 本节点运动学舵角以“逆时针（向左）为正”，与下位机相反，故乘 -1 取反。
        self.steering_sign = float(
            self.declare_parameter('steering_sign', -1.0).value
        )

        # ---- 命令 ----
        self.command_topic = self.declare_parameter(
            'command_topic', '/chassis/cmd_vel'
        ).value
        self.command_timeout = float(
            self.declare_parameter('command_timeout', 0.25).value
        )
        self.default_enabled = bool(
            self.declare_parameter('default_enabled', False).value
        )
        self.state_topic = self.declare_parameter(
            'state_topic', '/chassis/state'
        ).value
        self.estop_topic = self.declare_parameter(
            'estop_topic', '/chassis/emergency_stop'
        ).value
        self.enable_topic = self.declare_parameter(
            'enable_topic', '/chassis/controller_enabled'
        ).value
        self.fault_topic = self.declare_parameter(
            'fault_topic', '/chassis/fault_bits'
        ).value

        if not (math.isfinite(self.steering_sign) and abs(self.steering_sign) > 1e-9):
            raise ValueError('steering_sign 必须是非零有限数')
        positive_values = (
            self.wheel_radius,
            self.front_track,
            self.rear_track,
            self.max_steering_angle,
            self.max_wheel_speed_rps,
            self.command_timeout,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive_values):
            raise ValueError('底盘几何、限速和超时参数必须为有限正数')
        if not (math.isfinite(self.front_axle_x) and math.isfinite(self.rear_axle_x)):
            raise ValueError('前后轮轴位置必须为有限数')
        if self.front_axle_x <= self.rear_axle_x:
            raise ValueError('front_axle_x 必须大于 rear_axle_x（前轮轴在前）')
        if self.max_steering_angle > math.radians(60.0):
            # 硬件轮舵限幅 ±60°
            self.get_logger().warn(
                'max_steering_angle=%.2f° 超过轮舵硬件限幅 ±60°，将被限制在 ±60°'
                % math.degrees(self.max_steering_angle)
            )

        # ---- 初始化 H7 客户端 ----
        try:
            self.h7 = self._create_h7_client()
            self.get_logger().info(
                'H7 串口客户端已启动（%s，4 动力轮 + 4 舵轮）' % self.h7_backend
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('H7 串口客户端初始化失败: %s' % exc)
            raise

        # ---- 命令订阅与超时 ----
        self._last_command = None  # (vx, wz)
        self._last_command_time = None
        self.enabled = self.default_enabled
        self.estop = False
        self.external_fault_bits = 0
        self.subscription = self.create_subscription(
            TwistStamped, self.command_topic, self.on_command, 1
        )
        self.estop_subscription = self.create_subscription(
            Bool, self.estop_topic, self.on_estop, 1
        )
        self.enable_subscription = self.create_subscription(
            Bool, self.enable_topic, self.on_enable, 1
        )
        self.fault_subscription = self.create_subscription(
            UInt32, self.fault_topic, self.on_fault, 1
        )
        self.state_publisher = self.create_publisher(
            ChassisState, self.state_topic, 10
        )
        self.create_timer(0.05, self.on_timer)
        self.h7.set_enabled(self.enabled)
        self.get_logger().info(
            '底盘解算节点已启动，订阅 %s，初始%s'
            % (self.command_topic, '使能' if self.enabled else '禁用')
        )

    def _create_h7_client(self):
        if self.h7_backend == 'serial':
            return H7SerialClient(
                port=self.h7_port,
                baud=self.h7_baud,
                logger=self.get_logger(),
            )
        if self.h7_backend == 'dummy':
            return DummyH7Client(logger=self.get_logger())
        raise ValueError("h7_backend must be 'serial' or 'dummy'")

    def on_command(self, message):
        vx = message.twist.linear.x
        wz = message.twist.angular.z
        if not (math.isfinite(vx) and math.isfinite(wz)):
            self.get_logger().warn('丢弃非有限底盘命令')
            return
        self._last_command = (vx, wz)
        self._last_command_time = self.get_clock().now()

    def on_estop(self, message):
        self.estop = bool(message.data)
        if self.estop:
            self.enabled = False
            self.h7.set_enabled(False)
            self.h7.stop_drive()
            self.h7.stop_steering()
            self.get_logger().error('底盘急停已锁存；需先清除急停再重新使能')

    def on_enable(self, message):
        requested = bool(message.data)
        if requested and (self.estop or self._hardware_fault_bits() != 0):
            self.get_logger().error('拒绝使能：急停未清除或硬件链路未就绪')
            self.enabled = False
            self.h7.set_enabled(False)
            return
        self.enabled = requested
        self.h7.set_enabled(requested)
        if not requested:
            self.h7.stop_drive()
            self.h7.stop_steering()
        self.get_logger().info('底盘%s' % ('已使能' if requested else '已禁用'))

    def on_fault(self, message):
        self.external_fault_bits = int(message.data)
        if self.external_fault_bits:
            self.enabled = False
            self.h7.set_enabled(False)
            self.h7.stop_drive()
            self.h7.stop_steering()

    def _command_fresh(self):
        if self._last_command is None or self._last_command_time is None:
            return False
        age = (self.get_clock().now() - self._last_command_time).nanoseconds * 1e-9
        return 0.0 <= age <= self.command_timeout

    def _geometry(self):
        return {
            'wheel_radius': self.wheel_radius,
            'front_axle_x': self.front_axle_x,
            'rear_axle_x': self.rear_axle_x,
            'front_track': self.front_track,
            'rear_track': self.rear_track,
            'max_steering_angle': self.max_steering_angle,
        }

    def on_timer(self):
        hardware_faults = self._hardware_fault_bits()
        output_allowed = (
            self.enabled
            and not self.estop
            and self.external_fault_bits == 0
            and hardware_faults == 0
        )

        if output_allowed and self._command_fresh():
            vx, wz = self._last_command
        else:
            vx, wz = 0.0, 0.0

        targets, saturated = ackermann_inverse(vx, wz, self._geometry())

        drive_speeds = []
        steer_angles = []
        for steer_deg, wheel_speed_rps in targets:
            rps = max(
                -self.max_wheel_speed_rps,
                min(self.max_wheel_speed_rps, wheel_speed_rps),
            )
            drive_speeds.append(rps)
            steer_angles.append(steer_deg * self.steering_sign)

        # 动力轮速度模式（正 = 前进，方向由下位机处理）
        self.h7.set_drive_speeds(drive_speeds)
        # 舵轮位置模式（下位机单发目标、自持到位）
        if output_allowed:
            self.h7.set_steering_angles(steer_angles)

        if saturated:
            self.get_logger().debug('阿克曼解算限幅：vx=%.3f wz=%.3f' % (vx, wz))

        state = ChassisState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'base_link'
        state.fault_bits = int(self.external_fault_bits | hardware_faults)
        state.estop = self.estop
        state.enabled = self.enabled and state.fault_bits == 0 and not self.estop
        state.watchdog_ok = self._command_fresh()
        if self.estop or state.fault_bits:
            state.mode = ChassisState.MODE_FAULT
        elif state.enabled:
            state.mode = ChassisState.MODE_MANUAL
        else:
            state.mode = ChassisState.MODE_DISABLED
        self.state_publisher.publish(state)

    def _hardware_fault_bits(self):
        bits = 0
        if not self.h7.is_healthy():
            bits |= self.FAULT_SERIAL
        if not self.h7.all_motors_online():
            bits |= self.FAULT_DRIVE_FEEDBACK
        if not self.h7.all_steering_online():
            bits |= self.FAULT_STEERING
        return bits

    def shutdown(self):
        self.enabled = False
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
        node = ChassisDriverNode()
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


if __name__ == '__main__':
    main()
