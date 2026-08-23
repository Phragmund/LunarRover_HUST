#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M3508 / C620 电机的 CAN 控制 API（达妙 USB2CANFD + dmcan SDK）。

供 ROS2 底盘驱动节点复用。管理 1~4 个动力轮电机（C620 电调 ID1~4，
共用一条 1 Mbps Classic CAN 总线，控制帧 0x200），每个电机支持三种模式：

- 电流模式（current）：直接给定电流 raw（±16384 ≈ ±20A）
- 速度模式（speed）  ：给定输出轴转速 r/s（车体前进方向为正），速度环 PID -> 电流
- 位置模式（position）：给定输出轴角度 °（上电零点为 0，前进方向为正），
                        位置环 PID -> 速度目标，速度环 PID -> 电流（双环）

方向约定通过 `direction` 参数处理：direction=+1 表示"车体前进方向 = 转子正转"
（左轮），direction=-1 表示镜像安装（右轮，前进对应转子反转）。

运行依赖（由 handoff_dir 提供）：
- handoff_dir/dmcan/       官方 Python SDK 包
- handoff_dir/dlls/        后端库 libdm_device.so + 新版 libstdc++/libusb
"""

import ctypes
import os
import sys
import threading
import time

# ===== 常量 =====
REDUCTION_RATIO = 3591.0 / 187.0      # 减速比 ≈ 19.2
ENCODER_MAX = 8192                     # 转子编码器每圈线数
CONTROL_ID = 0x200                     # 控制帧 ID（控制电调 1-4）
FEEDBACK_TIMEOUT = 0.2                 # 反馈超时（s）
CONTROL_DT = 0.010                     # 控制周期 10ms（100Hz）

# ===== 默认 PID 参数（真车调试值）=====
DEFAULT_SPEED_PID = {
    'kp': 1.4,
    'ki': 0.7,
    'kd': 0.0025,
    'output_limit': 5600.0,      # 电流输出限幅（raw）
    'integral_limit': 500.0,
    'deadband': 3.0,             # 速度死区（转子 rpm）
}

DEFAULT_POS_PID = {
    'pos_kp': 0.01,              # 1° 角度误差 -> 0.01 r/s 速度目标
    'pos_ki': 0.3,
    'pos_kd': 0.001,
    'pos_integral_limit': 600.0,
    'pos_deadband': 30.0,        # 位置死区（°）
    'max_speed_rps': 1.0,        # 位置环输出速度限幅（输出轴 r/s）
}

DEFAULT_FRICTION = {
    'friction_comp': 200.0,      # 静摩擦补偿电流（raw）
    'friction_deadband_rpm': 30.0,  # 摩擦前馈生效的速度目标死区（转子 rpm）
}


def _int16(u):
    u &= 0xFFFF
    return u - 0x10000 if u >= 0x8000 else u


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class MotorState:
    """单个动力轮电机的控制状态 + 双环 PID。"""

    def __init__(self, esc_id, direction=1.0, params=None):
        self.esc_id = int(esc_id)
        # 车体前进方向 -> 转子正转 的映射（左轮 +1，右轮镜像 -1）
        self.direction = 1.0 if direction >= 0 else -1.0

        self.params = {}
        self.params.update(DEFAULT_SPEED_PID)
        self.params.update(DEFAULT_POS_PID)
        self.params.update(DEFAULT_FRICTION)
        if params:
            self.params.update(params)

        # 模式与目标
        self.mode = 'current'       # 'current' | 'speed' | 'position'
        self.target_current = 0
        self.target_rps = 0.0       # 输出轴转速（前进正）
        self.target_angle_deg = 0.0  # 输出轴角度（前进正）

        # 反馈（输出轴速度/角度统一为"车体前进方向为正"）
        self.encoder = 0
        self.rpm = 0.0              # 转子转速（原始）
        self.output_rps = 0.0
        self.output_angle_deg = 0.0
        self.temp = 0
        self.cur_fb = 0
        self.fb_ts = 0.0

        # 编码器多圈累计
        self.round_count = 0
        self.last_encoder = None
        self.zero_rotor_angle = None

        # 速度环 PID 状态
        self.integral = 0.0
        self.last_err = 0.0

        # 位置环 PID 状态
        self.pos_integral = 0.0
        self.pos_last_err = 0.0

        self.current_cmd = 0

    # ---------- 反馈 ----------
    def on_feedback(self, encoder, rpm, cur, temp, now):
        self.encoder = encoder
        self.rpm = float(rpm)
        self.cur_fb = cur
        self.temp = temp
        self.fb_ts = now

        # 编码器多圈累计 -> 转子角度
        if self.last_encoder is not None:
            diff = self.encoder - self.last_encoder
            if diff > ENCODER_MAX // 2 and self.rpm < -50.0:
                self.round_count -= 1
            elif diff < -(ENCODER_MAX // 2) and self.rpm > 50.0:
                self.round_count += 1
        self.last_encoder = self.encoder
        rotor_angle_deg = self.round_count * 360.0 + self.encoder * 360.0 / ENCODER_MAX
        if self.zero_rotor_angle is None:
            self.zero_rotor_angle = rotor_angle_deg
        # 输出轴角度/速度，前进方向为正
        self.output_angle_deg = (rotor_angle_deg - self.zero_rotor_angle) / REDUCTION_RATIO * self.direction
        self.output_rps = self.rpm / REDUCTION_RATIO / 60.0 * self.direction

    def is_online(self, now):
        return (now - self.fb_ts) < FEEDBACK_TIMEOUT

    # ---------- 模式设置 ----------
    def set_current(self, current_raw):
        self.mode = 'current'
        self.target_current = int(current_raw)

    def set_speed(self, rps):
        self.mode = 'speed'
        self.target_rps = float(rps)

    def set_position(self, angle_deg):
        self.mode = 'position'
        self.target_angle_deg = float(angle_deg)

    def stop(self):
        self.mode = 'current'
        self.target_current = 0
        self.integral = 0.0
        self.last_err = 0.0
        self.pos_integral = 0.0
        self.pos_last_err = 0.0
        self.current_cmd = 0

    def reset_position_zero(self):
        """把当前位置重新标定为位置零点。"""
        rotor_angle = self.round_count * 360.0 + self.encoder * 360.0 / ENCODER_MAX
        self.zero_rotor_angle = rotor_angle
        self.output_angle_deg = 0.0

    # ---------- 控制 ----------
    def step(self, now, dt):
        """一次控制迭代，返回电流 raw。"""
        if not self.is_online(now):
            self.integral = 0.0
            self.last_err = 0.0
            self.pos_integral = 0.0
            self.pos_last_err = 0.0
            self.current_cmd = 0
            return 0

        if self.mode == 'current':
            self.current_cmd = _clamp(self.target_current,
                                      -self.params['output_limit'],
                                      self.params['output_limit'])
            return int(self.current_cmd)

        # 速度目标（转子 rpm）
        if self.mode == 'position':
            speed_target = self._step_pos_pid(now, dt)
        else:  # speed
            speed_target = self.target_rps * 60.0 * REDUCTION_RATIO * self.direction

        # 速度环 -> 电流
        self.current_cmd = self._step_speed_pid(speed_target, dt)
        return int(self.current_cmd)

    def _step_speed_pid(self, speed_target_rotor_rpm, dt):
        p = self.params
        err = speed_target_rotor_rpm - self.rpm
        if abs(err) < p['deadband']:
            err = 0.0

        self.integral += err * dt
        self.integral = _clamp(self.integral, -p['integral_limit'], p['integral_limit'])
        deriv = (err - self.last_err) / dt if dt > 1e-6 else 0.0
        self.last_err = err

        out = p['kp'] * err + p['ki'] * self.integral + p['kd'] * deriv
        # 摩擦前馈
        if abs(speed_target_rotor_rpm) > p['friction_deadband_rpm']:
            out += p['friction_comp'] * (1.0 if speed_target_rotor_rpm > 0 else -1.0)
        # 限幅 + anti-windup
        if out > p['output_limit']:
            out = p['output_limit']
            self.integral -= err * dt
        elif out < -p['output_limit']:
            out = -p['output_limit']
            self.integral -= err * dt
        return out

    def _step_pos_pid(self, now, dt):
        p = self.params
        err = self.target_angle_deg - self.output_angle_deg
        if abs(err) < p['pos_deadband']:
            err = 0.0
            self.pos_integral = 0.0

        self.pos_integral += err * dt
        self.pos_integral = _clamp(self.pos_integral,
                                   -p['pos_integral_limit'], p['pos_integral_limit'])
        deriv = (err - self.pos_last_err) / dt if dt > 1e-6 else 0.0
        self.pos_last_err = err

        speed_rps = p['pos_kp'] * err + p['pos_ki'] * self.pos_integral + p['pos_kd'] * deriv
        # 限幅 + anti-windup
        if speed_rps > p['max_speed_rps']:
            speed_rps = p['max_speed_rps']
            self.pos_integral -= err * dt
        elif speed_rps < -p['max_speed_rps']:
            speed_rps = -p['max_speed_rps']
            self.pos_integral -= err * dt
        return speed_rps * 60.0 * REDUCTION_RATIO * self.direction


class M3508CanBridge:
    """CAN 总线桥接，管理 1~4 个 M3508 电机，后台线程持续发 0x200 帧。"""

    def __init__(self, handoff_dir, motors, dlls_dir=None):
        """
        Args:
            handoff_dir: 交接包目录（含 dmcan/ 与 dlls/）
            motors: dict {esc_id: MotorState}，esc_id 为 1~4
            dlls_dir: 可选，后端库目录，默认 handoff_dir/dlls
        """
        self.handoff_dir = handoff_dir
        self.dlls_dir = dlls_dir or os.path.join(handoff_dir, 'dlls')
        self.motors = motors
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._error = None
        self._closed = False

        self._load_backend()
        self._open_device()

    # ---------- 后端加载 ----------
    def _load_backend(self):
        # 1. 预加载新版运行时库（解决 Ubuntu 22.04 的 GLIBCXX/libusb 版本过低问题）
        if not sys.platform.startswith('win'):
            for lib in ('libstdc++.so.6', 'libusb-1.0.so.0'):
                path = os.path.join(self.dlls_dir, lib)
                if os.path.exists(path):
                    try:
                        ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
                    except OSError:
                        pass
        # 2. 把 handoff_dir 加入 sys.path，使 import dmcan 可用
        if self.handoff_dir not in sys.path:
            sys.path.insert(0, self.handoff_dir)

        global dmcan_channel_can_info, DmCanContext
        from dmcan import DmCanContext, dmcan_channel_can_info  # noqa: F401
        self._DmCanContext = DmCanContext
        self._dmcan_channel_can_info = dmcan_channel_can_info

    def _open_device(self):
        self.ctx = self._DmCanContext()
        n = self.ctx.find_devices()
        if n == 0:
            self.ctx.destroy()
            raise RuntimeError('未发现 USB2CANFD 设备')
        self.dev = self.ctx.get_device(0)
        if not self.dev.open():
            self.ctx.destroy()
            raise RuntimeError('打开 USB2CANFD 设备失败')
        self.dev.enable_channel(0, True)
        info = self._dmcan_channel_can_info()
        info.channel = 0
        info.canfd = False
        info.can_baudrate = 1000000
        info.can_sp = 0.75
        self.dev.set_channel_baudrate(0, info)
        self.dev.hook_recv_callback(self._on_feedback)

    # ---------- 反馈 ----------
    def _on_feedback(self, dev, frame):
        aid = frame.head.can_id
        if aid < 0x201 or aid > 0x208 or frame.head.dlc < 7:
            return
        esc_id = aid - 0x200
        motor = self.motors.get(esc_id)
        if motor is None:
            return
        p = frame.payload
        encoder = (p[0] << 8) | p[1]
        rpm = _int16((p[2] << 8) | p[3])
        cur = _int16((p[4] << 8) | p[5])
        temp = p[6]
        with self._lock:
            motor.on_feedback(encoder, rpm, cur, temp, time.monotonic())

    # ---------- 控制循环 ----------
    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    def stop(self):
        if self._closed:
            return

        # Change every target before sending the final frames so the control
        # thread cannot interleave a stale non-zero current during shutdown.
        self.stop_all()
        for _ in range(3):
            try:
                self._send_currents({})
            except Exception:  # noqa: BLE001
                break
            time.sleep(CONTROL_DT)

        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)

        try:
            self.dev.enable_channel(0, False)
        except Exception:  # noqa: BLE001
            pass
        try:
            self.dev.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.ctx.destroy()
        except Exception:  # noqa: BLE001
            pass
        self._closed = True

    def _control_loop(self):
        last_ctl = time.monotonic()
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                dt = now - last_ctl
                if dt >= CONTROL_DT:
                    last_ctl = now
                    with self._lock:
                        currents = {
                            esc_id: motor.step(now, dt)
                            for esc_id, motor in self.motors.items()
                        }
                    self._send_currents(currents)
                time.sleep(0.002)
        except Exception as exc:  # noqa: BLE001
            self._error = exc
            self._stop.set()
            try:
                self._send_currents({})
            except Exception:  # noqa: BLE001
                pass

    def _send_currents(self, currents):
        data = bytearray(8)
        for esc_id in range(1, 5):
            cur = int(currents.get(esc_id, 0)) & 0xFFFF
            offset = (esc_id - 1) * 2
            data[offset] = (cur >> 8) & 0xFF
            data[offset + 1] = cur & 0xFF
        sent = self.dev.send_can(
            0, CONTROL_ID, 8, bytes(data), False, False, False, False
        )
        if sent is False:
            raise RuntimeError('USB2CANFD 发送 0x200 控制帧失败')

    # ---------- 对外 API ----------
    def set_current(self, esc_id, current_raw):
        with self._lock:
            self.motors[esc_id].set_current(current_raw)

    def set_speed(self, esc_id, rps):
        with self._lock:
            self.motors[esc_id].set_speed(rps)

    def set_position(self, esc_id, angle_deg):
        with self._lock:
            self.motors[esc_id].set_position(angle_deg)

    def stop_all(self):
        with self._lock:
            for motor in self.motors.values():
                motor.stop()

    def is_healthy(self):
        return (
            not self._closed
            and self._error is None
            and self._thread is not None
            and self._thread.is_alive()
        )

    def all_motors_online(self):
        now = time.monotonic()
        with self._lock:
            return bool(self.motors) and all(
                motor.is_online(now) for motor in self.motors.values()
            )

    @property
    def error(self):
        return self._error

    def reset_position_zero(self, esc_id):
        with self._lock:
            self.motors[esc_id].reset_position_zero()

    def get_motor(self, esc_id):
        return self.motors.get(esc_id)
