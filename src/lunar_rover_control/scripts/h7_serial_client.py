#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""H7 单串口客户端：动力轮(M3508)与舵轮(转向)统一走一个 USB CDC 虚拟串口。

上下位机协议见交接包：
    src/lunar_rover_hardware/ubuntu_upper_dev_handoff_20260820/docs/01_comm_control_guide.md

下行（上位机 -> H7）：
    $M3508:SPD,s0,s1,s2,s3!   3508 速度模式，单位 输出轴 r/s，正 = 该轮前进
    $M3508:STOP!              3508 停止
    $ANG,a0,a1,a2,a3!         舵轮角度模式，单位 deg
    $STOP!                    舵轮停止
    $ZERO!                    舵轮当前位置置零

上行（H7 -> 上位机，周期遥测）：
    $M3508:FB,enabled,mode,onlineMask,a0..a3,s0..s3,g0..g3,f0..f3,t0..t3!
        其中 onlineMask 为 4 路在线位掩码（16 进制文本），s0..s3 缩放 0.001 r/s
    $STD,fb0..fb3,ang0..ang3,target0,out0,isr0,online0,tx0

轮序统一（与下位机一致）：0=右后 1=左后 2=右前 3=左前。

下位机超时约束（必须遵守，否则下位机会自动停）：
    - 3508 速度模式命令超时 500ms  -> 本模块以 drive_send_period(默认 50ms) 周期刷新
    - 舵轮位置模式命令超时 2s      -> 本模块以 steer_send_period(默认 500ms) 周期刷新

本模块只负责串口收发与协议编解码，不做任何运动学解算。
"""

import threading
import time


class H7SerialClient:
    """H7 USB CDC 单串口客户端（动力轮 + 舵轮）。"""

    DRIVE_COUNT = 4
    STEER_COUNT = 4

    def __init__(self, port, baud=115200, logger=None,
                 drive_send_period=0.05, steer_send_period=0.5,
                 feedback_timeout=0.6, steer_auto_send=True):
        """
        Args:
            port: H7 USB CDC 虚拟串口（Linux 通常 /dev/ttyACM0）
            baud: 波特率（USB CDC 忽略，物理串口才需要）
            drive_send_period: $M3508:SPD 下发周期（s），须 < 500ms
            steer_send_period: $ANG 周期下发间隔（s），须 < 2s（仅 steer_auto_send=True 时生效）
            feedback_timeout: 超过该时长未收到遥测即判定链路异常（s）
            steer_auto_send: True 时按 steer_send_period 周期重发 $ANG；
                             False 时只由 send_steering_once 单发（“一次即可”模式）
        """
        import serial  # 延迟导入，仅在真正使用串口时依赖 pyserial

        self._logger = logger
        self._drive_send_period = float(drive_send_period)
        self._steer_send_period = float(steer_send_period)
        self._feedback_timeout = float(feedback_timeout)
        self._steer_auto_send = bool(steer_auto_send)

        self._lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._error = None
        self._enabled = False
        self._enable_pending = False
        self._disable_pending = False

        # 目标值（由底盘解算写入，单位见类 docstring）
        self._drive_targets = [0.0] * self.DRIVE_COUNT
        self._steer_targets = [0.0] * self.STEER_COUNT

        # 反馈（从 $M3508:FB 与 $STD 解析）
        self._drive_online = [0] * self.DRIVE_COUNT
        self._steer_online = [0] * self.STEER_COUNT
        self._drive_speeds = [0.0] * self.DRIVE_COUNT
        self._steer_angles = [0.0] * self.STEER_COUNT
        self._last_rx_time = 0.0

        self._serial = serial.Serial(port, baud, timeout=0.05)
        time.sleep(0.6)  # 等 USB 枚举 / CDC 就绪
        self._serial.reset_input_buffer()

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ---------- 日志 ----------
    def _log(self, message):
        if self._logger is not None:
            self._logger.info(message)
        else:
            print(message)

    # ---------- 对外 API：目标写入 ----------
    def set_drive_speeds(self, rps_list):
        """设置 4 个动力轮输出轴转速（r/s，正=前进），顺序 右后/左后/右前/左前。"""
        with self._lock:
            for i in range(self.DRIVE_COUNT):
                self._drive_targets[i] = float(rps_list[i])

    def set_steering_angles(self, deg_list):
        """设置 4 个舵轮目标角度（deg），顺序 右后/左后/右前/左前。"""
        with self._lock:
            for i in range(self.STEER_COUNT):
                self._steer_targets[i] = float(deg_list[i])

    def send_steering_once(self, deg_list):
        """单发一条 $ANG 位置指令（不进入周期发送，用于“一次即可”的转向控制）。"""
        self._write(self._format_ang([float(v) for v in deg_list]))

    def send_raw(self, frame):
        """发送任意原始 ASCII 帧（如 $M3508:STOP! / $STOP! / $ZERO! / $ARM:TORQUE,...）。"""
        if isinstance(frame, str):
            frame = frame.encode("ascii")
        self._write(frame)

    def set_enabled(self, enabled):
        """使能/禁用。使能时发 $M3508:ENABLE,1，禁用时发 ENABLE,0 并停驱动/松舵。"""
        with self._lock:
            was_enabled = self._enabled
            self._enabled = bool(enabled)
            if self._enabled and not was_enabled:
                self._enable_pending = True
            elif not self._enabled and was_enabled:
                self._disable_pending = True

    def stop_drive(self):
        with self._lock:
            self._drive_targets = [0.0] * self.DRIVE_COUNT

    def stop_steering(self):
        with self._lock:
            self._steer_targets = [0.0] * self.STEER_COUNT

    # ---------- 对外 API：反馈读取 ----------
    def get_drive_online(self):
        with self._lock:
            return list(self._drive_online)

    def get_steer_online(self):
        with self._lock:
            return list(self._steer_online)

    def get_drive_speeds(self):
        with self._lock:
            return list(self._drive_speeds)

    def get_steer_angles(self):
        with self._lock:
            return list(self._steer_angles)

    def all_motors_online(self):
        with self._lock:
            return all(self._drive_online)

    def all_steering_online(self):
        with self._lock:
            return all(self._steer_online)

    def is_healthy(self):
        with self._lock:
            last_rx = self._last_rx_time
            error = self._error
            thread_alive = self._thread is not None and self._thread.is_alive()
        return (
            not self._stop.is_set()
            and thread_alive
            and error is None
            and (time.monotonic() - last_rx) <= self._feedback_timeout
        )

    @property
    def error(self):
        return self._error

    # ---------- 生命周期 ----------
    def stop(self):
        if self._stop.is_set():
            return
        for _ in range(2):
            try:
                self._write(b"$M3508:ENABLE,0")
                self._write(b"$M3508:SPD,0,0,0,0!")
                self._write(b"$STOP!")
                self._serial.flush()
            except Exception:  # noqa: BLE001
                break
            time.sleep(0.02)
        self._stop.set()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=0.5)
        try:
            self._serial.close()
        except Exception:  # noqa: BLE001
            pass

    # ---------- 后台循环 ----------
    def _loop(self):
        buf = b""
        last_drive_send = 0.0
        last_steer_send = 0.0
        while not self._stop.is_set():
            try:
                buf = self._read_available(buf)
                now = time.monotonic()

                with self._lock:
                    enabled = self._enabled
                    drive_targets = list(self._drive_targets)
                    steer_targets = list(self._steer_targets)
                    enable_pending = self._enable_pending
                    disable_pending = self._disable_pending
                    self._enable_pending = False
                    self._disable_pending = False

                if disable_pending:
                    self._write(b"$M3508:ENABLE,0")
                    self._write(b"$M3508:SPD,0,0,0,0!")
                    self._write(b"$STOP!")
                    continue

                if enable_pending:
                    self._write(b"$M3508:ENABLE,1")

                if enabled:
                    if now - last_drive_send >= self._drive_send_period:
                        last_drive_send = now
                        self._write(self._format_spd(drive_targets))
                    if self._steer_auto_send and now - last_steer_send >= self._steer_send_period:
                        last_steer_send = now
                        self._write(self._format_ang(steer_targets))
            except Exception as exc:  # noqa: BLE001
                self._error = exc
                self._log("H7 串口通信异常: %s" % exc)
                break
            time.sleep(0.005)

    def _read_available(self, buf):
        chunk = self._serial.read(4096)
        if chunk:
            buf += chunk
            # 所有帧都以 '!' 结尾，按 '!' 切分后逐帧解析
            while b"!" in buf:
                frame, buf = buf.split(b"!", 1)
                self._parse_frame(frame + b"!")
        return buf

    def _parse_frame(self, raw):
        try:
            text = raw.decode("ascii", "replace").strip()
        except Exception:  # noqa: BLE001
            return
        if text.startswith("$M3508:FB,"):
            self._parse_m3508_fb(text)
        elif text.startswith("$STD,"):
            self._parse_std(text)

    def _parse_m3508_fb(self, text):
        # $M3508:FB,enabled,mode,onlineMask,a0..a3,s0..s3,g0..g3,f0..f3,t0..t3!
        fields = text.split(",")
        if len(fields) < 12:
            return
        try:
            mask = int(fields[3], 16)
        except (ValueError, IndexError):
            mask = 0
        with self._lock:
            self._last_rx_time = time.monotonic()
            for i in range(self.DRIVE_COUNT):
                self._drive_online[i] = 1 if (mask & (1 << i)) else 0
            # s0..s3 位于字段 8..11，缩放 0.001 r/s
            for i in range(self.DRIVE_COUNT):
                try:
                    self._drive_speeds[i] = float(fields[8 + i]) * 0.001
                except (ValueError, IndexError):
                    pass

    def _parse_std(self, text):
        # $STD,fb0..fb3,ang0..ang3,target0,out0,isr0,online0,tx0
        fields = text.split(",")
        if len(fields) < 9:
            return
        with self._lock:
            self._last_rx_time = time.monotonic()
            for i in range(self.STEER_COUNT):
                try:
                    self._steer_online[i] = int(fields[1 + i])
                except (ValueError, IndexError):
                    pass
                try:
                    self._steer_angles[i] = float(fields[5 + i])
                except (ValueError, IndexError):
                    pass

    # ---------- 帧格式化 ----------
    @staticmethod
    def _format_spd(targets):
        payload = ",".join("%.4f" % v for v in targets)
        return ("$M3508:SPD," + payload + "!").encode("ascii")

    @staticmethod
    def _format_ang(targets):
        payload = ",".join("%.2f" % v for v in targets)
        return ("$ANG," + payload + "!").encode("ascii")

    def _write(self, data):
        with self._write_lock:
            self._serial.write(data)
            self._serial.flush()


class DummyH7Client:
    """占位实现：不打开串口，仅记录目标值，用于 ROS 图/命令路径测试。"""

    DRIVE_COUNT = 4
    STEER_COUNT = 4

    def __init__(self, logger=None, **kwargs):
        self._logger = logger
        self._drive_targets = [0.0] * self.DRIVE_COUNT
        self._steer_targets = [0.0] * self.STEER_COUNT
        self._enabled = False

    def _log(self, message):
        if self._logger is not None:
            self._logger.info(message)
        else:
            print(message)

    def set_drive_speeds(self, rps_list):
        self._drive_targets = [float(v) for v in rps_list]

    def set_steering_angles(self, deg_list):
        self._steer_targets = [float(v) for v in deg_list]

    def send_steering_once(self, deg_list):
        self._steer_targets = [float(v) for v in deg_list]

    def send_raw(self, frame):
        self._log("[h7] RAW %s" % frame)

    def set_enabled(self, enabled):
        changed = bool(enabled) != self._enabled
        self._enabled = bool(enabled)
        if changed:
            self._log("[h7] %s" % ("使能" if self._enabled else "禁用"))

    def stop_drive(self):
        self._drive_targets = [0.0] * self.DRIVE_COUNT

    def stop_steering(self):
        self._steer_targets = [0.0] * self.STEER_COUNT

    def get_drive_online(self):
        return [1] * self.DRIVE_COUNT

    def get_steer_online(self):
        return [1] * self.STEER_COUNT

    def get_drive_speeds(self):
        return list(self._drive_targets)

    def get_steer_angles(self):
        return list(self._steer_targets)

    def all_motors_online(self):
        return True

    def all_steering_online(self):
        return True

    def is_healthy(self):
        return True

    @property
    def error(self):
        return None

    def stop(self):
        self.set_enabled(False)
        self._log("[h7] 停止")
