#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""USB CDC 舵电机桥接（NUC <-> STM32F407 下位机）。

本机（NUC）通过 USB CDC 虚拟串口与下位机（Lunarover_f407）通信，
把底盘解算出的 4 个舵电机角度下发，并接收下位机的 $STD 运行反馈。

下行命令帧（与下位机 steer_motor_task.c 的 cmd_parse 对应）：
    $ANG,a0,a1,a2,a3!   位置模式，4 个舵电机目标角度（°）
    $SPD,s0,s1,s2,s3!   速度模式（开环 -1~1）
    $STOP!              停止
    $ZERO!              把当前位置设为零位

上行反馈帧（下位机每 100ms 发一次）：
    $STD,fbA,fbB,fbC,fbD,angA,angB,angC,angD,targetA,spdA,isr,online,tx\\r\\n
"""

import threading
import time

from steering_motor import SteeringMotorInterface


class UsbCdcSteeringMotor(SteeringMotorInterface):
    """通过 USB CDC 虚拟串口控制下位机舵电机。"""

    def __init__(self, port='/dev/ttyACM0', baud=115200, logger=None,
                 send_period=0.1, startup_timeout=3.0, auto_zero=True):
        """
        Args:
            port: USB CDC 虚拟串口设备（STM32 在 Linux 上通常为 /dev/ttyACM0）
            baud: 波特率（USB CDC 忽略，物理串口才需要）
            send_period: $ANG 命令下发周期（s），避免灌爆下位机
            startup_timeout: 等待四路舵机反馈在线的最长时间（s）
            auto_zero: 首次四路反馈在线后，把当前机械姿态设为零位
        """
        import serial  # 延迟导入，仅在真正使用 USB 时依赖 pyserial

        self._logger = logger
        self._send_period = send_period
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._error = None
        self._ready = False
        self._last_rx_time = 0.0

        # 目标角度（底盘解算写入）
        self._target_angles = [0.0] * self.STEER_COUNT
        self._last_sent_angles = [None] * self.STEER_COUNT

        # 反馈（从 $STD 解析）
        self._fb_online = [0] * self.STEER_COUNT
        self._angles = [0.0] * self.STEER_COUNT
        self._target_angle_0 = 0.0
        self._speed_0 = 0.0

        self._serial = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.6)  # 等 USB 枚举 / CDC 就绪
        self._serial.reset_input_buffer()

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

        if not self._wait_for_all_online(startup_timeout):
            self.stop()
            raise RuntimeError(
                'USB CDC 已打开，但未在 %.1fs 内收到四路舵机在线反馈' % startup_timeout
            )
        if auto_zero:
            self._write_frame('$ZERO!')
            time.sleep(0.15)
        self._ready = True
        self._log('USB CDC 舵机桥接已启动: %s @ %d' % (port, baud))

    def _log(self, message):
        if self._logger is not None:
            self._logger.info(message)
        else:
            print(message)

    # ---------- 接口实现 ----------
    def set_angle(self, index, angle_deg):
        with self._lock:
            self._target_angles[index] = float(angle_deg)

    def set_all_angles(self, angles_deg):
        with self._lock:
            for i in range(self.STEER_COUNT):
                self._target_angles[i] = float(angles_deg[i])

    def get_angle(self, index):
        with self._lock:
            return self._angles[index]

    def stop(self):
        if self._stop.is_set():
            return
        for _ in range(3):
            try:
                self._serial.write(b'$STOP!')
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

    def set_enabled(self, enabled):
        if enabled:
            if self._error is not None or self._stop.is_set():
                raise RuntimeError('USB CDC 舵机桥接已停止，不能重新使能')
            self._ready = True
            return
        self._ready = False
        self._write_frame('$STOP!')

    def is_healthy(self):
        with self._lock:
            last_rx_time = self._last_rx_time
            online = all(self._fb_online)
        return (
            not self._stop.is_set()
            and self._thread.is_alive()
            and self._error is None
            and online
            and (time.monotonic() - last_rx_time) <= 0.5
        )

    @property
    def error(self):
        return self._error

    # ---------- 后台循环 ----------
    def _loop(self):
        last_send = 0.0
        buf = b''
        while not self._stop.is_set():
            try:
                buf = self._read_available(buf)
                now = time.monotonic()
                if now - last_send >= self._send_period:
                    last_send = now
                    if self._ready:
                        self._send_angles()
            except Exception as exc:  # noqa: BLE001
                self._error = exc
                self._log('USB CDC 通信异常: %s' % exc)
                break
            time.sleep(0.01)

    def _read_available(self, buf):
        chunk = self._serial.read(1024)
        if chunk:
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self._parse_line(line)
        return buf

    def _parse_line(self, line):
        try:
            text = line.strip().decode('ascii', 'replace').strip()
        except Exception:  # noqa: BLE001
            return
        if not text.startswith('$STD,'):
            return
        fields = text.split(',')
        if len(fields) < 11:
            return
        with self._lock:
            self._last_rx_time = time.monotonic()
            for i in range(self.STEER_COUNT):
                try:
                    self._fb_online[i] = int(fields[1 + i])
                except (ValueError, IndexError):
                    pass
                try:
                    self._angles[i] = float(fields[5 + i])
                except (ValueError, IndexError):
                    pass
            try:
                self._target_angle_0 = float(fields[9])
                self._speed_0 = float(fields[10])
            except (ValueError, IndexError):
                pass

    def _send_angles(self):
        with self._lock:
            targets = list(self._target_angles)
        frame = '$ANG,%.2f,%.2f,%.2f,%.2f!' % tuple(targets)
        try:
            self._write_frame(frame)
            with self._lock:
                self._last_sent_angles = list(targets)
        except Exception as exc:  # noqa: BLE001
            self._log('发送 $ANG 失败: %s' % exc)
            raise

    def _write_frame(self, frame):
        self._serial.write(frame.encode('ascii'))
        self._serial.flush()

    def _wait_for_all_online(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and self._error is None:
            with self._lock:
                if all(self._fb_online):
                    return True
            time.sleep(0.02)
        return False
