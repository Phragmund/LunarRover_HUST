#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""舵电机（转向）接口。

4 个舵电机分别安装在前左、前右、后左、后右轮位（中间一对轮无舵）。
约定：上电位置为 0 点，顺时针转为正（角度单位：度）。

本模块只定义接口与一个占位实现；实际驱动（四路编码器电机驱动模块，
ASCII 帧 `$Car:...!` 或 CAN 舵机）在后续接入，替换占位实现即可。
"""


class SteeringMotorInterface:
    """舵电机位置控制接口（抽象）。"""

    STEER_COUNT = 4

    # 索引约定（与 lunar_rover_control 的 SteeringIndex 一致）
    FRONT_LEFT = 0
    FRONT_RIGHT = 1
    REAR_LEFT = 2
    REAR_RIGHT = 3

    def set_angle(self, index, angle_deg):
        """命令第 index 个舵电机转到指定角度（°，顺时针为正，0 为上电零点）。"""
        raise NotImplementedError

    def set_all_angles(self, angles_deg):
        """命令 4 个舵电机转到指定角度。angles_deg 为长度 4 的序列。"""
        for i, angle in enumerate(angles_deg):
            self.set_angle(i, angle)

    def get_angle(self, index):
        """返回第 index 个舵电机当前角度（°），无回读则返回 None。"""
        raise NotImplementedError

    def stop(self):
        """急停所有舵电机。"""
        raise NotImplementedError

    def set_enabled(self, enabled):
        """允许或禁止舵机接受位置命令。"""
        if not enabled:
            self.stop()


class DummySteeringMotor(SteeringMotorInterface):
    """占位实现：仅打印日志，用于无实际舵机驱动时的调试与联调。"""

    def __init__(self, logger=None):
        self._logger = logger
        self._angles = [0.0] * self.STEER_COUNT

    def _log(self, message):
        if self._logger is not None:
            self._logger.info(message)
        else:
            print(message)

    def set_angle(self, index, angle_deg):
        self._angles[index] = float(angle_deg)
        self._log(f"[steering] 舵机 {index} -> {angle_deg:+.2f}°")

    def get_angle(self, index):
        return self._angles[index]

    def stop(self):
        self._log("[steering] 舵机急停")

    def set_enabled(self, enabled):
        self._log("[steering] %s" % ("使能" if enabled else "停止"))
