import math
import sys
import threading
import time
from pathlib import Path


CONTROL_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
NAVIGATION_SCRIPTS = Path(__file__).resolve().parents[2] / "lunar_rover_navigation" / "scripts"
sys.path.insert(0, str(CONTROL_SCRIPTS))
sys.path.insert(0, str(NAVIGATION_SCRIPTS))

from chassis_driver_node import ackermann_inverse  # noqa: E402
from keyboard_rover_teleop import KeyboardRoverTeleop  # noqa: E402
from m3508_can_api import M3508CanBridge, MotorState  # noqa: E402
from usb_cdc_steering import UsbCdcSteeringMotor  # noqa: E402
from web_joystick_server import WebJoystickServer  # noqa: E402


GEOMETRY = {
    "wheel_radius": 0.125,
    "front_axle_x": 0.22,
    "rear_axle_x": -0.20,
    "front_track": 0.415,
    "rear_track": 0.350,
    "max_steering_angle": math.radians(60.0),
}


def test_ackermann_straight_and_spin_are_finite():
    # 直行：4 轮舵角为 0，轮速（r/s）= vx / (2*pi*r)
    straight, saturated = ackermann_inverse(0.16, 0.0, GEOMETRY)
    assert not saturated
    assert len(straight) == 4
    expected_rps = 0.16 / (2.0 * math.pi * GEOMETRY["wheel_radius"])
    for angle, speed in straight:
        assert angle == 0.0
        assert math.isclose(speed, expected_rps)

    # 原地转向：4 轮都输出非零轮速，且数值有限
    spin, _ = ackermann_inverse(0.0, 0.36, GEOMETRY)
    assert len(spin) == 4
    assert all(math.isfinite(value) for target in spin for value in target)
    assert all(abs(speed) > 0.0 for _, speed in spin)


def test_ackermann_spin_geometry():
    # 原地左转（wz>0）：右侧轮前进、左侧轮后退，舵角都在 ±90° 内
    spin, saturated = ackermann_inverse(0.0, 0.36, GEOMETRY)
    assert not saturated
    # 顺序：右后、左后、右前、左前
    rr, rl, rf, lf = spin
    for angle, _ in (rr, rl, rf, lf):
        assert -90.0 <= angle <= 90.0
    # 右后、右前（y<0）前进（速度为正），左后、左前（y>0）后退（速度为负）
    assert rr[1] > 0.0 and rf[1] > 0.0
    assert rl[1] < 0.0 and lf[1] < 0.0


def test_keyboard_new_key_mapping_and_boost():
    teleop = KeyboardRoverTeleop.__new__(KeyboardRoverTeleop)
    teleop.linear_speed = 0.16
    teleop.reverse_speed = 0.10
    teleop.spin_angular_speed = 0.36
    teleop.ackermann_speed = 0.12
    teleop.ackermann_angular_speed = 0.30
    teleop.boost_multiplier = 2.0

    teleop.held_keys = {"q"}
    assert teleop.compute_command()[:2] == (0.0, 0.36)
    teleop.held_keys = {"a"}
    assert teleop.compute_command()[:2] == (0.12, 0.30)
    teleop.held_keys = {"s", "d", "left_shift"}
    assert teleop.compute_command()[:2] == (-0.20, -0.60)


class _FakeDevice:
    def __init__(self):
        self.frames = []
        self.disabled = False
        self.closed = False

    def send_can(self, channel, can_id, length, payload, *flags):
        self.frames.append((channel, can_id, length, payload))
        return True

    def enable_channel(self, channel, enabled):
        self.disabled = not enabled

    def close(self):
        self.closed = True


class _FakeContext:
    def __init__(self):
        self.destroyed = False

    def destroy(self):
        self.destroyed = True


def test_can_shutdown_sends_zero_before_close():
    bridge = M3508CanBridge.__new__(M3508CanBridge)
    bridge.motors = {1: MotorState(1), 2: MotorState(2)}
    bridge._lock = threading.Lock()
    bridge._stop = threading.Event()
    bridge._thread = None
    bridge._error = None
    bridge._closed = False
    bridge.dev = _FakeDevice()
    bridge.ctx = _FakeContext()

    bridge.motors[1].set_current(1000)
    bridge.stop()

    assert len(bridge.dev.frames) == 3
    assert all(frame[3] == bytes(8) for frame in bridge.dev.frames)
    assert bridge.dev.disabled
    assert bridge.dev.closed
    assert bridge.ctx.destroyed


class _FakeSerial:
    def __init__(self):
        self.frames = []

    def write(self, payload):
        self.frames.append(payload)

    def flush(self):
        pass


def test_usb_feedback_parse_and_repeated_heartbeat():
    steering = UsbCdcSteeringMotor.__new__(UsbCdcSteeringMotor)
    steering._lock = threading.Lock()
    steering._fb_online = [0] * 4
    steering._angles = [0.0] * 4
    steering._target_angles = [1.0, 2.0, 3.0, 4.0]
    steering._last_sent_angles = [None] * 4
    steering._target_angle_0 = 0.0
    steering._speed_0 = 0.0
    steering._last_rx_time = 0.0
    steering._serial = _FakeSerial()

    steering._parse_line(b"$STD,1,1,1,1,10,20,30,40,1.0,0.1,5,1,9\r")
    assert steering._fb_online == [1, 1, 1, 1]
    assert steering._angles == [10.0, 20.0, 30.0, 40.0]
    assert time.monotonic() - steering._last_rx_time < 0.1

    steering._send_angles()
    steering._send_angles()
    assert steering._serial.frames == [
        b"$ANG,1.00,2.00,3.00,4.00!",
        b"$ANG,1.00,2.00,3.00,4.00!",
    ]


def test_web_joystick_ackermann_reverse_sign():
    server = WebJoystickServer.__new__(WebJoystickServer)
    server.deadzone = 0.05
    server.max_linear = 0.16
    server.max_reverse = 0.10
    server.spin_angular_speed = 0.36
    server.ackermann_wheelbase = 0.52
    server.ackermann_max_steering_angle = math.radians(60.0)
    server.max_angular_speed = 1.0

    forward = server.compute_command(-1.0, 1.0, False, False, False, False)
    reverse = server.compute_command(-1.0, -1.0, False, False, False, False)
    assert forward[0] > 0.0 and forward[1] > 0.0
    assert reverse[0] < 0.0 and reverse[1] < 0.0


if __name__ == "__main__":
    tests = [
        test_ackermann_straight_and_spin_are_finite,
        test_ackermann_spin_geometry,
        test_keyboard_new_key_mapping_and_boost,
        test_can_shutdown_sends_zero_before_close,
        test_usb_feedback_parse_and_repeated_heartbeat,
        test_web_joystick_ackermann_reverse_sign,
    ]
    for test in tests:
        test()
        print("PASS", test.__name__)
