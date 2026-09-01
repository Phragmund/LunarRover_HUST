#!/usr/bin/env python3
"""Hardware-free checks for the MoveIt-to-H7 arm mapping."""

import math
import sys
import threading
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from h7_serial_client import H7SerialClient  # noqa: E402
from motion_solver import MotionSolver  # noqa: E402


def make_solver_mapping():
    solver = MotionSolver.__new__(MotionSolver)
    solver.arm_zero_degrees = [90.0, 180.0, 135.0, 135.0, 0.0]
    solver.arm_ros_directions = [1.0, -1.0, 1.0, 1.0, 1.0]
    solver.arm_min_degrees = [0.0] * 5
    solver.arm_max_degrees = [270.0, 270.0, 270.0, 270.0, 90.0]
    solver.arm_min_pulses = [500] * 5
    solver.arm_max_pulses = [2500] * 5
    solver.arm_inverted = [False, True, False, True, False]
    solver.gripper_stroke_m = 0.045
    return solver


def test_home_mapping():
    solver = make_solver_mapping()
    expected_degrees = [90.0, 180.0, 135.0, 135.0]
    expected_pulses = [1167, 1167, 1500, 1500]
    for index, (degrees, pulse) in enumerate(
        zip(expected_degrees, expected_pulses)
    ):
        actual_degrees = solver._ros_to_degrees(index, 0.0)
        assert math.isclose(actual_degrees, degrees)
        assert solver._degrees_to_pulse(index, actual_degrees) == pulse
        assert math.isclose(solver._degrees_to_ros(index, actual_degrees), 0.0)


def test_gripper_mapping():
    solver = make_solver_mapping()
    assert math.isclose(solver._ros_to_degrees(4, 0.0), 0.0)
    assert math.isclose(solver._ros_to_degrees(4, 0.045), 90.0)
    assert solver._degrees_to_pulse(4, 45.0) == 1500
    assert math.isclose(solver._degrees_to_ros(4, 45.0), 0.0225)


def test_arm_frame_callback_dispatch():
    client = H7SerialClient.__new__(H7SerialClient)
    client._lock = threading.Lock()
    client._frame_callbacks = []
    received = []
    client.register_frame_callback(received.append)
    client._parse_frame(b"$ARM:RESP,#003P1500!")
    assert received == ["$ARM:RESP,#003P1500!"]


if __name__ == "__main__":
    tests = [test_home_mapping, test_gripper_mapping, test_arm_frame_callback_dispatch]
    for test in tests:
        test()
        print("PASS", test.__name__)
