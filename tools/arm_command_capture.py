#!/usr/bin/env python3
"""Passive, timestamped capture for real-arm command/feedback diagnosis.

This node never publishes a control message.  It records all ROS-visible arm
inputs and H7 responses to JSONL, then emits diagnostic event records for
large target steps, quick target reversals, malformed UART10 responses, and
large readback steps.  Browser Web Serial commands that bypass ROS cannot be
observed by this process; the log metadata records the ROS publishers visible
when capture begins.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


ARM_NAMES = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4"]
ARM_ZERO_DEGREES = [90.0, 180.0, 135.0, 135.0]
ARM_DIRECTIONS = [1.0, -1.0, 1.0, 1.0]
ARM_RESPONSE_RE = re.compile(r"\$ARM:RESP,#(\d{1,3})P(\d{1,4})!")
ARM_RESPONSE_BROKEN_RE = re.compile(r"\$ARM:RESP,([^!\r\n]*)!")
ARM_MULTI_RE = re.compile(r"^\$ARM:MULTI,(.+),(\d+)!$")


class ArmCommandCapture(Node):
    def __init__(self, output_path: Path, target_jump_deg: float, feedback_jump_deg: float):
        super().__init__("arm_command_capture")
        self.output = output_path.open("a", encoding="utf-8", buffering=1)
        self.target_jump_deg = float(target_jump_deg)
        self.feedback_jump_deg = float(feedback_jump_deg)
        self.command_history = []
        self.raw_command_history = []
        self.last_feedback = None
        self.sequence = 0

        self.create_subscription(
            JointTrajectory, "/arm_controller/joint_trajectory", self.on_trajectory, 50
        )
        self.create_subscription(String, "/h7/raw_command", self.on_raw_command, 50)
        self.create_subscription(String, "/h7/rx_frame", self.on_h7_feedback, 100)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 50)
        self.create_subscription(String, "/grasp/status", self.on_grasp_status, 20)

        self.write(
            "capture_start",
            {
                "target_jump_deg": self.target_jump_deg,
                "feedback_jump_deg": self.feedback_jump_deg,
                "ros_publishers": {
                    topic: self.publishers_for(topic)
                    for topic in (
                        "/arm_controller/joint_trajectory",
                        "/h7/raw_command",
                        "/gripper_controller/commands",
                    )
                },
            },
        )
        self.get_logger().info("arm capture writing to %s" % output_path)

    def publishers_for(self, topic: str):
        return sorted(
            {
                "%s%s" % (item.node_namespace, item.node_name)
                for item in self.get_publishers_info_by_topic(topic)
            }
        )

    def write(self, kind: str, payload: dict):
        self.sequence += 1
        record = {
            "sequence": self.sequence,
            "monotonic_sec": round(time.monotonic(), 6),
            "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "kind": kind,
            **payload,
        }
        self.output.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")

    @staticmethod
    def degrees_from_positions(positions):
        return [
            ARM_ZERO_DEGREES[index]
            + ARM_DIRECTIONS[index] * math.degrees(float(position))
            for index, position in enumerate(positions[:4])
        ]

    @staticmethod
    def degrees_from_pulses(pulses):
        degrees = []
        for index, pulse in enumerate(pulses[:4]):
            ratio = (float(pulse) - 500.0) / 2000.0
            if index in (1, 3):
                ratio = 1.0 - ratio
            degrees.append(270.0 * ratio)
        return degrees

    @staticmethod
    def parse_multi(frame: str):
        match = ARM_MULTI_RE.fullmatch(frame)
        if match is None:
            return None
        values = match.group(1).split(",")
        if len(values) % 2:
            return None
        pairs = {}
        try:
            for index in range(0, len(values), 2):
                pairs[int(values[index])] = int(values[index + 1])
        except ValueError:
            return None
        if not all(index in pairs for index in range(1, 5)):
            return None
        return [pairs[index] for index in range(1, 6) if index in pairs]

    def on_trajectory(self, message: JointTrajectory):
        if list(message.joint_names) != ARM_NAMES or not message.points:
            self.write(
                "trajectory_other",
                {"joint_names": list(message.joint_names), "point_count": len(message.points)},
            )
            return
        point = message.points[-1]
        if len(point.positions) != 4:
            self.write("trajectory_invalid", {"positions": list(point.positions)})
            return
        angles = self.degrees_from_positions(point.positions)
        duration_sec = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
        now = time.monotonic()
        record = {
            "source": self.publishers_for("/arm_controller/joint_trajectory"),
            "angles_deg": [round(value, 3) for value in angles],
            "duration_sec": round(duration_sec, 4),
        }
        if self.command_history:
            prior = self.command_history[-1]
            delta = [angles[index] - prior["angles"][index] for index in range(4)]
            record["delta_from_previous_deg"] = [round(value, 3) for value in delta]
            if max(abs(value) for value in delta) >= self.target_jump_deg:
                self.write("event_target_jump", {**record, "max_delta_deg": round(max(abs(value) for value in delta), 3)})
        self.write("trajectory", record)
        self.command_history.append({"time": now, "angles": angles, "record": record})
        self.command_history = self.command_history[-3:]
        if len(self.command_history) == 3:
            first, middle, last = self.command_history
            elapsed = last["time"] - first["time"]
            out = max(abs(middle["angles"][i] - first["angles"][i]) for i in range(4))
            back = max(abs(last["angles"][i] - first["angles"][i]) for i in range(4))
            if elapsed <= 3.0 and out >= self.target_jump_deg and back <= 3.0:
                self.write(
                    "event_target_reversal",
                    {
                        "elapsed_sec": round(elapsed, 3),
                        "first_deg": [round(v, 3) for v in first["angles"]],
                        "outlier_deg": [round(v, 3) for v in middle["angles"]],
                        "recovered_deg": [round(v, 3) for v in last["angles"]],
                    },
                )

    def on_raw_command(self, message: String):
        frame = str(message.data).strip()
        if frame.startswith("$ARM:"):
            record = {
                "source": self.publishers_for("/h7/raw_command"),
                "frame": frame,
            }
            pulses = self.parse_multi(frame)
            if pulses is not None:
                angles = self.degrees_from_pulses(pulses)
                record["pulses"] = pulses
                record["angles_deg"] = [round(value, 3) for value in angles]
                now = time.monotonic()
                if self.raw_command_history:
                    prior = self.raw_command_history[-1]
                    delta = [angles[index] - prior["angles"][index] for index in range(4)]
                    record["delta_from_previous_deg"] = [round(value, 3) for value in delta]
                    if max(abs(value) for value in delta) >= self.target_jump_deg:
                        self.write(
                            "event_raw_target_jump",
                            {
                                **record,
                                "max_delta_deg": round(max(abs(value) for value in delta), 3),
                            },
                        )
                self.raw_command_history.append({"time": now, "angles": angles, "record": record})
                self.raw_command_history = self.raw_command_history[-3:]
                if len(self.raw_command_history) == 3:
                    first, middle, last = self.raw_command_history
                    elapsed = last["time"] - first["time"]
                    out = max(abs(middle["angles"][i] - first["angles"][i]) for i in range(4))
                    back = max(abs(last["angles"][i] - first["angles"][i]) for i in range(4))
                    if elapsed <= 3.0 and out >= self.target_jump_deg and back <= 3.0:
                        self.write(
                            "event_raw_target_reversal",
                            {
                                "elapsed_sec": round(elapsed, 3),
                                "first_deg": [round(v, 3) for v in first["angles"]],
                                "outlier_deg": [round(v, 3) for v in middle["angles"]],
                                "recovered_deg": [round(v, 3) for v in last["angles"]],
                            },
                        )
            self.write("raw_arm_command", record)

    def on_h7_feedback(self, message: String):
        text = str(message.data)
        for frame in text.replace("\r", "").split("\n"):
            frame = frame.strip()
            if not frame.startswith("$ARM:"):
                continue
            valid = ARM_RESPONSE_RE.fullmatch(frame)
            broken = ARM_RESPONSE_BROKEN_RE.fullmatch(frame)
            self.write("h7_arm_feedback", {"frame": frame})
            if broken is not None and valid is None:
                self.write("event_malformed_arm_response", {"frame": frame})

    def on_joint_state(self, message: JointState):
        positions = {name: value for name, value in zip(message.name, message.position)}
        if any(name not in positions for name in ARM_NAMES):
            return
        angles = self.degrees_from_positions([positions[name] for name in ARM_NAMES])
        now = time.monotonic()
        if self.last_feedback is not None:
            elapsed = now - self.last_feedback["time"]
            delta = [angles[index] - self.last_feedback["angles"][index] for index in range(4)]
            if elapsed <= 0.25 and max(abs(value) for value in delta) >= self.feedback_jump_deg:
                self.write(
                    "event_feedback_jump",
                    {
                        "elapsed_sec": round(elapsed, 4),
                        "angles_deg": [round(value, 3) for value in angles],
                        "delta_deg": [round(value, 3) for value in delta],
                        "recent_command": self.command_history[-1]["record"] if self.command_history else None,
                        "recent_raw_command": self.raw_command_history[-1]["record"]
                        if self.raw_command_history
                        else None,
                    },
                )
        self.last_feedback = {"time": now, "angles": angles}

    def on_grasp_status(self, message: String):
        self.write("grasp_status", {"status": str(message.data)})

    def destroy_node(self):
        self.write("capture_stop", {})
        self.output.close()
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="JSONL path")
    parser.add_argument("--target-jump-deg", type=float, default=25.0)
    parser.add_argument("--feedback-jump-deg", type=float, default=12.0)
    args, ros_args = parser.parse_known_args()

    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=ros_args)
    node = ArmCommandCapture(output_path, args.target_jump_deg, args.feedback_jump_deg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
