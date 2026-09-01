#!/usr/bin/env python3
"""Read real arm servo positions synchronously and publish /joint_states."""

from __future__ import annotations

import math
import re
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmJointStateReader(Node):
    def __init__(self):
        super().__init__("arm_joint_state_reader")
        self.port = self.declare_parameter("port", "/dev/ttyACM0").value
        self.baud = int(self.declare_parameter("baud", 115200).value)
        self.poll_rate_hz = float(self.declare_parameter("poll_rate_hz", 4.0).value)
        self.read_timeout_sec = float(
            self.declare_parameter("read_timeout_sec", 0.25).value
        )
        self.inter_read_delay_sec = float(
            self.declare_parameter("inter_read_delay_sec", 0.02).value
        )
        self.joint_ids = [
            int(value)
            for value in self.declare_parameter("joint_ids", [1, 2, 3, 4, 5]).value
        ]
        self.joint_names = [
            str(value)
            for value in self.declare_parameter(
                "joint_names",
                ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"],
            ).value
        ]
        self.zero_degrees = [
            float(value)
            for value in self.declare_parameter(
                "arm_zero_degrees", [90.0, 180.0, 135.0, 135.0, 0.0]
            ).value
        ]
        self.ros_directions = [
            float(value)
            for value in self.declare_parameter(
                "arm_ros_directions", [1.0, -1.0, 1.0, 1.0, 1.0]
            ).value
        ]
        self.min_degrees = [
            float(value)
            for value in self.declare_parameter(
                "arm_min_degrees", [0.0, 0.0, 0.0, 0.0, 0.0]
            ).value
        ]
        self.max_degrees = [
            float(value)
            for value in self.declare_parameter(
                "arm_max_degrees", [270.0, 270.0, 270.0, 270.0, 90.0]
            ).value
        ]
        self.min_pulses = [
            int(value)
            for value in self.declare_parameter(
                "arm_min_pulses", [500, 500, 500, 500, 500]
            ).value
        ]
        self.max_pulses = [
            int(value)
            for value in self.declare_parameter(
                "arm_max_pulses", [2500, 2500, 2500, 2500, 2500]
            ).value
        ]
        self.inverted = [
            bool(value)
            for value in self.declare_parameter(
                "arm_inverted", [False, True, False, True, False]
            ).value
        ]
        if len(self.joint_ids) != len(self.joint_names):
            raise ValueError("joint_ids and joint_names must have the same length")

        import serial

        self.serial = serial.Serial(self.port, self.baud, timeout=0.02)
        time.sleep(0.6)
        self.serial.reset_input_buffer()
        self._rx_buffer = b""
        self._pending_pulses = {}
        self._frame_pattern = re.compile(rb"\$ARM:RESP,#(\d{1,3})P(\d{1,4})!")
        self.positions = [0.0] * len(self.joint_names)
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / max(0.1, self.poll_rate_hz), self.poll_once)
        self.get_logger().info(
            "arm joint state reader polling %s at %.2f full poses/s"
            % (self.port, self.poll_rate_hz)
        )

    def pulse_to_degrees(self, index, pulse):
        span_pulse = max(1, self.max_pulses[index] - self.min_pulses[index])
        ratio = (int(pulse) - self.min_pulses[index]) / span_pulse
        if self.inverted[index]:
            ratio = 1.0 - ratio
        return self.min_degrees[index] + ratio * (
            self.max_degrees[index] - self.min_degrees[index]
        )

    def degrees_to_ros(self, index, degrees):
        if index == 4:
            return max(0.0, min(0.045, float(degrees) / 90.0 * 0.045))
        return math.radians(
            (float(degrees) - self.zero_degrees[index]) / self.ros_directions[index]
        )

    def _consume_buffer(self):
        responses = []
        while True:
            match = self._frame_pattern.search(self._rx_buffer)
            if match is None:
                if len(self._rx_buffer) > 256:
                    self._rx_buffer = self._rx_buffer[-64:]
                return responses
            responses.append((int(match.group(1)), int(match.group(2))))
            self._rx_buffer = self._rx_buffer[match.end() :]

    def _read_serial_frames(self):
        chunk = self.serial.read(256)
        if chunk:
            self._rx_buffer += chunk
        return self._consume_buffer()

    def _stash_responses(self, responses):
        for response_id, pulse in responses:
            self._pending_pulses[response_id] = pulse

    def read_joint(self, joint_id):
        if joint_id in self._pending_pulses:
            return joint_id, self._pending_pulses.pop(joint_id)

        self.serial.write(("$ARM:READ,%d!" % joint_id).encode("ascii"))
        self.serial.flush()
        deadline = time.monotonic() + self.read_timeout_sec
        while time.monotonic() < deadline:
            responses = self._read_serial_frames()
            for response_id, pulse in responses:
                if response_id == joint_id:
                    self._stash_responses(
                        [
                            response
                            for response in responses
                            if response[0] != joint_id
                        ]
                    )
                    return response_id, pulse
                self._pending_pulses[response_id] = pulse
        return None

    def poll_once(self):
        updated = False
        for index, joint_id in enumerate(self.joint_ids):
            response = self.read_joint(joint_id)
            if response is None:
                self.get_logger().warning(
                    "no ARM READ response from joint %d" % joint_id,
                    throttle_duration_sec=2.0,
                )
                continue
            response_id, pulse = response
            if response_id != joint_id:
                continue
            degrees = self.pulse_to_degrees(index, pulse)
            self.positions[index] = self.degrees_to_ros(index, degrees)
            updated = True
            self.publish_joint_states()
            if self.inter_read_delay_sec > 0.0:
                time.sleep(self.inter_read_delay_sec)
        if updated:
            self.publish_joint_states()

    def publish_joint_states(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.joint_names)
        message.position = list(self.positions)
        self.publisher.publish(message)

    def destroy_node(self):
        try:
            self.serial.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ArmJointStateReader()
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
