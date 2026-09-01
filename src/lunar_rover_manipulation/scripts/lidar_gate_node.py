#!/usr/bin/env python3
"""Freeze FAST-LIO input safely while the arm occludes the Livox point cloud."""

import copy

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


class LidarGate(Node):
    def __init__(self):
        super().__init__("grasp_lidar_gate")
        cloud_input = self.declare_parameter(
            "cloud_input_topic", "/livox/lidar_raw"
        ).value
        cloud_output = self.declare_parameter(
            "cloud_output_topic", "/livox/lidar"
        ).value
        imu_input = self.declare_parameter("imu_input_topic", "/livox/imu_raw").value
        imu_output = self.declare_parameter("imu_output_topic", "/livox/imu").value
        active_topic = self.declare_parameter(
            "grasp_active_topic", "/grasp/active"
        ).value
        self.hold_imu_during_freeze = bool(
            self.declare_parameter("hold_imu_during_freeze", True).value
        )

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.frozen = False
        self.grasp_active = False
        self.search_active = False
        self.gravity_baseline = None
        self.cloud_publisher = self.create_publisher(CustomMsg, cloud_output, sensor_qos)
        self.imu_publisher = self.create_publisher(Imu, imu_output, sensor_qos)
        self.create_subscription(CustomMsg, cloud_input, self.on_cloud, sensor_qos)
        self.create_subscription(Imu, imu_input, self.on_imu, sensor_qos)
        self.create_subscription(Bool, active_topic, self.on_grasp_active, state_qos)
        self.create_subscription(Bool, "/search/active", self.on_search_active, state_qos)
        self.get_logger().info(
            f"Livox gate: {cloud_input} -> {cloud_output}, {imu_input} -> {imu_output}"
        )

    def on_grasp_active(self, message):
        self.grasp_active = bool(message.data)
        self.update_frozen()

    def on_search_active(self, message):
        self.search_active = bool(message.data)
        self.update_frozen()

    def update_frozen(self):
        frozen = self.grasp_active or self.search_active
        if frozen == self.frozen:
            return
        self.frozen = frozen
        self.get_logger().info(
            "FAST-LIO input frozen at current pose"
            if frozen
            else "FAST-LIO input resumed"
        )

    def on_cloud(self, message):
        if not self.frozen:
            self.cloud_publisher.publish(message)

    def on_imu(self, message):
        if not self.frozen:
            acceleration = message.linear_acceleration
            sample = (acceleration.x, acceleration.y, acceleration.z)
            if self.gravity_baseline is None:
                self.gravity_baseline = sample
            else:
                alpha = 0.02
                self.gravity_baseline = tuple(
                    (1.0 - alpha) * old + alpha * new
                    for old, new in zip(self.gravity_baseline, sample)
                )
            self.imu_publisher.publish(message)
            return
        if not self.hold_imu_during_freeze or self.gravity_baseline is None:
            return
        held = copy.deepcopy(message)
        held.angular_velocity.x = 0.0
        held.angular_velocity.y = 0.0
        held.angular_velocity.z = 0.0
        (
            held.linear_acceleration.x,
            held.linear_acceleration.y,
            held.linear_acceleration.z,
        ) = self.gravity_baseline
        self.imu_publisher.publish(held)


def main():
    rclpy.init()
    node = LidarGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
