#!/usr/bin/env python3
"""Prepare IMU messages for robot_localization.

The simulator publishes zero covariance matrices. robot_localization can ingest
them, but that silently turns into near-perfect measurements. This node keeps
the original IMU frame_id so TF can handle the sensor mounting offset, and fills
conservative covariance values before the EKF consumes the data.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def covariance_is_all_zero(values):
    return all(abs(value) < 1e-12 for value in values)


def diagonal_covariance(x, y, z):
    return [x, 0.0, 0.0, 0.0, y, 0.0, 0.0, 0.0, z]


class ImuEkfPreprocessor(Node):
    def __init__(self):
        super().__init__("imu_ekf_preprocessor")
        self.input_topic = self.declare_parameter("input_topic", "/livox/imu").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/localization/imu_for_ekf"
        ).value
        self.orientation_variance = float(
            self.declare_parameter("orientation_variance", 0.0025).value
        )
        self.angular_velocity_variance = float(
            self.declare_parameter("angular_velocity_variance", 0.0025).value
        )
        self.linear_acceleration_variance = float(
            self.declare_parameter("linear_acceleration_variance", 0.25).value
        )

        self.publisher = self.create_publisher(Imu, self.output_topic, 50)
        self.create_subscription(Imu, self.input_topic, self.on_imu, 100)
        self.get_logger().info(
            f"Preparing IMU for EKF: {self.input_topic} -> {self.output_topic}; "
            "frame_id is preserved for TF mounting correction"
        )

    def on_imu(self, msg):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        if msg.orientation_covariance[0] == -1.0:
            out.orientation_covariance = msg.orientation_covariance
        elif covariance_is_all_zero(msg.orientation_covariance):
            out.orientation_covariance = diagonal_covariance(
                self.orientation_variance,
                self.orientation_variance,
                self.orientation_variance,
            )
        else:
            out.orientation_covariance = msg.orientation_covariance

        if covariance_is_all_zero(msg.angular_velocity_covariance):
            out.angular_velocity_covariance = diagonal_covariance(
                self.angular_velocity_variance,
                self.angular_velocity_variance,
                self.angular_velocity_variance,
            )
        else:
            out.angular_velocity_covariance = msg.angular_velocity_covariance

        if covariance_is_all_zero(msg.linear_acceleration_covariance):
            out.linear_acceleration_covariance = diagonal_covariance(
                self.linear_acceleration_variance,
                self.linear_acceleration_variance,
                self.linear_acceleration_variance,
            )
        else:
            out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.publisher.publish(out)


def main():
    rclpy.init()
    node = ImuEkfPreprocessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
