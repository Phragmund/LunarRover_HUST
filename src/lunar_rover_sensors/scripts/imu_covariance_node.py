#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _covariance_from_diagonal(diagonal):
    covariance = [0.0] * 9
    for index, value in enumerate(diagonal):
        covariance[index * 3 + index] = float(value)
    return covariance


class ImuCovarianceNode(Node):
    def __init__(self):
        super().__init__("imu_covariance_node")
        self.declare_parameter("input_topic", "/livox/imu")
        self.declare_parameter("output_topic", "/livox/imu/sanitized")
        self.declare_parameter("orientation_covariance_diagonal", [0.01, 0.01, 0.02])
        self.declare_parameter("angular_velocity_covariance_diagonal", [0.0025, 0.0025, 0.0025])
        self.declare_parameter("linear_acceleration_covariance_diagonal", [0.04, 0.04, 0.04])

        self.orientation_covariance = _covariance_from_diagonal(
            self.get_parameter("orientation_covariance_diagonal").value
        )
        self.angular_velocity_covariance = _covariance_from_diagonal(
            self.get_parameter("angular_velocity_covariance_diagonal").value
        )
        self.linear_acceleration_covariance = _covariance_from_diagonal(
            self.get_parameter("linear_acceleration_covariance_diagonal").value
        )

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.publisher = self.create_publisher(Imu, output_topic, 50)
        self.subscription = self.create_subscription(Imu, input_topic, self._callback, 50)

    def _callback(self, message):
        output = Imu()
        output.header = message.header
        output.orientation = message.orientation
        output.angular_velocity = message.angular_velocity
        output.linear_acceleration = message.linear_acceleration

        if not self._finite_quaternion(output.orientation):
            self.get_logger().warn("Dropping IMU message with non-finite orientation")
            return

        output.orientation_covariance = self.orientation_covariance
        output.angular_velocity_covariance = self.angular_velocity_covariance
        output.linear_acceleration_covariance = self.linear_acceleration_covariance
        self.publisher.publish(output)

    @staticmethod
    def _finite_quaternion(quaternion):
        return all(
            math.isfinite(value)
            for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
