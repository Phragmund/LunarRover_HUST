#!/usr/bin/env python3
"""Publish a FAST-LIO-friendly IMU stream for Gazebo simulation.

Gazebo's IMU output in this model can report angular velocity and acceleration
that are inconsistent with its orientation. FAST-LIO only consumes gyro and
accelerometer channels, so for simulation we derive those channels from the
ground-truth pose orientation and the URDF mounting transform.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener


def q_normalize(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def q_inverse(q):
    x, y, z, w = q_normalize(q)
    return (-x, -y, -z, w)


def q_multiply_raw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_multiply(a, b):
    return q_normalize(q_multiply_raw(a, b))


def rotate(q, v):
    vx, vy, vz = v
    x, y, z, _ = q_multiply_raw(
        q_multiply_raw(q_normalize(q), (vx, vy, vz, 0.0)), q_inverse(q)
    )
    return (x, y, z)


def q_tuple(q):
    return (float(q.x), float(q.y), float(q.z), float(q.w))


def stamp_seconds(stamp):
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def diagonal_covariance(x, y, z):
    return [x, 0.0, 0.0, 0.0, y, 0.0, 0.0, 0.0, z]


class SimFastLioImuAdapter(Node):
    def __init__(self):
        super().__init__("sim_fast_lio_imu_adapter")
        self.input_topic = self.declare_parameter(
            "input_topic", "/simulation/ground_truth/odometry"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/localization/imu_for_fast_lio"
        ).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.imu_frame = self.declare_parameter("imu_frame", "livox_imu_frame").value
        self.gravity = float(self.declare_parameter("gravity_mps2", 9.80665).value)
        self.max_dt = float(self.declare_parameter("max_dt", 0.1).value)
        self.max_gyro = float(self.declare_parameter("max_gyro_radps", 4.0).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_from_imu = None
        self.prev_stamp = None
        self.prev_world_from_imu = None

        self.publisher = self.create_publisher(Imu, self.output_topic, 100)
        self.create_subscription(Odometry, self.input_topic, self.on_odom, 100)
        self.get_logger().info(
            f"Simulation FAST-LIO IMU adapter: {self.input_topic} -> {self.output_topic}"
        )

    def ensure_transform(self):
        if self.base_from_imu is not None:
            return True
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.imu_frame, Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for TF {self.base_frame} <- {self.imu_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return False
        self.base_from_imu = q_tuple(transform.transform.rotation)
        self.get_logger().info(
            f"Using URDF IMU mounting transform {self.base_frame} <- {self.imu_frame}"
        )
        return True

    def angular_velocity(self, stamp, world_from_imu):
        if self.prev_stamp is None or self.prev_world_from_imu is None:
            self.prev_stamp = stamp
            self.prev_world_from_imu = world_from_imu
            return (0.0, 0.0, 0.0)

        now = stamp_seconds(stamp)
        prev = stamp_seconds(self.prev_stamp)
        dt = now - prev
        if dt <= 0.0 or (self.max_dt > 0.0 and dt > self.max_dt):
            self.prev_stamp = stamp
            self.prev_world_from_imu = world_from_imu
            return (0.0, 0.0, 0.0)

        delta = q_multiply(q_inverse(self.prev_world_from_imu), world_from_imu)
        if delta[3] < 0.0:
            delta = (-delta[0], -delta[1], -delta[2], -delta[3])
        vec_norm = math.sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2])
        if vec_norm < 1e-12:
            gyro = (0.0, 0.0, 0.0)
        else:
            angle = 2.0 * math.atan2(vec_norm, max(-1.0, min(1.0, delta[3])))
            scale = angle / (vec_norm * dt)
            gyro = (delta[0] * scale, delta[1] * scale, delta[2] * scale)

        if self.max_gyro > 0.0:
            gyro = tuple(max(-self.max_gyro, min(self.max_gyro, value)) for value in gyro)

        self.prev_stamp = stamp
        self.prev_world_from_imu = world_from_imu
        return gyro

    def on_odom(self, msg):
        if not self.ensure_transform():
            return

        world_from_base = q_tuple(msg.pose.pose.orientation)
        world_from_imu = q_multiply(world_from_base, self.base_from_imu)
        gyro = self.angular_velocity(msg.header.stamp, world_from_imu)
        accel = rotate(q_inverse(world_from_imu), (0.0, 0.0, self.gravity))

        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.imu_frame
        out.orientation.x = world_from_imu[0]
        out.orientation.y = world_from_imu[1]
        out.orientation.z = world_from_imu[2]
        out.orientation.w = world_from_imu[3]
        out.angular_velocity.x = gyro[0]
        out.angular_velocity.y = gyro[1]
        out.angular_velocity.z = gyro[2]
        out.linear_acceleration.x = accel[0]
        out.linear_acceleration.y = accel[1]
        out.linear_acceleration.z = accel[2]
        out.orientation_covariance = diagonal_covariance(0.0025, 0.0025, 0.0025)
        out.angular_velocity_covariance = diagonal_covariance(0.0025, 0.0025, 0.0025)
        out.linear_acceleration_covariance = diagonal_covariance(0.04, 0.04, 0.04)
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = SimFastLioImuAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
