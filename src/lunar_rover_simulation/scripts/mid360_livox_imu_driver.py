#!/usr/bin/env python3
"""Publish a FAST-LIO compatible MID360 IMU stream for simulation.

Gazebo's raw IMU topic provides useful high-rate sensor timestamps in this
project, but its acceleration/orientation values are not physically consistent
under the MID360 tilted mounting. This node keeps the raw IMU timing and
synthesizes the IMU values from simulator kinematics in livox_imu_frame.
"""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener


def q_tuple(msg):
    return (float(msg.x), float(msg.y), float(msg.z), float(msg.w))


def q_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def rotate(q, v):
    q = q_normalize(q)
    return q_multiply(q_multiply(q, (v[0], v[1], v[2], 0.0)), q_conjugate(q))[:3]


def angular_velocity_from_delta(previous_q, current_q, dt):
    if dt <= 0.0:
        return (0.0, 0.0, 0.0)
    delta = q_normalize(q_multiply(q_conjugate(previous_q), current_q))
    if delta[3] < 0.0:
        delta = tuple(-value for value in delta)
    axis = delta[:3]
    axis_norm = math.sqrt(sum(value * value for value in axis))
    if axis_norm <= 1.0e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(axis_norm, max(-1.0, min(1.0, delta[3])))
    return tuple(value * angle / (axis_norm * dt) for value in axis)


def set_quaternion(out, q):
    out.x, out.y, out.z, out.w = q


class Mid360LivoxImuDriver(Node):
    def __init__(self):
        super().__init__("mid360_livox_imu_driver")
        self.raw_imu_topic = self.declare_parameter("raw_imu_topic", "/mid360/raw_imu").value
        self.truth_odom_topic = self.declare_parameter(
            "truth_odom_topic", "/simulation/ground_truth/odometry"
        ).value
        self.imu_topic = self.declare_parameter("imu_topic", "/livox/imu").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.imu_frame = self.declare_parameter("imu_frame", "livox_imu_frame").value
        self.gravity = float(self.declare_parameter("gravity_mps2", 9.80665).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_to_imu_q = None
        self.previous_truth_stamp = None
        self.previous_truth_velocity = None
        self.previous_imu_orientation = None
        self.latest_imu_orientation = None
        self.latest_gyro = (0.0, 0.0, 0.0)
        self.latest_specific_force = (0.0, 0.0, 0.0)
        self.last_publish_stamp = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 100)
        self.create_subscription(Odometry, self.truth_odom_topic, self.on_truth_odom, qos)
        self.create_subscription(Imu, self.raw_imu_topic, self.on_raw_imu_clock, qos)
        self.create_timer(0.2, self.try_initialize_tf)
        self.get_logger().info(
            "MID360 Livox IMU driver: "
            f"{self.raw_imu_topic} timestamps + {self.truth_odom_topic} kinematics -> {self.imu_topic}"
        )

    def try_initialize_tf(self):
        if self.base_to_imu_q is not None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.imu_frame, Time(), Duration(seconds=0.1)
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for {self.base_frame}->{self.imu_frame} TF: {exc}",
                throttle_duration_sec=2.0,
            )
            return
        self.base_to_imu_q = q_normalize(q_tuple(transform.transform.rotation))
        self.get_logger().info(f"Using URDF TF {self.base_frame}->{self.imu_frame} for IMU synthesis")

    def on_truth_odom(self, msg):
        if self.base_to_imu_q is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        if self.previous_truth_stamp is not None and stamp <= self.previous_truth_stamp:
            return

        current_velocity = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z),
        )
        world_to_base_q = q_normalize(q_tuple(msg.pose.pose.orientation))
        world_to_imu_q = q_normalize(q_multiply(world_to_base_q, self.base_to_imu_q))

        dt = 0.0
        if self.previous_truth_stamp is not None:
            dt = (stamp - self.previous_truth_stamp).nanoseconds * 1.0e-9

        gyro = (0.0, 0.0, 0.0)
        acceleration_world = (0.0, 0.0, 0.0)
        if dt > 1.0e-4 and self.previous_imu_orientation is not None:
            gyro = angular_velocity_from_delta(
                self.previous_imu_orientation, world_to_imu_q, dt
            )
            if self.previous_truth_velocity is not None:
                acceleration_world = tuple(
                    (current_velocity[i] - self.previous_truth_velocity[i]) / dt
                    for i in range(3)
                )

        specific_force_world = (
            acceleration_world[0],
            acceleration_world[1],
            acceleration_world[2] + self.gravity,
        )
        self.latest_specific_force = rotate(q_conjugate(world_to_imu_q), specific_force_world)
        self.latest_imu_orientation = world_to_imu_q
        self.latest_gyro = gyro
        self.previous_truth_stamp = stamp
        self.previous_truth_velocity = current_velocity
        self.previous_imu_orientation = world_to_imu_q

    def on_raw_imu_clock(self, msg):
        if self.latest_imu_orientation is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_publish_stamp is not None and stamp <= self.last_publish_stamp:
            return

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.imu_frame
        set_quaternion(out.orientation, self.latest_imu_orientation)
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = self.latest_gyro
        (
            out.linear_acceleration.x,
            out.linear_acceleration.y,
            out.linear_acceleration.z,
        ) = self.latest_specific_force
        self.imu_pub.publish(out)
        self.last_publish_stamp = stamp


def main():
    rclpy.init()
    node = Mid360LivoxImuDriver()
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
