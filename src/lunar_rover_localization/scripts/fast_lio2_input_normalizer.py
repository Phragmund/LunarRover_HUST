#!/usr/bin/env python3
"""Normalize simulated MID360 LiDAR/IMU samples into one FAST-LIO body frame."""

from copy import deepcopy
import math
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField
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
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / norm for v in q)


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
    if axis_norm <= 1e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(axis_norm, max(-1.0, min(1.0, delta[3])))
    return tuple(value * angle / (axis_norm * dt) for value in axis)


def field_offset(fields, name):
    for field in fields:
        if field.name == name:
            return field.offset, field.datatype
    return None, None


class FastLio2InputNormalizer(Node):
    def __init__(self):
        super().__init__("fast_lio2_input_normalizer")
        self.target_frame = self.declare_parameter("target_frame", "base_link").value
        self.lidar_frame = self.declare_parameter("lidar_frame", "livox_frame").value
        self.imu_frame = self.declare_parameter("imu_frame", "livox_imu_frame").value
        self.points_in = self.declare_parameter("points_in", "/livox/points").value
        self.points_out = self.declare_parameter(
            "points_out", "/livox/points_fast_lio"
        ).value
        self.imu_in = self.declare_parameter("imu_in", "/livox/imu").value
        self.imu_out = self.declare_parameter("imu_out", "/livox/imu_fast_lio").value
        self.gravity = float(self.declare_parameter("gravity_mps2", 9.80665).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_from_lidar = None
        self.target_from_imu = None
        self.previous_imu_stamp = None
        self.previous_orientation = None
        self.warned_bigendian = False
        self.warned_fields = False

        self.points_pub = self.create_publisher(PointCloud2, self.points_out, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_out, 100)
        self.create_subscription(PointCloud2, self.points_in, self.on_points, 10)
        self.create_subscription(Imu, self.imu_in, self.on_imu, 100)
        self.create_timer(0.1, self.try_initialize)

        self.get_logger().info(
            f"FAST-LIO input normalizer: {self.points_in}, {self.imu_in} -> "
            f"{self.points_out}, {self.imu_out} in {self.target_frame}"
        )

    def try_initialize(self):
        if self.target_from_lidar is not None and self.target_from_imu is not None:
            return
        try:
            lidar_tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.lidar_frame, Time(), Duration(seconds=0.2)
            )
            imu_tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.imu_frame, Time(), Duration(seconds=0.2)
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for static sensor transforms into {self.target_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        self.target_from_lidar = (
            (
                lidar_tf.transform.translation.x,
                lidar_tf.transform.translation.y,
                lidar_tf.transform.translation.z,
            ),
            q_tuple(lidar_tf.transform.rotation),
        )
        self.target_from_imu = (
            (
                imu_tf.transform.translation.x,
                imu_tf.transform.translation.y,
                imu_tf.transform.translation.z,
            ),
            q_tuple(imu_tf.transform.rotation),
        )
        self.get_logger().info(
            f"FAST-LIO input normalizer using {self.target_frame}<-{self.lidar_frame} "
            f"and {self.target_frame}<-{self.imu_frame}"
        )

    def initialized(self):
        return self.target_from_lidar is not None and self.target_from_imu is not None

    def on_imu(self, msg):
        if not self.initialized():
            return
        out = deepcopy(msg)
        out.header.frame_id = self.target_frame

        current_orientation = q_normalize(q_tuple(msg.orientation))
        current_stamp = Time.from_msg(msg.header.stamp)
        gyro = (0.0, 0.0, 0.0)
        if self.previous_imu_stamp is not None and self.previous_orientation is not None:
            dt = (current_stamp - self.previous_imu_stamp).nanoseconds * 1.0e-9
            gyro = angular_velocity_from_delta(
                self.previous_orientation, current_orientation, dt
            )
        self.previous_imu_stamp = current_stamp
        self.previous_orientation = current_orientation

        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = gyro
        (
            out.linear_acceleration.x,
            out.linear_acceleration.y,
            out.linear_acceleration.z,
        ) = rotate(
            q_conjugate(current_orientation),
            (0.0, 0.0, self.gravity),
        )
        self.imu_pub.publish(out)

    def on_points(self, msg):
        if not self.initialized():
            return
        if msg.is_bigendian:
            if not self.warned_bigendian:
                self.get_logger().error("Big-endian PointCloud2 is not supported")
                self.warned_bigendian = True
            return

        offsets = [field_offset(msg.fields, name) for name in ("x", "y", "z")]
        if any(
            offset is None or datatype != PointField.FLOAT32
            for offset, datatype in offsets
        ):
            if not self.warned_fields:
                self.get_logger().error("PointCloud2 must contain float32 x/y/z fields")
                self.warned_fields = True
            return

        (tx, ty, tz), q = self.target_from_lidar
        ox, oy, oz = (offset for offset, _ in offsets)
        out = deepcopy(msg)
        out.header.frame_id = self.target_frame
        data = bytearray(out.data)
        for index in range(msg.width * msg.height):
            base = index * msg.point_step
            point = (
                struct.unpack_from("<f", data, base + ox)[0],
                struct.unpack_from("<f", data, base + oy)[0],
                struct.unpack_from("<f", data, base + oz)[0],
            )
            x, y, z = rotate(q, point)
            struct.pack_into("<f", data, base + ox, x + tx)
            struct.pack_into("<f", data, base + oy, y + ty)
            struct.pack_into("<f", data, base + oz, z + tz)
        out.data = bytes(data)
        self.points_pub.publish(out)


def main():
    rclpy.init()
    node = FastLio2InputNormalizer()
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
