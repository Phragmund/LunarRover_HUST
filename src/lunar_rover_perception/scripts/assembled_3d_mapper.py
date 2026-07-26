#!/usr/bin/env python3
"""Accumulate lidar clouds into a voxelized 3D map for SLAM visualization tests."""

import math
from typing import Dict, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import tf2_ros


class Assembled3DMapper(Node):
    def __init__(self):
        super().__init__("assembled_3d_mapper")
        self.declare_parameter("input_topic", "/livox/points")
        self.declare_parameter("output_topic", "/mapping/assembled_3d_map")
        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("voxel_size", 0.08)
        self.declare_parameter("min_range", 0.30)
        self.declare_parameter("max_range", 14.0)
        self.declare_parameter("min_z", -0.8)
        self.declare_parameter("max_z", 2.5)
        self.declare_parameter("publish_every_n_scans", 2)
        self.declare_parameter("max_voxels", 300000)
        self.declare_parameter("stride", 2)

        self.map_frame = self.get_parameter("map_frame").value
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)
        self.publish_every_n_scans = int(
            self.get_parameter("publish_every_n_scans").value
        )
        self.max_voxels = int(self.get_parameter("max_voxels").value)
        self.stride = max(1, int(self.get_parameter("stride").value))
        if self.voxel_size <= 0.0:
            raise ValueError("voxel_size must be positive")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        output_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        sensor_qos = QoSProfile(
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, output_qos
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("input_topic").value,
            self.cloud_callback,
            sensor_qos,
        )
        self.voxels: Dict[Tuple[int, int, int], Tuple[float, float, float]] = {}
        self.scan_count = 0
        self.last_log_count = 0
        self.get_logger().info(
            f"Accumulating {self.get_parameter('input_topic').value} into "
            f"{self.get_parameter('output_topic').value} in {self.map_frame} "
            f"at {self.voxel_size:.2f} m"
        )

    def cloud_callback(self, msg: PointCloud2):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Cannot transform {msg.header.frame_id} to {self.map_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        translation = transform.transform.translation
        rotation = self._rotation_matrix(transform.transform.rotation)

        points = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        selected = []
        for i, point in enumerate(points):
            if i % self.stride:
                continue
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            radius = math.hypot(x, y)
            if radius < self.min_range or radius > self.max_range:
                continue
            mx, my, mz = self._transform_point(
                rotation, translation.x, translation.y, translation.z, x, y, z
            )
            if mz < self.min_z or mz > self.max_z:
                continue
            selected.append((mx, my, mz))

        if not selected:
            return

        scale = 1.0 / self.voxel_size
        for x, y, z in selected:
            key = (int(round(x * scale)), int(round(y * scale)), int(round(z * scale)))
            self.voxels[key] = (x, y, z)

        if len(self.voxels) > self.max_voxels:
            keep = list(self.voxels.items())[-self.max_voxels :]
            self.voxels = dict(keep)

        self.scan_count += 1
        if self.scan_count % self.publish_every_n_scans == 0:
            self.publish_map(msg.header.stamp)

    def publish_map(self, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = self.map_frame
        cloud = point_cloud2.create_cloud_xyz32(header, list(self.voxels.values()))
        self.publisher.publish(cloud)
        count = len(self.voxels)
        if count - self.last_log_count >= 1000 or count < 1000:
            self.get_logger().info(f"assembled_3d_map voxels={count}")
            self.last_log_count = count

    @staticmethod
    def _rotation_matrix(q):
        x, y, z, w = q.x, q.y, q.z, q.w
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return np.identity(3)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _transform_point(rotation, tx, ty, tz, x, y, z):
        p = rotation @ np.array([x, y, z], dtype=np.float64)
        return float(p[0] + tx), float(p[1] + ty), float(p[2] + tz)


def main():
    rclpy.init()
    node = Assembled3DMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
