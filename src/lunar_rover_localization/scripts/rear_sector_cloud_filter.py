#!/usr/bin/env python3
"""Keep only the rover-forward 190-degree Livox sector for real FAST-LIO.

The Mid-360 is pitched relative to the rover.  Classification is consequently
done on point directions transformed into ``base_link``.  The outgoing cloud
keeps the original Livox frame, fields, point order, and timestamps so the
FAST-LIO MID360 PointCloud2 input handler stays unchanged.
"""

import math
import struct

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


def field_offset(fields, name):
    for field in fields:
        if field.name == name:
            return field.offset, field.datatype
    return None, None


def rotation_matrix(q):
    """Return the 3x3 matrix represented by a geometry_msgs quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


class RearSectorCloudFilter(Node):
    def __init__(self):
        super().__init__("rear_sector_cloud_filter")
        self.input_topic = self.declare_parameter("input_topic", "/livox/lidar").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/livox/lidar_front190"
        ).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        # ±95° around rover +X retains front 180° plus 5° on each side.
        self.front_half_angle_deg = float(
            self.declare_parameter("front_half_angle_deg", 95.0).value
        )
        if not 0.0 < self.front_half_angle_deg <= 180.0:
            raise ValueError("front_half_angle_deg must be in (0, 180]")
        self.cos_limit = math.cos(math.radians(self.front_half_angle_deg))
        self.rotation = None
        self.source_frame = None
        self.warned_layout = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, self.input_topic, self.on_cloud, qos_profile_sensor_data
        )
        self.get_logger().info(
            "Rear-sector filter: %s -> %s; retaining forward ±%.1f° "
            "(blocking rear %.1f°)"
            % (
                self.input_topic,
                self.output_topic,
                self.front_half_angle_deg,
                360.0 - 2.0 * self.front_half_angle_deg,
            )
        )

    def initialize_rotation(self, source_frame):
        if self.rotation is not None and self.source_frame == source_frame:
            return True
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, source_frame, Time(), Duration(seconds=0.2)
            )
        except TransformException as exc:
            self.get_logger().warn(
                "Waiting for %s <- %s before sector filtering: %s"
                % (self.base_frame, source_frame, exc),
                throttle_duration_sec=2.0,
            )
            return False
        self.rotation = rotation_matrix(transform.transform.rotation)
        if self.rotation is None:
            self.get_logger().error("Invalid lidar-to-base quaternion; dropping cloud")
            return False
        self.source_frame = source_frame
        self.get_logger().info(
            "Using TF %s <- %s for forward/rear sector classification"
            % (self.base_frame, source_frame)
        )
        return True

    def on_cloud(self, msg):
        if not msg.header.frame_id or not self.initialize_rotation(msg.header.frame_id):
            return
        if msg.is_bigendian:
            self.get_logger().error("Big-endian PointCloud2 is unsupported")
            return
        xyz = [field_offset(msg.fields, name) for name in ("x", "y", "z")]
        if any(offset is None or datatype != PointField.FLOAT32 for offset, datatype in xyz):
            if not self.warned_layout:
                self.get_logger().error("PointCloud2 must contain float32 x/y/z fields")
                self.warned_layout = True
            return

        x_offset, y_offset, z_offset = (offset for offset, _ in xyz)
        row_x, row_y, _ = self.rotation
        raw = bytes(msg.data)
        retained = bytearray()
        for index in range(msg.width * msg.height):
            start = index * msg.point_step
            x = struct.unpack_from("<f", raw, start + x_offset)[0]
            y = struct.unpack_from("<f", raw, start + y_offset)[0]
            z = struct.unpack_from("<f", raw, start + z_offset)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            # A ray direction is rotated but not translated into base_link.
            base_x = row_x[0] * x + row_x[1] * y + row_x[2] * z
            base_y = row_y[0] * x + row_y[1] * y + row_y[2] * z
            horizontal_norm = math.hypot(base_x, base_y)
            # Keep vertical rays: they have no meaningful horizontal azimuth.
            if horizontal_norm <= 1.0e-6 or base_x / horizontal_norm >= self.cos_limit:
                retained.extend(raw[start : start + msg.point_step])

        output = PointCloud2()
        output.header = msg.header
        output.height = 1
        output.width = len(retained) // msg.point_step
        output.fields = msg.fields
        output.is_bigendian = msg.is_bigendian
        output.point_step = msg.point_step
        output.row_step = output.width * output.point_step
        output.data = bytes(retained)
        output.is_dense = msg.is_dense
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = RearSectorCloudFilter()
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
