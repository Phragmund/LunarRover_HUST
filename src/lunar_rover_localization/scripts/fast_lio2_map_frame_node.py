#!/usr/bin/env python3
"""Publish FAST-LIO2 map products in a clean, level navigation map frame."""

import math
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def q_tuple(q):
    return (float(q.x), float(q.y), float(q.z), float(q.w))


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


def transform_parts(transform):
    t = transform.transform.translation
    return (float(t.x), float(t.y), float(t.z)), q_tuple(transform.transform.rotation)


def compose(a_t, a_q, b_t, b_q):
    rotated_b_t = rotate(a_q, b_t)
    return (
        (
            a_t[0] + rotated_b_t[0],
            a_t[1] + rotated_b_t[1],
            a_t[2] + rotated_b_t[2],
        ),
        q_multiply(a_q, b_q),
    )


def point_field_size(datatype):
    return {
        PointField.INT8: 1,
        PointField.UINT8: 1,
        PointField.INT16: 2,
        PointField.UINT16: 2,
        PointField.INT32: 4,
        PointField.UINT32: 4,
        PointField.FLOAT32: 4,
        PointField.FLOAT64: 8,
    }.get(datatype)


class FastLio2MapFrameNode(Node):
    def __init__(self):
        super().__init__("fast_lio2_map_frame")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.lio_body_frame = self.declare_parameter("lio_body_frame", "livox_imu_frame").value
        self.raw_lio_map_frame = self.declare_parameter(
            "raw_lio_map_frame", "camera_init"
        ).value
        self.raw_odom_topic = self.declare_parameter(
            "raw_odom_topic", "/localization/lio_odometry_raw"
        ).value
        self.raw_path_topic = self.declare_parameter(
            "raw_path_topic", "/localization/lio_path_raw"
        ).value
        self.raw_registered_topic = self.declare_parameter(
            "raw_registered_topic", "/localization/cloud_registered_raw"
        ).value
        self.raw_map_cloud_topic = self.declare_parameter(
            "raw_map_cloud_topic", "/localization/lio_map_cloud_raw"
        ).value
        self.odom_topic = self.declare_parameter(
            "odom_topic", "/localization/lio_odometry"
        ).value
        self.path_topic = self.declare_parameter("path_topic", "/localization/lio_path").value
        self.registered_topic = self.declare_parameter(
            "registered_topic", "/localization/cloud_registered"
        ).value
        self.map_cloud_topic = self.declare_parameter(
            "map_cloud_topic", "/localization/lio_map_cloud"
        ).value
        self.restamp_outputs = bool(self.declare_parameter("restamp_outputs", True).value)
        self.max_abs_position = float(
            self.declare_parameter("max_abs_position", 1000.0).value
        )
        self.require_valid_pose_for_clouds = bool(
            self.declare_parameter("require_valid_pose_for_clouds", True).value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_from_lio = None
        self.lio_body_from_base = None
        self.last_pose_valid = False
        self.wait_timer = self.create_timer(0.1, self.try_initialize)
        self.cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.registered_pub = self.create_publisher(
            PointCloud2, self.registered_topic, self.cloud_qos
        )
        self.map_cloud_pub = self.create_publisher(
            PointCloud2, self.map_cloud_topic, self.cloud_qos
        )

        self.create_subscription(Odometry, self.raw_odom_topic, self.on_odom, 20)
        self.create_subscription(Path, self.raw_path_topic, self.on_path, 10)
        self.create_subscription(
            PointCloud2,
            self.raw_registered_topic,
            self.on_registered_cloud,
            self.cloud_qos,
        )
        self.create_subscription(
            PointCloud2,
            self.raw_map_cloud_topic,
            self.on_map_cloud,
            self.cloud_qos,
        )

        self.get_logger().info(
            "FAST-LIO2 map frame: raw camera_init products -> level map products"
        )

    def try_initialize(self):
        if self.map_from_lio is not None:
            return
        try:
            base_from_lio_body = self.tf_buffer.lookup_transform(
                self.base_frame, self.lio_body_frame, Time()
            )
            lio_body_from_base = self.tf_buffer.lookup_transform(
                self.lio_body_frame, self.base_frame, Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for static {self.base_frame}<->{self.lio_body_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        self.map_from_lio = transform_parts(base_from_lio_body)
        self.lio_body_from_base = transform_parts(lio_body_from_base)
        _, q = self.map_from_lio
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (q[3] * q[1] - q[2] * q[0])))))
        self.get_logger().info(
            f"Using URDF-derived {self.base_frame}->{self.lio_body_frame} as "
            f"{self.map_frame}->{self.raw_lio_map_frame}; pitch={pitch:.3f} deg"
        )
        self.wait_timer.cancel()

    def initialized(self):
        if self.map_from_lio is None or self.lio_body_from_base is None:
            self.get_logger().warn("FAST-LIO2 map frame is not initialized", throttle_duration_sec=2.0)
            return False
        return True

    def raw_pose_to_map_base(self, position, orientation):
        raw_t = (float(position.x), float(position.y), float(position.z))
        raw_q = q_tuple(orientation)
        map_body = compose(*self.map_from_lio, raw_t, raw_q)
        return compose(*map_body, *self.lio_body_from_base)

    def pose_is_valid(self, t, q):
        values = (*t, *q)
        if not all(math.isfinite(value) for value in values):
            return False
        if self.max_abs_position > 0.0 and max(abs(t[0]), abs(t[1]), abs(t[2])) > self.max_abs_position:
            return False
        q_norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        return math.isfinite(q_norm) and 0.5 <= q_norm <= 1.5

    def fill_pose(self, pose, t, q):
        pose.position.x = t[0]
        pose.position.y = t[1]
        pose.position.z = t[2]
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

    def on_odom(self, msg):
        if not self.initialized():
            return
        out = deepcopy(msg)
        out.header.frame_id = self.map_frame
        if self.restamp_outputs:
            out.header.stamp = self.get_clock().now().to_msg()
        out.child_frame_id = self.base_frame
        t, q = self.raw_pose_to_map_base(msg.pose.pose.position, msg.pose.pose.orientation)
        if not self.pose_is_valid(t, q):
            self.last_pose_valid = False
            self.get_logger().warn(
                "Dropping invalid FAST-LIO2 pose before publishing map-frame odometry: "
                f"position=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f})",
                throttle_duration_sec=2.0,
            )
            return
        self.last_pose_valid = True
        self.fill_pose(out.pose.pose, t, q)
        self.odom_pub.publish(out)

    def on_path(self, msg):
        if not self.initialized():
            return
        out = Path()
        out.header = msg.header
        out.header.frame_id = self.map_frame
        if self.restamp_outputs:
            out.header.stamp = self.get_clock().now().to_msg()
        for pose_stamped in msg.poses:
            pose_out = PoseStamped()
            pose_out.header = pose_stamped.header
            pose_out.header.frame_id = self.map_frame
            if self.restamp_outputs:
                pose_out.header.stamp = out.header.stamp
            t, q = self.raw_pose_to_map_base(
                pose_stamped.pose.position, pose_stamped.pose.orientation
            )
            self.fill_pose(pose_out.pose, t, q)
            out.poses.append(pose_out)
        self.path_pub.publish(out)

    def transform_cloud(self, msg):
        if not self.initialized():
            return None
        if self.require_valid_pose_for_clouds and not self.last_pose_valid:
            self.get_logger().warn(
                "Dropping FAST-LIO2 cloud because the latest LIO pose is invalid",
                throttle_duration_sec=2.0,
            )
            return None
        selected_fields = []
        compact_fields = []
        offset = 0
        for field in sorted(msg.fields, key=lambda candidate: candidate.offset):
            size = point_field_size(field.datatype)
            if size is None:
                continue
            compact = PointField()
            compact.name = field.name
            compact.offset = offset
            compact.datatype = field.datatype
            compact.count = field.count
            selected_fields.append(field)
            compact_fields.append(compact)
            offset += size * field.count

        names = [field.name for field in selected_fields]
        try:
            x_index = names.index("x")
            y_index = names.index("y")
            z_index = names.index("z")
        except ValueError:
            self.get_logger().warn("PointCloud2 has no x/y/z fields", throttle_duration_sec=2.0)
            return None

        map_t, map_q = self.map_from_lio
        points = []
        for point in point_cloud2.read_points(msg, field_names=names, skip_nans=True):
            values = list(point)
            x, y, z = rotate(map_q, (values[x_index], values[y_index], values[z_index]))
            values[x_index] = x + map_t[0]
            values[y_index] = y + map_t[1]
            values[z_index] = z + map_t[2]
            points.append(tuple(values))

        header = deepcopy(msg.header)
        header.frame_id = self.map_frame
        if self.restamp_outputs:
            header.stamp = self.get_clock().now().to_msg()
        return point_cloud2.create_cloud(header, compact_fields, points)

    def on_registered_cloud(self, msg):
        out = self.transform_cloud(msg)
        if out is not None:
            self.registered_pub.publish(out)

    def on_map_cloud(self, msg):
        out = self.transform_cloud(msg)
        if out is not None:
            self.map_cloud_pub.publish(out)


def main():
    rclpy.init()
    node = FastLio2MapFrameNode()
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
