#!/usr/bin/env python3
"""Publish map->odom from a map-frame LIO pose and local odometry."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


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


def pose_parts(odom):
    p = odom.pose.pose.position
    return (float(p.x), float(p.y), float(p.z)), q_tuple(odom.pose.pose.orientation)


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


def pose_as_base(odom, base_frame, tf_buffer, logger, source_name):
    pose_t, pose_q = pose_parts(odom)
    child_frame = odom.child_frame_id.strip()
    if not child_frame or child_frame == base_frame:
        return pose_t, pose_q

    try:
        child_base_tf = tf_buffer.lookup_transform(child_frame, base_frame, Time())
    except TransformException as exc:
        logger.warn(
            f"Cannot convert {source_name} child frame '{child_frame}' to "
            f"'{base_frame}': {exc}",
            throttle_duration_sec=2.0,
        )
        return None

    child_base_t, child_base_q = transform_parts(child_base_tf)
    return compose(pose_t, pose_q, child_base_t, child_base_q)


class MapOdomTfBridge(Node):
    def __init__(self):
        super().__init__("map_odom_tf_bridge")
        self.lio_topic = self.declare_parameter(
            "lio_topic", "/localization/lio_odometry"
        ).value
        self.local_odom_topic = self.declare_parameter(
            "local_odom_topic", "/localization/composite_odometry"
        ).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.max_stamp_delta = float(self.declare_parameter("max_stamp_delta", 0.0).value)
        self.max_lio_age = float(self.declare_parameter("max_lio_age", 0.75).value)
        self.max_local_age = float(self.declare_parameter("max_local_age", 0.75).value)
        self.max_abs_lio_position = float(
            self.declare_parameter("max_abs_lio_position", 1000.0).value
        )
        self.publish_period = float(self.declare_parameter("publish_period", 0.02).value)
        self.stamp_mode = (
            self.declare_parameter("stamp_mode", "now").value.strip().lower()
        )
        if self.stamp_mode not in ("now", "data"):
            self.get_logger().warn(
                f"Unsupported stamp_mode '{self.stamp_mode}', falling back to 'now'"
            )
            self.stamp_mode = "now"

        self.latest_lio = None
        self.latest_local = None
        self.cached_lio_stamp = None
        self.cached_transform_stamp = None
        self.cached_map_odom_t = None
        self.cached_map_odom_q = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.lio_topic, self.on_lio, 20)
        self.create_subscription(Odometry, self.local_odom_topic, self.on_local, 50)
        self.create_timer(self.publish_period, self.publish_transform)
        self.get_logger().info(
            f"map->odom bridge: {self.lio_topic} with {self.local_odom_topic}; "
            f"stamp_mode={self.stamp_mode}"
        )

    @staticmethod
    def stamp_seconds(msg):
        stamp = msg.header.stamp
        return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    def on_lio(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"LIO frame '{msg.header.frame_id}' != expected '{self.map_frame}'",
                throttle_duration_sec=2.0,
            )
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().warn("Ignoring non-finite LIO odometry", throttle_duration_sec=2.0)
            return
        if self.max_abs_lio_position > 0.0:
            max_abs = max(abs(float(p.x)), abs(float(p.y)), abs(float(p.z)))
            if max_abs > self.max_abs_lio_position:
                self.get_logger().warn(
                    "Ignoring out-of-range LIO odometry before map->odom update: "
                    f"position=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})",
                    throttle_duration_sec=2.0,
                )
                return
        self.latest_lio = msg

    def on_local(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.odom_frame:
            self.get_logger().warn(
                f"Local odom frame '{msg.header.frame_id}' != expected '{self.odom_frame}'",
                throttle_duration_sec=2.0,
            )
        self.latest_local = msg

    def publish_transform(self):
        if self.latest_lio is None or self.latest_local is None:
            return

        now_stamp = self.get_clock().now().to_msg()
        now_seconds = float(now_stamp.sec) + 1e-9 * float(now_stamp.nanosec)
        lio_stamp = self.stamp_seconds(self.latest_lio)
        local_stamp = self.stamp_seconds(self.latest_local)
        if self.max_stamp_delta > 0.0 and abs(lio_stamp - local_stamp) > self.max_stamp_delta:
            self.get_logger().warn(
                "LIO/local odom stamp delta "
                f"{abs(lio_stamp - local_stamp):.3f}s exceeds strict sync threshold; "
                "publishing latest fresh transform",
                throttle_duration_sec=2.0,
            )
        if self.max_lio_age > 0.0 and now_seconds > 0.0 and now_seconds - lio_stamp > self.max_lio_age:
            self.get_logger().warn(
                f"Skipping map->odom: LIO odometry age {now_seconds - lio_stamp:.3f}s",
                throttle_duration_sec=2.0,
            )
            return
        if self.max_local_age > 0.0 and now_seconds > 0.0 and now_seconds - local_stamp > self.max_local_age:
            self.get_logger().warn(
                f"Skipping map->odom: local odometry age {now_seconds - local_stamp:.3f}s",
                throttle_duration_sec=2.0,
            )
            return

        lio_stamp_key = (
            self.latest_lio.header.stamp.sec,
            self.latest_lio.header.stamp.nanosec,
        )
        if self.cached_lio_stamp != lio_stamp_key:
            self.update_cached_transform(lio_stamp_key)
        if self.cached_map_odom_t is None or self.cached_map_odom_q is None:
            return

        transform = TransformStamped()
        transform.header.stamp = (
            now_stamp if self.stamp_mode == "now" else self.cached_transform_stamp
        )
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = self.cached_map_odom_t[0]
        transform.transform.translation.y = self.cached_map_odom_t[1]
        transform.transform.translation.z = self.cached_map_odom_t[2]
        transform.transform.rotation.x = self.cached_map_odom_q[0]
        transform.transform.rotation.y = self.cached_map_odom_q[1]
        transform.transform.rotation.z = self.cached_map_odom_q[2]
        transform.transform.rotation.w = self.cached_map_odom_q[3]
        self.broadcaster.sendTransform(transform)

    def update_cached_transform(self, lio_stamp_key):
        map_base = pose_as_base(
            self.latest_lio, self.base_frame, self.tf_buffer, self.get_logger(), "LIO odometry"
        )
        odom_base = pose_as_base(
            self.latest_local,
            self.base_frame,
            self.tf_buffer,
            self.get_logger(),
            "local odometry",
        )
        if map_base is None or odom_base is None:
            return
        map_base_t, map_base_q = map_base
        odom_base_t, odom_base_q = odom_base

        base_odom_q = q_inverse(odom_base_q)
        base_odom_t = rotate(base_odom_q, (-odom_base_t[0], -odom_base_t[1], -odom_base_t[2]))
        map_odom_q = q_multiply(map_base_q, base_odom_q)
        rotated_base_odom_t = rotate(map_base_q, base_odom_t)
        map_odom_t = (
            map_base_t[0] + rotated_base_odom_t[0],
            map_base_t[1] + rotated_base_odom_t[1],
            map_base_t[2] + rotated_base_odom_t[2],
        )

        # map->odom is a correction between the global LIO pose and the local
        # odom frame. Recomputing it at timer rate with a stale LIO pose and a
        # fresh local odom pose cancels odom->base_link motion, so RViz sees
        # base_link freeze until the next LIO update. Cache the correction only
        # when a new LIO pose arrives; the high-rate odom->base_link transform
        # then provides smooth motion between global corrections.
        self.cached_lio_stamp = lio_stamp_key
        self.cached_transform_stamp = self.latest_lio.header.stamp
        self.cached_map_odom_t = map_odom_t
        self.cached_map_odom_q = map_odom_q


def main():
    rclpy.init()
    node = MapOdomTfBridge()
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
