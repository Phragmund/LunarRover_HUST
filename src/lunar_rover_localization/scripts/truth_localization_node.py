#!/usr/bin/env python3
"""Ground-truth localization: replace FAST-LIO2 with the Gazebo ground-truth pose.

Publishes the ground-truth pose as a map-frame LIO odometry and registers each
raw LiDAR scan into the map frame using that pose.  The resulting
/localization/cloud_registered has no localization drift, giving a "theoretically
perfect" point cloud for validating the downstream terrain costmap rules.

Inputs:
  /simulation/ground_truth/odometry  (nav_msgs/Odometry, world -> base_link)
  /livox/points                      (sensor_msgs/PointCloud2, livox_frame)

Outputs:
  /localization/lio_odometry         (nav_msgs/Odometry, map -> base_link)
  /localization/cloud_registered     (sensor_msgs/PointCloud2, map frame)
"""

import math
from collections import deque
from threading import Lock

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)


def q_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


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


def yaw_to_q(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def stamp_seconds(stamp):
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


def q_slerp(a, b, ratio):
    ax, ay, az, aw = q_normalize(a)
    bx, by, bz, bw = q_normalize(b)
    dot = ax * bx + ay * by + az * bz + aw * bw
    if dot < 0.0:
        bx, by, bz, bw = -bx, -by, -bz, -bw
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return q_normalize(
            (
                ax + ratio * (bx - ax),
                ay + ratio * (by - ay),
                az + ratio * (bz - az),
                aw + ratio * (bw - aw),
            )
        )
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * ratio
    sin_theta = math.sin(theta)
    scale_a = math.cos(theta) - dot * sin_theta / sin_theta_0
    scale_b = sin_theta / sin_theta_0
    return (
        scale_a * ax + scale_b * bx,
        scale_a * ay + scale_b * by,
        scale_a * az + scale_b * bz,
        scale_a * aw + scale_b * bw,
    )


def quat_to_rot_matrix(q):
    """Return a 3x3 rotation matrix from a unit quaternion (x, y, z, w)."""
    x, y, z, w = q_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class TruthLocalizationNode(Node):
    def __init__(self):
        super().__init__("truth_localization")
        self.map_origin_world_x = float(
            self.declare_parameter("map_origin_world_x", 0.0).value
        )
        self.map_origin_world_y = float(
            self.declare_parameter("map_origin_world_y", 0.0).value
        )
        self.map_origin_world_z = float(
            self.declare_parameter("map_origin_world_z", 0.0).value
        )
        self.map_origin_world_yaw = float(
            self.declare_parameter("map_origin_world_yaw", 0.0).value
        )
        self.truth_topic = self.declare_parameter(
            "truth_topic", "/simulation/ground_truth/odometry"
        ).value
        self.lidar_topic = self.declare_parameter("lidar_topic", "/livox/points").value
        self.lio_topic = self.declare_parameter(
            "lio_topic", "/localization/lio_odometry"
        ).value
        self.cloud_topic = self.declare_parameter(
            "cloud_topic", "/localization/cloud_registered"
        ).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.truth_buffer_duration = float(
            self.declare_parameter("truth_buffer_duration_sec", 3.0).value
        )
        self.max_truth_extrapolation = float(
            self.declare_parameter("max_truth_extrapolation_sec", 0.08).value
        )

        # world -> map inverse transform (the spawn pose maps world to map).
        self.q_wm = yaw_to_q(self.map_origin_world_yaw)
        self.q_mw = q_inverse(self.q_wm)
        self.p_wm = (
            self.map_origin_world_x,
            self.map_origin_world_y,
            self.map_origin_world_z,
        )
        r = rotate(self.q_mw, self.p_wm)
        self.p_mw = (-r[0], -r[1], -r[2])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.latest_truth = None
        self.latest_map_stamp = None
        self.truth_buffer = deque()
        self.truth_lock = Lock()

        # In truth mode odom is identical to map (no drift), so publish a
        # static map->odom identity and broadcast map->base_link from the
        # ground-truth pose.  This gives the full map->odom->base_link tree
        # without depending on composite odometry or the FAST-LIO bridge.
        odom_identity = TransformStamped()
        odom_identity.header.stamp = self.get_clock().now().to_msg()
        odom_identity.header.frame_id = self.map_frame
        odom_identity.child_frame_id = "odom"
        odom_identity.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(odom_identity)

        # on_cloud is heavy (per-scan point transform); run it in its own
        # callback group so it never blocks the on_truth TF broadcast.
        self.cloud_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Odometry, self.truth_topic, self.on_truth, 10)
        self.create_subscription(
            PointCloud2, self.lidar_topic, self.on_cloud, 10,
            callback_group=self.cloud_group,
        )
        self.lio_pub = self.create_publisher(Odometry, self.lio_topic, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, self.cloud_topic, 1)

        self.get_logger().info(
            "truth_localization: "
            f"{self.truth_topic} -> {self.lio_topic}, "
            f"{self.lidar_topic} -> {self.cloud_topic}; "
            f"map origin world=({self.map_origin_world_x:.3f}, "
            f"{self.map_origin_world_y:.3f}, {self.map_origin_world_z:.3f}) "
            f"yaw={self.map_origin_world_yaw:.3f}"
        )

    def truth_to_map(self, odom=None):
        """Return (position, quaternion) of base_link in the map frame."""
        if odom is None:
            odom = self.latest_truth
        pose = odom.pose.pose
        p_wb = (pose.position.x, pose.position.y, pose.position.z)
        q_wb = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        q_map_base = q_multiply(self.q_mw, q_wb)
        r = rotate(self.q_mw, p_wb)
        p_map_base = (self.p_mw[0] + r[0], self.p_mw[1] + r[1], self.p_mw[2] + r[2])
        return p_map_base, q_map_base

    def append_truth_pose(self, msg, p, q):
        stamp = stamp_seconds(msg.header.stamp)
        with self.truth_lock:
            self.truth_buffer.append((stamp, msg.header.stamp, p, q))
            cutoff = stamp - self.truth_buffer_duration
            while len(self.truth_buffer) > 2 and self.truth_buffer[0][0] < cutoff:
                self.truth_buffer.popleft()

    def pose_for_stamp(self, stamp_msg):
        target = stamp_seconds(stamp_msg)
        with self.truth_lock:
            samples = list(self.truth_buffer)
        if not samples:
            return None

        if target <= samples[0][0]:
            error = samples[0][0] - target
            if error > self.max_truth_extrapolation:
                return None
            return samples[0][1], samples[0][2], samples[0][3]

        if target >= samples[-1][0]:
            error = target - samples[-1][0]
            if error > self.max_truth_extrapolation:
                return None
            return samples[-1][1], samples[-1][2], samples[-1][3]

        for index in range(1, len(samples)):
            previous_sample = samples[index - 1]
            next_sample = samples[index]
            if previous_sample[0] <= target <= next_sample[0]:
                span = next_sample[0] - previous_sample[0]
                ratio = 0.0 if span <= 0.0 else (target - previous_sample[0]) / span
                p0 = previous_sample[2]
                p1 = next_sample[2]
                p = (
                    p0[0] + ratio * (p1[0] - p0[0]),
                    p0[1] + ratio * (p1[1] - p0[1]),
                    p0[2] + ratio * (p1[2] - p0[2]),
                )
                q = q_slerp(previous_sample[3], next_sample[3], ratio)
                return stamp_msg, p, q
        return None

    def on_truth(self, msg):
        self.latest_truth = msg
        p, q = self.truth_to_map()
        self.append_truth_pose(msg, p, q)

        # Keep the truth TF timeline aligned with the LiDAR stamps so downstream
        # raytracing can recover the exact base pose used to register a cloud.
        stamp = msg.header.stamp
        self.latest_map_stamp = stamp
        lio = Odometry()
        lio.header.stamp = stamp
        lio.header.frame_id = self.map_frame
        lio.child_frame_id = self.base_frame
        lio.pose.pose.position.x = p[0]
        lio.pose.pose.position.y = p[1]
        lio.pose.pose.position.z = p[2]
        lio.pose.pose.orientation.x = q[0]
        lio.pose.pose.orientation.y = q[1]
        lio.pose.pose.orientation.z = q[2]
        lio.pose.pose.orientation.w = q[3]
        self.lio_pub.publish(lio)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = p[0]
        tf.transform.translation.y = p[1]
        tf.transform.translation.z = p[2]
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf)

    def on_cloud(self, msg):
        if self.latest_truth is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Cannot lookup {self.base_frame} -> {msg.header.frame_id}: {exc}",
                throttle_duration_sec=2.0,
            )
            return
        p_bl = (
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        )
        q_bl = (
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        )
        stamped_pose = self.pose_for_stamp(msg.header.stamp)
        if stamped_pose is None:
            self.get_logger().warn(
                "Dropping LiDAR cloud because no time-aligned truth pose is available: "
                f"cloud_stamp={stamp_seconds(msg.header.stamp):.3f}",
                throttle_duration_sec=2.0,
            )
            return
        pose_stamp, p_map_base, q_map_base = stamped_pose

        # Fast vectorized read: build a structured dtype from the field
        # offsets so the whole buffer is decoded without a Python loop.
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [np.float32, np.float32, np.float32],
                "offsets": [0, 4, 8],
                "itemsize": msg.point_step,
            }
        )
        arr = np.frombuffer(msg.data, dtype=dtype)
        pts = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float64)
        if pts.size == 0:
            return

        # livox -> base_link -> map (vectorized).
        r_bl = quat_to_rot_matrix(q_bl)
        r_mb = quat_to_rot_matrix(q_map_base)
        base = pts @ r_bl.T + np.asarray(p_bl, dtype=np.float64)
        map_pts = base @ r_mb.T + np.asarray(p_map_base, dtype=np.float64)

        out_header = Header()
        out_header.stamp = pose_stamp
        out_header.frame_id = self.map_frame
        out = point_cloud2.create_cloud_xyz32(out_header, map_pts.astype(np.float32))
        self.cloud_pub.publish(out)


def main():
    rclpy.init()
    node = TruthLocalizationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
