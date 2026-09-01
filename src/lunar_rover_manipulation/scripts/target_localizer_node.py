#!/usr/bin/env python3
"""Fuse an instance mask with registered depth and classify its support surface."""

import math
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from lunar_rover_msgs.msg import TargetObservation
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2D


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _rotation_matrix(quaternion):
    q = np.array(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=np.float64
    )
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        raise ValueError("invalid zero-length TF quaternion")
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _depth_to_points(depth_m, valid, camera_info):
    rows, cols = np.nonzero(valid)
    z = depth_m[rows, cols].astype(np.float64)
    fx, fy = float(camera_info.k[0]), float(camera_info.k[4])
    cx, cy = float(camera_info.k[2]), float(camera_info.k[5])
    x = (cols.astype(np.float64) - cx) * z / fx
    y = (rows.astype(np.float64) - cy) * z / fy
    return np.column_stack((x, y, z))


def _robust_center_and_extent(points):
    center = np.median(points, axis=0)
    low = np.percentile(points, 5.0, axis=0)
    high = np.percentile(points, 95.0, axis=0)
    return center, np.maximum(0.0, high - low)


def _fit_plane_normal(points):
    if len(points) < 30:
        return None
    center = np.median(points, axis=0)
    distance = np.linalg.norm(points - center, axis=1)
    filtered = points[distance <= np.percentile(distance, 85.0)]
    if len(filtered) < 30:
        return None
    centered = filtered - np.mean(filtered, axis=0)
    covariance = centered.T @ centered / max(len(filtered) - 1, 1)
    values, vectors = np.linalg.eigh(covariance)
    if values[1] <= 1e-10 or values[0] / values[1] > 0.18:
        return None
    normal = vectors[:, int(np.argmin(values))]
    normal /= max(np.linalg.norm(normal), 1e-9)
    if float(np.dot(normal, np.mean(filtered, axis=0))) > 0.0:
        normal = -normal
    return normal


class TargetLocalizer(Node):
    def __init__(self):
        super().__init__("grasp_target_localizer")
        self.depth_topic = self.declare_parameter(
            "depth_topic", "/camera/depth/image_raw"
        ).value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/color/camera_info"
        ).value
        self.mask_topic = self.declare_parameter(
            "mask_topic", "/perception/target_mask"
        ).value
        self.detection_topic = self.declare_parameter(
            "detection_topic", "/perception/target_detection"
        ).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.min_depth_m = float(self.declare_parameter("min_depth_m", 0.10).value)
        self.max_depth_m = float(self.declare_parameter("max_depth_m", 1.20).value)
        self.sync_tolerance_sec = float(
            self.declare_parameter("sync_tolerance_sec", 0.08).value
        )
        self.tf_timeout_sec = float(
            self.declare_parameter("tf_timeout_sec", 0.12).value
        )
        self.min_target_points = int(
            self.declare_parameter("min_target_points", 30).value
        )
        self.max_target_extent_m = float(
            self.declare_parameter("max_target_extent_m", 0.09).value
        )
        self.support_ring_pixels = int(
            self.declare_parameter("support_ring_pixels", 25).value
        )
        self.ground_normal_min_z = float(
            self.declare_parameter("ground_normal_min_z", 0.70).value
        )
        self.wall_normal_max_z = float(
            self.declare_parameter("wall_normal_max_z", 0.35).value
        )
        self.ground_height_fallback_m = float(
            self.declare_parameter("ground_height_fallback_m", 0.18).value
        )

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_mask = None
        self.latest_detection = None
        self.latest_depth = None
        self.depth_cache = deque(maxlen=12)
        self.camera_info = None
        self.last_publish_stamp = -math.inf
        self.observation_publisher = self.create_publisher(
            TargetObservation, "/perception/target_observation", 10
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped, "/grasp/target_pose", 10
        )
        self.debug_publisher = self.create_publisher(
            Image, "/perception/debug/support_ring", sensor_qos
        )
        self.create_subscription(Image, self.mask_topic, self.on_mask, sensor_qos)
        self.create_subscription(
            Detection2D, self.detection_topic, self.on_detection, 10
        )
        self.create_subscription(
            Image, self.depth_topic, self.on_depth, sensor_qos
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.on_camera_info, sensor_qos
        )

    def on_mask(self, message):
        self.latest_mask = message
        self._try_process_latest()

    def on_detection(self, message):
        self.latest_detection = message
        self._try_process_latest()

    def on_camera_info(self, message):
        self.camera_info = message

    def _matching_inputs(self, depth_message):
        if self.latest_mask is None or self.latest_detection is None:
            return None
        depth_stamp = _stamp_seconds(depth_message.header.stamp)
        mask_stamp = _stamp_seconds(self.latest_mask.header.stamp)
        detection_stamp = _stamp_seconds(self.latest_detection.header.stamp)
        if abs(depth_stamp - mask_stamp) > self.sync_tolerance_sec:
            return None
        if abs(mask_stamp - detection_stamp) > self.sync_tolerance_sec:
            return None
        if depth_stamp <= self.last_publish_stamp:
            return None
        return self.latest_mask, self.latest_detection, depth_stamp

    def on_depth(self, depth_message):
        self.latest_depth = depth_message
        self.depth_cache.append(depth_message)
        self._try_process_latest()

    def _best_depth_for_latest_mask(self):
        if self.latest_mask is None or not self.depth_cache:
            return self.latest_depth
        mask_stamp = _stamp_seconds(self.latest_mask.header.stamp)
        best = min(
            self.depth_cache,
            key=lambda message: abs(_stamp_seconds(message.header.stamp) - mask_stamp),
        )
        if abs(_stamp_seconds(best.header.stamp) - mask_stamp) <= self.sync_tolerance_sec:
            return best
        return self.latest_depth

    def _try_process_latest(self):
        depth_message = self._best_depth_for_latest_mask()
        if depth_message is None:
            return
        if self.camera_info is None:
            return
        matched = self._matching_inputs(depth_message)
        if matched is None:
            return
        mask_message, detection, depth_stamp = matched
        try:
            mask = self.bridge.imgmsg_to_cv2(mask_message, desired_encoding="mono8")
            depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"RGB-D conversion failed: {exc}")
            return
        if mask.shape[:2] != depth.shape[:2]:
            self.get_logger().warning("target mask and registered depth dimensions differ")
            return
        fx, fy = float(self.camera_info.k[0]), float(self.camera_info.k[4])
        if fx <= 0.0 or fy <= 0.0:
            return

        depth_m = depth.astype(np.float32)
        if depth.dtype == np.uint16:
            depth_m *= 0.001
        finite_depth = np.isfinite(depth_m)
        finite_depth &= (depth_m >= self.min_depth_m) & (depth_m <= self.max_depth_m)
        target_valid = (mask > 0) & finite_depth
        if np.count_nonzero(target_valid) < self.min_target_points:
            return
        target_points_camera = _depth_to_points(
            depth_m, target_valid, self.camera_info
        )
        target_center_camera, _ = _robust_center_and_extent(target_points_camera)

        ring_size = max(3, self.support_ring_pixels | 1)
        inner_size = max(3, min(9, ring_size - 2) | 1)
        outer = cv2.dilate(mask, np.ones((ring_size, ring_size), np.uint8)) > 0
        inner = cv2.dilate(mask, np.ones((inner_size, inner_size), np.uint8)) > 0
        support_valid = outer & ~inner & finite_depth
        support_valid &= depth_m <= float(target_center_camera[2] + 0.15)
        support_points_camera = _depth_to_points(
            depth_m, support_valid, self.camera_info
        )
        support_normal_camera = _fit_plane_normal(support_points_camera)

        source_frame = depth_message.header.frame_id or self.camera_info.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                Time.from_msg(depth_message.header.stamp),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec),
                )
            except TransformException:
                self.get_logger().warning(f"camera TF unavailable: {exc}")
                return
            self.get_logger().warning(
                f"time-aligned camera TF unavailable, using latest TF: {exc}",
                throttle_duration_sec=2.0,
            )
        rotation = _rotation_matrix(transform.transform.rotation)
        translation = transform.transform.translation
        offset = np.array([translation.x, translation.y, translation.z])
        target_points_base = (rotation @ target_points_camera.T).T + offset
        target_center_base, target_extent_base = _robust_center_and_extent(
            target_points_base
        )
        if float(np.max(target_extent_base)) > self.max_target_extent_m:
            self.get_logger().warning(
                f"rejecting oversized target extent {target_extent_base.tolist()}"
            )
            return

        surface_type = TargetObservation.SURFACE_UNKNOWN
        surface_normal_base = np.zeros(3, dtype=np.float64)
        if support_normal_camera is not None:
            surface_normal_base = rotation @ support_normal_camera
            surface_normal_base /= max(np.linalg.norm(surface_normal_base), 1e-9)
            abs_z = abs(float(surface_normal_base[2]))
            if abs_z >= self.ground_normal_min_z:
                surface_type = TargetObservation.SURFACE_GROUND
                if surface_normal_base[2] < 0.0:
                    surface_normal_base = -surface_normal_base
            elif abs_z <= self.wall_normal_max_z:
                surface_type = TargetObservation.SURFACE_WALL
        elif target_center_base[2] <= self.ground_height_fallback_m:
            surface_type = TargetObservation.SURFACE_GROUND
            surface_normal_base = np.array([0.0, 0.0, 1.0])

        class_id = "unknown"
        confidence = 0.0
        if detection.results:
            class_id = detection.results[0].hypothesis.class_id
            confidence = float(detection.results[0].hypothesis.score)
        observation = TargetObservation()
        observation.header.stamp = depth_message.header.stamp
        observation.header.frame_id = self.base_frame
        observation.observation_id = detection.id
        observation.class_id = class_id
        observation.confidence = confidence
        observation.pose.position.x = float(target_center_base[0])
        observation.pose.position.y = float(target_center_base[1])
        observation.pose.position.z = float(target_center_base[2])
        observation.pose.orientation.w = 1.0
        observation.surface_normal.x = float(surface_normal_base[0])
        observation.surface_normal.y = float(surface_normal_base[1])
        observation.surface_normal.z = float(surface_normal_base[2])
        observation.dimensions.x = float(target_extent_base[0])
        observation.dimensions.y = float(target_extent_base[1])
        observation.dimensions.z = float(target_extent_base[2])
        observation.surface_type = surface_type
        observation.pose_is_precise = False
        observation.source = "rgbd_mask_geometry"
        self.observation_publisher.publish(observation)

        pose = PoseStamped()
        pose.header = observation.header
        pose.pose = observation.pose
        self.pose_publisher.publish(pose)
        self.last_publish_stamp = depth_stamp

        debug = np.zeros(mask.shape, dtype=np.uint8)
        debug[support_valid] = 120
        debug[target_valid] = 255
        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="mono8")
        debug_message.header = depth_message.header
        self.debug_publisher.publish(debug_message)


def main():
    rclpy.init()
    node = TargetLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
