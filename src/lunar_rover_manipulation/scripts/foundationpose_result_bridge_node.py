#!/usr/bin/env python3
"""Convert FoundationPose Detection3D output into the grasp observation API."""

import math
import time

import numpy as np
import rclpy
from lunar_rover_msgs.msg import TargetObservation
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray


def _rotation_matrix(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    result = np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=np.float64,
    )
    return result / max(np.linalg.norm(result), 1e-9)


class FoundationPoseResultBridge(Node):
    def __init__(self):
        super().__init__("foundationpose_result_bridge")
        self.input_topic = self.declare_parameter(
            "input_topic", "/foundationpose/output"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/perception/target_pose_precise"
        ).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.rough_observation_max_age_sec = float(
            self.declare_parameter("rough_observation_max_age_sec", 1.0).value
        )
        self.rough_match_distance_m = float(
            self.declare_parameter("rough_match_distance_m", 0.10).value
        )
        self.object_class_id = str(
            self.declare_parameter("object_class_id", "").value
        ).strip()
        self.tf_timeout_sec = float(
            self.declare_parameter("tf_timeout_sec", 0.12).value
        )
        self.latest_rough = None
        self.latest_rough_received = -math.inf
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(TargetObservation, self.output_topic, 10)
        self.create_subscription(
            TargetObservation,
            "/perception/target_observation",
            self.on_rough_observation,
            10,
        )
        self.create_subscription(Detection3DArray, self.input_topic, self.on_result, 10)

    def on_rough_observation(self, message):
        self.latest_rough = message
        self.latest_rough_received = time.monotonic()

    def on_result(self, message):
        if not message.detections:
            return
        detection = max(
            message.detections,
            key=lambda item: max(
                (result.hypothesis.score for result in item.results), default=0.0
            ),
        )
        source_header = detection.header
        if not source_header.frame_id:
            source_header = message.header
        if not source_header.frame_id:
            self.get_logger().warning("FoundationPose output has no frame_id")
            return
        pose = detection.results[0].pose.pose if detection.results else detection.bbox.center
        score = (
            float(detection.results[0].hypothesis.score)
            if detection.results
            else 1.0
        )
        class_id = (
            detection.results[0].hypothesis.class_id
            if detection.results and detection.results[0].hypothesis.class_id
            else "foundationpose_object"
        )
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_header.frame_id,
                Time.from_msg(source_header.stamp),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warning(f"FoundationPose TF unavailable: {exc}")
            return

        rotation = _rotation_matrix(transform.transform.rotation)
        translation = transform.transform.translation
        source_position = np.array(
            [pose.position.x, pose.position.y, pose.position.z], dtype=np.float64
        )
        target_position = rotation @ source_position + np.array(
            [translation.x, translation.y, translation.z], dtype=np.float64
        )
        tf_q = transform.transform.rotation
        pose_q = pose.orientation
        target_q = _quaternion_multiply(
            np.array([tf_q.x, tf_q.y, tf_q.z, tf_q.w]),
            np.array([pose_q.x, pose_q.y, pose_q.z, pose_q.w]),
        )

        output = TargetObservation()
        output.header.stamp = source_header.stamp
        output.header.frame_id = self.base_frame
        output.observation_id = detection.id
        output.class_id = self.object_class_id or class_id
        output.confidence = score
        output.pose.position.x = float(target_position[0])
        output.pose.position.y = float(target_position[1])
        output.pose.position.z = float(target_position[2])
        output.pose.orientation.x = float(target_q[0])
        output.pose.orientation.y = float(target_q[1])
        output.pose.orientation.z = float(target_q[2])
        output.pose.orientation.w = float(target_q[3])
        output.dimensions = detection.bbox.size
        if (
            self.latest_rough is not None
            and time.monotonic() - self.latest_rough_received
            <= self.rough_observation_max_age_sec
        ):
            rough = self.latest_rough
            rough_point = rough.pose.position
            rough_position = np.array(
                [rough_point.x, rough_point.y, rough_point.z], dtype=np.float64
            )
            if np.linalg.norm(rough_position - target_position) <= self.rough_match_distance_m:
                output.surface_normal = rough.surface_normal
                output.surface_type = rough.surface_type
                if rough.class_id and class_id == "foundationpose_object":
                    output.class_id = rough.class_id
                if max(output.dimensions.x, output.dimensions.y, output.dimensions.z) <= 0.0:
                    output.dimensions = rough.dimensions
        output.pose_is_precise = True
        output.source = "foundationpose"
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = FoundationPoseResultBridge()
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
