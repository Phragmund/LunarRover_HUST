#!/usr/bin/env python3
"""Republish synchronized RGB-D and segmentation inputs for FoundationPose."""

import copy
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class FoundationPoseInputAdapter(Node):
    def __init__(self):
        super().__init__("foundationpose_input_adapter")
        self.color_topic = self.declare_parameter(
            "color_topic", "/camera/color/image_raw"
        ).value
        self.depth_topic = self.declare_parameter(
            "depth_topic", "/camera/depth/image_raw"
        ).value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/color/camera_info"
        ).value
        self.mask_topic = self.declare_parameter(
            "mask_topic", "/perception/target_mask"
        ).value
        self.output_namespace = str(
            self.declare_parameter("output_namespace", "/foundationpose").value
        ).rstrip("/")
        self.sync_tolerance_sec = float(
            self.declare_parameter("sync_tolerance_sec", 0.10).value
        )

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.color_publisher = self.create_publisher(
            Image, f"{self.output_namespace}/image", sensor_qos
        )
        self.depth_publisher = self.create_publisher(
            Image, f"{self.output_namespace}/depth_image", sensor_qos
        )
        self.info_publisher = self.create_publisher(
            CameraInfo, f"{self.output_namespace}/camera_info", sensor_qos
        )
        self.mask_publisher = self.create_publisher(
            Image, f"{self.output_namespace}/segmentation", sensor_qos
        )
        self.latest = {"color": None, "depth": None, "info": None, "mask": None}
        self.last_output_stamp = -math.inf
        self.create_subscription(
            Image, self.color_topic, lambda msg: self._receive("color", msg), sensor_qos
        )
        self.create_subscription(
            Image, self.depth_topic, lambda msg: self._receive("depth", msg), sensor_qos
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            lambda msg: self._receive("info", msg),
            sensor_qos,
        )
        self.create_subscription(
            Image, self.mask_topic, lambda msg: self._receive("mask", msg), sensor_qos
        )

    def _receive(self, key, message):
        self.latest[key] = message
        self._publish_if_synchronized()

    def _publish_if_synchronized(self):
        if any(message is None for message in self.latest.values()):
            return
        mask_stamp = _stamp_seconds(self.latest["mask"].header.stamp)
        if mask_stamp <= self.last_output_stamp:
            return
        for key in ("color", "depth"):
            delta = abs(_stamp_seconds(self.latest[key].header.stamp) - mask_stamp)
            if delta > self.sync_tolerance_sec:
                return

        # FoundationPose expects one synchronized bundle. CameraInfo is often
        # latched or low-rate, so only its calibration payload is reused.
        color = copy.deepcopy(self.latest["color"])
        depth = copy.deepcopy(self.latest["depth"])
        info = copy.deepcopy(self.latest["info"])
        mask = copy.deepcopy(self.latest["mask"])
        common_stamp = mask.header.stamp
        common_frame = depth.header.frame_id or color.header.frame_id
        for message in (color, depth, info, mask):
            message.header.stamp = common_stamp
            if common_frame:
                message.header.frame_id = common_frame
        self.color_publisher.publish(color)
        self.depth_publisher.publish(depth)
        self.info_publisher.publish(info)
        self.mask_publisher.publish(mask)
        self.last_output_stamp = mask_stamp


def main():
    rclpy.init()
    node = FoundationPoseInputAdapter()
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
