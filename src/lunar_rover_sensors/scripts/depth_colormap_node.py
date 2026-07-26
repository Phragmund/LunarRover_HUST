#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthColormapNode(Node):
    def __init__(self):
        super().__init__("depth_colormap_node")
        self.declare_parameter("input_topic", "/camera/depth/image_raw")
        self.declare_parameter("output_topic", "/camera/depth/colored")
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 6.0)
        self.declare_parameter("colormap", "turbo")

        self.min_depth = float(self.get_parameter("min_depth_m").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)
        if not math.isfinite(self.min_depth) or not math.isfinite(self.max_depth):
            raise ValueError("Depth range must be finite")
        if self.max_depth <= self.min_depth:
            raise ValueError("max_depth_m must be greater than min_depth_m")

        self.bridge = CvBridge()
        self.colormap = self._colormap_id(str(self.get_parameter("colormap").value))
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(Image, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image, input_topic, self._callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Colorizing depth image {input_topic} -> {output_topic} "
            f"range=[{self.min_depth:.2f}, {self.max_depth:.2f}] m"
        )

    @staticmethod
    def _colormap_id(name):
        normalized = name.strip().lower()
        if normalized == "jet":
            return cv2.COLORMAP_JET
        if normalized == "viridis":
            return cv2.COLORMAP_VIRIDIS
        if normalized == "plasma":
            return cv2.COLORMAP_PLASMA
        return getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)

    def _depth_to_meters(self, image, encoding):
        depth = np.asarray(image)
        if depth.dtype == np.uint16 or encoding in ("16UC1", "mono16"):
            return depth.astype(np.float32) * 0.001
        return depth.astype(np.float32)

    def _callback(self, message):
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            depth_m = self._depth_to_meters(depth_raw, message.encoding)
        except Exception as exc:
            self.get_logger().warn(f"Failed to decode depth image: {exc}")
            return

        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        clipped = np.clip(depth_m, self.min_depth, self.max_depth)
        normalized = ((clipped - self.min_depth) * 255.0 / (self.max_depth - self.min_depth))
        mono = normalized.astype(np.uint8)
        colored = cv2.applyColorMap(mono, self.colormap)
        colored[~valid] = (0, 0, 0)

        output = self.bridge.cv2_to_imgmsg(colored, encoding="bgr8")
        output.header = message.header
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = DepthColormapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
