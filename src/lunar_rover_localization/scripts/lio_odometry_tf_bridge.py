#!/usr/bin/env python3
"""Broadcast a TF chain from the corrected FAST-LIO odometry.

This is intended for the real-vehicle bringup before wheel odometry is wired into
the stack.  It exposes the existing /localization/lio_odometry pose as:

    map -> odom  (identity)
    odom -> base_link  (FAST-LIO pose)

With the identity map->odom transform, map->base_link is equal to the corrected
FAST-LIO pose while Nav2 local costmaps can still use odom as their global frame.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def finite_pose(message):
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    values = (
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )
    return all(math.isfinite(float(value)) for value in values)


class LioOdometryTfBridge(Node):
    def __init__(self):
        super().__init__("lio_odometry_tf_bridge")
        self.odom_topic = self.declare_parameter(
            "odom_topic", "/localization/lio_odometry"
        ).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.stamp_mode = (
            self.declare_parameter("stamp_mode", "now").value.strip().lower()
        )
        if self.stamp_mode not in ("now", "data"):
            self.get_logger().warn(
                "Unsupported stamp_mode '%s', falling back to 'now'" % self.stamp_mode
            )
            self.stamp_mode = "now"

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, self.odom_topic, self.on_odom, 20
        )
        self.publish_map_odom_identity()
        self.get_logger().info(
            "LIO odometry TF bridge: %s -> %s -> %s from %s"
            % (self.map_frame, self.odom_frame, self.base_frame, self.odom_topic)
        )

    def publish_map_odom_identity(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(transform)

    def on_odom(self, message):
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().warn(
                "LIO odometry frame '%s' != expected '%s'"
                % (message.header.frame_id, self.map_frame),
                throttle_duration_sec=2.0,
            )
        if not finite_pose(message):
            self.get_logger().warn(
                "Ignoring non-finite LIO odometry for TF", throttle_duration_sec=2.0
            )
            return

        transform = TransformStamped()
        transform.header.stamp = (
            self.get_clock().now().to_msg()
            if self.stamp_mode == "now"
            else message.header.stamp
        )
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = (
            message.child_frame_id.strip() if message.child_frame_id.strip() else self.base_frame
        )
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = LioOdometryTfBridge()
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
