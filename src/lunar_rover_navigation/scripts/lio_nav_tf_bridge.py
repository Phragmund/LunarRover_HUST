#!/usr/bin/env python3
"""从 FAST-LIO 里程计发布 map->odom(恒等) + map->base_link TF。

实车没有轮速+IMU 组合里程计(composite_odometry)，因此用 FAST-LIO 的
map->base_link 直接作为定位，并发布恒等 map->odom，使 Nav2 的
map->odom->base_link 链成立(odom 与 map 重合)。
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class LioNavTfBridge(Node):
    def __init__(self):
        super().__init__("lio_nav_tf_bridge")
        self.odom_topic = self.declare_parameter(
            "odom_topic", "/localization/lio_odometry"
        ).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, self.odom_topic, self.on_odom, 10
        )
        # 周期性重发恒等 map->odom，避免单次 TF 过期导致 Nav2 一直等待
        self.create_timer(0.1, self._on_timer)
        self._publish_identity()
        self.get_logger().info(
            "Publishing map->odom (identity) + map->base_link from %s"
            % self.odom_topic
        )

    def _on_timer(self):
        self._publish_identity()

    def _publish_identity(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t)

    def on_odom(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)
        self._publish_identity()


def main():
    rclpy.init()
    node = LioNavTfBridge()
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
