#!/usr/bin/env python3
"""Closed-loop online mapping navigation smoke test without Gazebo truth."""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import MapMetaData
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class NavigationSmokeTest(Node):
    def __init__(self):
        super().__init__("navigation_smoke_test")
        self.map_metadata = None
        self.map_subscription = self.create_subscription(
            MapMetaData, "/map_metadata", self._map_callback, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.goal_tolerance = self.declare_parameter("goal_tolerance", 0.07).value
        self.goals = [(0.0, 0.20, 0.0), (0.0, 0.45, 0.0)]

    def _map_callback(self, message):
        self.map_metadata = message

    def wait_until_ready(self):
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.map_metadata and self.client.wait_for_server(timeout_sec=0.1):
                try:
                    self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                    return True
                except TransformException:
                    pass
        return False

    def send_goal(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Goal ({x:.2f}, {y:.2f}) was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        result = result_future.result()
        if result is None:
            raise RuntimeError(f"Goal ({x:.2f}, {y:.2f}) timed out")
        if result.status != 4:
            raise RuntimeError(f"Goal ({x:.2f}, {y:.2f}) finished with status {result.status}")

        transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        dx = transform.transform.translation.x - x
        dy = transform.transform.translation.y - y
        error = math.hypot(dx, dy)
        self.get_logger().info(
            f"Goal ({x:.2f}, {y:.2f}) reached with {error:.3f} m map-frame error"
        )
        if error > self.goal_tolerance:
            raise RuntimeError(
                f"Goal ({x:.2f}, {y:.2f}) error {error:.3f} exceeds {self.goal_tolerance:.3f}"
            )

    def run(self):
        if not self.wait_until_ready():
            raise RuntimeError("Timed out waiting for Nav2, /map_metadata, and map->base_link")
        for goal in self.goals:
            self.send_goal(*goal)
        if self.map_metadata is None:
            raise RuntimeError("No map metadata received")
        width_m = self.map_metadata.width * self.map_metadata.resolution
        height_m = self.map_metadata.height * self.map_metadata.resolution
        self.get_logger().info(
            f"Final online map size: {width_m:.2f} m x {height_m:.2f} m"
        )


def main():
    rclpy.init()
    node = NavigationSmokeTest()
    try:
        node.run()
    except Exception as error:  # noqa: BLE001
        node.get_logger().error(str(error))
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
