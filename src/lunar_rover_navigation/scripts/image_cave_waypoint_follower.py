#!/usr/bin/env python3
"""Send the generated image_cave centerline loop to Nav2 as sequential goals."""

from __future__ import annotations

import csv
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class ImageCaveWaypointFollower(Node):
    def __init__(self):
        super().__init__("image_cave_waypoint_follower")
        default_route = (
            Path(get_package_share_directory("lunar_rover_navigation"))
            / "routes"
            / "image_cave_centerline_loop.csv"
        )
        self.route_csv = Path(
            self.declare_parameter("route_csv", str(default_route)).value
        )
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.loop = bool(self.declare_parameter("loop", True).value)
        self.start_index = int(self.declare_parameter("start_index", 0).value)
        self.start_from_nearest = bool(
            self.declare_parameter("start_from_nearest", True).value
        )
        self.skip_reached_radius = float(
            self.declare_parameter("skip_reached_radius", 0.35).value
        )
        self.goal_timeout_sec = float(
            self.declare_parameter("goal_timeout_sec", 180.0).value
        )
        self.retry_failed_goals = bool(
            self.declare_parameter("retry_failed_goals", True).value
        )
        self.max_retries = int(self.declare_parameter("max_retries", 1).value)

        self.waypoints = self.load_waypoints(self.route_csv)
        if not self.waypoints:
            raise RuntimeError(f"No waypoints loaded from {self.route_csv}")
        self.index = max(0, min(self.start_index, len(self.waypoints) - 1))
        self.current_retries = 0
        self.current_goal_handle = None
        self.goal_request_pending = False
        self.goal_start_time = None
        self.route_initialized = not self.start_from_nearest

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.path_pub = self.create_publisher(NavPath, "/image_cave/centerline_route", 1)
        self.create_timer(1.0, self.publish_path)
        self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f"Loaded {len(self.waypoints)} image_cave waypoints from {self.route_csv}"
        )

    def load_waypoints(self, route_csv: Path):
        waypoints = []
        with route_csv.open(newline="", encoding="ascii") as route_file:
            reader = csv.DictReader(route_file)
            for row in reader:
                waypoints.append(
                    (
                        float(row["x"]),
                        float(row["y"]),
                        float(row["yaw"]),
                    )
                )
        return waypoints

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def publish_path(self) -> None:
        path = NavPath()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in self.waypoints:
            path.poses.append(self.make_pose(x, y, yaw))
        self.path_pub.publish(path)

    def current_robot_xy(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id, self.base_frame, rclpy.time.Time()
            )
        except TransformException:
            return None
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
        )

    def advance_index(self) -> bool:
        self.index += 1
        self.current_retries = 0
        if self.index < len(self.waypoints):
            return True
        if not self.loop:
            self.get_logger().info("Completed image_cave waypoint route")
            return False
        self.index = 0
        self.get_logger().info("Restarting image_cave waypoint loop")
        return True

    def skip_already_reached(self) -> bool:
        robot = self.current_robot_xy()
        if robot is None:
            return True
        while True:
            x, y, _yaw = self.waypoints[self.index]
            if math.hypot(robot[0] - x, robot[1] - y) > self.skip_reached_radius:
                return True
            self.get_logger().info(
                f"Skipping reached waypoint {self.index + 1}/{len(self.waypoints)}"
            )
            if not self.advance_index():
                return False

    def initialize_route_position(self) -> bool:
        if self.route_initialized:
            return True
        robot = self.current_robot_xy()
        if robot is None:
            return False
        self.index = min(
            range(len(self.waypoints)),
            key=lambda i: math.hypot(
                robot[0] - self.waypoints[i][0], robot[1] - self.waypoints[i][1]
            ),
        )
        self.route_initialized = True
        self.get_logger().info(
            f"Starting at nearest waypoint {self.index + 1}/{len(self.waypoints)}"
        )
        return True

    def tick(self) -> None:
        if self.goal_request_pending or self.current_goal_handle is not None:
            if self.goal_start_time is None or self.goal_timeout_sec <= 0.0:
                return
            elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds * 1e-9
            if elapsed > self.goal_timeout_sec:
                self.get_logger().warn(
                    f"Canceling waypoint {self.index + 1}: timeout {elapsed:.1f}s"
                )
                self.current_goal_handle.cancel_goal_async()
            return

        if not self.client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn("Waiting for /navigate_to_pose", throttle_duration_sec=2.0)
            return
        if not self.initialize_route_position():
            return
        if not self.skip_already_reached():
            return
        self.send_current_goal()

    def send_current_goal(self) -> None:
        x, y, yaw = self.waypoints[self.index]
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw)
        self.get_logger().info(
            f"Sending waypoint {self.index + 1}/{len(self.waypoints)}: "
            f"x={x:.2f} y={y:.2f} yaw={yaw:.2f}"
        )
        self.goal_request_pending = True
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future) -> None:
        self.goal_request_pending = False
        try:
            handle = future.result()
        except Exception as error:
            self.get_logger().warn(
                f"Waypoint {self.index + 1} request failed: {error}"
            )
            self.handle_failure()
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn(f"Waypoint {self.index + 1} rejected")
            self.handle_failure()
            return
        self.current_goal_handle = handle
        self.goal_start_time = self.get_clock().now()
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future) -> None:
        result = future.result()
        self.current_goal_handle = None
        self.goal_start_time = None
        if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Waypoint {self.index + 1} reached")
            self.advance_index()
            return
        status = result.status if result is not None else "none"
        self.get_logger().warn(f"Waypoint {self.index + 1} failed: status={status}")
        self.handle_failure()

    def handle_failure(self) -> None:
        if self.retry_failed_goals and self.current_retries < self.max_retries:
            self.current_retries += 1
            self.get_logger().warn(
                f"Retrying waypoint {self.index + 1}; retry "
                f"{self.current_retries}/{self.max_retries}"
            )
            return
        self.advance_index()


def main():
    rclpy.init()
    node = ImageCaveWaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
