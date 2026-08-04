#!/usr/bin/env python3
"""Terrain-aware frontier exploration over /terrain/traversability."""

import collections
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_ros import Buffer, TransformException, TransformListener


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.grid = None
        self.create_subscription(OccupancyGrid, "/terrain/traversability", self.on_grid, qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.path_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

        self.free_cost = int(self.declare_parameter("free_cost", 35).value)
        self.max_goal_cost = int(self.declare_parameter("max_goal_cost", 55).value)
        self.min_cluster_size = int(self.declare_parameter("min_cluster_size", 8).value)
        self.max_goals = int(self.declare_parameter("max_goals", 24).value)
        self.goal_timeout = float(self.declare_parameter("goal_timeout", 80.0).value)
        self.path_timeout = float(self.declare_parameter("path_timeout", 4.0).value)
        self.settle_time = float(self.declare_parameter("settle_time", 1.5).value)
        self.goal_backoff = float(self.declare_parameter("goal_backoff", 0.45).value)
        self.blacklist_radius = float(self.declare_parameter("blacklist_radius", 0.45).value)
        self.max_path_checks = int(self.declare_parameter("max_path_checks", 24).value)
        self.info_gain_weight = float(self.declare_parameter("info_gain_weight", 0.07).value)
        self.terrain_risk_weight = float(self.declare_parameter("terrain_risk_weight", 0.018).value)
        self.return_risk_weight = float(self.declare_parameter("return_risk_weight", 0.20).value)
        self.min_known_ratio = float(self.declare_parameter("min_known_ratio", 0.80).value)
        self.failure_return_threshold = int(self.declare_parameter("failure_return_threshold", 4).value)
        self.start_search_radius = float(self.declare_parameter("start_search_radius", 1.2).value)
        self.home_frame = self.declare_parameter("home_frame", "map").value
        self.robot_frame = self.declare_parameter("robot_frame", "base_link").value

        self.blacklist = []
        self.completed = []
        self.home_pose = None
        self.consecutive_failures = 0

    def on_grid(self, msg):
        self.grid = msg

    def wait_ready(self):
        deadline = time.monotonic() + 60.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.grid is None:
                continue
            if not self.nav_client.wait_for_server(timeout_sec=0.1):
                continue
            if not self.path_client.wait_for_server(timeout_sec=0.1):
                continue
            try:
                self.robot_xy()
                self.home_pose = self.robot_xy()
                return True
            except TransformException:
                continue
        return False

    def robot_xy(self):
        transform = self.tf_buffer.lookup_transform(
            self.grid.header.frame_id, self.robot_frame, rclpy.time.Time()
        )
        return transform.transform.translation.x, transform.transform.translation.y

    def index(self, col, row):
        return row * self.grid.info.width + col

    def in_bounds(self, col, row):
        return 0 <= col < self.grid.info.width and 0 <= row < self.grid.info.height

    def value(self, col, row):
        return self.grid.data[self.index(col, row)]

    def is_unknown(self, col, row):
        return self.value(col, row) < 0

    def is_free(self, col, row):
        value = self.value(col, row)
        return 0 <= value <= self.free_cost

    def is_goal_cell(self, col, row):
        value = self.value(col, row)
        return 0 <= value <= self.max_goal_cost

    def world_to_cell(self, x, y):
        return (
            int(math.floor((x - self.grid.info.origin.position.x) / self.grid.info.resolution)),
            int(math.floor((y - self.grid.info.origin.position.y) / self.grid.info.resolution)),
        )

    def cell_to_world(self, col, row):
        return (
            self.grid.info.origin.position.x + (col + 0.5) * self.grid.info.resolution,
            self.grid.info.origin.position.y + (row + 0.5) * self.grid.info.resolution,
        )

    def neighbors8(self, col, row):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nc = col + dx
                nr = row + dy
                if self.in_bounds(nc, nr):
                    yield nc, nr

    def reachable_free(self, start):
        if not self.in_bounds(*start) or not self.is_free(*start):
            return set()
        reachable = {start}
        queue = collections.deque([start])
        while queue:
            cell = queue.popleft()
            for neighbor in self.neighbors8(*cell):
                if neighbor in reachable or not self.is_free(*neighbor):
                    continue
                reachable.add(neighbor)
                queue.append(neighbor)
        return reachable

    def nearest_free_cell(self, start):
        if self.in_bounds(*start) and self.is_free(*start):
            return start
        max_radius = max(1, int(math.ceil(self.start_search_radius / self.grid.info.resolution)))
        best = None
        best_distance = math.inf
        start_col, start_row = start
        for row in range(start_row - max_radius, start_row + max_radius + 1):
            for col in range(start_col - max_radius, start_col + max_radius + 1):
                if not self.in_bounds(col, row) or not self.is_free(col, row):
                    continue
                distance = math.hypot(col - start_col, row - start_row)
                if distance < best_distance:
                    best = (col, row)
                    best_distance = distance
        if best is not None:
            self.get_logger().warn(
                "Robot cell is not free in terrain grid; using nearest free seed "
                f"{best} at {best_distance * self.grid.info.resolution:.2f} m"
            )
        return best

    def frontier_clusters(self, reachable):
        frontier = {
            cell
            for cell in reachable
            if any(self.is_unknown(*neighbor) for neighbor in self.neighbors8(*cell))
        }
        clusters = []
        while frontier:
            seed = frontier.pop()
            cluster = [seed]
            queue = collections.deque([seed])
            while queue:
                cell = queue.popleft()
                for neighbor in self.neighbors8(*cell):
                    if neighbor not in frontier:
                        continue
                    frontier.remove(neighbor)
                    cluster.append(neighbor)
                    queue.append(neighbor)
            if len(cluster) >= self.min_cluster_size:
                clusters.append(cluster)
        return clusters

    def backed_off_candidates(self, cluster, reachable):
        min_cells = max(1, int(math.ceil(self.goal_backoff / self.grid.info.resolution)))
        queue = collections.deque((cell, 0) for cell in cluster)
        visited = set(cluster)
        candidates = []
        while queue:
            cell, distance = queue.popleft()
            if distance >= min_cells and cell in reachable and self.is_goal_cell(*cell):
                candidates.append(cell)
            if distance > 2 * min_cells:
                continue
            for neighbor in self.neighbors8(*cell):
                if neighbor in visited or neighbor not in reachable:
                    continue
                visited.add(neighbor)
                queue.append((neighbor, distance + 1))
        return candidates

    def is_blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < self.blacklist_radius for bx, by in self.blacklist)

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.grid.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    def terrain_cost_at(self, x, y):
        cell = self.world_to_cell(x, y)
        if not self.in_bounds(*cell):
            return 100
        value = self.value(*cell)
        return 80 if value < 0 else value

    def path_score(self, x, y, yaw, cluster_size):
        goal = ComputePathToPose.Goal()
        goal.goal = self.make_pose(x, y, yaw)
        goal.planner_id = "GridBased"
        future = self.path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return None
        result_future = handle.get_result_async()
        deadline = time.monotonic() + self.path_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.1)
            if not result_future.done():
                continue
            result = result_future.result()
            if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
                return None
            poses = result.result.path.poses
            if len(poses) < 2:
                return None
            length = 0.0
            risk = 0.0
            previous = poses[0].pose.position
            for pose in poses[1:]:
                current = pose.pose.position
                segment = math.hypot(current.x - previous.x, current.y - previous.y)
                length += segment
                risk += self.terrain_cost_at(current.x, current.y) * segment
                previous = current
            home_risk = 0.0
            if self.home_pose is not None:
                home_risk = math.hypot(x - self.home_pose[0], y - self.home_pose[1])
            return (
                length
                + self.terrain_risk_weight * risk
                + self.return_risk_weight * home_risk
                - self.info_gain_weight * math.sqrt(cluster_size)
            )
        return None

    def option_priority(self, option, robot_x, robot_y):
        x, y, _yaw, cluster_size = option
        distance = math.hypot(x - robot_x, y - robot_y)
        terrain_risk = self.terrain_cost_at(x, y)
        return (
            distance
            + self.terrain_risk_weight * terrain_risk
            + self.return_risk_weight * math.hypot(x - self.home_pose[0], y - self.home_pose[1])
            - self.info_gain_weight * math.sqrt(cluster_size)
        )

    def select_goal(self):
        robot_x, robot_y = self.robot_xy()
        start = self.world_to_cell(robot_x, robot_y)
        start = self.nearest_free_cell(start)
        if start is None:
            self.get_logger().warn("No free terrain cell near robot; cannot seed frontier search")
            return None
        reachable = self.reachable_free(start)
        options = []
        for cluster in self.frontier_clusters(reachable):
            for cell in self.backed_off_candidates(cluster, reachable)[:8]:
                x, y = self.cell_to_world(*cell)
                if self.is_blacklisted(x, y):
                    continue
                yaw = math.atan2(y - robot_y, x - robot_x)
                options.append((x, y, yaw, len(cluster)))
        if self.home_pose is not None:
            options.sort(key=lambda option: self.option_priority(option, robot_x, robot_y))

        scored = []
        for x, y, yaw, cluster_size in options[: self.max_path_checks]:
            score = self.path_score(x, y, yaw, cluster_size)
            if score is None:
                self.blacklist.append((x, y))
                continue
            scored.append((score, x, y, yaw, cluster_size))
        if not scored:
            return None
        scored.sort(key=lambda item: item[0])
        return scored[0][1:]

    def send_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw)
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return False, "rejected"
        result_future = handle.get_result_async()
        deadline = time.monotonic() + self.goal_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.2)
            if result_future.done():
                result = result_future.result()
                if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
                    return True, "succeeded"
                return False, f"status {result.status if result else 'none'}"
        rclpy.spin_until_future_complete(self, handle.cancel_goal_async(), timeout_sec=3.0)
        return False, "timeout"

    def known_ratio(self):
        if self.grid is None or not self.grid.data:
            return 0.0
        return sum(1 for value in self.grid.data if value >= 0) / len(self.grid.data)

    def return_home(self):
        if self.home_pose is None:
            return
        x, y = self.home_pose
        robot_x, robot_y = self.robot_xy()
        self.get_logger().warn("Returning home after exploration stop condition")
        self.send_goal(x, y, math.atan2(y - robot_y, x - robot_x))

    def run(self):
        if not self.wait_ready():
            raise RuntimeError("Timed out waiting for terrain grid, TF, and Nav2 actions")
        for goal_index in range(1, self.max_goals + 1):
            if self.known_ratio() >= self.min_known_ratio:
                self.get_logger().info(f"Exploration complete: known_ratio={self.known_ratio():.3f}")
                self.return_home()
                return
            if self.consecutive_failures >= self.failure_return_threshold:
                self.return_home()
                return
            selected = self.select_goal()
            if selected is None:
                self.get_logger().info("No reachable terrain-aware frontier remains")
                self.return_home()
                return
            x, y, yaw, cluster_size = selected
            self.get_logger().info(
                f"Goal {goal_index}/{self.max_goals}: ({x:.2f}, {y:.2f}), "
                f"cluster={cluster_size}, known={self.known_ratio():.3f}"
            )
            success, reason = self.send_goal(x, y, yaw)
            if success:
                self.completed.append((x, y))
                self.consecutive_failures = 0
            else:
                self.blacklist.append((x, y))
                self.consecutive_failures += 1
                self.get_logger().warn(f"Frontier goal failed: {reason}")
            end = time.monotonic() + self.settle_time
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
        self.return_home()


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        node.run()
    except Exception as error:  # noqa: BLE001
        node.get_logger().error(str(error))
        node.destroy_node()
        rclpy.shutdown()
        return 1
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
