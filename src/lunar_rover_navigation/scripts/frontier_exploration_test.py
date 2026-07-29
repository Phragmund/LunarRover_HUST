#!/usr/bin/env python3
"""Frontier-based online mapping navigation test without Gazebo truth."""

import collections
import math
import sys
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


class FrontierExplorationTest(Node):
    def __init__(self):
        super().__init__("frontier_exploration_test")
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map = None
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, map_qos
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.path_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

        self.free_threshold = self.declare_parameter("free_threshold", 25).value
        self.occupied_threshold = self.declare_parameter("occupied_threshold", 65).value
        self.min_cluster_size = self.declare_parameter("min_cluster_size", 6).value
        self.max_goals = self.declare_parameter("max_goals", 14).value
        self.goal_timeout = self.declare_parameter("goal_timeout", 75.0).value
        self.settle_time = self.declare_parameter("settle_time", 2.0).value
        self.goal_blacklist_radius = self.declare_parameter(
            "goal_blacklist_radius", 0.35
        ).value
        self.robot_clearance = self.declare_parameter("robot_clearance", 0.10).value
        self.goal_backoff = self.declare_parameter("goal_backoff", 0.35).value
        self.map_edge_margin = self.declare_parameter("map_edge_margin", 0.35).value
        self.cluster_gain_weight = self.declare_parameter(
            "cluster_gain_weight", 0.035
        ).value
        self.max_candidates_per_cluster = self.declare_parameter(
            "max_candidates_per_cluster", 8
        ).value
        self.max_path_checks = self.declare_parameter("max_path_checks", 18).value
        self.path_check_timeout = self.declare_parameter("path_check_timeout", 4.0).value
        self.max_navigation_step = self.declare_parameter(
            "max_navigation_step", 0.45
        ).value
        self.empty_frontier_retries = self.declare_parameter(
            "empty_frontier_retries", 5
        ).value
        self.min_known_ratio = self.declare_parameter("min_known_ratio", 0.82).value

        self.blacklisted_goals = []
        self.completed_goals = []

    def _map_callback(self, message):
        self.map = message

    def wait_until_ready(self):
        deadline = time.monotonic() + 45.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.map is None:
                continue
            if not self.client.wait_for_server(timeout_sec=0.1):
                continue
            if not self.path_client.wait_for_server(timeout_sec=0.1):
                continue
            try:
                self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                return True
            except TransformException:
                continue
        return False

    def robot_xy(self):
        transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def cell_index(self, column, row):
        return row * self.map.info.width + column

    def in_bounds(self, column, row):
        return 0 <= column < self.map.info.width and 0 <= row < self.map.info.height

    def cell_value(self, column, row):
        return self.map.data[self.cell_index(column, row)]

    def is_free(self, column, row):
        value = self.cell_value(column, row)
        return 0 <= value <= self.free_threshold

    def is_unknown(self, column, row):
        return self.cell_value(column, row) < 0

    def is_occupied(self, column, row):
        return self.cell_value(column, row) >= self.occupied_threshold

    def world_to_cell(self, x, y):
        resolution = self.map.info.resolution
        origin = self.map.info.origin.position
        return (
            int(math.floor((x - origin.x) / resolution)),
            int(math.floor((y - origin.y) / resolution)),
        )

    def cell_to_world(self, column, row):
        resolution = self.map.info.resolution
        origin = self.map.info.origin.position
        return (
            origin.x + (column + 0.5) * resolution,
            origin.y + (row + 0.5) * resolution,
        )

    def neighbors8(self, column, row):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                next_column = column + dx
                next_row = row + dy
                if self.in_bounds(next_column, next_row):
                    yield next_column, next_row

    def is_frontier_free_cell(self, column, row):
        if not self.is_free(column, row):
            return False
        return any(self.is_unknown(nx, ny) for nx, ny in self.neighbors8(column, row))

    def is_clear_enough(self, column, row):
        radius_cells = max(
            1, int(math.ceil(self.robot_clearance / self.map.info.resolution))
        )
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                nx = column + dx
                ny = row + dy
                if not self.in_bounds(nx, ny):
                    return False
                if self.is_occupied(nx, ny):
                    return False
        return True

    def is_inside_map_margin(self, column, row):
        margin_cells = max(1, int(math.ceil(self.map_edge_margin / self.map.info.resolution)))
        return (
            margin_cells <= column < self.map.info.width - margin_cells
            and margin_cells <= row < self.map.info.height - margin_cells
        )

    def is_blacklisted(self, x, y):
        for bx, by in self.blacklisted_goals + self.completed_goals:
            if math.hypot(x - bx, y - by) < self.goal_blacklist_radius:
                return True
        return False

    def reachable_free_cells(self, start_column, start_row):
        if not self.in_bounds(start_column, start_row) or not self.is_free(start_column, start_row):
            return set()
        reachable = set()
        queue = collections.deque([(start_column, start_row)])
        reachable.add((start_column, start_row))
        while queue:
            column, row = queue.popleft()
            for nx, ny in self.neighbors8(column, row):
                if (nx, ny) in reachable:
                    continue
                if not self.is_free(nx, ny):
                    continue
                reachable.add((nx, ny))
                queue.append((nx, ny))
        return reachable

    def frontier_clusters(self, reachable):
        frontier_cells = {
            cell for cell in reachable if self.is_frontier_free_cell(cell[0], cell[1])
        }
        clusters = []
        while frontier_cells:
            seed = frontier_cells.pop()
            cluster = [seed]
            queue = collections.deque([seed])
            while queue:
                column, row = queue.popleft()
                for neighbor in self.neighbors8(column, row):
                    if neighbor not in frontier_cells:
                        continue
                    frontier_cells.remove(neighbor)
                    cluster.append(neighbor)
                    queue.append(neighbor)
            if len(cluster) >= self.min_cluster_size:
                clusters.append(cluster)
        return clusters

    def backed_off_candidates(self, cluster, reachable):
        min_backoff_cells = max(1, int(math.ceil(self.goal_backoff / self.map.info.resolution)))
        max_backoff_cells = max(min_backoff_cells, int(math.ceil(1.6 * self.goal_backoff / self.map.info.resolution)))
        cluster_set = set(cluster)
        queue = collections.deque((cell, 0) for cell in cluster)
        visited = set(cluster)
        candidates = []
        while queue:
            cell, distance = queue.popleft()
            if distance >= min_backoff_cells and cell not in cluster_set:
                candidates.append((cell, distance))
            if distance >= max_backoff_cells:
                continue
            column, row = cell
            for neighbor in self.neighbors8(column, row):
                if neighbor in visited or neighbor not in reachable:
                    continue
                visited.add(neighbor)
                queue.append((neighbor, distance + 1))
        return candidates

    def select_goal(self):
        robot_x, robot_y = self.robot_xy()
        free_cells = {
            (column, row)
            for row in range(self.map.info.height)
            for column in range(self.map.info.width)
            if self.is_free(column, row)
        }
        if not free_cells:
            return None

        goal_options = []
        for cluster in self.frontier_clusters(free_cells):
            centroid_column = sum(cell[0] for cell in cluster) / len(cluster)
            centroid_row = sum(cell[1] for cell in cluster) / len(cluster)
            candidates = sorted(
                self.backed_off_candidates(cluster, free_cells),
                key=lambda item: abs(item[1] - int(math.ceil(self.goal_backoff / self.map.info.resolution)))
                + 0.03
                * ((item[0][0] - centroid_column) ** 2 + (item[0][1] - centroid_row) ** 2),
            )
            added = 0
            for (column, row), _ in candidates:
                if not self.is_inside_map_margin(column, row):
                    continue
                if not self.is_clear_enough(column, row):
                    continue
                x, y = self.cell_to_world(column, row)
                if self.is_blacklisted(x, y):
                    continue
                distance = math.hypot(x - robot_x, y - robot_y)
                score = distance - self.cluster_gain_weight * math.sqrt(len(cluster))
                goal_options.append(
                    (score, x, y, math.atan2(y - robot_y, x - robot_x), len(cluster))
                )
                added += 1
                if added >= self.max_candidates_per_cluster:
                    break

        goal_options.sort(key=lambda item: item[0])
        for _, x, y, yaw, cluster_size in goal_options[: self.max_path_checks]:
            step_goal = self.planned_step_goal(x, y, yaw)
            if step_goal is not None:
                step_x, step_y, step_yaw = step_goal
                return step_x, step_y, step_yaw, cluster_size
            self.blacklisted_goals.append((x, y))
            self.get_logger().warn(f"Rejected frontier candidate ({x:.2f}, {y:.2f}): no path")
        return None

    def map_stats(self):
        total = len(self.map.data)
        unknown = sum(1 for value in self.map.data if value < 0)
        occupied = sum(1 for value in self.map.data if value >= self.occupied_threshold)
        free = total - unknown - occupied
        known_ratio = 1.0 - unknown / total if total else 0.0
        width_m = self.map.info.width * self.map.info.resolution
        height_m = self.map.info.height * self.map.info.resolution
        return known_ratio, free, occupied, unknown, width_m, height_m

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    def truncate_path_to_step(self, poses):
        if len(poses) < 2:
            return None
        previous = poses[0].pose.position
        traveled = 0.0
        selected = poses[-1].pose.position
        selected_previous = previous
        for pose in poses[1:]:
            current = pose.pose.position
            segment = math.hypot(current.x - previous.x, current.y - previous.y)
            if traveled + segment >= self.max_navigation_step:
                selected = current
                selected_previous = previous
                break
            traveled += segment
            previous = current

        yaw = math.atan2(selected.y - selected_previous.y, selected.x - selected_previous.x)
        return selected.x, selected.y, yaw

    def planned_step_goal(self, x, y, yaw):
        goal = ComputePathToPose.Goal()
        goal.goal = self.make_pose(x, y, yaw)
        goal.planner_id = "GridBased"
        goal.use_start = False

        send_future = self.path_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=2.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + self.path_check_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.1)
            if result_future.done():
                result = result_future.result()
                feasible = (
                    result is not None
                    and result.status == GoalStatus.STATUS_SUCCEEDED
                    and len(result.result.path.poses) > 1
                )
                if not feasible:
                    return None
                return self.truncate_path_to_step(result.result.path.poses)
        return None

    def send_goal(self, x, y, yaw):
        pose = self.make_pose(x, y, yaw)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, "rejected"

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + self.goal_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.2)
            if result_future.done():
                result = result_future.result()
                if result is None:
                    return False, "no result"
                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    return True, "succeeded"
                return False, f"status {result.status}"

        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
        return False, "timeout"

    def run(self):
        if not self.wait_until_ready():
            raise RuntimeError("Timed out waiting for Nav2, /map, and map->base_link")

        for goal_index in range(1, self.max_goals + 1):
            rclpy.spin_once(self, timeout_sec=0.2)
            known_ratio, free, occupied, unknown, width_m, height_m = self.map_stats()
            self.get_logger().info(
                f"Map {width_m:.2f} m x {height_m:.2f} m, "
                f"known {known_ratio * 100.0:.1f}%, "
                f"free={free} occupied={occupied} unknown={unknown}"
            )
            if known_ratio >= self.min_known_ratio:
                self.get_logger().info(
                    f"Stopping: known map ratio {known_ratio * 100.0:.1f}% "
                    f"reached target {self.min_known_ratio * 100.0:.1f}%"
                )
                return

            selected = self.select_goal()
            retry = 0
            while selected is None and retry < self.empty_frontier_retries:
                retry += 1
                self.get_logger().info(
                    f"No valid frontier goal yet; waiting for map update "
                    f"({retry}/{self.empty_frontier_retries})"
                )
                end_time = time.monotonic() + self.settle_time
                while rclpy.ok() and time.monotonic() < end_time:
                    rclpy.spin_once(self, timeout_sec=0.1)
                selected = self.select_goal()
            if selected is None:
                self.get_logger().info("Stopping: no reachable frontier goal remains")
                return
            x, y, yaw, cluster_size = selected
            self.get_logger().info(
                f"Goal {goal_index}/{self.max_goals}: "
                f"frontier at ({x:.2f}, {y:.2f}), cluster={cluster_size}"
            )
            success, reason = self.send_goal(x, y, yaw)
            if success:
                self.completed_goals.append((x, y))
                self.get_logger().info(f"Goal {goal_index} succeeded")
            else:
                self.blacklisted_goals.append((x, y))
                self.get_logger().warn(f"Goal {goal_index} failed: {reason}")

            end_time = time.monotonic() + self.settle_time
            while rclpy.ok() and time.monotonic() < end_time:
                rclpy.spin_once(self, timeout_sec=0.1)

        known_ratio, free, occupied, unknown, width_m, height_m = self.map_stats()
        self.get_logger().info(
            f"Finished after {self.max_goals} attempted goals. "
            f"Map {width_m:.2f} m x {height_m:.2f} m, "
            f"known {known_ratio * 100.0:.1f}%, "
            f"free={free} occupied={occupied} unknown={unknown}"
        )


def main():
    rclpy.init()
    node = FrontierExplorationTest()
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
