#!/usr/bin/env python3
"""Terrain-aware frontier exploration with async goals and dead-end backtracking.

Key improvements over the original:
- Async goal sending: no blocking wait during navigation.
- Fork-point recording: when multiple frontier clusters exist the robot position
  is saved so the explorer can backtrack later instead of returning all the way home.
- Single-phase scoring: every candidate is scored via a real Nav2 path query;
  the misleading Euclidean pre-sort has been removed.
- Cluster-aware blacklisting: failing one candidate blacklists the whole frontier
  cluster, avoiding repeated failures against the same unreachable region.
"""

import collections
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class PathQueryHelper:
    """Runs a tiny ROS node solely for calling /compute_path_to_pose.

    Uses its own SingleThreadedExecutor so that spin_until_future_complete
    can be safely called from any thread — even from within a timer callback
    of the main node.  No background spin thread is used; the executor is
    only driven during query calls.
    """

    def __init__(self):
        self._node = Node(
            "_path_query_helper",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
        )
        self._client = ActionClient(self._node, ComputePathToPose, "/compute_path_to_pose")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        # Spin once to check server availability (does not start background spin)
        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("PathQueryHelper: /compute_path_to_pose not available")
            raise RuntimeError("PathQueryHelper: /compute_path_to_pose not available")

    def get_logger(self):
        return self._node.get_logger()

    def compute_path(self, x, y, yaw, frame_id, timeout_sec=4.0):
        """Block the calling thread until the planner returns.

        Returns ``(poses, planning_time)`` on success or ``(None, 0.0)`` on
        any failure.
        """
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = frame_id
        goal.goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.z = math.sin(0.5 * yaw)
        goal.goal.pose.orientation.w = math.cos(0.5 * yaw)
        goal.planner_id = "GridBased"
        goal.use_start = False

        future = self._client.send_goal_async(goal)
        self._executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        if not future.done():
            return None, 0.0
        handle = future.result()
        if handle is None or not handle.accepted:
            return None, 0.0
        result_future = handle.get_result_async()
        self._executor.spin_until_future_complete(result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            return None, 0.0
        result = result_future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            return None, 0.0
        return result.result.path.poses, float(
            result.result.planning_time.sec + 1e-9 * result.result.planning_time.nanosec
        )

    def shutdown(self):
        self._executor.shutdown()
        self._node.destroy_node()


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
        self.mission_enabled = True
        self.active_goal_handle = None
        self.create_subscription(
            Bool, "/mission/exploration_enabled", self.on_mission_enabled, 10
        )
        # Use a helper node (separate executor) for path queries to avoid
        # the executor-conflict problem when spin_until_future_complete is
        # called from within a timer callback.
        self._path_helper = PathQueryHelper()

        # ── tunable parameters ──────────────────────────────────────────
        self.free_cost = int(self.declare_parameter("free_cost", 35).value)
        self.max_goal_cost = int(self.declare_parameter("max_goal_cost", 60).value)
        self.min_cluster_size = int(self.declare_parameter("min_cluster_size", 3).value)
        self.max_goals = int(self.declare_parameter("max_goals", 40).value)
        self.goal_timeout = float(self.declare_parameter("goal_timeout", 100.0).value)
        self.path_timeout = float(self.declare_parameter("path_timeout", 4.0).value)
        self.settle_time = float(self.declare_parameter("settle_time", 1.5).value)
        self.goal_backoff = float(self.declare_parameter("goal_backoff", 0.45).value)
        self.blacklist_radius = float(self.declare_parameter("blacklist_radius", 0.45).value)
        self.max_path_checks = int(self.declare_parameter("max_path_checks", 32).value)
        self.info_gain_weight = float(self.declare_parameter("info_gain_weight", 0.07).value)
        self.terrain_risk_weight = float(
            self.declare_parameter("terrain_risk_weight", 0.018).value
        )
        self.return_risk_weight = float(
            self.declare_parameter("return_risk_weight", 0.20).value
        )
        self.min_known_ratio = float(self.declare_parameter("min_known_ratio", 0.55).value)
        self.failure_return_threshold = int(
            self.declare_parameter("failure_return_threshold", 6).value
        )
        self.start_search_radius = float(
            self.declare_parameter("start_search_radius", 1.2).value
        )
        self.min_goal_distance = float(self.declare_parameter("min_goal_distance", 1.00).value)
        self.no_frontier_wait = float(self.declare_parameter("no_frontier_wait", 8.0).value)
        self.max_no_frontier_cycles = int(
            self.declare_parameter("max_no_frontier_cycles", 6).value
        )
        self.home_frame = self.declare_parameter("home_frame", "map").value
        self.robot_frame = self.declare_parameter("robot_frame", "base_link").value

        # ── runtime state ───────────────────────────────────────────────
        self.blacklist = []          # list of (x, y) world coords
        self.cluster_blacklist = []  # list of (cx, cy) cluster centroids
        self.home_pose = None
        self.consecutive_failures = 0
        self.no_frontier_cycles = 0
        self.state = "IDLE"          # IDLE | NAVIGATING | SETTLING | RETURNING
        self.current_goal = None     # (x, y) of active goal
        self.goal_index = 0
        self.settle_deadline = 0.0
        self.fork_stack = []         # [(x, y, set of explored cluster centroids), ...]
        self._return_triggered = False
        self._selecting = False  # guard against re-entrant select_goal calls

        # Dedicated MultiThreadedExecutor — avoids the global executor and its
        # timer-priority starvation of action client waitables.
        from rclpy.executors import MultiThreadedExecutor
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)

    def on_mission_enabled(self, message):
        enabled = bool(message.data)
        if enabled == self.mission_enabled:
            return
        self.mission_enabled = enabled
        if not enabled:
            if self.active_goal_handle is not None:
                self.active_goal_handle.cancel_goal_async()
            self.current_goal = None
            self.state = "IDLE"
            self.get_logger().info("Frontier exploration paused by mission executive")
        else:
            self.get_logger().info("Frontier exploration enabled by mission executive")

    # ── grid / TF helpers ───────────────────────────────────────────────

    def on_grid(self, msg):
        self.grid = msg

    def wait_ready(self):
        """Spin until terrain grid, TF, and Nav2 actions are available."""
        deadline = time.monotonic() + 60.0
        while rclpy.ok() and time.monotonic() < deadline:
            self._executor.spin_once(timeout_sec=0.1)
            if self.grid is None:
                continue
            if not self.nav_client.wait_for_server(timeout_sec=0.1):
                continue
            try:
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

    # ── reachability & frontier detection ──────────────────────────────

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
        if not candidates:
            candidates = [cell for cell in cluster if cell in reachable and self.is_goal_cell(*cell)]
        candidates.sort()
        return candidates

    def cluster_centroid(self, cluster):
        if not cluster:
            return (0.0, 0.0)
        cx = sum(c[0] for c in cluster) / len(cluster)
        cy = sum(c[1] for c in cluster) / len(cluster)
        # Return world coordinates so the meter-based blacklist radius in
        # is_cluster_blacklisted() compares like-for-like.  The previous
        # cell-coordinate return made blacklist_radius*2 (0.9 m) act as
        # 0.9 *cells*, so the cluster-aware blacklisting never actually
        # covered neighbouring frontier clusters.
        return self.cell_to_world(cx, cy)

    # ── blacklisting ────────────────────────────────────────────────────

    def is_blacklisted(self, x, y):
        if any(math.hypot(x - bx, y - by) < self.blacklist_radius for bx, by in self.blacklist):
            return True
        return False

    def is_cluster_blacklisted(self, centroid):
        return any(
            math.hypot(centroid[0] - cx, centroid[1] - cy) < self.blacklist_radius * 2.0
            for cx, cy in self.cluster_blacklist
        )

    def blacklist_cluster(self, centroid):
        self.cluster_blacklist.append(centroid)
        self.get_logger().info(
            f"Blacklisting cluster near ({centroid[0]:.2f}, {centroid[1]:.2f})"
        )

    # ── pose / path helpers ─────────────────────────────────────────────

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
        """Score a candidate goal by querying Nav2 for a real path.

        Returns (combined_score, path_length) or None when the planner
        cannot find a path.

        Uses a separate PathQueryHelper node/executor to avoid the
        executor-conflict problem when called from within a timer callback.
        """
        frame_id = self.grid.header.frame_id
        poses, planning_time = self._path_helper.compute_path(
            x, y, yaw, frame_id, timeout_sec=self.path_timeout
        )
        if poses is None or len(poses) < 2:
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

    # ── goal selection with fork-aware backtracking ─────────────────────

    def select_goal(self):
        """Find the best frontier goal, recording fork points on the way.

        Returns (x, y, yaw, cluster_size) or None when no reachable frontier
        remains.  Callers should then attempt backtracking via the fork stack.
        """
        if self.grid is None:
            return None
        self._selecting = True
        try:
            return self._select_goal_impl()
        finally:
            self._selecting = False

    def _select_goal_impl(self):
        if self.grid is None:
            return None
        try:
            robot_x, robot_y = self.robot_xy()
        except TransformException:
            return None
        start = self.world_to_cell(robot_x, robot_y)
        start = self.nearest_free_cell(start)
        if start is None:
            self.get_logger().warn("No free terrain cell near robot; cannot seed frontier search")
            return None
        reachable = self.reachable_free(start)
        clusters = self.frontier_clusters(reachable)

        # ── record fork point when multiple clusters exist ──────────────
        if len(clusters) >= 2:
            fork_key = (round(robot_x, 2), round(robot_y, 2))
            if not self.fork_stack or (
                abs(self.fork_stack[-1][0] - fork_key[0]) > 0.5
                and abs(self.fork_stack[-1][1] - fork_key[1]) > 0.5
            ):
                self.fork_stack.append((robot_x, robot_y, set()))
                self.get_logger().info(
                    f"Fork recorded at ({robot_x:.2f}, {robot_y:.2f}) "
                    f"with {len(clusters)} frontier clusters"
                )

        # ── build candidates across all non-blacklisted clusters ────────
        all_candidates = []  # (x, y, yaw, cluster_size, centroid)
        for cluster in clusters:
            centroid = self.cluster_centroid(cluster)
            if self.is_cluster_blacklisted(centroid):
                continue
            # Skip clusters already explored from the current fork
            if self.fork_stack and centroid in self.fork_stack[-1][2]:
                continue
            for cell in self.backed_off_candidates(cluster, reachable):
                x, y = self.cell_to_world(*cell)
                if self.is_blacklisted(x, y):
                    continue
                yaw = math.atan2(y - robot_y, x - robot_x)
                all_candidates.append((x, y, yaw, len(cluster), centroid))

        if not all_candidates:
            return None

        # Filter candidates too close to the robot
        filtered = [
            c for c in all_candidates
            if math.hypot(c[0] - robot_x, c[1] - robot_y) >= self.min_goal_distance
        ]
        candidates = filtered if filtered else all_candidates

        # ── single-phase path scoring (no misleading Euclidean pre-sort) ─
        scored = []
        for x, y, yaw, cluster_size, centroid in candidates[: self.max_path_checks]:
            score = self.path_score(x, y, yaw, cluster_size)
            if score is None:
                # Planner could not find a path — blacklist the whole cluster
                self.blacklist_cluster(centroid)
                continue
            scored.append((score, x, y, yaw, cluster_size, centroid))

        if not scored:
            return None

        scored.sort(key=lambda item: item[0])
        best = scored[0]

        # Mark cluster as visited in the current fork
        if self.fork_stack:
            self.fork_stack[-1][2].add(best[5])

        self.get_logger().info(
            f"Selected goal {self.goal_index + 1}/{self.max_goals}: "
            f"({best[1]:.2f}, {best[2]:.2f}), cluster_size={best[4]}, "
            f"score={best[0]:.2f}, known={self.known_ratio():.3f}"
        )
        return best[1:5]  # x, y, yaw, cluster_size

    # ── async goal dispatch ─────────────────────────────────────────────

    def send_goal_async(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw)
        self.current_goal = (x, y)
        self.state = "NAVIGATING"
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        if self.state == "RETURNING":
            return  # shutdown in progress
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("NavigateToPose goal rejected by Nav2")
            if self.current_goal is not None:
                self.blacklist.append(self.current_goal)
            self.consecutive_failures += 1
            self._enter_settle()
            return
        self.active_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self.active_goal_handle = None
        if self.state == "RETURNING":
            return
        result = future.result()
        if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.consecutive_failures = 0
            self.get_logger().info("Goal reached successfully")
        else:
            status_str = f"status {result.status if result else 'none'}"
            self.get_logger().warn(f"Goal failed: {status_str}")
            if self.current_goal is not None:
                self.blacklist.append(self.current_goal)
            self.consecutive_failures += 1
        self.goal_index += 1
        self._enter_settle()

    def _enter_settle(self):
        self.state = "SETTLING"
        self.settle_deadline = time.monotonic() + self.settle_time
        self.current_goal = None

    # ── known ratio & diagnostics ───────────────────────────────────────

    def known_ratio(self):
        if self.grid is None or not self.grid.data:
            return 0.0
        return sum(1 for value in self.grid.data if value >= 0) / len(self.grid.data)

    # ── backtracking ────────────────────────────────────────────────────

    def try_backtrack(self):
        """Pop the most recent fork and navigate back to it.

        Returns True when a backtrack goal was dispatched, False when the
        fork stack is empty (no more forks to try).
        """
        # Clean exhausted forks (all clusters explored or blacklisted)
        while self.fork_stack:
            _x, _y, explored = self.fork_stack[-1]
            # We cannot know remaining clusters without re-detecting frontiers
            # at the fork location.  We simply navigate to the fork; once
            # arrived select_goal() will pick from whatever remains.
            if not explored:
                # Fork was recorded but we never consumed any branch — stale.
                self.fork_stack.pop()
                continue
            break

        if not self.fork_stack:
            return False

        fork_x, fork_y, explored = self.fork_stack.pop()
        self.get_logger().info(
            f"Backtracking to fork ({fork_x:.2f}, {fork_y:.2f}), "
            f"{len(explored)} branch(es) already visited"
        )
        try:
            robot_x, robot_y = self.robot_xy()
        except TransformException:
            return False
        yaw = math.atan2(fork_y - robot_y, fork_x - robot_x)
        self.send_goal_async(fork_x, fork_y, yaw)
        return True

    # ── return home ─────────────────────────────────────────────────────

    def return_home(self):
        if self.home_pose is None:
            self.get_logger().error("No home pose recorded; cannot return")
            return
        if self._return_triggered:
            return
        self._return_triggered = True
        x, y = self.home_pose
        try:
            robot_x, robot_y = self.robot_xy()
        except TransformException:
            self.get_logger().warn("Cannot determine robot pose for return home")
            return
        dist = math.hypot(x - robot_x, y - robot_y)
        if dist < 0.5:
            self.get_logger().info("Already home")
            return
        self.get_logger().info(f"Returning home, distance={dist:.2f} m")
        self.state = "RETURNING"
        yaw = math.atan2(y - robot_y, x - robot_x)
        self.send_goal_async(x, y, yaw)

    def wait_for_map_update(self):
        deadline = time.monotonic() + self.no_frontier_wait
        while rclpy.ok() and time.monotonic() < deadline:
            self._executor.spin_once(timeout_sec=0.1)

    # ── main exploration loop (timer-driven) ────────────────────────────

    def exploration_tick(self):
        """Called periodically.  Advances the exploration state machine."""
        if not rclpy.ok():
            return
        if not self.mission_enabled:
            return

        # Prevent re-entrancy: select_goal → path_score can block the executor
        # with spin_until_future_complete, and queued timer callbacks would
        # otherwise re-enter select_goal and corrupt the ActionClient state.
        if self._selecting:
            return

        # Let active navigation finish
        if self.state == "NAVIGATING":
            return
        if self.state == "SETTLING":
            if time.monotonic() < self.settle_deadline:
                return
            self.state = "IDLE"
        if self.state == "RETURNING":
            # The goal result callback will fire when we arrive home.
            return

        # ── stop conditions ─────────────────────────────────────────────
        if self.known_ratio() >= self.min_known_ratio:
            self.get_logger().info(
                f"Exploration complete: known_ratio={self.known_ratio():.3f}"
            )
            self.return_home()
            return

        if self.consecutive_failures >= self.failure_return_threshold:
            self.get_logger().warn(
                f"Exceeded {self.failure_return_threshold} consecutive failures; "
                "returning home"
            )
            self.return_home()
            return

        if self.goal_index >= self.max_goals:
            self.get_logger().info(f"Reached max goals ({self.max_goals})")
            self.return_home()
            return

        # ── select next goal ────────────────────────────────────────────
        selected = self.select_goal()
        if selected is not None:
            self.no_frontier_cycles = 0
            x, y, yaw, _cluster_size = selected
            self.send_goal_async(x, y, yaw)
            return

        # ── no reachable frontier — try backtrack or wait ───────────────
        if self.try_backtrack():
            self.no_frontier_cycles = 0
            return

        self.no_frontier_cycles += 1
        if self.no_frontier_cycles >= self.max_no_frontier_cycles:
            self.get_logger().info(
                "No reachable terrain-aware frontier remains "
                f"after {self.max_no_frontier_cycles} cycles; returning home"
            )
            self.return_home()
            return

        self.get_logger().warn(
            "No reachable frontier in current grid; waiting for map update "
            f"({self.no_frontier_cycles}/{self.max_no_frontier_cycles})"
        )
        self.wait_for_map_update()

    def run(self):
        if not self.wait_ready():
            raise RuntimeError("Timed out waiting for terrain grid, TF, and Nav2 actions")
        self.home_pose = self.robot_xy()
        self.get_logger().info(
            f"Frontier explorer ready. Home: ({self.home_pose[0]:.2f}, {self.home_pose[1]:.2f})"
        )
        # Drive the state machine at 2 Hz; action callbacks handle goal completion.
        self.create_timer(0.5, self.exploration_tick)
        self._executor.spin()


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
