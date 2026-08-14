#!/usr/bin/env python3
"""Build a 2.5D traversability grid from registered LiDAR points."""

import math
from dataclasses import dataclass, field

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class TerrainStats:
    count: int = 0
    min_z: float = math.inf
    max_z: float = -math.inf
    sum_z: float = 0.0
    sum_z2: float = 0.0
    ceiling_filtered: bool = False

    @property
    def mean_z(self):
        return self.sum_z / self.count if self.count else math.nan

    @property
    def variance(self):
        if self.count <= 1:
            return 0.0
        mean = self.mean_z
        return max(0.0, self.sum_z2 / self.count - mean * mean)


@dataclass
class TerrainCell:
    count: int = 0
    free_count: int = 0
    min_z: float = math.inf
    max_z: float = -math.inf
    sum_z: float = 0.0
    sum_z2: float = 0.0
    last_seen: float = 0.0
    z_bins: dict = field(default_factory=dict)

    def add(self, z, stamp, z_bin=None):
        self.count += 1
        self.min_z = min(self.min_z, z)
        self.max_z = max(self.max_z, z)
        self.sum_z += z
        self.sum_z2 += z * z
        self.last_seen = stamp
        if z_bin is not None:
            stats = self.z_bins.get(z_bin)
            if stats is None:
                self.z_bins[z_bin] = [1, z, z * z, z, z]
            else:
                stats[0] += 1
                stats[1] += z
                stats[2] += z * z
                stats[3] = min(stats[3], z)
                stats[4] = max(stats[4], z)

    def mark_free(self, stamp):
        self.free_count += 1
        self.last_seen = stamp

    def reset(self):
        self.count = 0
        self.free_count = 0
        self.min_z = math.inf
        self.max_z = -math.inf
        self.sum_z = 0.0
        self.sum_z2 = 0.0
        self.last_seen = 0.0
        self.z_bins.clear()

    @property
    def mean_z(self):
        return self.sum_z / self.count if self.count else math.nan

    @property
    def variance(self):
        if self.count <= 1:
            return 0.0
        mean = self.mean_z
        return max(0.0, self.sum_z2 / self.count - mean * mean)


def q_tuple(q):
    if isinstance(q, Quaternion):
        return (q.x, q.y, q.z, q.w)
    return q


def q_normalize(q):
    x, y, z, w = q_tuple(q)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def q_inverse(q):
    x, y, z, w = q_normalize(q)
    return (-x, -y, -z, w)


def q_multiply(a, b):
    ax, ay, az, aw = q_tuple(a)
    bx, by, bz, bw = q_tuple(b)
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate(q, vector):
    rotated = q_multiply(
        q_multiply(q_normalize(q), (vector[0], vector[1], vector[2], 0.0)),
        q_inverse(q),
    )
    return rotated[0], rotated[1], rotated[2]


class TerrainMapper(Node):
    def __init__(self):
        super().__init__("terrain_mapper")
        self.input_topic = self.declare_parameter("input_topic", "/livox/points").value
        self.traversability_topic = self.declare_parameter(
            "traversability_topic", "/terrain/traversability"
        ).value
        self.diagnostics_topic = self.declare_parameter(
            "diagnostics_topic", "/terrain/diagnostics"
        ).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.sensor_origin_frame = self.declare_parameter(
            "sensor_origin_frame", self.base_frame
        ).value
        self.resolution = float(self.declare_parameter("resolution", 0.10).value)
        self.width = int(self.declare_parameter("width_cells", 400).value)
        self.height = int(self.declare_parameter("height_cells", 400).value)
        self.origin_x = float(self.declare_parameter("origin_x", -20.0).value)
        self.origin_y = float(self.declare_parameter("origin_y", -20.0).value)
        self.min_z = float(self.declare_parameter("min_z", -0.80).value)
        self.max_z = float(self.declare_parameter("max_z", 1.80).value)
        self.min_points = int(self.declare_parameter("min_points_per_cell", 2).value)
        self.cautious_step = float(self.declare_parameter("cautious_step_height", 0.06).value)
        self.lethal_step = float(self.declare_parameter("lethal_step_height", 0.10).value)
        self.max_slope_deg = float(self.declare_parameter("max_slope_deg", 8.0).value)
        self.lethal_slope_deg = float(
            self.declare_parameter(
                "lethal_slope_deg", max(self.max_slope_deg * 2.0, self.max_slope_deg + 8.0)
            ).value
        )
        self.roughness_warn = float(self.declare_parameter("roughness_warn", 0.025).value)
        self.ceiling_filter_enabled = bool(
            self.declare_parameter("ceiling_filter_enabled", True).value
        )
        self.ceiling_filter_bin_size = max(
            0.01, float(self.declare_parameter("ceiling_filter_bin_size", 0.05).value)
        )
        self.ceiling_filter_min_gap = float(
            self.declare_parameter("ceiling_filter_min_gap", 0.40).value
        )
        self.ceiling_filter_min_cluster_points = int(
            self.declare_parameter("ceiling_filter_min_cluster_points", 2).value
        )
        self.ceiling_filter_ground_cluster_max_height = float(
            self.declare_parameter("ceiling_filter_ground_cluster_max_height", 0.45).value
        )
        self.unknown_is_risky = bool(self.declare_parameter("unknown_is_risky", False).value)
        self.max_points_per_cloud = int(self.declare_parameter("max_points_per_cloud", 60000).value)
        self.publish_period = float(self.declare_parameter("publish_period", 0.5).value)
        self.rolling_window = bool(self.declare_parameter("rolling_window", True).value)
        self.recenter_margin = float(self.declare_parameter("recenter_margin_m", 6.0).value)
        self.cell_decay_time = float(self.declare_parameter("cell_decay_time", 20.0).value)
        self.raytrace_free_space = bool(
            self.declare_parameter("raytrace_free_space", True).value
        )
        self.raytrace_stride = max(1, int(self.declare_parameter("raytrace_stride", 6).value))
        self.raytrace_max_range = float(
            self.declare_parameter("raytrace_max_range", 6.0).value
        )
        self.raytrace_endpoint_margin = float(
            self.declare_parameter("raytrace_endpoint_margin", 0.20).value
        )
        self.self_filter_enabled = bool(
            self.declare_parameter("self_filter_enabled", True).value
        )
        self.self_filter_min_x = float(self.declare_parameter("self_filter_min_x", -0.45).value)
        self.self_filter_max_x = float(self.declare_parameter("self_filter_max_x", 0.45).value)
        self.self_filter_min_y = float(self.declare_parameter("self_filter_min_y", -0.35).value)
        self.self_filter_max_y = float(self.declare_parameter("self_filter_max_y", 0.35).value)
        self.self_filter_min_z = float(self.declare_parameter("self_filter_min_z", -0.35).value)
        self.self_filter_max_z = float(self.declare_parameter("self_filter_max_z", 0.35).value)
        self.clear_robot_footprint = bool(
            self.declare_parameter("clear_robot_footprint", True).value
        )

        self.cells = [TerrainCell() for _ in range(self.width * self.height)]
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        grid_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.grid_pub = self.create_publisher(
            OccupancyGrid, self.traversability_topic, grid_qos
        )
        self.diag_pub = self.create_publisher(String, self.diagnostics_topic, 2)
        self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, 10)
        self.create_timer(self.publish_period, self.publish_grid)
        self.last_cloud_stamp = None
        self.get_logger().info(
            f"Terrain mapper: {self.input_topic} -> {self.traversability_topic}, "
            f"{self.width}x{self.height}@{self.resolution:.2f}m"
        )

    @staticmethod
    def stamp_seconds(stamp):
        return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    def lookup_robot_xy(self):
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            self.get_clock().now().to_msg(),
            timeout=Duration(seconds=0.05),
        )
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        if not math.isfinite(x) or not math.isfinite(y):
            raise TransformException("robot pose contains non-finite translation")
        return x, y

    def maybe_recenter(self):
        if not self.rolling_window:
            return
        try:
            robot_x, robot_y = self.lookup_robot_xy()
        except TransformException as exc:
            self.get_logger().warn(
                f"Cannot recenter terrain map without {self.map_frame} <- {self.base_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        max_x = self.origin_x + self.width * self.resolution
        max_y = self.origin_y + self.height * self.resolution
        if (
            self.origin_x + self.recenter_margin <= robot_x <= max_x - self.recenter_margin
            and self.origin_y + self.recenter_margin <= robot_y <= max_y - self.recenter_margin
        ):
            return

        new_origin_x = math.floor(
            (robot_x - 0.5 * self.width * self.resolution) / self.resolution
        ) * self.resolution
        new_origin_y = math.floor(
            (robot_y - 0.5 * self.height * self.resolution) / self.resolution
        ) * self.resolution
        shift_cols = int(round((self.origin_x - new_origin_x) / self.resolution))
        shift_rows = int(round((self.origin_y - new_origin_y) / self.resolution))
        if shift_cols == 0 and shift_rows == 0:
            return

        old_cells = self.cells
        self.cells = [TerrainCell() for _ in range(self.width * self.height)]
        for old_row in range(self.height):
            new_row = old_row + shift_rows
            if new_row < 0 or new_row >= self.height:
                continue
            for old_col in range(self.width):
                new_col = old_col + shift_cols
                if new_col < 0 or new_col >= self.width:
                    continue
                self.cells[new_row * self.width + new_col] = old_cells[
                    old_row * self.width + old_col
                ]
        self.origin_x = new_origin_x
        self.origin_y = new_origin_y
        self.get_logger().info(
            f"Recentered terrain map origin to ({self.origin_x:.2f}, {self.origin_y:.2f})"
        )

    def cell_index(self, x, y):
        column = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        if column < 0 or column >= self.width or row < 0 or row >= self.height:
            return None
        return row * self.width + column

    def transform_point(self, transform, point):
        t = transform.transform.translation
        rotated = rotate(transform.transform.rotation, point)
        return rotated[0] + t.x, rotated[1] + t.y, rotated[2] + t.z

    def is_self_point(self, point_base):
        if not self.self_filter_enabled:
            return False
        x, y, z = point_base
        return (
            self.self_filter_min_x <= x <= self.self_filter_max_x
            and self.self_filter_min_y <= y <= self.self_filter_max_y
            and self.self_filter_min_z <= z <= self.self_filter_max_z
        )

    def clear_robot_cells(self, data):
        if not self.clear_robot_footprint:
            return
        try:
            now = self.get_clock().now().to_msg()
            map_from_base = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                now,
                timeout=Duration(seconds=0.05),
            )
            base_from_map = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.map_frame,
                now,
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Cannot clear robot footprint in terrain grid: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        corners_base = (
            (self.self_filter_min_x, self.self_filter_min_y, 0.0),
            (self.self_filter_min_x, self.self_filter_max_y, 0.0),
            (self.self_filter_max_x, self.self_filter_min_y, 0.0),
            (self.self_filter_max_x, self.self_filter_max_y, 0.0),
        )
        corners_map = [self.transform_point(map_from_base, corner) for corner in corners_base]
        min_x = min(point[0] for point in corners_map) - self.resolution
        max_x = max(point[0] for point in corners_map) + self.resolution
        min_y = min(point[1] for point in corners_map) - self.resolution
        max_y = max(point[1] for point in corners_map) + self.resolution
        min_col = max(0, int(math.floor((min_x - self.origin_x) / self.resolution)))
        max_col = min(self.width - 1, int(math.floor((max_x - self.origin_x) / self.resolution)))
        min_row = max(0, int(math.floor((min_y - self.origin_y) / self.resolution)))
        max_row = min(self.height - 1, int(math.floor((max_y - self.origin_y) / self.resolution)))

        for row in range(min_row, max_row + 1):
            y = self.origin_y + (row + 0.5) * self.resolution
            row_offset = row * self.width
            for column in range(min_col, max_col + 1):
                x = self.origin_x + (column + 0.5) * self.resolution
                bx, by, _bz = self.transform_point(base_from_map, (x, y, 0.0))
                if (
                    self.self_filter_min_x <= bx <= self.self_filter_max_x
                    and self.self_filter_min_y <= by <= self.self_filter_max_y
                ):
                    data[row_offset + column] = 0

    def mark_ray_free(self, origin, endpoint, stamp):
        dx = endpoint[0] - origin[0]
        dy = endpoint[1] - origin[1]
        distance = math.hypot(dx, dy)
        if distance <= self.resolution:
            return
        usable_distance = distance - max(0.0, self.raytrace_endpoint_margin)
        if self.raytrace_max_range > 0.0:
            usable_distance = min(usable_distance, self.raytrace_max_range)
        if usable_distance <= 0.0:
            return

        steps = max(1, int(math.floor(usable_distance / self.resolution)))
        previous_index = None
        for step in range(steps + 1):
            ratio = step / steps
            x = origin[0] + ratio * dx * usable_distance / distance
            y = origin[1] + ratio * dy * usable_distance / distance
            index = self.cell_index(x, y)
            if index is None or index == previous_index:
                continue
            self.cells[index].mark_free(stamp)
            previous_index = index

    def on_cloud(self, msg):
        self.maybe_recenter()
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=0.10),
            )
        except TransformException as exc:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    msg.header.frame_id,
                    self.get_clock().now().to_msg(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException:
                self.get_logger().warn(
                    f"Cannot transform cloud from {msg.header.frame_id} to {self.map_frame}: {exc}",
                    throttle_duration_sec=2.0,
                )
                return

        base_from_cloud = None
        if self.self_filter_enabled:
            try:
                base_from_cloud = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    msg.header.frame_id,
                    msg.header.stamp,
                    timeout=Duration(seconds=0.05),
                )
            except TransformException:
                try:
                    base_from_cloud = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        msg.header.frame_id,
                        self.get_clock().now().to_msg(),
                        timeout=Duration(seconds=0.05),
                    )
                except TransformException as exc:
                    self.get_logger().warn(
                        f"Cannot self-filter cloud from {msg.header.frame_id}: {exc}",
                        throttle_duration_sec=2.0,
                    )

        used = 0
        stamp = self.stamp_seconds(msg.header.stamp)
        sensor_origin = None
        if self.raytrace_free_space:
            try:
                origin_transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.sensor_origin_frame,
                    msg.header.stamp,
                    timeout=Duration(seconds=0.05),
                )
                sensor_origin = (
                    origin_transform.transform.translation.x,
                    origin_transform.transform.translation.y,
                    origin_transform.transform.translation.z,
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f"Cannot locate ray origin {self.sensor_origin_frame} in "
                    f"{self.map_frame}; raytracing disabled for this cloud: {exc}",
                    throttle_duration_sec=2.0,
                )
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if used >= self.max_points_per_cloud:
                break
            x, y, z = self.transform_point(transform, point)
            if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
                continue
            if base_from_cloud is not None and self.is_self_point(
                self.transform_point(base_from_cloud, point)
            ):
                continue
            if sensor_origin is not None and used % self.raytrace_stride == 0:
                self.mark_ray_free(sensor_origin, (x, y, z), stamp)
            if z < self.min_z or z > self.max_z:
                continue
            index = self.cell_index(x, y)
            if index is None:
                continue
            z_bin = int(math.floor(z / self.ceiling_filter_bin_size))
            self.cells[index].add(z, stamp, z_bin)
            used += 1
        self.last_cloud_stamp = msg.header.stamp

    def decay_stale_cells(self, now_seconds):
        if self.cell_decay_time <= 0.0:
            return
        for cell in self.cells:
            if cell.count and now_seconds - cell.last_seen > self.cell_decay_time:
                cell.reset()

    def raw_cell_stats(self, cell):
        return TerrainStats(
            count=cell.count,
            min_z=cell.min_z,
            max_z=cell.max_z,
            sum_z=cell.sum_z,
            sum_z2=cell.sum_z2,
        )

    @staticmethod
    def add_bin_stats(total, stats):
        total.count += stats[0]
        total.sum_z += stats[1]
        total.sum_z2 += stats[2]
        total.min_z = min(total.min_z, stats[3])
        total.max_z = max(total.max_z, stats[4])

    @staticmethod
    def clone_stats(stats):
        return TerrainStats(
            count=stats.count,
            min_z=stats.min_z,
            max_z=stats.max_z,
            sum_z=stats.sum_z,
            sum_z2=stats.sum_z2,
            ceiling_filtered=stats.ceiling_filtered,
        )

    def filtered_cell_stats(self, cell):
        raw = self.raw_cell_stats(cell)
        if (
            not self.ceiling_filter_enabled
            or cell.count < self.min_points
            or len(cell.z_bins) < 2
        ):
            return raw

        occupied_bins = sorted(cell.z_bins)
        best_lower = None
        best_gap = 0.0
        min_cluster_points = max(1, self.ceiling_filter_min_cluster_points)

        prefix = []
        running = TerrainStats()
        for bin_index in occupied_bins:
            self.add_bin_stats(running, cell.z_bins[bin_index])
            prefix.append(self.clone_stats(running))

        suffix = [None] * len(occupied_bins)
        running = TerrainStats()
        for pos in range(len(occupied_bins) - 1, -1, -1):
            self.add_bin_stats(running, cell.z_bins[occupied_bins[pos]])
            suffix[pos] = self.clone_stats(running)

        for pos in range(len(occupied_bins) - 1):
            lower = prefix[pos]
            upper = suffix[pos + 1]
            if lower.count < min_cluster_points or upper.count < min_cluster_points:
                continue
            vertical_gap = upper.min_z - lower.max_z
            if vertical_gap < self.ceiling_filter_min_gap:
                continue
            if lower.max_z - lower.min_z > self.ceiling_filter_ground_cluster_max_height:
                continue
            if vertical_gap > best_gap:
                best_gap = vertical_gap
                best_lower = lower

        if best_lower is None:
            return raw
        best_lower.ceiling_filtered = True
        return best_lower

    def neighbor_slope_cost(self, index, stats_cache):
        row = index // self.width
        column = index - row * self.width
        cell_stats = stats_cache[index]
        if cell_stats.count < self.min_points:
            return 0
        max_slope = 0.0
        unknown_neighbors = 0
        # 8-connected neighbourhood so diagonal slopes are also detected.
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nr = row + dy
                nc = column + dx
                if nr < 0 or nr >= self.height or nc < 0 or nc >= self.width:
                    continue
                neighbor = stats_cache[nr * self.width + nc]
                if neighbor.count < self.min_points:
                    unknown_neighbors += 1
                    continue
                dist = self.resolution * math.hypot(dx, dy)  # diagonal cells are farther
                slope = math.degrees(math.atan2(abs(cell_stats.mean_z - neighbor.mean_z), dist))
                max_slope = max(max_slope, slope)
        # Conservative penalty: each unseen neighbour adds an assumed slope risk.
        # At map edges / frontiers we lack data — assume worst-case safe slope.
        unknown_penalty = 0
        if unknown_neighbors > 0:
            unknown_penalty = min(35, 8 * unknown_neighbors)
        if max_slope >= self.lethal_slope_deg:
            return min(100, 100 + unknown_penalty)  # already lethal, penalty is moot
        if max_slope <= self.max_slope_deg:
            return min(100, int(round(30.0 * max_slope / max(0.1, self.max_slope_deg))) + unknown_penalty)
        slope_band = max(0.1, self.lethal_slope_deg - self.max_slope_deg)
        return min(100, 30 + int(round(45.0 * (max_slope - self.max_slope_deg) / slope_band)) + unknown_penalty)

    def cell_cost(self, index, stats_cache):
        cell = self.cells[index]
        cell_stats = stats_cache[index]
        if cell_stats.count < self.min_points:
            if cell.free_count > 0:
                return 0
            return 70 if self.unknown_is_risky else -1
        height_range = cell_stats.max_z - cell_stats.min_z
        if height_range >= self.lethal_step:
            return 100
        step_cost = 0
        if height_range >= self.cautious_step:
            step_cost = 35 + int(round(40.0 * (height_range - self.cautious_step) / max(0.01, self.lethal_step - self.cautious_step)))
        roughness_cost = int(round(35.0 * min(1.0, math.sqrt(cell_stats.variance) / max(0.001, self.roughness_warn))))
        slope_cost = self.neighbor_slope_cost(index, stats_cache)
        # Sum accumulated risks instead of taking only the worst factor.
        # A cell with moderate step, roughness, AND slope is riskier than any
        # single factor — the combined cost better reflects real traversability.
        return min(100, step_cost + roughness_cost + slope_cost)

    def publish_grid(self):
        now = self.get_clock().now().to_msg()
        self.decay_stale_cells(self.stamp_seconds(now))
        grid = OccupancyGrid()
        grid.header.stamp = now
        grid.header.frame_id = self.map_frame
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0
        stats_cache = [self.filtered_cell_stats(cell) for cell in self.cells]
        grid.data = [
            self.cell_cost(index, stats_cache) for index in range(len(self.cells))
        ]
        self.clear_robot_cells(grid.data)
        self.grid_pub.publish(grid)

        known = sum(1 for value in grid.data if value >= 0)
        lethal = sum(1 for value in grid.data if value >= 100)
        ceiling_filtered = sum(1 for stats in stats_cache if stats.ceiling_filtered)
        self.diag_pub.publish(
            String(
                data=(
                    f"known_ratio={known / len(grid.data):.3f} "
                    f"lethal_ratio={lethal / len(grid.data):.3f} "
                    f"ceiling_filtered={ceiling_filtered} "
                    f"last_cloud={'yes' if self.last_cloud_stamp else 'no'}"
                )
            )
        )


def main():
    rclpy.init()
    node = TerrainMapper()
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
