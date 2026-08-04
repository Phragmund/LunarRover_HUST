#!/usr/bin/env python3
"""Build a 2.5D traversability grid from registered LiDAR points."""

import math
from dataclasses import dataclass

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
class TerrainCell:
    count: int = 0
    free_count: int = 0
    min_z: float = math.inf
    max_z: float = -math.inf
    sum_z: float = 0.0
    sum_z2: float = 0.0
    last_seen: float = 0.0

    def add(self, z, stamp):
        self.count += 1
        self.min_z = min(self.min_z, z)
        self.max_z = max(self.max_z, z)
        self.sum_z += z
        self.sum_z2 += z * z
        self.last_seen = stamp

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
        self.roughness_warn = float(self.declare_parameter("roughness_warn", 0.025).value)
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
            rclpy.time.Time(),
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
                    rclpy.time.Time(),
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
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.05),
                    )
                except TransformException as exc:
                    self.get_logger().warn(
                        f"Cannot self-filter cloud from {msg.header.frame_id}: {exc}",
                        throttle_duration_sec=2.0,
                    )

        used = 0
        stamp = self.stamp_seconds(msg.header.stamp)
        sensor_origin = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
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
            if self.raytrace_free_space and used % self.raytrace_stride == 0:
                self.mark_ray_free(sensor_origin, (x, y, z), stamp)
            if z < self.min_z or z > self.max_z:
                continue
            index = self.cell_index(x, y)
            if index is None:
                continue
            self.cells[index].add(z, stamp)
            used += 1
        self.last_cloud_stamp = msg.header.stamp

    def decay_stale_cells(self, now_seconds):
        if self.cell_decay_time <= 0.0:
            return
        for cell in self.cells:
            if cell.count and now_seconds - cell.last_seen > self.cell_decay_time:
                cell.reset()

    def neighbor_slope_cost(self, index):
        row = index // self.width
        column = index - row * self.width
        cell = self.cells[index]
        if cell.count < self.min_points:
            return 0
        max_slope = 0.0
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr = row + dy
            nc = column + dx
            if nr < 0 or nr >= self.height or nc < 0 or nc >= self.width:
                continue
            neighbor = self.cells[nr * self.width + nc]
            if neighbor.count < self.min_points:
                continue
            slope = math.degrees(math.atan2(abs(cell.mean_z - neighbor.mean_z), self.resolution))
            max_slope = max(max_slope, slope)
        if max_slope >= self.max_slope_deg:
            return 100
        return int(round(70.0 * max_slope / max(0.1, self.max_slope_deg)))

    def cell_cost(self, index):
        cell = self.cells[index]
        if cell.count < self.min_points:
            if cell.free_count > 0:
                return 0
            return 70 if self.unknown_is_risky else -1
        height_range = cell.max_z - cell.min_z
        if height_range >= self.lethal_step:
            return 100
        step_cost = 0
        if height_range >= self.cautious_step:
            step_cost = 65 + int(round(25.0 * (height_range - self.cautious_step) / max(0.01, self.lethal_step - self.cautious_step)))
        roughness_cost = int(round(60.0 * min(1.0, math.sqrt(cell.variance) / max(0.001, self.roughness_warn))))
        return min(100, max(step_cost, roughness_cost, self.neighbor_slope_cost(index)))

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
        grid.data = [self.cell_cost(index) for index in range(len(self.cells))]
        self.grid_pub.publish(grid)

        known = sum(1 for value in grid.data if value >= 0)
        lethal = sum(1 for value in grid.data if value >= 100)
        self.diag_pub.publish(
            String(
                data=(
                    f"known_ratio={known / len(grid.data):.3f} "
                    f"lethal_ratio={lethal / len(grid.data):.3f} "
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
