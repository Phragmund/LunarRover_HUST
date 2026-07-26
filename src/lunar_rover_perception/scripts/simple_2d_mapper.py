#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Simple2DMapper(Node):
    def __init__(self):
        super().__init__("simple_2d_mapper")
        self.declare_parameter("cloud_topic", "/livox/points")
        self.declare_parameter("odometry_topic", "/simulation/ground_truth/odometry")
        self.declare_parameter("map_topic", "/mapping/simple_2d_map")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("width", 240)
        self.declare_parameter("height", 240)
        self.declare_parameter("origin_x", -6.0)
        self.declare_parameter("origin_y", -6.0)
        self.declare_parameter("min_range", 0.20)
        self.declare_parameter("max_range", 8.0)
        self.declare_parameter("min_obstacle_z", -0.35)
        self.declare_parameter("max_obstacle_z", 1.20)
        self.declare_parameter("publish_rate", 2.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.resolution = float(self.get_parameter("resolution").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.min_obstacle_z = float(self.get_parameter("min_obstacle_z").value)
        self.max_obstacle_z = float(self.get_parameter("max_obstacle_z").value)

        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self.pose = None
        self.map_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter("map_topic").value, 1
        )
        self.create_subscription(
            Odometry, self.get_parameter("odometry_topic").value, self._odom_cb, 20
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("cloud_topic").value,
            self._cloud_cb,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self._publish)
        self.get_logger().info(
            f"Publishing simple occupancy map {self.width}x{self.height} "
            f"at {self.resolution:.2f} m/cell"
        )

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose = (float(p.x), float(p.y), yaw)

    def _cell(self, x, y):
        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return ix, iy
        return None

    def _mark_ray(self, start, end):
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while (x, y) != (x1, y1):
            if 0 <= x < self.width and 0 <= y < self.height and self.grid[y, x] < 50:
                self.grid[y, x] = 0
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

    def _cloud_cb(self, msg):
        if self.pose is None:
            return
        robot_x, robot_y, robot_yaw = self.pose
        start = self._cell(robot_x, robot_y)
        if start is None:
            return

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        accepted = 0
        for point in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            x_l = float(point[0])
            y_l = float(point[1])
            z_l = float(point[2])
            rng = math.sqrt(x_l * x_l + y_l * y_l + z_l * z_l)
            if (
                not math.isfinite(rng)
                or rng < self.min_range
                or rng > self.max_range
                or z_l < self.min_obstacle_z
                or z_l > self.max_obstacle_z
            ):
                continue

            # livox_frame is aligned with base_link in the rover URDF.
            x_b = x_l - 0.05
            y_b = y_l
            x_w = robot_x + cos_yaw * x_b - sin_yaw * y_b
            y_w = robot_y + sin_yaw * x_b + cos_yaw * y_b
            end = self._cell(x_w, y_w)
            if end is None:
                continue
            self._mark_ray(start, end)
            self.grid[end[1], end[0]] = 100
            accepted += 1

        if accepted:
            sx, sy = start
            self.grid[max(0, sy - 1) : min(self.height, sy + 2), max(0, sx - 1) : min(self.width, sx + 2)] = 0

    def _publish(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.reshape(-1).astype(np.int8).tolist()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Simple2DMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
