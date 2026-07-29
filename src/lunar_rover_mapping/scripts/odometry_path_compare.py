#!/usr/bin/env python3
"""Publish odometry comparison paths in one common frame."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    q = PoseStamped().pose.orientation
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OdometryPathCompare(Node):
    def __init__(self):
        super().__init__("odometry_path_compare")

        self.wheel_topic = self.declare_parameter(
            "wheel_odom_topic", "/localization/odometry"
        ).value
        self.truth_topic = self.declare_parameter(
            "ground_truth_topic", "/simulation/ground_truth/odometry"
        ).value
        self.imu_topic = self.declare_parameter(
            "imu_odom_topic", "/localization/imu_odometry"
        ).value
        self.fused_topic = self.declare_parameter(
            "fused_odom_topic", "/localization/composite_odometry"
        ).value
        self.output_frame = self.declare_parameter("output_frame", "odom").value
        self.max_points = int(self.declare_parameter("max_points", 0).value)
        self.log_period = float(self.declare_parameter("log_period", 2.0).value)

        self.wheel_path_pub = self.create_publisher(Path, "/debug/wheel_odom_path", 1)
        self.truth_path_pub = self.create_publisher(Path, "/debug/ground_truth_path", 1)
        self.imu_path_pub = self.create_publisher(Path, "/debug/imu_odom_path", 1)
        self.fused_path_pub = self.create_publisher(
            Path, "/debug/fused_odom_path", 1
        )

        self.create_subscription(Odometry, self.wheel_topic, self.on_wheel_odom, 20)
        self.create_subscription(Odometry, self.truth_topic, self.on_truth_odom, 20)
        self.create_subscription(Odometry, self.imu_topic, self.on_imu_odom, 20)
        self.create_subscription(Odometry, self.fused_topic, self.on_fused_odom, 20)

        self.wheel_origin = None
        self.truth_origin = None
        self.imu_origin = None
        self.fused_origin = None
        self.latest_wheel = None
        self.latest_truth = None
        self.latest_imu = None
        self.latest_fused = None
        self.wheel_path = Path()
        self.truth_path = Path()
        self.imu_path = Path()
        self.fused_path = Path()
        self.wheel_path.header.frame_id = self.output_frame
        self.truth_path.header.frame_id = self.output_frame
        self.imu_path.header.frame_id = self.output_frame
        self.fused_path.header.frame_id = self.output_frame

        self.create_timer(self.log_period, self.log_error)
        self.get_logger().info(
            f"Comparing wheel {self.wheel_topic}, IMU {self.imu_topic}, "
            f"composite {self.fused_topic}, "
            f"and Gazebo truth {self.truth_topic}; "
            f"paths are normalized into {self.output_frame}"
        )

    def relative_pose(self, pose, origin):
        x0, y0, yaw0 = origin
        x = pose.position.x
        y = pose.position.y
        yaw = yaw_from_quaternion(pose.orientation)

        dx = x - x0
        dy = y - y0
        cos0 = math.cos(yaw0)
        sin0 = math.sin(yaw0)
        return (
            cos0 * dx + sin0 * dy,
            -sin0 * dx + cos0 * dy,
            normalize_angle(yaw - yaw0),
        )

    def append_path_point(self, path, stamp, rel_pose, z_offset):
        x, y, yaw = rel_pose
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.output_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z_offset
        pose.pose.orientation = quaternion_from_yaw(yaw)
        path.header.stamp = stamp
        path.header.frame_id = self.output_frame
        path.poses.append(pose)
        if self.max_points > 0 and len(path.poses) > self.max_points:
            path.poses = path.poses[-self.max_points :]

    def on_wheel_odom(self, msg):
        pose = msg.pose.pose
        if self.wheel_origin is None:
            self.wheel_origin = (
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            )
            self.get_logger().info("Wheel odometry origin captured")
        self.latest_wheel = self.relative_pose(pose, self.wheel_origin)
        self.append_path_point(self.wheel_path, msg.header.stamp, self.latest_wheel, 0.04)
        self.wheel_path_pub.publish(self.wheel_path)

    def on_truth_odom(self, msg):
        pose = msg.pose.pose
        if self.truth_origin is None:
            self.truth_origin = (
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            )
            self.get_logger().info("Gazebo truth origin captured")
        self.latest_truth = self.relative_pose(pose, self.truth_origin)
        self.append_path_point(self.truth_path, msg.header.stamp, self.latest_truth, 0.07)
        self.truth_path_pub.publish(self.truth_path)

    def on_imu_odom(self, msg):
        pose = msg.pose.pose
        if self.imu_origin is None:
            self.imu_origin = (
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            )
            self.get_logger().info("IMU odometry origin captured")
        self.latest_imu = self.relative_pose(pose, self.imu_origin)
        self.append_path_point(self.imu_path, msg.header.stamp, self.latest_imu, 0.10)
        self.imu_path_pub.publish(self.imu_path)

    def on_fused_odom(self, msg):
        pose = msg.pose.pose
        if self.fused_origin is None:
            self.fused_origin = (
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            )
            self.get_logger().info("Composite wheel+IMU odometry origin captured")
        self.latest_fused = self.relative_pose(pose, self.fused_origin)
        self.append_path_point(
            self.fused_path, msg.header.stamp, self.latest_fused, 0.13
        )
        self.fused_path_pub.publish(self.fused_path)

    def log_error(self):
        if self.latest_wheel is None or self.latest_truth is None:
            return
        wx, wy, wyaw = self.latest_wheel
        tx, ty, tyaw = self.latest_truth
        wheel_xy_error = math.hypot(wx - tx, wy - ty)
        wheel_yaw_error = normalize_angle(wyaw - tyaw)
        message = (
            f"odom error: wheel xy={wheel_xy_error:.3f} m, "
            f"yaw={math.degrees(wheel_yaw_error):.2f} deg"
        )

        if self.latest_imu is not None:
            ix, iy, iyaw = self.latest_imu
            imu_xy_error = math.hypot(ix - tx, iy - ty)
            imu_yaw_error = normalize_angle(iyaw - tyaw)
            message += (
                f" | imu xy={imu_xy_error:.3f} m, "
                f"yaw={math.degrees(imu_yaw_error):.2f} deg"
            )

        if self.latest_fused is not None:
            fx, fy, fyaw = self.latest_fused
            fused_xy_error = math.hypot(fx - tx, fy - ty)
            fused_yaw_error = normalize_angle(fyaw - tyaw)
            message += (
                f" | composite xy={fused_xy_error:.3f} m, "
                f"yaw={math.degrees(fused_yaw_error):.2f} deg"
            )

        message += (
            f" | truth=({tx:.2f}, {ty:.2f}, {math.degrees(tyaw):.1f}deg)"
        )
        self.get_logger().info(message)


def main():
    rclpy.init()
    node = OdometryPathCompare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
