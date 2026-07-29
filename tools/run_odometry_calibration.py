#!/usr/bin/env python3
"""Run a short odometry calibration sequence and print relative error metrics."""

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_pose(pose, origin):
    x0, y0, yaw0 = origin
    dx = pose.position.x - x0
    dy = pose.position.y - y0
    cos0 = math.cos(yaw0)
    sin0 = math.sin(yaw0)
    return (
        cos0 * dx + sin0 * dy,
        -sin0 * dx + cos0 * dy,
        normalize_angle(yaw_from_quaternion(pose.orientation) - yaw0),
    )


def xy_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_length(points):
    if len(points) < 2:
        return 0.0
    return sum(xy_distance(a, b) for a, b in zip(points[:-1], points[1:]))


class OdometryCalibration(Node):
    def __init__(self):
        super().__init__("odometry_calibration_runner")
        self.cmd_topic = self.declare_parameter("cmd_topic", "/chassis/cmd_vel").value
        self.truth_topic = self.declare_parameter(
            "truth_topic", "/simulation/ground_truth/odometry"
        ).value
        self.wheel_topic = self.declare_parameter(
            "wheel_topic", "/localization/odometry"
        ).value
        self.composite_topic = self.declare_parameter(
            "composite_topic", "/localization/composite_odometry"
        ).value

        self.publisher = self.create_publisher(TwistStamped, self.cmd_topic, 1)
        self.create_subscription(Odometry, self.truth_topic, self.on_truth, 50)
        self.create_subscription(Odometry, self.wheel_topic, self.on_wheel, 50)
        self.create_subscription(Odometry, self.composite_topic, self.on_composite, 50)

        self.origins = {}
        self.latest = {}
        self.samples = []

    def pose_tuple(self, msg, key):
        pose = msg.pose.pose
        if key not in self.origins:
            self.origins[key] = (
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            )
        return relative_pose(pose, self.origins[key])

    def on_truth(self, msg):
        self.latest["truth"] = self.pose_tuple(msg, "truth")
        if "wheel" in self.latest and "composite" in self.latest:
            self.samples.append(
                (self.latest["truth"], self.latest["wheel"], self.latest["composite"])
            )

    def on_wheel(self, msg):
        self.latest["wheel"] = self.pose_tuple(msg, "wheel")

    def on_composite(self, msg):
        self.latest["composite"] = self.pose_tuple(msg, "composite")

    def publish_command(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.publisher.publish(msg)

    def run_phase(self, name, duration, linear_x, angular_z, rate=20.0):
        self.get_logger().info(
            f"phase={name} duration={duration:.1f}s v={linear_x:.3f} wz={angular_z:.3f}"
        )
        period = 1.0 / rate
        end_time = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_command(linear_x, angular_z)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def stop(self):
        for _ in range(10):
            self.publish_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def report(self):
        if not self.samples:
            print("No synchronized odometry samples collected.")
            return

        truth_points = [item[0] for item in self.samples]
        wheel_points = [item[1] for item in self.samples]
        composite_points = [item[2] for item in self.samples]

        def summarize(name, points):
            xy_errors = [xy_distance(point, truth) for truth, point in zip(truth_points, points)]
            yaw_errors = [
                abs(normalize_angle(point[2] - truth[2]))
                for truth, point in zip(truth_points, points)
            ]
            final_xy = xy_errors[-1]
            final_yaw = math.degrees(normalize_angle(points[-1][2] - truth_points[-1][2]))
            rms_xy = math.sqrt(sum(value * value for value in xy_errors) / len(xy_errors))
            rms_yaw = math.degrees(
                math.sqrt(sum(value * value for value in yaw_errors) / len(yaw_errors))
            )
            print(
                f"{name}: final_xy={final_xy:.3f} m, rms_xy={rms_xy:.3f} m, "
                f"final_yaw={final_yaw:.2f} deg, rms_yaw={rms_yaw:.2f} deg, "
                f"path_length={path_length(points):.3f} m"
            )

        truth_length = path_length(truth_points)
        wheel_length = path_length(wheel_points)
        composite_length = path_length(composite_points)
        print(f"truth: path_length={truth_length:.3f} m, samples={len(self.samples)}")
        summarize("wheel", wheel_points)
        summarize("composite", composite_points)
        if composite_length > 1.0e-6:
            print(
                "suggested forward_velocity_scale ~= "
                f"{truth_length / composite_length:.4f} "
                "(use only if this was a mostly straight calibration run)"
            )


def main():
    rclpy.init()
    node = OdometryCalibration()
    try:
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if {"truth", "wheel", "composite"}.issubset(node.latest):
                break

        node.run_phase("settle", 2.0, 0.0, 0.0)
        node.run_phase("straight", 12.0, 0.15, 0.0)
        node.run_phase("stop_after_straight", 2.0, 0.0, 0.0)
        node.run_phase("spin_left", 5.0, 0.0, 0.36)
        node.run_phase("stop_after_spin", 2.0, 0.0, 0.0)
        node.run_phase("straight_after_spin", 8.0, 0.15, 0.0)
        node.run_phase("final_stop", 2.0, 0.0, 0.0)
        node.stop()
        node.report()
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
