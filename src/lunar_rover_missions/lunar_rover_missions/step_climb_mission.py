import math
from statistics import median

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class StepClimbMission(Node):
    def __init__(self):
        super().__init__("step_climb_mission")
        self.declare_parameter("cloud_topic", "/livox/points")
        self.declare_parameter("odometry_topic", "/simulation/ground_truth/odometry")
        self.declare_parameter("command_topic", "/cmd_vel_safe")
        self.declare_parameter("status_topic", "/mission/status")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("search_speed", 0.0)
        self.declare_parameter("approach_speed", 0.22)
        self.declare_parameter("drive_command_sign", 1.0)
        self.declare_parameter("scan_axis_sign", 1.0)
        self.declare_parameter("max_angular_speed", 0.25)
        self.declare_parameter("lane_half_width", 0.55)
        self.declare_parameter("detect_min_distance", 0.35)
        self.declare_parameter("detect_max_distance", 5.00)
        self.declare_parameter("min_step_height", 0.022)
        self.declare_parameter("max_step_height", 0.16)
        self.declare_parameter("min_elevated_points", 8)
        self.declare_parameter("range_fallback_points", 40)
        self.declare_parameter("climb_height", 0.018)
        self.declare_parameter("step_clearance_distance", 0.75)
        self.declare_parameter("one_step_drive_distance", 2.65)
        self.declare_parameter("finish_world_x", -0.95)
        self.declare_parameter("settle_time", 1.0)
        self.declare_parameter("mission_timeout", 180.0)

        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.odom_topic = self.get_parameter("odometry_topic").value
        self.command_topic = self.get_parameter("command_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.search_speed = self.get_parameter("search_speed").value
        self.approach_speed = self.get_parameter("approach_speed").value
        self.drive_sign = float(self.get_parameter("drive_command_sign").value)
        self.scan_sign = float(self.get_parameter("scan_axis_sign").value)
        self.max_angular = self.get_parameter("max_angular_speed").value
        self.lane_half_width = self.get_parameter("lane_half_width").value
        self.detect_min_distance = self.get_parameter("detect_min_distance").value
        self.detect_max_distance = self.get_parameter("detect_max_distance").value
        self.min_step_height = self.get_parameter("min_step_height").value
        self.max_step_height = self.get_parameter("max_step_height").value
        self.min_elevated_points = self.get_parameter("min_elevated_points").value
        self.range_fallback_points = self.get_parameter("range_fallback_points").value
        self.climb_height = self.get_parameter("climb_height").value
        self.step_clearance_distance = self.get_parameter("step_clearance_distance").value
        self.one_step_drive_distance = self.get_parameter("one_step_drive_distance").value
        self.finish_world_x = self.get_parameter("finish_world_x").value
        self.settle_time = self.get_parameter("settle_time").value
        self.timeout = self.get_parameter("mission_timeout").value

        self.command_pub = self.create_publisher(Twist, self.command_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(
            PointCloud2, self.cloud_topic, self._cloud_callback, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 20)

        self.phase = "waiting"
        self.initial_pose = None
        self.current_pose = None
        self.detected = None
        self.latched_step = None
        self.climb_start_pose = None
        self.climb_target_distance = None
        self.climb_started_at = None
        self.started_at = self.get_clock().now()
        self.done = False

        period = 1.0 / self.get_parameter("publish_rate").value
        self.timer = self.create_timer(period, self._tick)
        self._publish_status("waiting_for_lidar_and_odometry")

    def _publish_status(self, text):
        msg = String()
        msg.data = f"step_climb:{text}"
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def _odom_callback(self, message):
        position = message.pose.pose.position
        self.current_pose = (position.x, position.y, position.z)
        if self.initial_pose is None:
            self.initial_pose = self.current_pose

    def _cloud_callback(self, message):
        lane_points = []
        candidates = []
        for point in point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        ):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if abs(y) > self.lane_half_width:
                continue
            distance = self.scan_sign * x
            if distance <= 0.05:
                continue
            lane_points.append((distance, y, z))
            if self.detect_min_distance <= distance <= self.detect_max_distance:
                candidates.append((distance, y, z))

        if len(lane_points) < 20 or len(candidates) < 8:
            self.detected = None
            return

        z_values = sorted(z for _, _, z in lane_points)
        low_count = max(8, len(z_values) // 4)
        ground_z = median(z_values[:low_count])
        elevated = [
            (distance, y, z)
            for distance, y, z in candidates
            if self.min_step_height <= z - ground_z <= self.max_step_height
        ]
        if len(elevated) < self.min_elevated_points:
            if len(candidates) < self.range_fallback_points:
                self.detected = None
                return
            front_edge = min(distance for distance, _, _ in candidates)
            close_band = [
                (distance, y, z)
                for distance, y, z in candidates
                if abs(distance - front_edge) <= 0.35
            ] or candidates
            detection_mode = "range"
        else:
            front_edge = min(distance for distance, _, _ in elevated)
            close_band = [
                (distance, y, z)
                for distance, y, z in elevated
                if abs(distance - front_edge) <= 0.25
            ] or elevated
            detection_mode = "height"

        center_y = sum(y for _, y, _ in close_band) / len(close_band)
        top_z = median([z for _, _, z in close_band])
        self.detected = {
            "distance": front_edge,
            "center_y": center_y,
            "height": top_z - ground_z,
            "points": len(elevated) if detection_mode == "height" else len(candidates),
            "mode": detection_mode,
        }

    def _command(self, linear_x, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.command_pub.publish(msg)

    def _distance_from(self, pose):
        if pose is None or self.current_pose is None:
            return 0.0
        return math.hypot(self.current_pose[0] - pose[0], self.current_pose[1] - pose[1])

    def _height_delta(self):
        if self.initial_pose is None or self.current_pose is None:
            return 0.0
        return self.current_pose[2] - self.initial_pose[2]

    def _finish(self, reason):
        if self.done:
            return
        self.done = True
        self._command(0.0, 0.0)
        self._publish_status(reason)
        self.timer.cancel()

    def _tick(self):
        now = self.get_clock().now()
        elapsed = (now - self.started_at).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self._finish("timeout_stopped")
            return

        if self.initial_pose is None or self.current_pose is None:
            self._command(0.0, 0.0)
            return

        climbed = self._height_delta()
        if self.phase == "climbing" and climbed >= self.climb_height:
            if self.climb_started_at is None:
                self.climb_started_at = now
                self._publish_status(f"climb_detected dz={climbed:.3f}")
            settled = (now - self.climb_started_at).nanoseconds * 1e-9
            if settled >= self.settle_time:
                self._finish(f"completed dz={climbed:.3f}")
                return
        else:
            self.climb_started_at = None

        if self.phase == "climbing":
            driven = self._distance_from(self.climb_start_pose)
            if self.current_pose[0] >= self.finish_world_x:
                self._finish(
                    f"completed_on_first_step x={self.current_pose[0]:.2f} dz={climbed:.3f}"
                )
                return
            if self.climb_target_distance is not None and driven >= self.climb_target_distance:
                self._finish(f"completed_by_distance driven={driven:.2f} dz={climbed:.3f}")
                return
            center_y = self.latched_step["center_y"] if self.latched_step else 0.0
            angular = clamp(0.15 * center_y, -self.max_angular, self.max_angular)
            self._command(self.drive_sign * self.approach_speed, angular)
            return

        if self.detected is None:
            if self.phase != "searching":
                self.phase = "searching"
                self._publish_status("searching_for_step")
            self._command(self.drive_sign * self.search_speed, 0.0)
            return

        distance = self.detected["distance"]
        center_y = self.detected["center_y"]
        height = self.detected["height"]
        mode = self.detected["mode"]
        self.phase = "climbing"
        self.latched_step = dict(self.detected)
        self.climb_start_pose = self.current_pose
        self.climb_target_distance = min(
            self.one_step_drive_distance,
            max(0.50, distance + self.step_clearance_distance),
        )
        self._publish_status(
            "step_found "
            f"mode={mode} distance={distance:.2f} height={height:.3f} y={center_y:.2f} "
            f"target_drive={self.climb_target_distance:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StepClimbMission()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node._command(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
