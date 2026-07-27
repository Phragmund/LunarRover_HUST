import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def rotate(point, q):
    x, y, z = point
    tx = 2.0 * (q.y * z - q.z * y)
    ty = 2.0 * (q.z * x - q.x * z)
    tz = 2.0 * (q.x * y - q.y * x)
    return (
        x + q.w * tx + (q.y * tz - q.z * ty),
        y + q.w * ty + (q.z * tx - q.x * tz),
        z + q.w * tz + (q.x * ty - q.y * tx),
    )


class ShuttleMappingMission(Node):
    """Repeat a fixed out-and-back drive pattern for mapping drift tests."""

    def __init__(self):
        super().__init__("shuttle_mapping_mission")
        self.declare_parameter("odometry_topic", "/simulation/ground_truth/odometry")
        self.declare_parameter("cloud_topic", "/livox/points")
        self.declare_parameter("command_topic", "/chassis/cmd_vel")
        self.declare_parameter("status_topic", "/mission/status")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("travel_distance", 2.0)
        self.declare_parameter("return_tolerance", 0.12)
        self.declare_parameter("forward_speed", 0.12)
        self.declare_parameter("turn_speed", 0.45)
        self.declare_parameter("yaw_gain", 1.8)
        self.declare_parameter("heading_gain", 1.1)
        self.declare_parameter("max_heading_correction", 0.30)
        self.declare_parameter("turn_tolerance_degrees", 5.0)
        self.declare_parameter("settle_time", 0.4)
        self.declare_parameter("search_turn_speed", 0.28)
        self.declare_parameter("search_wall_min_z", 0.12)
        self.declare_parameter("search_front_clear_distance", 1.35)
        self.declare_parameter("search_front_half_width", 0.32)
        self.declare_parameter("search_min_x", 0.25)
        self.declare_parameter("search_max_x", 1.60)
        self.declare_parameter("search_min_blocking_points", 16)
        self.declare_parameter("search_stable_open_ticks", 4)
        self.declare_parameter("mission_timeout", 600.0)

        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.command_topic = self.get_parameter("command_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.travel_distance = float(self.get_parameter("travel_distance").value)
        self.return_tolerance = float(self.get_parameter("return_tolerance").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.max_heading_correction = float(
            self.get_parameter("max_heading_correction").value
        )
        self.turn_tolerance = math.radians(
            float(self.get_parameter("turn_tolerance_degrees").value)
        )
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.search_turn_speed = float(self.get_parameter("search_turn_speed").value)
        self.search_wall_min_z = float(self.get_parameter("search_wall_min_z").value)
        self.search_front_clear = float(
            self.get_parameter("search_front_clear_distance").value
        )
        self.search_front_half_width = float(
            self.get_parameter("search_front_half_width").value
        )
        self.search_min_x = float(self.get_parameter("search_min_x").value)
        self.search_max_x = float(self.get_parameter("search_max_x").value)
        self.search_min_blocking_points = int(
            self.get_parameter("search_min_blocking_points").value
        )
        self.search_stable_open_ticks = int(
            self.get_parameter("search_stable_open_ticks").value
        )
        self.timeout = float(self.get_parameter("mission_timeout").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.command_pub = self.create_publisher(TwistStamped, self.command_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(
            Odometry, self.get_parameter("odometry_topic").value, self._odom_callback, 20
        )
        self.create_subscription(
            PointCloud2, self.cloud_topic, self._cloud_callback, qos_profile_sensor_data
        )

        self.pose = None
        self.yaw = None
        self.front_distance = None
        self.front_open = False
        self.open_ticks = 0
        self.last_cloud_time = None
        self.origin = None
        self.start = None
        self.out_heading = None
        self.back_heading = None
        self.mode = "waiting"
        self.cycle = 0
        self.last_status = None
        self.started_at = self.get_clock().now()
        self.settle_until = None
        self.next_mode = None

        period = 1.0 / float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(period, self._tick)
        self._publish_status("waiting_for_odometry_and_lidar")

    def _odom_callback(self, message):
        position = message.pose.pose.position
        self.pose = (position.x, position.y)
        self.yaw = yaw_from_quaternion(message.pose.pose.orientation)

    def _lookup_cloud_transform(self, source_frame):
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, rclpy.time.Time()
            ).transform
        except Exception as exc:
            self.get_logger().warn(
                f"Waiting for TF {self.target_frame} <- {source_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

    def _transform_point(self, point, transform):
        rotated = rotate(point, transform.rotation)
        return (
            rotated[0] + transform.translation.x,
            rotated[1] + transform.translation.y,
            rotated[2] + transform.translation.z,
        )

    def _cloud_callback(self, message):
        transform = self._lookup_cloud_transform(message.header.frame_id)
        if transform is None:
            return

        front = []
        for point in point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        ):
            x, y, z = self._transform_point(
                (float(point[0]), float(point[1]), float(point[2])), transform
            )
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if z < self.search_wall_min_z:
                continue
            if (
                self.search_min_x <= x <= self.search_max_x
                and abs(y) <= self.search_front_half_width
            ):
                front.append(x)

        self.front_distance = (
            min(front) if len(front) >= self.search_min_blocking_points else None
        )
        self.front_open = (
            self.front_distance is None or self.front_distance >= self.search_front_clear
        )
        self.open_ticks = self.open_ticks + 1 if self.front_open else 0
        self.last_cloud_time = self.get_clock().now()

    def _publish_status(self, text):
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = f"shuttle_mapping:{text}"
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def _command(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.command_pub.publish(msg)

    def _distance_from(self, point):
        return math.hypot(self.pose[0] - point[0], self.pose[1] - point[1])

    def _cloud_age(self):
        if self.last_cloud_time is None:
            return float("inf")
        return (self.get_clock().now() - self.last_cloud_time).nanoseconds * 1e-9

    def _start_shuttle(self):
        self.origin = self.pose
        self.start = self.pose
        self.out_heading = self.yaw
        self.back_heading = normalize_angle(self.out_heading + math.pi)
        self.mode = "forward_out"
        self._publish_status(
            "road_locked:"
            f"heading={math.degrees(self.out_heading):.0f}deg "
            f"front={self.front_distance if self.front_distance is not None else -1.0:.2f}m"
        )

    def _start_settle(self, next_mode):
        self._command(0.0, 0.0)
        self.mode = "settling"
        self.next_mode = next_mode
        self.settle_until = self.get_clock().now().nanoseconds * 1e-9 + self.settle_time

    def _finish_settle(self):
        if self.next_mode == "turn_back":
            self.mode = "turn_back"
            self._publish_status(
                f"cycle={self.cycle}:turn_back:target={math.degrees(self.back_heading):.0f}deg"
            )
        elif self.next_mode == "forward_back":
            self.start = self.pose
            self.mode = "forward_back"
            self._publish_status(f"cycle={self.cycle}:forward_back:return_origin")
        elif self.next_mode == "turn_out":
            self.mode = "turn_out"
            self._publish_status(
                f"cycle={self.cycle}:turn_out:target={math.degrees(self.out_heading):.0f}deg"
            )
        elif self.next_mode == "forward_out":
            self.cycle += 1
            self.origin = self.pose
            self.start = self.pose
            self.out_heading = self.yaw
            self.back_heading = normalize_angle(self.out_heading + math.pi)
            self.mode = "forward_out"
            self._publish_status(
                f"cycle={self.cycle}:forward_out:target={self.travel_distance:.2f}m"
            )
        elif self.next_mode == "start_shuttle":
            self._start_shuttle()
        self.next_mode = None
        self.settle_until = None

    def _drive_straight(self, heading):
        yaw_error = normalize_angle(heading - self.yaw)
        angular = clamp(
            self.heading_gain * yaw_error,
            -self.max_heading_correction,
            self.max_heading_correction,
        )
        self._command(self.forward_speed, angular)

    def _turn_to(self, heading, next_mode):
        yaw_error = normalize_angle(heading - self.yaw)
        if abs(yaw_error) <= self.turn_tolerance:
            self._start_settle(next_mode)
            return
        angular = clamp(self.yaw_gain * yaw_error, -self.turn_speed, self.turn_speed)
        if abs(angular) < 0.16:
            angular = math.copysign(0.16, angular)
        self._command(0.0, angular)

    def _tick(self):
        if (self.get_clock().now() - self.started_at).nanoseconds * 1e-9 > self.timeout:
            self._command(0.0, 0.0)
            self._publish_status("timeout_stopped")
            self.timer.cancel()
            return
        if self.pose is None or self.yaw is None:
            self._command(0.0, 0.0)
            return

        if self.mode == "waiting":
            if self._cloud_age() > 0.8:
                self._command(0.0, 0.0)
                self._publish_status("waiting_for_fresh_lidar")
                return
            if self.open_ticks >= self.search_stable_open_ticks:
                self._command(0.0, 0.0)
                self._start_settle("start_shuttle")
                return
            self._command(0.0, self.search_turn_speed)
            self._publish_status(
                "searching_road "
                f"front={self.front_distance if self.front_distance is not None else -1.0:.2f} "
                f"open_ticks={self.open_ticks}"
            )
            return

        if self.mode == "settling":
            self._command(0.0, 0.0)
            if self.get_clock().now().nanoseconds * 1e-9 >= self.settle_until:
                self._finish_settle()
            return

        if self.mode == "forward_out":
            if self._distance_from(self.start) >= self.travel_distance:
                self._start_settle("turn_back")
                return
            self._drive_straight(self.out_heading)
            return

        if self.mode == "turn_back":
            self._turn_to(self.back_heading, "forward_back")
            return

        if self.mode == "forward_back":
            if self._distance_from(self.origin) <= self.return_tolerance:
                self._start_settle("turn_out")
                return
            if self._distance_from(self.start) >= self.travel_distance + 0.35:
                self._start_settle("turn_out")
                return
            self._drive_straight(self.back_heading)
            return

        if self.mode == "turn_out":
            self._turn_to(self.out_heading, "forward_out")
            return


def main(args=None):
    rclpy.init(args=args)
    node = ShuttleMappingMission()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node._command(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
