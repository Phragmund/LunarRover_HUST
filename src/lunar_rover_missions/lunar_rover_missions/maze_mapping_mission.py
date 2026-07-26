import math
from statistics import median

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


def roll_pitch_from_quaternion(q):
    sin_roll = 2.0 * (q.w * q.x + q.y * q.z)
    cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)
    return roll, math.asin(sin_pitch)


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


class MazeMappingMission(Node):
    """Simple online SLAM drive task for the generated maze.

    The controller follows maze walls using only points high enough to be walls.
    Low ground bumps are intentionally ignored as obstacles; yaw and corridor
    centering feedback correct the heading drift they introduce while driving.
    """

    def __init__(self):
        super().__init__("maze_mapping_mission")
        self.declare_parameter("cloud_topic", "/livox/points")
        self.declare_parameter("odometry_topic", "/simulation/ground_truth/odometry")
        self.declare_parameter("command_topic", "/chassis/cmd_vel")
        self.declare_parameter("status_topic", "/mission/status")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("forward_speed", 0.13)
        self.declare_parameter("turn_speed", 0.48)
        self.declare_parameter("creep_speed", 0.025)
        self.declare_parameter("max_angular_speed", 0.55)
        self.declare_parameter("angular_command_sign", 1.0)
        self.declare_parameter("wall_min_z", 0.08)
        self.declare_parameter("front_stop_distance", 0.58)
        self.declare_parameter("front_clear_distance", 0.86)
        self.declare_parameter("side_target_distance", 0.47)
        self.declare_parameter("side_open_distance", 0.78)
        self.declare_parameter("front_half_width", 0.30)
        self.declare_parameter("side_min_y", 0.22)
        self.declare_parameter("side_max_y", 1.08)
        self.declare_parameter("side_x_min", -0.15)
        self.declare_parameter("side_x_max", 1.00)
        self.declare_parameter("minimum_sector_points", 18)
        self.declare_parameter("stable_open_ticks", 5)
        self.declare_parameter("yaw_gain", 1.65)
        self.declare_parameter("center_gain", 0.58)
        self.declare_parameter("rough_tilt_degrees", 7.0)
        self.declare_parameter("mission_timeout", 300.0)

        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.odom_topic = self.get_parameter("odometry_topic").value
        self.command_topic = self.get_parameter("command_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.creep_speed = float(self.get_parameter("creep_speed").value)
        self.max_angular = float(self.get_parameter("max_angular_speed").value)
        self.angular_command_sign = float(
            self.get_parameter("angular_command_sign").value
        )
        self.wall_min_z = float(self.get_parameter("wall_min_z").value)
        self.front_stop = float(self.get_parameter("front_stop_distance").value)
        self.front_clear = float(self.get_parameter("front_clear_distance").value)
        self.side_target = float(self.get_parameter("side_target_distance").value)
        self.side_open = float(self.get_parameter("side_open_distance").value)
        self.front_half_width = float(self.get_parameter("front_half_width").value)
        self.side_min_y = float(self.get_parameter("side_min_y").value)
        self.side_max_y = float(self.get_parameter("side_max_y").value)
        self.side_x_min = float(self.get_parameter("side_x_min").value)
        self.side_x_max = float(self.get_parameter("side_x_max").value)
        self.min_sector_points = int(self.get_parameter("minimum_sector_points").value)
        self.stable_open_ticks = int(self.get_parameter("stable_open_ticks").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.center_gain = float(self.get_parameter("center_gain").value)
        self.rough_tilt = math.radians(
            float(self.get_parameter("rough_tilt_degrees").value)
        )
        self.timeout = float(self.get_parameter("mission_timeout").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.command_pub = self.create_publisher(TwistStamped, self.command_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(
            PointCloud2, self.cloud_topic, self._cloud_callback, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 20)

        self.current_yaw = None
        self.current_tilt = 0.0
        self.target_heading = None
        self.mode = "waiting"
        self.observation = None
        self.last_cloud_time = None
        self.started_at = self.get_clock().now()
        self.open_counts = {"left": 0, "right": 0, "front": 0}
        self.last_status = None

        period = 1.0 / float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(period, self._tick)
        self._publish_status("waiting_for_lidar_and_odometry")

    def _publish_status(self, text):
        if text == self.last_status:
            return
        self.last_status = text
        msg = String()
        msg.data = f"maze_mapping:{text}"
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def _odom_callback(self, message):
        q = message.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(q)
        roll, pitch = roll_pitch_from_quaternion(q)
        self.current_tilt = max(abs(roll), abs(pitch))
        if self.target_heading is None:
            self.target_heading = self._cardinal_heading(self.current_yaw)

    def _cardinal_heading(self, yaw):
        return normalize_angle(round(yaw / (math.pi / 2.0)) * (math.pi / 2.0))

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
        left = []
        right = []
        wall_points = 0
        ignored_low_points = 0

        for point in point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        ):
            x, y, z = self._transform_point(
                (float(point[0]), float(point[1]), float(point[2])), transform
            )
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if z < self.wall_min_z:
                if -0.30 <= z <= self.wall_min_z and 0.0 <= x <= 1.2:
                    ignored_low_points += 1
                continue

            wall_points += 1
            if 0.12 <= x <= 1.45 and abs(y) <= self.front_half_width:
                front.append(x)
            if self.side_x_min <= x <= self.side_x_max:
                if self.side_min_y <= y <= self.side_max_y:
                    left.append(y)
                elif -self.side_max_y <= y <= -self.side_min_y:
                    right.append(-y)

        front_distance = min(front) if len(front) >= self.min_sector_points else None
        left_distance = median(left) if len(left) >= self.min_sector_points else None
        right_distance = median(right) if len(right) >= self.min_sector_points else None

        left_open = left_distance is None or left_distance >= self.side_open
        right_open = right_distance is None or right_distance >= self.side_open
        front_open = front_distance is None or front_distance >= self.front_clear
        for name, is_open in (
            ("left", left_open),
            ("right", right_open),
            ("front", front_open),
        ):
            self.open_counts[name] = self.open_counts[name] + 1 if is_open else 0

        self.observation = {
            "front": front_distance,
            "left": left_distance,
            "right": right_distance,
            "wall_points": wall_points,
            "ignored_low_points": ignored_low_points,
        }
        self.last_cloud_time = self.get_clock().now()

    def _command(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = self.angular_command_sign * angular_z
        self.command_pub.publish(msg)

    def _turn(self, direction, reason):
        self.mode = "turning"
        self.target_heading = normalize_angle(
            self.target_heading + direction * math.pi / 2.0
        )
        self.open_counts = {"left": 0, "right": 0, "front": 0}
        direction_name = "left" if direction > 0 else "right"
        self._publish_status(
            f"turn_{direction_name}:{reason}:target={math.degrees(self.target_heading):.0f}deg"
        )

    def _age(self, stamp):
        if stamp is None:
            return float("inf")
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _center_error(self):
        if self.observation is None:
            return 0.0
        left = self.observation["left"]
        right = self.observation["right"]
        if left is not None and right is not None:
            return clamp(left - right, -0.35, 0.35)
        if left is not None:
            return clamp(left - self.side_target, -0.25, 0.25)
        if right is not None:
            return clamp(self.side_target - right, -0.25, 0.25)
        return 0.0

    def _tick(self):
        if (self.get_clock().now() - self.started_at).nanoseconds * 1e-9 > self.timeout:
            self._command(0.0, 0.0)
            self._publish_status("timeout_stopped")
            self.timer.cancel()
            return

        if self.current_yaw is None or self.target_heading is None:
            self._command(0.0, 0.0)
            return
        if self.observation is None or self._age(self.last_cloud_time) > 0.8:
            self._command(0.0, 0.0)
            self._publish_status("waiting_for_fresh_lidar")
            return

        yaw_error = normalize_angle(self.target_heading - self.current_yaw)
        if self.mode == "turning":
            if abs(yaw_error) < math.radians(7.0):
                self.target_heading = self._cardinal_heading(self.current_yaw)
                self.mode = "driving"
                self._publish_status(
                    f"drive:heading={math.degrees(self.target_heading):.0f}deg"
                )
            else:
                angular = clamp(self.yaw_gain * yaw_error, -self.turn_speed, self.turn_speed)
                if abs(angular) < 0.18:
                    angular = math.copysign(0.18, angular)
                self._command(self.creep_speed, angular)
                return

        front = self.observation["front"]
        front_blocked = front is not None and front <= self.front_stop
        right_stably_open = self.open_counts["right"] >= self.stable_open_ticks
        left_stably_open = self.open_counts["left"] >= self.stable_open_ticks

        if front_blocked:
            if right_stably_open:
                self._turn(-1.0, f"front_blocked_{front:.2f}m_right_open")
            elif left_stably_open:
                self._turn(1.0, f"front_blocked_{front:.2f}m_left_open")
            else:
                self._turn(1.0, f"front_blocked_{front:.2f}m_no_side_open")
            self._command(0.0, 0.0)
            return

        if (
            right_stably_open
            and self.open_counts["front"] >= self.stable_open_ticks
            and self.mode != "turning"
        ):
            self._turn(-1.0, "right_hand_opening")
            self._command(0.0, 0.0)
            return

        self.mode = "driving"
        center_error = self._center_error()
        rough_scale = 0.70 if self.current_tilt >= self.rough_tilt else 1.0
        angular = self.yaw_gain * yaw_error + self.center_gain * center_error
        angular = clamp(angular, -self.max_angular, self.max_angular)
        self._command(self.forward_speed * rough_scale, angular)
        self._publish_status(
            "drive "
            f"front={front if front is not None else -1.0:.2f} "
            f"left={self.observation['left'] if self.observation['left'] is not None else -1.0:.2f} "
            f"right={self.observation['right'] if self.observation['right'] is not None else -1.0:.2f} "
            f"ignored_low={self.observation['ignored_low_points']}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MazeMappingMission()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node._command(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
