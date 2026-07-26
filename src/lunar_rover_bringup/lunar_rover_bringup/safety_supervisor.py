import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
from lunar_rover_msgs.msg import ChassisState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def roll_pitch(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(sin_pitch)


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__("safety_supervisor")
        self.declare_parameter("input_cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("output_cmd_topic", "/chassis/cmd_vel")
        self.declare_parameter("lidar_topic", "/livox/points")
        self.declare_parameter("terrain_topic", "/terrain/traversability")
        self.declare_parameter("odometry_topic", "/odometry/filtered")
        self.declare_parameter("chassis_state_topic", "/chassis/state")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("lidar_timeout", 0.5)
        self.declare_parameter("terrain_timeout", 0.5)
        self.declare_parameter("odometry_timeout", 0.5)
        self.declare_parameter("chassis_timeout", 0.3)
        self.declare_parameter("stop_tilt_degrees", 25.0)
        self.declare_parameter("slow_tilt_degrees", 15.0)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 0.8)

        self.timeout = {
            "command": self.get_parameter("command_timeout").value,
            "lidar": self.get_parameter("lidar_timeout").value,
            "terrain": self.get_parameter("terrain_timeout").value,
            "odometry": self.get_parameter("odometry_timeout").value,
            "chassis": self.get_parameter("chassis_timeout").value,
        }
        self.last_seen = {name: None for name in self.timeout}
        self.command = Twist()
        self.chassis = None
        self.tilt = (0.0, 0.0)
        self.max_linear = self.get_parameter("max_linear_speed").value
        self.max_angular = self.get_parameter("max_angular_speed").value
        self.stop_tilt = math.radians(self.get_parameter("stop_tilt_degrees").value)
        self.slow_tilt = math.radians(self.get_parameter("slow_tilt_degrees").value)

        self.command_pub = self.create_publisher(
            TwistStamped, self.get_parameter("output_cmd_topic").value, 10
        )
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/safety/diagnostics", 10
        )
        self.create_subscription(
            Twist,
            self.get_parameter("input_cmd_topic").value,
            self._command_callback,
            10,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("lidar_topic").value,
            lambda _: self._mark("lidar"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("terrain_topic").value,
            lambda _: self._mark("terrain"),
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odometry_topic").value,
            self._odom_callback,
            20,
        )
        self.create_subscription(
            ChassisState,
            self.get_parameter("chassis_state_topic").value,
            self._chassis_callback,
            10,
        )
        period = 1.0 / self.get_parameter("publish_rate").value
        self.create_timer(period, self._publish)

    def _now(self):
        return self.get_clock().now()

    def _mark(self, name):
        self.last_seen[name] = self._now()

    def _command_callback(self, message):
        self.command = message
        self._mark("command")

    def _odom_callback(self, message):
        self.tilt = roll_pitch(message.pose.pose.orientation)
        self._mark("odometry")

    def _chassis_callback(self, message):
        self.chassis = message
        self._mark("chassis")

    def _age(self, name):
        stamp = self.last_seen[name]
        if stamp is None:
            return float("inf")
        return (self._now() - stamp).nanoseconds * 1e-9

    def _health(self):
        reasons = []
        for name in ("lidar", "terrain", "odometry", "chassis"):
            if self._age(name) > self.timeout[name]:
                reasons.append(f"{name}_stale")
        if self.chassis is not None:
            if self.chassis.estop:
                reasons.append("estop")
            if not self.chassis.enabled:
                reasons.append("chassis_disabled")
            if not self.chassis.watchdog_ok:
                reasons.append("lower_watchdog")
            if self.chassis.fault_bits:
                reasons.append(f"fault_0x{self.chassis.fault_bits:08x}")
        if max(abs(self.tilt[0]), abs(self.tilt[1])) >= self.stop_tilt:
            reasons.append("tilt_limit")
        return reasons

    def _publish(self):
        reasons = self._health()
        command_fresh = self._age("command") <= self.timeout["command"]
        output = TwistStamped()
        output.header.stamp = self._now().to_msg()
        output.header.frame_id = "base_footprint"
        if not reasons and command_fresh:
            tilt = max(abs(self.tilt[0]), abs(self.tilt[1]))
            scale = 1.0
            if tilt > self.slow_tilt:
                scale = max(
                    0.0,
                    (self.stop_tilt - tilt) / (self.stop_tilt - self.slow_tilt),
                )
            output.twist.linear.x = max(
                -self.max_linear,
                min(self.max_linear, self.command.linear.x * scale),
            )
            output.twist.angular.z = max(
                -self.max_angular,
                min(self.max_angular, self.command.angular.z * scale),
            )
        self.command_pub.publish(output)

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = output.header.stamp
        status = DiagnosticStatus()
        status.name = "lunar_rover/safety_gate"
        status.hardware_id = "upper_controller"
        status.level = DiagnosticStatus.OK if not reasons else DiagnosticStatus.ERROR
        status.message = "ready" if not reasons else ",".join(reasons)
        status.values = [
            KeyValue(key=name + "_age_s", value=f"{self._age(name):.3f}")
            for name in self.timeout
        ]
        diagnostics.status = [status]
        self.diagnostic_pub.publish(diagnostics)


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
