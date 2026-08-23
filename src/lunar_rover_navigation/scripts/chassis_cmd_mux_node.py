#!/usr/bin/env python3
"""Arbitrate navigation, physical-keyboard, and web chassis commands."""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class ChassisCmdMux(Node):
    def __init__(self):
        super().__init__("chassis_cmd_mux")
        self.nav_topic = self.declare_parameter(
            "nav_topic", "/navigation/chassis_cmd_vel"
        ).value
        self.keyboard_topic = self.declare_parameter(
            "keyboard_topic", "/keyboard/chassis_cmd_vel"
        ).value
        self.web_topic = self.declare_parameter(
            "web_topic", "/web/chassis_cmd_vel"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/chassis/cmd_vel"
        ).value
        self.keyboard_timeout = float(
            self.declare_parameter("keyboard_timeout", 0.5).value
        )
        self.web_timeout = float(
            self.declare_parameter("web_timeout", 0.5).value
        )
        self.status_period = float(self.declare_parameter("status_period", 1.0).value)
        if (
            not math.isfinite(self.keyboard_timeout)
            or self.keyboard_timeout <= 0.0
            or not math.isfinite(self.web_timeout)
            or self.web_timeout <= 0.0
        ):
            raise ValueError("manual-source timeouts must be finite and positive")

        self.last_keyboard_time = None
        self.last_web_time = None
        self.mode = "navigation"
        self.publisher = self.create_publisher(TwistStamped, self.output_topic, 1)
        self.nav_subscription = self.create_subscription(
            TwistStamped, self.nav_topic, self.on_nav_command, 10
        )
        self.keyboard_subscription = self.create_subscription(
            TwistStamped, self.keyboard_topic, self.on_keyboard_command, 10
        )
        self.web_subscription = self.create_subscription(
            TwistStamped, self.web_topic, self.on_web_command, 10
        )
        self.create_timer(max(0.1, self.status_period), self.on_timer)
        self.get_logger().info(
            "Chassis command mux: navigation %s + keyboard %s + web %s -> %s"
            % (
                self.nav_topic,
                self.keyboard_topic,
                self.web_topic,
                self.output_topic,
            )
        )

    def keyboard_active(self):
        if self.last_keyboard_time is None:
            return False
        age = (self.get_clock().now() - self.last_keyboard_time).nanoseconds * 1e-9
        return 0.0 <= age <= self.keyboard_timeout

    def web_active(self):
        if self.last_web_time is None:
            return False
        age = (self.get_clock().now() - self.last_web_time).nanoseconds * 1e-9
        return 0.0 <= age <= self.web_timeout

    def manual_active(self):
        return self.web_active() or self.keyboard_active()

    def set_mode(self, mode):
        if mode == self.mode:
            return
        self.mode = mode
        self.get_logger().info("Chassis command source: %s" % mode)

    def on_nav_command(self, message):
        if self.manual_active():
            return
        self.set_mode("navigation")
        self.publisher.publish(message)

    def on_keyboard_command(self, message):
        self.last_keyboard_time = self.get_clock().now()
        if self.web_active():
            return
        self.set_mode("keyboard")
        self.publisher.publish(message)

    def on_web_command(self, message):
        self.last_web_time = self.get_clock().now()
        self.set_mode("web")
        self.publisher.publish(message)

    def on_timer(self):
        if self.mode == "web" and not self.web_active():
            if self.keyboard_active():
                self.set_mode("keyboard")
            else:
                self.set_mode("navigation")
        elif self.mode == "keyboard" and not self.keyboard_active():
            self.set_mode("navigation")


def main():
    rclpy.init()
    node = ChassisCmdMux()
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
