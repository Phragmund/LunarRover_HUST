#!/usr/bin/env python3
"""Arbitrate navigation and keyboard chassis commands."""

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
        self.output_topic = self.declare_parameter(
            "output_topic", "/chassis/cmd_vel"
        ).value
        self.keyboard_timeout = float(
            self.declare_parameter("keyboard_timeout", 5.0).value
        )
        self.status_period = float(self.declare_parameter("status_period", 1.0).value)
        if not math.isfinite(self.keyboard_timeout) or self.keyboard_timeout <= 0.0:
            raise ValueError("keyboard_timeout must be finite and positive")

        self.last_keyboard_time = None
        self.mode = "navigation"
        self.publisher = self.create_publisher(TwistStamped, self.output_topic, 1)
        self.nav_subscription = self.create_subscription(
            TwistStamped, self.nav_topic, self.on_nav_command, 10
        )
        self.keyboard_subscription = self.create_subscription(
            TwistStamped, self.keyboard_topic, self.on_keyboard_command, 10
        )
        self.create_timer(max(0.1, self.status_period), self.on_timer)
        self.get_logger().info(
            "Chassis command mux: navigation %s + keyboard %s -> %s, "
            "keyboard timeout %.1fs"
            % (
                self.nav_topic,
                self.keyboard_topic,
                self.output_topic,
                self.keyboard_timeout,
            )
        )

    def keyboard_active(self):
        if self.last_keyboard_time is None:
            return False
        age = (self.get_clock().now() - self.last_keyboard_time).nanoseconds * 1e-9
        return 0.0 <= age <= self.keyboard_timeout

    def set_mode(self, mode):
        if mode == self.mode:
            return
        self.mode = mode
        self.get_logger().info("Chassis command source: %s" % mode)

    def on_nav_command(self, message):
        if self.keyboard_active():
            self.set_mode("keyboard")
            return
        self.set_mode("navigation")
        self.publisher.publish(message)

    def on_keyboard_command(self, message):
        self.last_keyboard_time = self.get_clock().now()
        self.set_mode("keyboard")
        self.publisher.publish(message)

    def on_timer(self):
        if self.mode == "keyboard" and not self.keyboard_active():
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
