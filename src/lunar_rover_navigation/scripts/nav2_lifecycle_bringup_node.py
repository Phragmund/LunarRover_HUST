#!/usr/bin/env python3
"""Bring up Nav2 lifecycle nodes with state polling between transitions."""

import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class LifecycleBringup(Node):
    def __init__(self):
        super().__init__("nav2_lifecycle_bringup")
        self.node_names = list(
            self.declare_parameter(
                "node_names",
                [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                ],
            ).value
        )
        self.wait_timeout = float(self.declare_parameter("wait_timeout", 90.0).value)
        self.call_timeout = float(self.declare_parameter("call_timeout", 8.0).value)
        self.poll_period = float(self.declare_parameter("poll_period", 0.2).value)
        self._lifecycle_clients = {}

    def service_name(self, node_name, service):
        node = node_name if node_name.startswith("/") else f"/{node_name}"
        return f"{node}/{service}"

    def get_clients(self, node_name):
        if node_name not in self._lifecycle_clients:
            self._lifecycle_clients[node_name] = (
                self.create_client(GetState, self.service_name(node_name, "get_state")),
                self.create_client(ChangeState, self.service_name(node_name, "change_state")),
            )
        return self._lifecycle_clients[node_name]

    def wait_for_services(self, node_name, deadline):
        get_client, change_client = self.get_clients(node_name)
        while rclpy.ok() and time.monotonic() < deadline:
            if get_client.wait_for_service(timeout_sec=0.1) and change_client.wait_for_service(
                timeout_sec=0.1
            ):
                return True
        return False

    def get_state(self, node_name):
        get_client, _ = self.get_clients(node_name)
        request = GetState.Request()
        future = get_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.call_timeout)
        if not future.done() or future.result() is None:
            return None
        return future.result().current_state.id

    def wait_for_state(self, node_name, expected, deadline):
        while rclpy.ok() and time.monotonic() < deadline:
            state = self.get_state(node_name)
            if state == expected:
                return True
            if state == State.PRIMARY_STATE_FINALIZED:
                return False
            time.sleep(self.poll_period)
        return False

    def change_state(self, node_name, transition_id, expected_state):
        _, change_client = self.get_clients(node_name)
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = change_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.call_timeout)
        deadline = time.monotonic() + self.call_timeout
        result = future.result() if future.done() else None
        if result is not None and not result.success:
            self.get_logger().warn(f"{node_name}: lifecycle transition returned unsuccessful")
        return self.wait_for_state(node_name, expected_state, deadline)

    def bringup_node(self, node_name, deadline):
        if not self.wait_for_services(node_name, deadline):
            raise RuntimeError(f"{node_name}: lifecycle services not available")

        # The lifecycle node may still be inside its constructor (large
        # costmap / plugin load under cold-start I/O), so get_state can time
        # out and return None even though the node is healthy. Poll for a real
        # state instead of treating a transient None as fatal.
        state = None
        while rclpy.ok() and time.monotonic() < deadline and state is None:
            state = self.get_state(node_name)
            if state is None:
                time.sleep(self.poll_period)
        if state is None:
            raise RuntimeError(
                f"{node_name}: could not read lifecycle state within {self.wait_timeout}s"
            )

        if state == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info(f"{node_name}: already active")
            return
        if state == State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().info(f"{node_name}: configuring")
            if not self.change_state(
                node_name,
                Transition.TRANSITION_CONFIGURE,
                State.PRIMARY_STATE_INACTIVE,
            ):
                raise RuntimeError(f"{node_name}: failed to reach inactive")
            state = State.PRIMARY_STATE_INACTIVE
        if state == State.PRIMARY_STATE_INACTIVE:
            self.get_logger().info(f"{node_name}: activating")
            if not self.change_state(
                node_name,
                Transition.TRANSITION_ACTIVATE,
                State.PRIMARY_STATE_ACTIVE,
            ):
                raise RuntimeError(f"{node_name}: failed to reach active")
            return
        raise RuntimeError(f"{node_name}: unexpected lifecycle state {state}")

    def run(self):
        deadline = time.monotonic() + self.wait_timeout
        for node_name in self.node_names:
            self.bringup_node(node_name, deadline)
        self.get_logger().info("Nav2 lifecycle bringup complete")


def main():
    rclpy.init()
    node = LifecycleBringup()
    try:
        node.run()
    except Exception as error:  # noqa: BLE001
        node.get_logger().error(str(error))
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
