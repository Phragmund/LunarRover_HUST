import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def load_mission(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    frame = data.get("frame_id", "map")
    waypoints = []
    for item in data.get("waypoints", []):
        waypoints.append(
            {
                "frame_id": item.get("frame_id", frame),
                "x": float(item["x"]),
                "y": float(item["y"]),
                "yaw": float(item.get("yaw", 0.0)),
                "dwell": float(item.get("dwell", 0.0)),
            }
        )
    if not waypoints:
        raise ValueError("Mission contains no waypoints")
    return waypoints


class MissionRunner(Node):
    def __init__(self):
        super().__init__("mission_runner")
        self.declare_parameter("mission_file", "")
        self.declare_parameter("auto_start", True)
        mission_file = self.get_parameter("mission_file").value
        self.waypoints = load_mission(mission_file) if mission_file else []
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.status_pub = self.create_publisher(String, "/mission/status", 10)
        self.create_service(Trigger, "/mission/pause", self._pause)
        self.create_service(Trigger, "/mission/resume", self._resume)
        self.create_service(Trigger, "/mission/cancel", self._cancel)
        self.index = 0
        self.paused = False
        self.cancelled = False
        self.goal_handle = None
        self.started = False
        if self.waypoints and self.get_parameter("auto_start").value:
            self.create_timer(0.5, self._try_start)

    def _publish_status(self, text):
        message = String()
        message.data = text
        self.status_pub.publish(message)
        self.get_logger().info(text)

    def _try_start(self):
        if not self.started and self.client.server_is_ready():
            self.started = True
            self._send_current()

    def _pose(self, waypoint):
        pose = PoseStamped()
        pose.header.frame_id = waypoint["frame_id"]
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint["x"]
        pose.pose.position.y = waypoint["y"]
        pose.pose.orientation.z = math.sin(waypoint["yaw"] * 0.5)
        pose.pose.orientation.w = math.cos(waypoint["yaw"] * 0.5)
        return pose

    def _send_current(self):
        if self.paused or self.cancelled:
            return
        if self.index >= len(self.waypoints):
            self._publish_status("completed")
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(self.waypoints[self.index])
        self._publish_status(f"navigating:{self.index}")
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self._publish_status(f"rejected:{self.index}")
            return
        result = self.goal_handle.get_result_async()
        result.add_done_callback(self._result)

    def _result(self, future):
        self.goal_handle = None
        if self.paused or self.cancelled:
            return
        status = future.result().status
        if status != 4:
            self._publish_status(f"failed:{self.index}:status={status}")
            return
        dwell = self.waypoints[self.index]["dwell"]
        self._publish_status(f"reached:{self.index}")
        self.index += 1
        if dwell > 0.0:
            timer = self.create_timer(dwell, lambda: self._finish_dwell(timer))
        else:
            self._send_current()

    def _finish_dwell(self, timer):
        timer.cancel()
        self.destroy_timer(timer)
        self._send_current()

    def _pause(self, _, response):
        self.paused = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self._publish_status(f"paused:{self.index}")
        response.success = True
        response.message = "mission paused"
        return response

    def _resume(self, _, response):
        self.paused = False
        self.cancelled = False
        self._publish_status(f"resumed:{self.index}")
        self._send_current()
        response.success = True
        response.message = "mission resumed"
        return response

    def _cancel(self, _, response):
        self.cancelled = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self._publish_status(f"cancelled:{self.index}")
        response.success = True
        response.message = "mission cancelled"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MissionRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

