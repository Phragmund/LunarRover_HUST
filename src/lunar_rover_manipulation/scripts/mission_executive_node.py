#!/usr/bin/env python3
"""Ten-minute collect-and-return executive for the physical rover.

The node deliberately owns *task* decisions only.  FAST-LIO2/terrain/Nav2
continue to own long-range motion, ``active_search`` continues to validate
RGB-D observations, and ``grasp_state_machine`` owns the guarded 0.1 m visual
approach plus arm motion.  This prevents two nodes from commanding the chassis
at the same time.

States:
  READY -> SEARCHING -> TARGET_PENDING -> GRASPING -> SEARCHING
                                      -> RETURNING -> UNLOADING -> COMPLETE
An elapsed-time watchdog leaves the configured unload reserve and an estimated
Nav2 return duration before the ten-minute hard limit.
"""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from lunar_rover_msgs.msg import TargetObservation
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String, UInt32
from tf2_ros import Buffer, TransformException, TransformListener


class MissionExecutive(Node):
    def __init__(self):
        super().__init__("mission_executive")
        self.total_duration_sec = float(self.declare_parameter("mission_duration_sec", 600.0).value)
        self.unload_reserve_sec = float(self.declare_parameter("unload_reserve_sec", 120.0).value)
        self.return_safety_sec = float(self.declare_parameter("return_safety_sec", 20.0).value)
        self.return_speed_mps = float(self.declare_parameter("return_speed_mps", 0.035).value)
        self.return_turn_allowance_sec = float(self.declare_parameter("return_turn_allowance_sec", 12.0).value)
        self.home_arrival_tolerance_m = float(self.declare_parameter("home_arrival_tolerance_m", 0.20).value)
        self.grasp_timeout_sec = float(self.declare_parameter("grasp_timeout_sec", 110.0).value)
        self.target_confirm_max_age_sec = float(self.declare_parameter("target_confirm_max_age_sec", 1.0).value)
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.nav_cancel = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")

        self.status_pub = self.create_publisher(String, "/mission/status", 10)
        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.autonomy_pub = self.create_publisher(Bool, "/mission/autonomy_active", 10)
        self.exploration_pub = self.create_publisher(Bool, "/mission/exploration_enabled", 10)
        self.grasp_trigger_pub = self.create_publisher(Empty, "/grasp/trigger", 10)
        self.grasp_command_pub = self.create_publisher(String, "/grasp/command", 10)
        self.count_pub = self.create_publisher(UInt32, "/mission/collected_count", 10)

        self.create_subscription(String, "/mission/command", self.on_command, 10)
        self.create_subscription(TargetObservation, "/search/target_confirmed", self.on_target, 10)
        self.create_subscription(String, "/grasp/result", self.on_grasp_result, 10)
        self.create_subscription(String, "/grasp/status", self.on_grasp_status, 10)

        self.state = "READY"
        self.started_at = None
        self.home_pose = None
        self.return_goal_handle = None
        self.grasp_started_at = None
        self.collected_count = 0
        self._last_status = ""
        self.publish_modes(False, False)
        self.publish_state("READY", "等待 /mission/command=start；不会自行启动车体")
        self.create_timer(0.25, self.tick)

    def publish_modes(self, autonomy, exploration):
        msg = Bool()
        msg.data = bool(autonomy)
        self.autonomy_pub.publish(msg)
        msg.data = bool(exploration)
        self.exploration_pub.publish(msg)

    def publish_state(self, state, detail=""):
        self.state = state
        message = String()
        message.data = state
        self.state_pub.publish(message)
        text = state if not detail else f"{state}: {detail}"
        if text != self._last_status:
            self._last_status = text
            message.data = text
            self.status_pub.publish(message)
            self.get_logger().info(text)

    def publish_count(self):
        message = UInt32()
        message.data = self.collected_count
        self.count_pub.publish(message)

    def robot_pose(self):
        transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

    def elapsed(self):
        return 0.0 if self.started_at is None else time.monotonic() - self.started_at

    def return_estimate(self):
        if self.home_pose is None:
            return math.inf
        try:
            x, y, _ = self.robot_pose()
        except TransformException:
            return math.inf
        distance = math.hypot(self.home_pose[0] - x, self.home_pose[1] - y)
        return distance / max(self.return_speed_mps, 0.005) + self.return_turn_allowance_sec

    def must_return_now(self):
        return self.elapsed() + self.return_estimate() + self.unload_reserve_sec + self.return_safety_sec >= self.total_duration_sec

    def on_command(self, message):
        command = (message.data or "").strip().lower()
        if command in ("start", "resume") and self.state in ("READY", "ABORTED", "COMPLETE"):
            try:
                self.home_pose = self.robot_pose()
            except TransformException:
                self.publish_state("READY", "未获得 map->base_link，无法记录起点")
                return
            self.started_at = time.monotonic()
            self.collected_count = 0
            self.publish_count()
            self.publish_modes(True, True)
            self.publish_state("SEARCHING", "FAST-LIO2/地形建图+前沿探索运行；RGB-D 持续辅助检出")
        elif command in ("return", "return_home") and self.state not in ("READY", "COMPLETE"):
            self.begin_return("人工要求返航")
        elif command in ("abort", "stop", "cancel"):
            self.cancel_navigation()
            self.grasp_command_pub.publish(String(data="abort"))
            self.publish_modes(False, False)
            self.publish_state("ABORTED", "任务已中止，底盘自治与探索均已释放")

    def on_target(self, observation):
        if self.state != "SEARCHING" or self.must_return_now():
            return
        stamp = observation.header.stamp
        age = self.get_clock().now().nanoseconds * 1e-9 - (stamp.sec + stamp.nanosec * 1e-9)
        if age > self.target_confirm_max_age_sec:
            return
        # Frontier action must be cancelled before the grasp state machine
        # freezes lidar / performs its image-centred base approach.
        self.cancel_navigation()
        self.publish_modes(True, False)
        self.publish_state("TARGET_PENDING", f"确认目标 {observation.class_id}；停止探索底盘，交给视觉抓取")
        self.grasp_trigger_pub.publish(Empty())
        self.grasp_started_at = time.monotonic()
        self.publish_state("GRASPING", "抓取状态机负责停车、RGB-D 重判距和 0.1m 步进接近")

    def on_grasp_result(self, message):
        result = (message.data or "").strip().upper()
        if self.state == "UNLOADING":
            if result == "UNLOAD_COMPLETE":
                self.publish_state(
                    "COMPLETE",
                    f"任务完成：共回收 {self.collected_count} 个物块，用时 {self.elapsed():.0f}s",
                )
            elif result == "FAILED":
                self.publish_state("ABORTED", "卸货动作失败，等待人工确认")
            return
        if self.state != "GRASPING":
            return
        if result == "SUCCESS":
            self.collected_count += 1
            self.publish_count()
            if self.must_return_now():
                self.begin_return("抓取完成后返航预算已触发")
            else:
                self.publish_modes(True, True)
                self.publish_state("SEARCHING", f"第 {self.collected_count} 个物块已计数，恢复刚才的前沿搜索")
        elif result in ("FAILED", "UNCONFIRMED", "ABORTED"):
            self.publish_modes(True, True)
            self.publish_state("SEARCHING", f"抓取未成功({result})，恢复搜索且不增加计数")

    def on_grasp_status(self, message):
        # Compatibility with the existing visual confirmation path: it emits a
        # precise failure status before the result publisher is available.
        if self.state == "GRASPING" and "GRASP_UNCONFIRMED" in message.data:
            self.on_grasp_result(String(data="UNCONFIRMED"))

    def cancel_navigation(self):
        if self.nav_cancel.service_is_ready():
            self.nav_cancel.call_async(CancelGoal.Request())

    def begin_return(self, reason):
        if self.state in ("RETURNING", "UNLOADING", "COMPLETE"):
            return
        self.cancel_navigation()
        self.publish_modes(True, False)
        if self.home_pose is None:
            self.publish_state("ABORTED", "未记录起点，无法自动返航")
            return
        try:
            x, y, _ = self.robot_pose()
        except TransformException:
            self.publish_state("ABORTED", "无法取得当前地图位姿，无法自动返航")
            return
        distance = math.hypot(self.home_pose[0] - x, self.home_pose[1] - y)
        if distance <= self.home_arrival_tolerance_m:
            self.begin_unload("已在起点")
            return
        yaw = math.atan2(self.home_pose[1] - y, self.home_pose[0] - x)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.home_pose[0]
        goal.pose.pose.position.y = self.home_pose[1]
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.publish_state("RETURNING", f"{reason}；距起点 {distance:.2f}m，估计 {self.return_estimate():.0f}s")
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_return_goal)

    def on_return_goal(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.publish_state("ABORTED", "返航 Nav2 目标被拒绝")
            self.publish_modes(False, False)
            return
        self.return_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_return_result)

    def on_return_result(self, future):
        if self.state != "RETURNING":
            return
        result = future.result()
        if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
            self.begin_unload("已到起点")
        else:
            self.publish_state("ABORTED", "返航未完成；保持底盘停止，等待人工接管")
            self.publish_modes(False, False)

    def begin_unload(self, reason):
        self.publish_modes(False, False)
        self.grasp_command_pub.publish(String(data="unload"))
        self.publish_state("UNLOADING", f"{reason}；执行卸货序列并预留 {self.unload_reserve_sec:.0f}s")

    def tick(self):
        if self.state == "SEARCHING" and self.must_return_now():
            self.begin_return("达到返航时间预算")
        elif self.state == "GRASPING" and self.grasp_started_at is not None:
            if time.monotonic() - self.grasp_started_at > self.grasp_timeout_sec:
                self.grasp_command_pub.publish(String(data="abort"))
                self.publish_modes(True, True)
                self.publish_state("SEARCHING", "抓取超时，恢复搜索")
        elif self.state == "UNLOADING":
            # The arm node emits this once its configured release sequence ends.
            pass


def main():
    rclpy.init()
    node = MissionExecutive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_modes(False, False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
