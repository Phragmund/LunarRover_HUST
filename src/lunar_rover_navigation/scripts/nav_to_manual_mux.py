#!/usr/bin/env python3
"""Nav2 -> ManualControl 桥接 + 手机优先仲裁。

订阅:
  - /navigation/chassis_cmd_vel  (TwistStamped, 来自 twist_stamper)
  - /grasp/chassis_cmd_vel       (TwistStamped, 抓取状态机的受限步进接近)
  - /web/manual_control           (ManualControl, 来自 web_joystick_server)
发布:
  - /manual_control               (ManualControl, 由 motion_solver 消费)

仲裁逻辑(手机网页有「电脑接管」按钮):
  电脑接管关闭(默认): 手机杆量控制底盘, 松开即停(不自动切 Nav2)。
  电脑接管开启: Nav2 接管底盘; 手机一旦有非零杆量即临时手动接管, 松开交还 Nav2。

TwistStamped -> ManualControl 映射(motion_solver 当前为阿克曼转向):
  ly =  linear.x / v_max * 255               (油门, 正=前进)
  rx = -angular.z / w_max * 255              (近静止时原地转向)
  lx = 阿克曼舵角 delta = atan(wheelbase*w/|v|)  (行进中, LX>0=右转)
"""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from lunar_rover_msgs.msg import ManualControl
from rclpy.node import Node
from std_msgs.msg import Bool


def clamp_int(v):
    return max(-255, min(255, int(round(v))))


class NavToManualMux(Node):
    def __init__(self):
        super().__init__("nav_to_manual_mux")
        self.nav_topic = self.declare_parameter(
            "nav_topic", "/navigation/chassis_cmd_vel"
        ).value
        self.web_topic = self.declare_parameter(
            "web_topic", "/web/manual_control"
        ).value
        self.grasp_topic = self.declare_parameter(
            "grasp_topic", "/grasp/chassis_cmd_vel"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/manual_control"
        ).value
        # 手机满杆量线性速度: max_speed 1.5 r/s * 轮半径 0.0625 m = 0.09375 m/s
        self.v_max = float(self.declare_parameter("v_max", 0.09375).value)
        # 手机满杆量原地转角速度(约): 1.5*0.0625*2/0.4 ≈ 0.47 rad/s
        self.w_max = float(self.declare_parameter("w_max", 0.47).value)
        self.wheelbase = float(self.declare_parameter("wheelbase", 0.42).value)
        self.max_steer_deg = float(self.declare_parameter("max_steer_deg", 30.0).value)
        self.spin_linear_eps = float(
            self.declare_parameter("spin_linear_eps", 0.01).value
        )
        self.max_steer_rad = math.radians(self.max_steer_deg)
        self.grasp_timeout = float(self.declare_parameter("grasp_timeout", 0.35).value)
        self.mission_autonomy_topic = self.declare_parameter(
            "mission_autonomy_topic", "/mission/autonomy_active"
        ).value

        self.auto_mode = False      # 电脑(Nav2)接管状态, 默认关闭
        self.manual_active = False  # 手机当前是否有非零杆量
        self.last_nav = None
        self.last_grasp = None
        self.last_grasp_time = None
        self.mission_autonomy_active = False

        self.publisher = self.create_publisher(ManualControl, self.output_topic, 1)
        self.nav_sub = self.create_subscription(
            TwistStamped, self.nav_topic, self.on_nav, 10
        )
        self.web_sub = self.create_subscription(
            ManualControl, self.web_topic, self.on_web, 10
        )
        self.grasp_sub = self.create_subscription(
            TwistStamped, self.grasp_topic, self.on_grasp, 10
        )
        self.create_subscription(
            Bool, self.mission_autonomy_topic, self.on_mission_autonomy, 10
        )
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info(
            "NavToManualMux: nav %s + web %s -> %s (v_max=%.4f, w_max=%.3f, "
            "wheelbase=%.2f, steer=%.1f deg)"
            % (self.nav_topic, self.web_topic, self.output_topic,
               self.v_max, self.w_max, self.wheelbase, self.max_steer_deg)
        )

    def _convert_nav(self, msg):
        out = ManualControl()
        v = msg.twist.linear.x
        w = msg.twist.angular.z
        out.ly = clamp_int(v / self.v_max * 255.0)
        if abs(v) < self.spin_linear_eps:
            # 近静止: 原地转向 (rx), w>0(逆时针) -> rx<0
            out.rx = clamp_int(-w / self.w_max * 255.0)
            out.lx = 0
        else:
            # 行进中: 阿克曼转向角 delta = atan(L*w/|v|)
            delta = math.atan2(self.wheelbase * w, abs(v))
            lx_sign = -1.0 if v >= 0.0 else 1.0  # 前进左转 LX<0; 倒车反向
            out.lx = clamp_int(lx_sign * delta / self.max_steer_rad * 255.0)
            out.rx = 0
        return out

    def on_web(self, msg):
        self.auto_mode = bool(msg.auto_takeover)
        self.manual_active = (
            msg.lx != 0 or msg.ly != 0 or msg.rx != 0
            or msg.relax or msg.rearm or msg.calibrate
        )
        if self.manual_active:
            # 手机手动接管(两种模式下均有效)
            self.publisher.publish(msg)
        elif self.grasp_active():
            # A zero web frame must not interrupt a running guarded grasp
            # approach. Any non-zero web input above has already preempted it.
            return
        elif self.autonomy_enabled():
            # 电脑接管 + 手机空闲 -> Nav2
            if self.last_nav is not None:
                self.publisher.publish(self._convert_nav(self.last_nav))
            else:
                self.publisher.publish(ManualControl())
        else:
            # 手动模式 + 手机空闲 -> 停止(不自动切 Nav2)
            self.publisher.publish(ManualControl())

    def on_nav(self, msg):
        self.last_nav = msg
        if self.autonomy_enabled() and not self.manual_active and not self.grasp_active():
            self.publisher.publish(self._convert_nav(msg))
        # 手动模式或手机接管时忽略 Nav2

    def grasp_active(self):
        if self.last_grasp_time is None:
            return False
        age = (self.get_clock().now() - self.last_grasp_time).nanoseconds * 1e-9
        return 0.0 <= age <= self.grasp_timeout

    def on_grasp(self, msg):
        self.last_grasp = msg
        self.last_grasp_time = self.get_clock().now()
        if not self.manual_active:
            self.publisher.publish(self._convert_nav(msg))

    def on_mission_autonomy(self, msg):
        """Let the task executive enable Nav2 without spoofing web input."""
        self.mission_autonomy_active = bool(msg.data)

    def autonomy_enabled(self):
        return self.auto_mode or self.mission_autonomy_active

    def on_timer(self):
        if self.last_grasp_time is None or self.grasp_active():
            return
        self.last_grasp_time = None
        if self.manual_active:
            return
        if self.autonomy_enabled() and self.last_nav is not None:
            self.publisher.publish(self._convert_nav(self.last_nav))
        else:
            self.publisher.publish(ManualControl())


def main():
    rclpy.init()
    node = NavToManualMux()
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
