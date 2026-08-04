#!/usr/bin/env python3
"""Publish Gazebo MID360 samples with the Livox ROS driver message contract.

This node keeps the public simulation interface close to the mature
livox_laser_simulation_RO2 / livox_ros_driver2 path used by FAST-LIO:
  * /livox/lidar: livox_ros_driver2/msg/CustomMsg with per-point offset_time
  * /livox/points: sensor_msgs/msg/PointCloud2 with time, tag, and line fields
  * /livox/imu: sensor_msgs/msg/Imu in the MID360 IMU frame
"""

from copy import deepcopy
import math
import struct

from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


FLOAT32 = PointField.FLOAT32
UINT8 = PointField.UINT8
UINT16 = PointField.UINT16


def field_info(fields, name):
    for field in fields:
        if field.name == name:
            return field.offset, field.datatype
    return None, None


def read_float32(data, offset):
    return struct.unpack_from("<f", data, offset)[0]


def read_uint(data, offset, datatype):
    if datatype == UINT8:
        return struct.unpack_from("<B", data, offset)[0]
    if datatype == UINT16:
        return struct.unpack_from("<H", data, offset)[0]
    if datatype == PointField.UINT32:
        return struct.unpack_from("<I", data, offset)[0]
    if datatype == PointField.INT8:
        return struct.unpack_from("<b", data, offset)[0]
    if datatype == PointField.INT16:
        return struct.unpack_from("<h", data, offset)[0]
    if datatype == PointField.INT32:
        return struct.unpack_from("<i", data, offset)[0]
    return 0


def q_tuple(msg):
    return (float(msg.x), float(msg.y), float(msg.z), float(msg.w))


def q_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_normalize(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(value / norm for value in q)


def rotate(q, v):
    q = q_normalize(q)
    return q_multiply(q_multiply(q, (v[0], v[1], v[2], 0.0)), q_conjugate(q))[:3]


def angular_velocity_from_delta(previous_q, current_q, dt):
    if dt <= 0.0:
        return (0.0, 0.0, 0.0)
    delta = q_normalize(q_multiply(q_conjugate(previous_q), current_q))
    if delta[3] < 0.0:
        delta = tuple(-value for value in delta)
    axis = delta[:3]
    axis_norm = math.sqrt(sum(value * value for value in axis))
    if axis_norm <= 1.0e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(axis_norm, max(-1.0, min(1.0, delta[3])))
    return tuple(value * angle / (axis_norm * dt) for value in axis)


def set_quaternion(out, q):
    out.x, out.y, out.z, out.w = q


class Mid360LivoxSimDriver(Node):
    def __init__(self):
        super().__init__("mid360_livox_sim_driver")
        self.raw_points_topic = self.declare_parameter(
            "raw_points_topic", "/mid360/raw_points"
        ).value
        self.raw_imu_topic = self.declare_parameter("raw_imu_topic", "/mid360/raw_imu").value
        self.truth_odom_topic = self.declare_parameter(
            "truth_odom_topic", "/simulation/ground_truth/odometry"
        ).value
        self.imu_source = self.declare_parameter(
            "imu_source", "ground_truth_kinematics"
        ).value
        self.custom_topic = self.declare_parameter("custom_topic", "/livox/lidar").value
        self.points_topic = self.declare_parameter("points_topic", "/livox/points").value
        self.imu_topic = self.declare_parameter("imu_topic", "/livox/imu").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.lidar_frame = self.declare_parameter("lidar_frame", "livox_frame").value
        self.imu_frame = self.declare_parameter("imu_frame", "livox_imu_frame").value
        self.scan_rate = float(self.declare_parameter("scan_rate", 10.0).value)
        self.scan_lines = int(self.declare_parameter("scan_lines", 11).value)
        self.lidar_id = int(self.declare_parameter("lidar_id", 1).value)
        self.gravity = float(self.declare_parameter("gravity_mps2", 9.80665).value)
        self.publish_imu = bool(self.declare_parameter("publish_imu", True).value)
        self.use_raw_imu_clock = bool(
            self.declare_parameter("use_raw_imu_clock", True).value
        )
        self.publish_pointcloud2 = bool(
            self.declare_parameter("publish_pointcloud2", True).value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_to_imu_q = None
        self.scan_period_us = max(1, int(round(1.0e6 / max(self.scan_rate, 1.0e-3))))
        self.warned_fields = False
        self.warned_bigendian = False
        self.last_points_stamp = None
        self.last_imu_stamp = None
        self.previous_truth_stamp = None
        self.previous_truth_velocity = None
        self.previous_imu_orientation = None
        self.latest_imu_orientation = None
        self.latest_gyro = (0.0, 0.0, 0.0)
        self.latest_specific_force = (0.0, 0.0, 0.0)
        self.raw_imu_clock_seen = False

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.custom_pub = self.create_publisher(CustomMsg, self.custom_topic, 5)
        self.points_pub = self.create_publisher(PointCloud2, self.points_topic, 5)
        self.imu_pub = (
            self.create_publisher(Imu, self.imu_topic, 50) if self.publish_imu else None
        )
        self.create_subscription(PointCloud2, self.raw_points_topic, self.on_points, sensor_qos)
        if self.publish_imu:
            if self.imu_source == "raw":
                self.create_subscription(Imu, self.raw_imu_topic, self.on_imu, sensor_qos)
            else:
                self.create_subscription(
                    Odometry, self.truth_odom_topic, self.on_truth_odom, sensor_qos
                )
                if self.use_raw_imu_clock:
                    self.create_subscription(
                        Imu, self.raw_imu_topic, self.on_raw_imu_clock, sensor_qos
                    )
                self.create_timer(0.2, self.try_initialize_tf)

        self.get_logger().info(
            "MID360 Livox sim driver: "
            f"{self.raw_points_topic}->{self.custom_topic},{self.points_topic}; "
            f"publish_imu={self.publish_imu}; imu_source={self.imu_source}->{self.imu_topic}; "
            f"use_raw_imu_clock={self.use_raw_imu_clock}"
        )

    def try_initialize_tf(self):
        if self.base_to_imu_q is not None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.imu_frame, Time(), Duration(seconds=0.1)
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for {self.base_frame}->{self.imu_frame} TF: {exc}",
                throttle_duration_sec=2.0,
            )
            return
        self.base_to_imu_q = q_normalize(q_tuple(transform.transform.rotation))
        self.get_logger().info(f"Using URDF TF {self.base_frame}->{self.imu_frame} for IMU synthesis")

    def on_imu(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_imu_stamp is not None and stamp <= self.last_imu_stamp:
            return
        self.last_imu_stamp = stamp
        out = deepcopy(msg)
        out.header.frame_id = self.imu_frame
        self.imu_pub.publish(out)

    def on_truth_odom(self, msg):
        if self.base_to_imu_q is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        if self.previous_truth_stamp is not None and stamp <= self.previous_truth_stamp:
            return

        current_velocity = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z),
        )
        world_to_base_q = q_normalize(q_tuple(msg.pose.pose.orientation))
        world_to_imu_q = q_normalize(q_multiply(world_to_base_q, self.base_to_imu_q))

        dt = 0.0
        if self.previous_truth_stamp is not None:
            dt = (stamp - self.previous_truth_stamp).nanoseconds * 1.0e-9

        gyro = (0.0, 0.0, 0.0)
        acceleration_world = (0.0, 0.0, 0.0)
        if dt > 1.0e-4 and self.previous_imu_orientation is not None:
            gyro = angular_velocity_from_delta(
                self.previous_imu_orientation, world_to_imu_q, dt
            )
            if self.previous_truth_velocity is not None:
                acceleration_world = tuple(
                    (current_velocity[i] - self.previous_truth_velocity[i]) / dt
                    for i in range(3)
                )

        specific_force_world = (
            acceleration_world[0],
            acceleration_world[1],
            acceleration_world[2] + self.gravity,
        )
        specific_force_imu = rotate(q_conjugate(world_to_imu_q), specific_force_world)

        self.latest_imu_orientation = world_to_imu_q
        self.latest_gyro = gyro
        self.latest_specific_force = specific_force_imu
        if not self.use_raw_imu_clock or not self.raw_imu_clock_seen:
            self.publish_synthetic_imu(msg.header.stamp)

        self.previous_truth_stamp = stamp
        self.previous_truth_velocity = current_velocity
        self.previous_imu_orientation = world_to_imu_q

    def on_raw_imu_clock(self, msg):
        self.raw_imu_clock_seen = True
        self.publish_synthetic_imu(msg.header.stamp)

    def publish_synthetic_imu(self, stamp_msg):
        if self.imu_pub is None:
            return
        if self.latest_imu_orientation is None:
            return
        stamp = Time.from_msg(stamp_msg)
        if self.last_imu_stamp is not None and stamp <= self.last_imu_stamp:
            return

        out = Imu()
        out.header.stamp = stamp_msg
        out.header.frame_id = self.imu_frame
        set_quaternion(out.orientation, self.latest_imu_orientation)
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = self.latest_gyro
        (
            out.linear_acceleration.x,
            out.linear_acceleration.y,
            out.linear_acceleration.z,
        ) = self.latest_specific_force
        self.imu_pub.publish(out)
        self.last_imu_stamp = stamp

    def on_points(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_points_stamp is not None and stamp <= self.last_points_stamp:
            return
        self.last_points_stamp = stamp
        if msg.is_bigendian:
            if not self.warned_bigendian:
                self.get_logger().error("Big-endian PointCloud2 is not supported")
                self.warned_bigendian = True
            return

        x_info = field_info(msg.fields, "x")
        y_info = field_info(msg.fields, "y")
        z_info = field_info(msg.fields, "z")
        intensity_info = field_info(msg.fields, "intensity")
        ring_info = field_info(msg.fields, "ring")
        if (
            x_info[0] is None
            or y_info[0] is None
            or z_info[0] is None
            or x_info[1] != FLOAT32
            or y_info[1] != FLOAT32
            or z_info[1] != FLOAT32
        ):
            if not self.warned_fields:
                self.get_logger().error("Raw MID360 cloud must contain float32 x/y/z fields")
                self.warned_fields = True
            return

        total = msg.width * msg.height
        if total == 0:
            return

        custom = CustomMsg()
        custom.header.stamp = msg.header.stamp
        custom.header.frame_id = self.lidar_frame
        custom.timebase = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        custom.lidar_id = self.lidar_id
        custom.rsvd = [0, 0, 0]

        pc_rows = []
        data = msg.data
        horizontal_steps = msg.width if msg.height > 1 else total
        last_column = max(horizontal_steps - 1, 1)
        for index in range(total):
            base = index * msg.point_step
            x = read_float32(data, base + x_info[0])
            y = read_float32(data, base + y_info[0])
            z = read_float32(data, base + z_info[0])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            intensity = 0.0
            if intensity_info[0] is not None and intensity_info[1] == FLOAT32:
                intensity = read_float32(data, base + intensity_info[0])
            reflectivity = max(0, min(255, int(round(intensity))))

            if ring_info[0] is not None:
                line = read_uint(data, base + ring_info[0], ring_info[1]) % max(
                    self.scan_lines, 1
                )
            elif msg.height > 1 and msg.width > 0:
                line = (index // msg.width) % max(self.scan_lines, 1)
            else:
                line = index % max(self.scan_lines, 1)
            column = index % horizontal_steps
            offset_time = int(round(column * self.scan_period_us / last_column))

            point = CustomPoint()
            point.offset_time = offset_time
            point.x = x
            point.y = y
            point.z = z
            point.reflectivity = reflectivity
            point.tag = 0x10
            point.line = line
            custom.points.append(point)
            pc_rows.append((x, y, z, float(reflectivity), offset_time * 1.0e-6, 0x10, line))

        custom.point_num = len(custom.points)
        self.custom_pub.publish(custom)

        if self.publish_pointcloud2:
            header = deepcopy(msg.header)
            header.frame_id = self.lidar_frame
            fields = [
                PointField(name="x", offset=0, datatype=FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=FLOAT32, count=1),
                PointField(name="time", offset=16, datatype=FLOAT32, count=1),
                PointField(name="tag", offset=20, datatype=UINT8, count=1),
                PointField(name="line", offset=21, datatype=UINT8, count=1),
            ]
            self.points_pub.publish(point_cloud2.create_cloud(header, fields, pc_rows))


def main():
    rclpy.init()
    node = Mid360LivoxSimDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
