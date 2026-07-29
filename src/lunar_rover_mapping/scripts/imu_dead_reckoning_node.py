#!/usr/bin/env python3
"""Planar IMU dead-reckoning diagnostic odometry.

This is intentionally a diagnostic baseline, not a production localization
source.  It integrates planar acceleration for position and uses the IMU
orientation yaw when available.  IMU-only position estimates drift quickly; the
node exists so wheel odometry, IMU dead reckoning, and Gazebo truth can be
visualized side-by-side.

Important: IMU measurements are first transformed from the IMU sensor frame into
the rover base frame.  This is required because the Mid-360/IMU is mounted with a
fixed pitch offset relative to base_link.
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(q):
    x, y, z, w = quaternion_tuple(q)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def quaternion_tuple(q):
    if isinstance(q, (tuple, list)):
        return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    return (q.x, q.y, q.z, q.w)


def quaternion_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def quaternion_inverse(q):
    x, y, z, w = quaternion_normalize(q)
    return (-x, -y, -z, w)


def quaternion_multiply(a, b):
    return quaternion_normalize(quaternion_multiply_raw(a, b))


def quaternion_multiply_raw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(q, v):
    """Rotate vector v by quaternion q."""
    q = quaternion_normalize(q)
    vq = (v[0], v[1], v[2], 0.0)
    rq = quaternion_multiply_raw(
        quaternion_multiply_raw(q, vq), quaternion_inverse(q)
    )
    return (rq[0], rq[1], rq[2])


def euler_from_quaternion(q):
    x, y, z, w = quaternion_normalize(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    yaw = yaw_from_quaternion(q)
    return roll, pitch, yaw


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


class ImuDeadReckoningNode(Node):
    def __init__(self):
        super().__init__("imu_dead_reckoning")

        self.imu_topic = self.declare_parameter("imu_topic", "/livox/imu").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/localization/imu_odometry"
        ).value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.imu_frame = self.declare_parameter("imu_frame", "livox_imu_frame").value
        self.child_frame = self.declare_parameter("child_frame", "imu_dead_reckoning").value
        self.calibration_samples = int(
            self.declare_parameter("calibration_samples", 200).value
        )
        self.max_dt = float(self.declare_parameter("max_dt", 0.1).value)
        self.max_accel = float(self.declare_parameter("max_accel_mps2", 5.0).value)
        self.max_speed = float(self.declare_parameter("max_speed_mps", 2.0).value)
        self.velocity_damping = float(
            self.declare_parameter("velocity_damping_per_second", 0.03).value
        )

        self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.create_subscription(Imu, self.imu_topic, self.on_imu, 100)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_from_imu_quat = None
        self.transform_source_frame = None
        self.last_tf_warning = 0.0

        self.samples = 0
        self.accel_bias_x = 0.0
        self.accel_bias_y = 0.0
        self.gyro_bias_z = 0.0
        self.initial_yaw_sum_x = 0.0
        self.initial_yaw_sum_y = 0.0
        self.initial_yaw = None
        self.orientation_available = False

        self.last_stamp = None
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0

        self.get_logger().info(
            f"IMU dead reckoning from {self.imu_topic}; transforming IMU frame into "
            f"{self.base_frame}; calibrating first {self.calibration_samples} samples"
        )

    def ensure_static_transform(self, frame_id):
        source_frame = frame_id.strip() if frame_id.strip() else self.imu_frame
        if (
            self.base_from_imu_quat is not None
            and self.transform_source_frame == source_frame
        ):
            return True

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, source_frame, Time()
            )
        except Exception as exc:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_tf_warning > 2.0:
                self.get_logger().warn(
                    f"Waiting for TF {self.base_frame} <- {source_frame}: {exc}"
                )
                self.last_tf_warning = now
            return False

        self.base_from_imu_quat = quaternion_tuple(transform.transform.rotation)
        self.transform_source_frame = source_frame
        roll, pitch, yaw = euler_from_quaternion(self.base_from_imu_quat)
        self.get_logger().info(
            f"Using IMU extrinsic TF {self.base_frame} <- {source_frame}: "
            f"rpy=({math.degrees(roll):.2f}, {math.degrees(pitch):.2f}, "
            f"{math.degrees(yaw):.2f}) deg"
        )
        return True

    def corrected_measurement(self, msg):
        """Return IMU values expressed in base_frame coordinates."""
        if not self.ensure_static_transform(msg.header.frame_id):
            return None

        accel_base = rotate_vector(
            self.base_from_imu_quat,
            (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ),
        )
        gyro_base = rotate_vector(
            self.base_from_imu_quat,
            (
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ),
        )

        base_orientation = None
        if msg.orientation_covariance[0] != -1.0:
            imu_orientation = quaternion_tuple(msg.orientation)
            # If the IMU reports world<-imu orientation and base<-imu is the
            # fixed mounting transform, then world<-base = world<-imu * imu<-base.
            base_orientation = quaternion_multiply(
                imu_orientation, quaternion_inverse(self.base_from_imu_quat)
            )

        return accel_base, gyro_base, base_orientation

    def reset_integration_state(self):
        self.samples = 0
        self.accel_bias_x = 0.0
        self.accel_bias_y = 0.0
        self.gyro_bias_z = 0.0
        self.initial_yaw_sum_x = 0.0
        self.initial_yaw_sum_y = 0.0
        self.initial_yaw = None
        self.orientation_available = False
        self.last_stamp = None
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0
        self.get_logger().info(
            "IMU transform source changed; resetting integration and calibration, "
            f"calibrating first {self.calibration_samples} samples"
        )

    def stamp_to_seconds(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def update_calibration(self, msg, corrected):
        accel_base, gyro_base, base_orientation = corrected
        self.samples += 1
        n = float(self.samples)
        ax = accel_base[0]
        ay = accel_base[1]
        gz = gyro_base[2]
        self.accel_bias_x += (ax - self.accel_bias_x) / n
        self.accel_bias_y += (ay - self.accel_bias_y) / n
        self.gyro_bias_z += (gz - self.gyro_bias_z) / n

        if base_orientation is not None:
            yaw = yaw_from_quaternion(base_orientation)
            self.initial_yaw_sum_x += math.cos(yaw)
            self.initial_yaw_sum_y += math.sin(yaw)
            self.orientation_available = True

        if self.samples == self.calibration_samples:
            if self.orientation_available:
                self.initial_yaw = math.atan2(
                    self.initial_yaw_sum_y, self.initial_yaw_sum_x
                )
            else:
                self.initial_yaw = 0.0
            self.get_logger().info(
                "IMU calibration complete: "
                f"accel_bias=({self.accel_bias_x:.3f}, {self.accel_bias_y:.3f}) m/s^2, "
                f"gyro_z_bias={self.gyro_bias_z:.5f} rad/s, "
                f"orientation_yaw={'enabled' if self.orientation_available else 'gyro-integrated'}"
            )

    def current_yaw(self, corrected, dt):
        _, gyro_base, base_orientation = corrected
        if self.orientation_available and base_orientation is not None:
            return normalize_angle(yaw_from_quaternion(base_orientation) - self.initial_yaw)
        return normalize_angle(self.yaw + (gyro_base[2] - self.gyro_bias_z) * dt)

    def on_imu(self, msg):
        corrected = self.corrected_measurement(msg)
        if corrected is None:
            self.publish(msg.header.stamp)
            return

        stamp = self.stamp_to_seconds(msg.header.stamp)

        if self.samples < self.calibration_samples:
            self.update_calibration(msg, corrected)
            self.last_stamp = stamp
            self.publish(msg.header.stamp)
            return

        if self.last_stamp is None:
            self.last_stamp = stamp
            self.publish(msg.header.stamp)
            return

        dt = stamp - self.last_stamp
        self.last_stamp = stamp
        if dt <= 0.0:
            self.publish(msg.header.stamp)
            return
        if dt > self.max_dt:
            self.get_logger().warn(f"Skipping IMU integration gap dt={dt:.3f}s")
            self.publish(msg.header.stamp)
            return

        self.yaw = self.current_yaw(corrected, dt)

        accel_base, _, _ = corrected
        body_ax = clamp(accel_base[0] - self.accel_bias_x, self.max_accel)
        body_ay = clamp(accel_base[1] - self.accel_bias_y, self.max_accel)

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        world_ax = cos_yaw * body_ax - sin_yaw * body_ay
        world_ay = sin_yaw * body_ax + cos_yaw * body_ay

        if self.velocity_damping > 0.0:
            damping = math.exp(-self.velocity_damping * dt)
            self.vx *= damping
            self.vy *= damping

        self.vx += world_ax * dt
        self.vy += world_ay * dt

        speed = math.hypot(self.vx, self.vy)
        if self.max_speed > 0.0 and speed > self.max_speed:
            scale = self.max_speed / speed
            self.vx *= scale
            self.vy *= scale

        self.x += self.vx * dt
        self.y += self.vy * dt
        self.publish(msg.header.stamp)

    def publish(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.child_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = quaternion_from_yaw(self.yaw)
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.angular.z = self.gyro_bias_z
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ImuDeadReckoningNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
