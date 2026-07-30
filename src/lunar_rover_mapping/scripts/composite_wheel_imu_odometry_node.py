#!/usr/bin/env python3
"""Wheel-distance + IMU-heading planar odometry.

This node is intentionally simpler than a general-purpose EKF:

* wheel feedback odometry contributes only forward body-frame speed;
* IMU orientation / angular velocity contributes heading;
* IMU acceleration is not double-integrated for position.

The goal is to remove the weakest part of the current wheel odometry stack
-- wheel-derived yaw during slip-heavy turns -- while avoiding unreliable
IMU-only position integration.
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def quaternion_tuple(q):
    if isinstance(q, (tuple, list)):
        return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    return (q.x, q.y, q.z, q.w)


def quaternion_normalize(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quaternion_inverse(q):
    x, y, z, w = quaternion_normalize(q)
    return (-x, -y, -z, w)


def quaternion_multiply_raw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_multiply(a, b):
    return quaternion_normalize(quaternion_multiply_raw(a, b))


def rotate_vector(q, v):
    vq = (v[0], v[1], v[2], 0.0)
    rotated = quaternion_multiply_raw(
        quaternion_multiply_raw(quaternion_normalize(q), vq),
        quaternion_inverse(q),
    )
    return (rotated[0], rotated[1], rotated[2])


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


def euler_from_quaternion(q):
    x, y, z, w = quaternion_normalize(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = yaw_from_quaternion((x, y, z, w))
    return roll, pitch, yaw


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def diagonal_covariance(values):
    matrix = [0.0] * 36
    for index, value in enumerate(values):
        matrix[index * 6 + index] = value
    return matrix


DEFAULT_STEERING_JOINTS = [
    "left_front_steering_joint",
    "right_front_steering_joint",
    "left_rear_steering_joint",
    "right_rear_steering_joint",
]

DEFAULT_WHEEL_JOINTS = [
    "left_front_wheel_joint",
    "right_front_wheel_joint",
    "left_middle_wheel_joint",
    "right_middle_wheel_joint",
    "left_rear_wheel_joint",
    "right_rear_wheel_joint",
]


def wheel_locations(wheelbase, track_width):
    front_x = 0.5 * wheelbase
    rear_x = -front_x
    left_y = 0.5 * track_width
    right_y = -left_y
    return [
        (front_x, left_y, 0),
        (front_x, right_y, 1),
        (0.0, left_y, None),
        (0.0, right_y, None),
        (rear_x, left_y, 2),
        (rear_x, right_y, 3),
    ]


def solve_planar_twist(equations):
    ata00 = ata01 = ata11 = 0.0
    atb0 = atb1 = 0.0
    for a0, a1, b in equations:
        ata00 += a0 * a0
        ata01 += a0 * a1
        ata11 += a1 * a1
        atb0 += a0 * b
        atb1 += a1 * b

    determinant = ata00 * ata11 - ata01 * ata01
    if abs(determinant) <= 1.0e-12 or not math.isfinite(determinant):
        return None

    vx = (atb0 * ata11 - atb1 * ata01) / determinant
    wz = (ata00 * atb1 - ata01 * atb0) / determinant
    if not math.isfinite(vx) or not math.isfinite(wz):
        return None
    return vx, wz


def residuals_for_solution(equations, vx, wz):
    return [abs(b - a0 * vx - a1 * wz) for a0, a1, b in equations]


class CompositeWheelImuOdometry(Node):
    def __init__(self):
        super().__init__("composite_wheel_imu_odometry")

        self.joint_topic = self.declare_parameter("joint_topic", "/joint_states").value
        self.steering_joints = list(
            self.declare_parameter("steering_joints", DEFAULT_STEERING_JOINTS).value
        )
        self.wheel_joints = list(
            self.declare_parameter("wheel_joints", DEFAULT_WHEEL_JOINTS).value
        )
        self.imu_topic = self.declare_parameter("imu_topic", "/livox/imu").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/localization/composite_odometry"
        ).value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.imu_frame = self.declare_parameter("imu_frame", "livox_imu_frame").value
        self.child_frame = self.declare_parameter("child_frame", "base_link").value
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self.calibration_samples = int(
            self.declare_parameter("calibration_samples", 80).value
        )
        self.wheel_radius = float(self.declare_parameter("wheel_radius", 0.0625).value)
        self.wheelbase = float(self.declare_parameter("wheelbase", 0.39).value)
        self.track_width = float(self.declare_parameter("track_width", 0.345).value)
        self.forward_velocity_scale = float(
            self.declare_parameter("forward_velocity_scale", 1.0).value
        )
        self.max_wheel_angular_velocity = float(
            self.declare_parameter("max_wheel_angular_velocity", 9.0).value
        )
        self.wheel_outlier_residual = float(
            self.declare_parameter("wheel_outlier_residual_mps", 0.08).value
        )
        self.min_wheels_for_fit = int(self.declare_parameter("min_wheels_for_fit", 4).value)
        self.stationary_wheel_speed_threshold = float(
            self.declare_parameter("stationary_wheel_speed_threshold_mps", 0.012).value
        )
        self.stationary_yaw_rate_threshold = float(
            self.declare_parameter("stationary_yaw_rate_threshold_radps", 0.012).value
        )
        self.wheel_velocity_timeout = float(
            self.declare_parameter("wheel_velocity_timeout", 0.35).value
        )
        self.max_dt = float(self.declare_parameter("max_dt", 0.10).value)
        self.max_forward_speed = float(
            self.declare_parameter("max_forward_speed_mps", 0.8).value
        )
        self.max_forward_accel = float(
            self.declare_parameter("max_forward_accel_mps2", 0.8).value
        )
        self.pose_covariance = diagonal_covariance(
            self.declare_parameter(
                "pose_covariance_diagonal",
                [0.04, 0.04, 1.0e6, 1.0e6, 1.0e6, 0.02],
            ).value
        )
        self.twist_covariance = diagonal_covariance(
            self.declare_parameter(
                "twist_covariance_diagonal",
                [0.04, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 0.02],
            ).value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.base_from_imu_quat = None
        self.transform_source_frame = None
        self.last_tf_warning = 0.0

        self.latest_joint_speed = 0.0
        self.latest_joint_yaw_rate = 0.0
        self.latest_joint_stamp = None
        self.latest_joint_fit_rms = 0.0
        self.latest_joint_fit_used_wheels = 0
        self.filtered_speed = 0.0

        self.samples = 0
        self.initial_yaw_sum_x = 0.0
        self.initial_yaw_sum_y = 0.0
        self.initial_yaw = None
        self.gyro_z_bias = 0.0
        self.orientation_available = False

        self.last_imu_stamp = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0

        self.publisher = self.create_publisher(Odometry, self.output_topic, 30)
        self.create_subscription(JointState, self.joint_topic, self.on_joint_state, 50)
        self.create_subscription(Imu, self.imu_topic, self.on_imu, 100)

        self.get_logger().info(
            f"Composite wheel+IMU odometry: wheel speed from {self.joint_topic}, "
            f"heading from {self.imu_topic}, output {self.output_topic}"
        )

    def validate_geometry(self):
        return (
            self.wheel_radius > 0.0
            and self.wheelbase > 0.0
            and self.track_width > 0.0
            and len(self.steering_joints) == 4
            and len(self.wheel_joints) == 6
        )

    def on_joint_state(self, msg):
        if not self.validate_geometry():
            self.get_logger().error(
                "Invalid composite odometry geometry or joint list sizes"
            )
            return

        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        try:
            steering = [
                float(positions[name])
                for name in self.steering_joints
            ]
            wheel_velocity = [
                float(velocities[name])
                for name in self.wheel_joints
            ]
        except (KeyError, ValueError):
            return

        fit = self.fit_forward_speed(steering, wheel_velocity)
        if fit is None:
            return

        speed, wheel_yaw_rate, rms, used_wheels = fit
        speed *= self.forward_velocity_scale
        if abs(speed) < self.stationary_wheel_speed_threshold:
            speed = 0.0
        if self.max_forward_speed > 0.0:
            speed = max(-self.max_forward_speed, min(self.max_forward_speed, speed))

        self.latest_joint_speed = speed
        self.latest_joint_yaw_rate = wheel_yaw_rate
        self.latest_joint_fit_rms = rms
        self.latest_joint_fit_used_wheels = used_wheels
        self.latest_joint_stamp = stamp_to_seconds(msg.header.stamp)

    def fit_forward_speed(self, steering, wheel_velocity):
        equations = []
        for index, (x, y, steering_index) in enumerate(
            wheel_locations(self.wheelbase, self.track_width)
        ):
            velocity = wheel_velocity[index]
            if not math.isfinite(velocity):
                continue
            if self.max_wheel_angular_velocity > 0.0:
                velocity = max(
                    -self.max_wheel_angular_velocity,
                    min(self.max_wheel_angular_velocity, velocity),
                )

            angle = steering[steering_index] if steering_index is not None else 0.0
            if not math.isfinite(angle):
                continue

            a0 = math.cos(angle)
            a1 = -y * math.cos(angle) + x * math.sin(angle)
            b = self.wheel_radius * velocity
            equations.append((a0, a1, b))

        if len(equations) < self.min_wheels_for_fit:
            return None

        solution = solve_planar_twist(equations)
        if solution is None:
            return None
        vx, wz = solution
        residuals = residuals_for_solution(equations, vx, wz)

        if self.wheel_outlier_residual > 0.0 and len(equations) > self.min_wheels_for_fit:
            filtered = [
                equation
                for equation, residual in zip(equations, residuals)
                if residual <= self.wheel_outlier_residual
            ]
            if len(filtered) >= self.min_wheels_for_fit and len(filtered) < len(equations):
                refined = solve_planar_twist(filtered)
                if refined is not None:
                    equations = filtered
                    vx, wz = refined
                    residuals = residuals_for_solution(equations, vx, wz)

        rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
        return vx, wz, rms, len(equations)

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
        self.reset_state()
        self.get_logger().info(
            f"Using IMU extrinsic TF {self.base_frame} <- {source_frame}: "
            f"rpy=({math.degrees(roll):.2f}, {math.degrees(pitch):.2f}, "
            f"{math.degrees(yaw):.2f}) deg"
        )
        return True

    def reset_state(self):
        self.samples = 0
        self.initial_yaw_sum_x = 0.0
        self.initial_yaw_sum_y = 0.0
        self.initial_yaw = None
        self.gyro_z_bias = 0.0
        self.orientation_available = False
        self.last_imu_stamp = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.filtered_speed = 0.0

    def corrected_imu(self, msg):
        if not self.ensure_static_transform(msg.header.frame_id):
            return None

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
            # Gazebo/ROS IMU orientation is interpreted as world<-imu.
            # With fixed base<-imu from TF: world<-base = world<-imu * imu<-base.
            base_orientation = quaternion_multiply(
                imu_orientation, quaternion_inverse(self.base_from_imu_quat)
            )

        return gyro_base, base_orientation

    def update_calibration(self, corrected):
        gyro_base, base_orientation = corrected
        self.samples += 1
        n = float(self.samples)
        self.gyro_z_bias += (gyro_base[2] - self.gyro_z_bias) / n

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
                "Composite odometry calibration complete: "
                f"gyro_z_bias={self.gyro_z_bias:.6f} rad/s, "
                f"heading={'IMU orientation' if self.orientation_available else 'gyro integration'}"
            )

    def current_wheel_speed(self, now_stamp):
        if self.latest_joint_stamp is None:
            return 0.0
        if now_stamp - self.latest_joint_stamp > self.wheel_velocity_timeout:
            return 0.0
        return self.latest_joint_speed

    def rate_limited_speed(self, target_speed, dt):
        if self.max_forward_accel <= 0.0 or dt <= 0.0:
            self.filtered_speed = target_speed
            return self.filtered_speed
        max_step = self.max_forward_accel * dt
        delta = target_speed - self.filtered_speed
        self.filtered_speed += max(-max_step, min(max_step, delta))
        return self.filtered_speed

    def heading_from_imu(self, corrected, dt):
        gyro_base, base_orientation = corrected
        self.yaw_rate = gyro_base[2] - self.gyro_z_bias
        if self.orientation_available and base_orientation is not None:
            return normalize_angle(yaw_from_quaternion(base_orientation) - self.initial_yaw)
        return normalize_angle(self.yaw + self.yaw_rate * dt)

    def on_imu(self, msg):
        corrected = self.corrected_imu(msg)
        if corrected is None:
            self.publish(msg.header.stamp)
            return

        stamp = stamp_to_seconds(msg.header.stamp)
        if self.samples < self.calibration_samples:
            self.update_calibration(corrected)
            self.last_imu_stamp = stamp
            self.publish(msg.header.stamp)
            return

        if self.last_imu_stamp is None:
            self.last_imu_stamp = stamp
            self.publish(msg.header.stamp)
            return

        dt = stamp - self.last_imu_stamp
        self.last_imu_stamp = stamp
        if dt <= 0.0:
            self.publish(msg.header.stamp)
            return
        if dt > self.max_dt:
            self.get_logger().warn(f"Skipping composite odometry gap dt={dt:.3f}s")
            self.publish(msg.header.stamp)
            return

        previous_yaw = self.yaw
        self.yaw = self.heading_from_imu(corrected, dt)
        target_speed = self.current_wheel_speed(stamp)
        if (
            abs(target_speed) < self.stationary_wheel_speed_threshold
            and abs(self.yaw_rate) < self.stationary_yaw_rate_threshold
        ):
            target_speed = 0.0
            self.filtered_speed = 0.0
        speed = self.rate_limited_speed(target_speed, dt)
        midpoint_yaw = previous_yaw + 0.5 * normalize_angle(self.yaw - previous_yaw)
        self.x += speed * math.cos(midpoint_yaw) * dt
        self.y += speed * math.sin(midpoint_yaw) * dt
        self.publish(msg.header.stamp)

    def publish(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.child_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = quaternion_from_yaw(self.yaw)
        msg.pose.covariance = self.pose_covariance
        msg.twist.twist.linear.x = self.filtered_speed
        msg.twist.twist.angular.z = self.yaw_rate
        msg.twist.covariance = self.twist_covariance
        self.publisher.publish(msg)
        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.child_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = CompositeWheelImuOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
