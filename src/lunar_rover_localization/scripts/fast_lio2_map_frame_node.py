#!/usr/bin/env python3
"""Publish FAST-LIO2 map products in a clean, level navigation map frame."""

import math
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def q_tuple(q):
    return (float(q.x), float(q.y), float(q.z), float(q.w))


def bool_value(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def q_normalize(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def q_inverse(q):
    x, y, z, w = q_normalize(q)
    return (-x, -y, -z, w)


def q_multiply_raw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_multiply(a, b):
    return q_normalize(q_multiply_raw(a, b))


def q_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_q(q):
    x, y, z, w = q_normalize(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def rotate(q, v):
    vx, vy, vz = v
    x, y, z, _ = q_multiply_raw(
        q_multiply_raw(q_normalize(q), (vx, vy, vz, 0.0)), q_inverse(q)
    )
    return (x, y, z)


def transform_parts(transform):
    t = transform.transform.translation
    return (float(t.x), float(t.y), float(t.z)), q_tuple(transform.transform.rotation)


def compose(a_t, a_q, b_t, b_q):
    rotated_b_t = rotate(a_q, b_t)
    return (
        (
            a_t[0] + rotated_b_t[0],
            a_t[1] + rotated_b_t[1],
            a_t[2] + rotated_b_t[2],
        ),
        q_multiply(a_q, b_q),
    )


def inverse_transform(t, q):
    q_inv = q_inverse(q)
    return rotate(q_inv, (-t[0], -t[1], -t[2])), q_inv


def point_field_size(datatype):
    return {
        PointField.INT8: 1,
        PointField.UINT8: 1,
        PointField.INT16: 2,
        PointField.UINT16: 2,
        PointField.INT32: 4,
        PointField.UINT32: 4,
        PointField.FLOAT32: 4,
        PointField.FLOAT64: 8,
    }.get(datatype)


class FastLio2MapFrameNode(Node):
    def __init__(self):
        super().__init__("fast_lio2_map_frame")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.lio_body_frame = self.declare_parameter("lio_body_frame", "livox_imu_frame").value
        self.imu_topic = self.declare_parameter("imu_topic", "/livox/imu").value
        self.imu_mount_pitch_source = (
            self.declare_parameter("imu_mount_pitch_source", "urdf")
            .value.strip()
            .lower()
        )
        self.imu_mount_pitch_samples = max(
            1, int(self.declare_parameter("imu_mount_pitch_samples", 80).value)
        )
        self.imu_mount_pitch_max_gyro = float(
            self.declare_parameter("imu_mount_pitch_max_gyro_radps", 0.02).value
        )
        self.imu_mount_pitch_correction = math.radians(
            float(self.declare_parameter("imu_mount_pitch_correction_deg", 0.0).value)
        )
        self.raw_lio_map_frame = self.declare_parameter(
            "raw_lio_map_frame", "camera_init"
        ).value
        self.raw_odom_topic = self.declare_parameter(
            "raw_odom_topic", "/localization/lio_odometry_raw"
        ).value
        self.raw_path_topic = self.declare_parameter(
            "raw_path_topic", "/localization/lio_path_raw"
        ).value
        self.raw_registered_topic = self.declare_parameter(
            "raw_registered_topic", "/localization/cloud_registered_raw"
        ).value
        self.raw_map_cloud_topic = self.declare_parameter(
            "raw_map_cloud_topic", "/localization/lio_map_cloud_raw"
        ).value
        self.odom_topic = self.declare_parameter(
            "odom_topic", "/localization/lio_odometry"
        ).value
        self.path_topic = self.declare_parameter("path_topic", "/localization/lio_path").value
        self.registered_topic = self.declare_parameter(
            "registered_topic", "/localization/cloud_registered"
        ).value
        self.map_cloud_topic = self.declare_parameter(
            "map_cloud_topic", "/localization/lio_map_cloud"
        ).value
        self.restamp_outputs = bool(self.declare_parameter("restamp_outputs", True).value)
        self.max_abs_position = float(
            self.declare_parameter("max_abs_position", 1000.0).value
        )
        self.require_valid_pose_for_clouds = bool(
            self.declare_parameter("require_valid_pose_for_clouds", True).value
        )
        self.level_correction_source = (
            self.declare_parameter("level_correction_source", "none")
            .value.strip()
            .lower()
        )
        self.level_correction_samples = max(
            1, int(self.declare_parameter("level_correction_samples", 40).value)
        )
        self.level_correction_start_delay = max(
            0.0,
            float(self.declare_parameter("level_correction_start_delay_sec", 20.0).value),
        )
        self.level_correction_max_abs_tilt = math.radians(
            float(self.declare_parameter("level_correction_max_abs_tilt_deg", 10.0).value)
        )
        self.level_correction_preserve_yaw = bool_value(
            self.declare_parameter("level_correction_preserve_yaw", True).value
        )
        self.level_correction_anchor_position = bool_value(
            self.declare_parameter("level_correction_anchor_position", True).value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_from_lio = None
        self.lio_body_from_base = None
        self.level_correction = (
            None
            if self.level_correction_source in ("startup_lio_pose", "lio", "startup")
            else ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        )
        self.level_sample_count = 0
        self.level_roll_sin_sum = 0.0
        self.level_roll_cos_sum = 0.0
        self.level_pitch_sin_sum = 0.0
        self.level_pitch_cos_sum = 0.0
        self.level_yaw_sin_sum = 0.0
        self.level_yaw_cos_sum = 0.0
        self.level_position_sum = [0.0, 0.0, 0.0]
        self.level_first_raw_odom_time = None
        self.calibrated_imu_pitch = None
        self.imu_pitch_sample_count = 0
        self.imu_pitch_sum = 0.0
        self.last_pose_valid = False
        self.wait_timer = self.create_timer(0.1, self.try_initialize)
        self.cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.registered_pub = self.create_publisher(
            PointCloud2, self.registered_topic, self.cloud_qos
        )
        self.map_cloud_pub = self.create_publisher(
            PointCloud2, self.map_cloud_topic, self.cloud_qos
        )

        self.create_subscription(Odometry, self.raw_odom_topic, self.on_odom, 20)
        if self.imu_mount_pitch_source in ("startup_imu_pitch", "imu", "measured"):
            self.create_subscription(Imu, self.imu_topic, self.on_imu, 50)
        self.create_subscription(Path, self.raw_path_topic, self.on_path, 10)
        self.create_subscription(
            PointCloud2,
            self.raw_registered_topic,
            self.on_registered_cloud,
            self.cloud_qos,
        )
        self.create_subscription(
            PointCloud2,
            self.raw_map_cloud_topic,
            self.on_map_cloud,
            self.cloud_qos,
        )

        self.get_logger().info(
            "FAST-LIO2 map frame: raw camera_init products -> level map products"
        )

    def on_imu(self, msg):
        if self.calibrated_imu_pitch is not None:
            return
        gyro_norm = math.sqrt(
            msg.angular_velocity.x * msg.angular_velocity.x
            + msg.angular_velocity.y * msg.angular_velocity.y
            + msg.angular_velocity.z * msg.angular_velocity.z
        )
        if self.imu_mount_pitch_max_gyro > 0.0 and gyro_norm > self.imu_mount_pitch_max_gyro:
            return
        _, pitch, _ = rpy_from_q(q_tuple(msg.orientation))
        if not math.isfinite(pitch):
            return
        self.imu_pitch_sample_count += 1
        self.imu_pitch_sum += pitch
        if self.imu_pitch_sample_count >= self.imu_mount_pitch_samples:
            self.calibrated_imu_pitch = self.imu_pitch_sum / float(self.imu_pitch_sample_count)
            self.get_logger().info(
                "Calibrated effective IMU mount pitch from startup IMU orientation: "
                f"{math.degrees(self.calibrated_imu_pitch):.3f} deg "
                f"from {self.imu_pitch_sample_count} samples"
            )

    def try_initialize(self):
        if self.map_from_lio is not None:
            return
        if (
            self.imu_mount_pitch_source in ("startup_imu_pitch", "imu", "measured")
            and self.calibrated_imu_pitch is None
        ):
            self.get_logger().warn(
                "Waiting for startup IMU pitch calibration from "
                f"{self.imu_topic}: {self.imu_pitch_sample_count}/"
                f"{self.imu_mount_pitch_samples} samples",
                throttle_duration_sec=2.0,
            )
            return
        try:
            base_from_lio_body = self.tf_buffer.lookup_transform(
                self.base_frame, self.lio_body_frame, Time()
            )
            lio_body_from_base = self.tf_buffer.lookup_transform(
                self.lio_body_frame, self.base_frame, Time()
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for static {self.base_frame}<->{self.lio_body_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        base_from_lio_body_t, base_from_lio_body_q = transform_parts(base_from_lio_body)
        pitch_source = "urdf"
        if self.imu_mount_pitch_source in ("startup_imu_pitch", "imu", "measured"):
            urdf_roll, _, urdf_yaw = rpy_from_q(base_from_lio_body_q)
            base_from_lio_body_q = q_normalize(
                q_from_rpy(urdf_roll, self.calibrated_imu_pitch, urdf_yaw)
            )
            pitch_source = "startup_imu_pitch"
        elif abs(self.imu_mount_pitch_correction) > 0.0:
            base_from_lio_body_q = q_normalize(
                q_multiply(
                    base_from_lio_body_q,
                    q_from_rpy(0.0, -self.imu_mount_pitch_correction, 0.0),
                )
            )
            pitch_source = "manual_correction"
        self.map_from_lio = (base_from_lio_body_t, base_from_lio_body_q)
        self.lio_body_from_base = inverse_transform(*self.map_from_lio)
        _, q = self.map_from_lio
        _, pitch, _ = rpy_from_q(q)
        self.get_logger().info(
            f"Using URDF-derived {self.base_frame}->{self.lio_body_frame} as "
            f"{self.map_frame}->{self.raw_lio_map_frame}; "
            f"pitch={math.degrees(pitch):.3f} deg; pitch_source={pitch_source}; "
            f"pitch_correction={math.degrees(self.imu_mount_pitch_correction):.3f} deg"
        )
        self.wait_timer.cancel()

    def initialized(self):
        if self.map_from_lio is None or self.lio_body_from_base is None:
            self.get_logger().warn("FAST-LIO2 map frame is not initialized", throttle_duration_sec=2.0)
            return False
        return True

    def level_initialized(self):
        if self.level_correction is not None:
            return True
        self.get_logger().warn(
            "Waiting for startup FAST-LIO pose level calibration: "
            f"{self.level_sample_count}/{self.level_correction_samples} samples",
            throttle_duration_sec=2.0,
        )
        return False

    def raw_pose_to_map_base_uncorrected(self, position, orientation):
        raw_t = (float(position.x), float(position.y), float(position.z))
        raw_q = q_tuple(orientation)
        map_body = compose(*self.map_from_lio, raw_t, raw_q)
        return compose(*map_body, *self.lio_body_from_base)

    def raw_pose_to_map_base(self, position, orientation):
        t, q = self.raw_pose_to_map_base_uncorrected(position, orientation)
        return self.apply_level_correction(t, q)

    def apply_level_correction(self, t, q):
        if self.level_correction is None:
            return t, q
        return compose(*self.level_correction, t, q)

    def average_angle(self, sin_sum, cos_sum):
        return math.atan2(sin_sum, cos_sum)

    def record_startup_level_sample(self, t, q):
        if self.level_correction is not None:
            return True
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        if self.level_first_raw_odom_time is None:
            self.level_first_raw_odom_time = now_sec
        elapsed = now_sec - self.level_first_raw_odom_time
        if elapsed < self.level_correction_start_delay:
            self.get_logger().warn(
                "Waiting before startup FAST-LIO pose level calibration: "
                f"{elapsed:.1f}/{self.level_correction_start_delay:.1f}s",
                throttle_duration_sec=2.0,
            )
            return False
        if not self.pose_is_valid(t, q):
            self.get_logger().warn(
                "Skipping invalid startup FAST-LIO pose while calibrating level correction",
                throttle_duration_sec=2.0,
            )
            return False

        roll, pitch, yaw = rpy_from_q(q)
        self.level_roll_sin_sum += math.sin(roll)
        self.level_roll_cos_sum += math.cos(roll)
        self.level_pitch_sin_sum += math.sin(pitch)
        self.level_pitch_cos_sum += math.cos(pitch)
        self.level_yaw_sin_sum += math.sin(yaw)
        self.level_yaw_cos_sum += math.cos(yaw)
        self.level_position_sum[0] += t[0]
        self.level_position_sum[1] += t[1]
        self.level_position_sum[2] += t[2]
        self.level_sample_count += 1

        if self.level_sample_count < self.level_correction_samples:
            self.get_logger().warn(
                "Collecting startup FAST-LIO pose samples for level correction: "
                f"{self.level_sample_count}/{self.level_correction_samples}",
                throttle_duration_sec=2.0,
            )
            return False

        avg_roll = self.average_angle(self.level_roll_sin_sum, self.level_roll_cos_sum)
        avg_pitch = self.average_angle(self.level_pitch_sin_sum, self.level_pitch_cos_sum)
        avg_yaw = self.average_angle(self.level_yaw_sin_sum, self.level_yaw_cos_sum)
        avg_t = (
            self.level_position_sum[0] / float(self.level_sample_count),
            self.level_position_sum[1] / float(self.level_sample_count),
            self.level_position_sum[2] / float(self.level_sample_count),
        )
        tilt = max(abs(avg_roll), abs(avg_pitch))
        if self.level_correction_max_abs_tilt > 0.0 and tilt > self.level_correction_max_abs_tilt:
            self.level_correction = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
            self.get_logger().error(
                "Startup FAST-LIO level correction rejected because tilt is too large: "
                f"roll={math.degrees(avg_roll):.3f} deg, "
                f"pitch={math.degrees(avg_pitch):.3f} deg, "
                f"limit={math.degrees(self.level_correction_max_abs_tilt):.3f} deg"
            )
            return True

        measured_q = q_from_rpy(avg_roll, avg_pitch, avg_yaw)
        desired_yaw = avg_yaw if self.level_correction_preserve_yaw else 0.0
        desired_q = q_from_rpy(0.0, 0.0, desired_yaw)
        correction_q = q_multiply(desired_q, q_inverse(measured_q))
        if self.level_correction_anchor_position:
            rotated_anchor = rotate(correction_q, avg_t)
            correction_t = (
                avg_t[0] - rotated_anchor[0],
                avg_t[1] - rotated_anchor[1],
                avg_t[2] - rotated_anchor[2],
            )
        else:
            correction_t = (0.0, 0.0, 0.0)
        self.level_correction = (correction_t, correction_q)
        corr_roll, corr_pitch, corr_yaw = rpy_from_q(correction_q)
        self.get_logger().info(
            "Startup FAST-LIO level correction calibrated: "
            f"measured rpy=({math.degrees(avg_roll):+.3f}, "
            f"{math.degrees(avg_pitch):+.3f}, {math.degrees(avg_yaw):+.3f}) deg; "
            f"correction rpy=({math.degrees(corr_roll):+.3f}, "
            f"{math.degrees(corr_pitch):+.3f}, {math.degrees(corr_yaw):+.3f}) deg; "
            f"samples={self.level_sample_count}; "
            f"preserve_yaw={self.level_correction_preserve_yaw}"
        )
        return True

    def pose_is_valid(self, t, q):
        values = (*t, *q)
        if not all(math.isfinite(value) for value in values):
            return False
        if self.max_abs_position > 0.0 and max(abs(t[0]), abs(t[1]), abs(t[2])) > self.max_abs_position:
            return False
        q_norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        return math.isfinite(q_norm) and 0.5 <= q_norm <= 1.5

    def fill_pose(self, pose, t, q):
        pose.position.x = t[0]
        pose.position.y = t[1]
        pose.position.z = t[2]
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

    def on_odom(self, msg):
        if not self.initialized():
            return
        out = deepcopy(msg)
        out.header.frame_id = self.map_frame
        if self.restamp_outputs:
            out.header.stamp = self.get_clock().now().to_msg()
        out.child_frame_id = self.base_frame
        uncorrected_t, uncorrected_q = self.raw_pose_to_map_base_uncorrected(
            msg.pose.pose.position, msg.pose.pose.orientation
        )
        if not self.record_startup_level_sample(uncorrected_t, uncorrected_q):
            return
        t, q = self.apply_level_correction(uncorrected_t, uncorrected_q)
        if not self.pose_is_valid(t, q):
            self.last_pose_valid = False
            self.get_logger().warn(
                "Dropping invalid FAST-LIO2 pose before publishing map-frame odometry: "
                f"position=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f})",
                throttle_duration_sec=2.0,
            )
            return
        self.last_pose_valid = True
        self.fill_pose(out.pose.pose, t, q)
        self.odom_pub.publish(out)

    def on_path(self, msg):
        if not self.initialized() or not self.level_initialized():
            return
        out = Path()
        out.header = msg.header
        out.header.frame_id = self.map_frame
        if self.restamp_outputs:
            out.header.stamp = self.get_clock().now().to_msg()
        for pose_stamped in msg.poses:
            pose_out = PoseStamped()
            pose_out.header = pose_stamped.header
            pose_out.header.frame_id = self.map_frame
            if self.restamp_outputs:
                pose_out.header.stamp = out.header.stamp
            t, q = self.raw_pose_to_map_base(
                pose_stamped.pose.position, pose_stamped.pose.orientation
            )
            self.fill_pose(pose_out.pose, t, q)
            out.poses.append(pose_out)
        self.path_pub.publish(out)

    def transform_cloud(self, msg):
        if not self.initialized() or not self.level_initialized():
            return None
        if self.require_valid_pose_for_clouds and not self.last_pose_valid:
            self.get_logger().warn(
                "Dropping FAST-LIO2 cloud because the latest LIO pose is invalid",
                throttle_duration_sec=2.0,
            )
            return None
        selected_fields = []
        compact_fields = []
        offset = 0
        for field in sorted(msg.fields, key=lambda candidate: candidate.offset):
            size = point_field_size(field.datatype)
            if size is None:
                continue
            compact = PointField()
            compact.name = field.name
            compact.offset = offset
            compact.datatype = field.datatype
            compact.count = field.count
            selected_fields.append(field)
            compact_fields.append(compact)
            offset += size * field.count

        names = [field.name for field in selected_fields]
        try:
            x_index = names.index("x")
            y_index = names.index("y")
            z_index = names.index("z")
        except ValueError:
            self.get_logger().warn("PointCloud2 has no x/y/z fields", throttle_duration_sec=2.0)
            return None

        map_t, map_q = compose(*self.level_correction, *self.map_from_lio)
        points = []
        for point in point_cloud2.read_points(msg, field_names=names, skip_nans=True):
            values = list(point)
            x, y, z = rotate(map_q, (values[x_index], values[y_index], values[z_index]))
            values[x_index] = x + map_t[0]
            values[y_index] = y + map_t[1]
            values[z_index] = z + map_t[2]
            points.append(tuple(values))

        header = deepcopy(msg.header)
        header.frame_id = self.map_frame
        if self.restamp_outputs:
            header.stamp = self.get_clock().now().to_msg()
        return point_cloud2.create_cloud(header, compact_fields, points)

    def on_registered_cloud(self, msg):
        out = self.transform_cloud(msg)
        if out is not None:
            self.registered_pub.publish(out)

    def on_map_cloud(self, msg):
        out = self.transform_cloud(msg)
        if out is not None:
            self.map_cloud_pub.publish(out)


def main():
    rclpy.init()
    node = FastLio2MapFrameNode()
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
