#!/usr/bin/env python3
"""Fuse registered LiDAR and RGB-D clouds with LiDAR-anchored refinement."""

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener


FLOAT_DTYPES = {
    PointField.FLOAT32: "f4",
    PointField.FLOAT64: "f8",
}


@dataclass
class FusionParameters:
    lidar_voxel_size: float = 0.04
    camera_voxel_size: float = 0.03
    output_voxel_size: float = 0.04
    association_radius: float = 0.15
    consistency_radius: float = 0.08
    conflict_radius: float = 0.22
    min_overlap_matches: int = 30
    max_depth_bias: float = 0.10
    bias_smoothing: float = 0.25
    refinement_gain: float = 0.35
    lidar_weight: float = 4.0
    overlap_camera_weight: float = 1.5
    complementary_camera_weight: float = 1.0
    max_output_points: int = 60000


def voxel_reduce(points, voxel_size, weights=None, values=None):
    """Return weighted centroids and optional weighted scalar values per voxel."""
    if points.size == 0:
        empty_values = None if values is None else np.empty((0,), dtype=np.float64)
        return np.empty((0, 3), dtype=np.float64), empty_values

    points = np.asarray(points, dtype=np.float64)
    if weights is None:
        weights = np.ones(points.shape[0], dtype=np.float64)
    else:
        weights = np.asarray(weights, dtype=np.float64)
    keys = np.floor(points / voxel_size).astype(np.int64)
    _unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    count = int(inverse.max()) + 1
    weight_sum = np.bincount(inverse, weights=weights, minlength=count)
    sums = np.zeros((count, 3), dtype=np.float64)
    for axis in range(3):
        sums[:, axis] = np.bincount(
            inverse,
            weights=points[:, axis] * weights,
            minlength=count,
        )
    centroids = sums / weight_sum[:, None]

    reduced_values = None
    if values is not None:
        values = np.asarray(values, dtype=np.float64)
        reduced_values = np.bincount(
            inverse,
            weights=values * weights,
            minlength=count,
        ) / weight_sum
    return centroids, reduced_values


def robust_depth_bias(radial_residuals, max_depth_bias):
    residuals = np.asarray(radial_residuals, dtype=np.float64)
    residuals = residuals[np.isfinite(residuals)]
    residuals = residuals[np.abs(residuals) <= max_depth_bias]
    if residuals.size == 0:
        return None
    median = float(np.median(residuals))
    mad = float(np.median(np.abs(residuals - median)))
    if mad > 1.0e-6:
        residuals = residuals[np.abs(residuals - median) <= 3.5 * 1.4826 * mad]
    if residuals.size == 0:
        return None
    return float(np.median(residuals))


def fuse_point_sets(lidar_points, camera_points, camera_origin, params, previous_bias=0.0):
    """
    Fuse points already expressed in one frame.

    LiDAR remains the geometric anchor. Camera points near a LiDAR surface are
    corrected only along their viewing ray. Camera points that disagree with a
    nearby LiDAR return are rejected, while points in LiDAR sampling gaps remain.
    """
    lidar_points, _ = voxel_reduce(lidar_points, params.lidar_voxel_size)
    camera_points, _ = voxel_reduce(camera_points, params.camera_voxel_size)
    origin = np.asarray(camera_origin, dtype=np.float64).reshape(3)
    bias = float(previous_bias)

    stats = {
        "lidar": int(lidar_points.shape[0]),
        "camera": int(camera_points.shape[0]),
        "matches": 0,
        "overlap": 0,
        "conflicts": 0,
        "complementary": 0,
        "bias": bias,
    }

    if camera_points.size == 0:
        confidence = np.full(lidar_points.shape[0], 100.0)
        output, output_confidence = voxel_reduce(
            lidar_points,
            params.output_voxel_size,
            weights=np.full(lidar_points.shape[0], params.lidar_weight),
            values=confidence,
        )
        stats["output"] = int(output.shape[0])
        return output, output_confidence, bias, stats

    rays = camera_points - origin
    ranges = np.linalg.norm(rays, axis=1)
    valid_rays = ranges > 1.0e-6
    camera_points = camera_points[valid_rays]
    rays = rays[valid_rays]
    ranges = ranges[valid_rays]
    directions = rays / ranges[:, None]

    if lidar_points.size:
        tree = cKDTree(lidar_points)
        distances, indices = tree.query(camera_points, k=1, workers=-1)
        matched = distances <= params.association_radius
        stats["matches"] = int(np.count_nonzero(matched))
        if stats["matches"] >= params.min_overlap_matches:
            residual = lidar_points[indices[matched]] - camera_points[matched]
            radial = np.einsum("ij,ij->i", residual, directions[matched])
            estimate = robust_depth_bias(radial, params.max_depth_bias)
            if estimate is not None:
                bias += params.bias_smoothing * (estimate - bias)

        bias = float(np.clip(bias, -params.max_depth_bias, params.max_depth_bias))
        camera_points = camera_points + bias * directions
        distances, indices = tree.query(camera_points, k=1, workers=-1)
        overlap = distances <= params.consistency_radius
        conflicts = (distances > params.consistency_radius) & (
            distances <= params.conflict_radius
        )
        complementary = distances > params.conflict_radius

        refined_overlap = camera_points[overlap].copy()
        if refined_overlap.size:
            residual = lidar_points[indices[overlap]] - refined_overlap
            radial = np.einsum("ij,ij->i", residual, directions[overlap])
            refined_overlap += (
                params.refinement_gain * radial[:, None] * directions[overlap]
            )
    else:
        overlap = np.zeros(camera_points.shape[0], dtype=bool)
        conflicts = np.zeros(camera_points.shape[0], dtype=bool)
        complementary = np.ones(camera_points.shape[0], dtype=bool)
        refined_overlap = np.empty((0, 3), dtype=np.float64)

    complementary_points = camera_points[complementary]
    stats.update(
        overlap=int(np.count_nonzero(overlap)),
        conflicts=int(np.count_nonzero(conflicts)),
        complementary=int(np.count_nonzero(complementary)),
        bias=bias,
    )

    pieces = [lidar_points, refined_overlap, complementary_points]
    weights = [
        np.full(lidar_points.shape[0], params.lidar_weight),
        np.full(refined_overlap.shape[0], params.overlap_camera_weight),
        np.full(
            complementary_points.shape[0],
            params.complementary_camera_weight,
        ),
    ]
    # Intensity carries confidence/provenance without changing the XYZ contract.
    confidence = [
        np.full(lidar_points.shape[0], 100.0),
        np.full(refined_overlap.shape[0], 75.0),
        np.full(complementary_points.shape[0], 40.0),
    ]
    merged = np.concatenate(pieces, axis=0)
    merged_weights = np.concatenate(weights)
    merged_confidence = np.concatenate(confidence)
    output, output_confidence = voxel_reduce(
        merged,
        params.output_voxel_size,
        weights=merged_weights,
        values=merged_confidence,
    )
    if params.max_output_points > 0 and output.shape[0] > params.max_output_points:
        selected = np.linspace(
            0,
            output.shape[0] - 1,
            params.max_output_points,
            dtype=np.int64,
        )
        output = output[selected]
        output_confidence = output_confidence[selected]
    stats["output"] = int(output.shape[0])
    return output, output_confidence, bias, stats


def quaternion_matrix(rotation):
    x = float(rotation.x)
    y = float(rotation.y)
    z = float(rotation.z)
    w = float(rotation.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class MultisensorCloudFusionNode(Node):
    def __init__(self):
        super().__init__("multisensor_cloud_fusion")
        self.lidar_topic = self.declare_parameter(
            "lidar_topic", "/localization/cloud_registered"
        ).value
        self.camera_topic = self.declare_parameter(
            "camera_topic", "/camera/depth_registered/points"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/localization/cloud_fused"
        ).value
        self.diagnostics_topic = self.declare_parameter(
            "diagnostics_topic", "/localization/cloud_fusion/diagnostics"
        ).value
        self.target_frame = self.declare_parameter("target_frame", "map").value
        self.camera_min_range = float(
            self.declare_parameter("camera_min_range", 0.26).value
        )
        self.camera_max_range = float(
            self.declare_parameter("camera_max_range", 6.0).value
        )
        self.camera_pixel_stride = max(
            1, int(self.declare_parameter("camera_pixel_stride", 2).value)
        )
        self.max_camera_points = max(
            1, int(self.declare_parameter("max_camera_points", 90000).value)
        )
        self.camera_max_age = max(
            0.0, float(self.declare_parameter("camera_max_age_sec", 0.25).value)
        )
        self.transform_timeout = max(
            0.01, float(self.declare_parameter("transform_timeout_sec", 0.10).value)
        )
        self.params = FusionParameters(
            lidar_voxel_size=float(self.declare_parameter("lidar_voxel_size", 0.04).value),
            camera_voxel_size=float(self.declare_parameter("camera_voxel_size", 0.03).value),
            output_voxel_size=float(self.declare_parameter("output_voxel_size", 0.04).value),
            association_radius=float(self.declare_parameter("association_radius", 0.15).value),
            consistency_radius=float(self.declare_parameter("consistency_radius", 0.08).value),
            conflict_radius=float(self.declare_parameter("conflict_radius", 0.22).value),
            min_overlap_matches=int(self.declare_parameter("min_overlap_matches", 30).value),
            max_depth_bias=float(self.declare_parameter("max_depth_bias", 0.10).value),
            bias_smoothing=float(self.declare_parameter("bias_smoothing", 0.25).value),
            refinement_gain=float(self.declare_parameter("refinement_gain", 0.35).value),
            lidar_weight=float(self.declare_parameter("lidar_weight", 4.0).value),
            overlap_camera_weight=float(
                self.declare_parameter("overlap_camera_weight", 1.5).value
            ),
            complementary_camera_weight=float(
                self.declare_parameter("complementary_camera_weight", 1.0).value
            ),
            max_output_points=int(self.declare_parameter("max_output_points", 60000).value),
        )
        self._validate_parameters()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_camera = None
        self.depth_bias = 0.0
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, output_qos)
        self.diagnostics_publisher = self.create_publisher(
            String, self.diagnostics_topic, 2
        )
        self.create_subscription(PointCloud2, self.camera_topic, self.on_camera, sensor_qos)
        self.create_subscription(PointCloud2, self.lidar_topic, self.on_lidar, sensor_qos)
        self.get_logger().info(
            f"Cloud fusion: {self.lidar_topic} + {self.camera_topic} -> "
            f"{self.output_topic} ({self.target_frame})"
        )

    def _validate_parameters(self):
        positive = {
            "lidar_voxel_size": self.params.lidar_voxel_size,
            "camera_voxel_size": self.params.camera_voxel_size,
            "output_voxel_size": self.params.output_voxel_size,
            "association_radius": self.params.association_radius,
            "consistency_radius": self.params.consistency_radius,
            "conflict_radius": self.params.conflict_radius,
            "max_depth_bias": self.params.max_depth_bias,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not (
            self.params.consistency_radius
            <= self.params.association_radius
            <= self.params.conflict_radius
        ):
            raise ValueError(
                "radii must satisfy consistency_radius <= association_radius <= conflict_radius"
            )
        if not 0.0 <= self.params.bias_smoothing <= 1.0:
            raise ValueError("bias_smoothing must be in [0, 1]")
        if not 0.0 <= self.params.refinement_gain <= 1.0:
            raise ValueError("refinement_gain must be in [0, 1]")

    @staticmethod
    def stamp_seconds(stamp):
        return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)

    def on_camera(self, message):
        self.latest_camera = message

    def cloud_xyz(self, message, organized_stride=1, max_points=0):
        fields = {field.name: field for field in message.fields}
        arrays = []
        byte_order = ">" if message.is_bigendian else "<"
        row_step = message.row_step or message.point_step * message.width
        try:
            for name in ("x", "y", "z"):
                field = fields[name]
                dtype_name = FLOAT_DTYPES[field.datatype]
                array = np.ndarray(
                    shape=(message.height, message.width),
                    dtype=np.dtype(byte_order + dtype_name),
                    buffer=memoryview(message.data),
                    offset=field.offset,
                    strides=(row_step, message.point_step),
                )
                arrays.append(array[::organized_stride, ::organized_stride].reshape(-1))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"PointCloud2 requires float x/y/z fields: {exc}") from exc

        points = np.column_stack(arrays).astype(np.float64, copy=False)
        points = points[np.all(np.isfinite(points), axis=1)]
        if max_points > 0 and points.shape[0] > max_points:
            step = int(math.ceil(points.shape[0] / max_points))
            points = points[::step]
        return points

    def transform_to_target(self, message, points):
        if message.header.frame_id == self.target_frame:
            return points, np.zeros(3, dtype=np.float64)
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            message.header.frame_id,
            Time.from_msg(message.header.stamp),
            timeout=Duration(seconds=self.transform_timeout),
        )
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        rotation = quaternion_matrix(transform.transform.rotation)
        return points @ rotation.T + translation, translation

    def publish_cloud(self, lidar_message, points, confidence):
        header = Header()
        header.stamp = lidar_message.header.stamp
        header.frame_id = self.target_frame
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        rows = np.column_stack((points, confidence)).astype(np.float32, copy=False)
        self.publisher.publish(point_cloud2.create_cloud(header, fields, rows))

    def publish_diagnostics(self, mode, stats, camera_age=None):
        age_text = "n/a" if camera_age is None else f"{camera_age:.3f}"
        self.diagnostics_publisher.publish(
            String(
                data=(
                    f"mode={mode} camera_age={age_text} "
                    f"lidar={stats.get('lidar', 0)} camera={stats.get('camera', 0)} "
                    f"matches={stats.get('matches', 0)} overlap={stats.get('overlap', 0)} "
                    f"conflicts={stats.get('conflicts', 0)} "
                    f"complementary={stats.get('complementary', 0)} "
                    f"bias={stats.get('bias', self.depth_bias):+.4f} "
                    f"output={stats.get('output', 0)}"
                )
            )
        )

    def lidar_only(self, lidar_message, lidar_points, reason, camera_age=None):
        points, confidence, self.depth_bias, stats = fuse_point_sets(
            lidar_points,
            np.empty((0, 3)),
            np.zeros(3),
            self.params,
            self.depth_bias,
        )
        self.publish_cloud(lidar_message, points, confidence)
        self.publish_diagnostics(reason, stats, camera_age)

    def on_lidar(self, lidar_message):
        try:
            lidar_points = self.cloud_xyz(lidar_message)
            lidar_points, _lidar_origin = self.transform_to_target(
                lidar_message, lidar_points
            )
        except (ValueError, TransformException) as exc:
            self.get_logger().warn(
                f"Cannot prepare LiDAR cloud for fusion: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        camera_message = self.latest_camera
        if camera_message is None:
            self.lidar_only(lidar_message, lidar_points, "lidar_only_no_camera")
            return
        camera_age = abs(
            self.stamp_seconds(lidar_message.header.stamp)
            - self.stamp_seconds(camera_message.header.stamp)
        )
        if camera_age > self.camera_max_age:
            self.lidar_only(
                lidar_message,
                lidar_points,
                "lidar_only_stale_camera",
                camera_age,
            )
            return

        try:
            camera_points = self.cloud_xyz(
                camera_message,
                organized_stride=self.camera_pixel_stride,
                max_points=self.max_camera_points,
            )
            camera_ranges = np.linalg.norm(camera_points, axis=1)
            in_range = (camera_ranges >= self.camera_min_range) & (
                camera_ranges <= self.camera_max_range
            )
            camera_points = camera_points[in_range]
            camera_points, camera_origin = self.transform_to_target(
                camera_message, camera_points
            )
        except (ValueError, TransformException) as exc:
            self.get_logger().warn(
                f"Camera cloud unavailable for fusion; using LiDAR only: {exc}",
                throttle_duration_sec=2.0,
            )
            self.lidar_only(
                lidar_message,
                lidar_points,
                "lidar_only_camera_transform",
                camera_age,
            )
            return

        points, confidence, self.depth_bias, stats = fuse_point_sets(
            lidar_points,
            camera_points,
            camera_origin,
            self.params,
            self.depth_bias,
        )
        self.publish_cloud(lidar_message, points, confidence)
        self.publish_diagnostics("fused", stats, camera_age)


def main():
    rclpy.init()
    node = MultisensorCloudFusionNode()
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
