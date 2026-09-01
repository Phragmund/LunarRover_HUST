#!/usr/bin/env python3
"""Preview the initial hand-eye calibration result in RViz.

This temporary utility publishes:

  base_link -> arm_base_link
  arm_base_link -> handeye_tool_frame
  handeye_tool_frame -> camera_color_optical_frame

The FK model matches the current ROS IK keyboard controller geometry.

Geometry note (real arm, confirmed 2026-08-30):
  - J1 is a vertical yaw axis.
  - J2/J3/J4 are parallel pitch axes.
  - ``j1_j2_axis_offset`` is the shortest offset between the J1 and J2 axes.
    It is part of the planar radial model, not a "vertical riser height".
"""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def rotation_matrix_to_quaternion(matrix):
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2, 1] - matrix[1, 2]) / s
        y = (matrix[0, 2] - matrix[2, 0]) / s
        z = (matrix[1, 0] - matrix[0, 1]) / s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / s
        x = 0.25 * s
        y = (matrix[0, 1] + matrix[1, 0]) / s
        z = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / s
        x = (matrix[0, 1] + matrix[1, 0]) / s
        y = 0.25 * s
        z = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / s
        x = (matrix[0, 2] + matrix[2, 0]) / s
        y = (matrix[1, 2] + matrix[2, 1]) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return x / norm, y / norm, z / norm, w / norm


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def rotation_vector_to_matrix(vector):
    theta = float(np.linalg.norm(vector))
    if theta < 1e-12:
        return np.eye(3)
    axis = np.asarray(vector, dtype=float) / theta
    x, y, z = axis
    c, s = math.cos(theta), math.sin(theta)
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return c * np.eye(3) + (1.0 - c) * np.outer(axis, axis) + s * cross


def make_transform(parent, child, translation, rotation_matrix, stamp):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation_matrix)
    transform.transform.rotation.x = qx
    transform.transform.rotation.y = qy
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw
    return transform


class HandeyeTfPreview(Node):
    def __init__(self):
        super().__init__("handeye_tf_preview")
        self.zero_degrees = [90.0, 180.0, 135.0, 135.0, 0.0]
        self.ros_directions = [1.0, -1.0, 1.0, 1.0, 1.0]
        self.j1_j2_axis_offset = 0.07545
        self.upper_arm = 0.11391
        self.forearm = 0.11000
        self.tool = 0.15601
        self.arm_base_xyz = (0.13, 0.0, 0.056)
        # Updated from the 2026-08-31 ChArUco session.
        self.tool_t_camera = (-0.03559743396701177, -0.04235515197231568, 0.034338323438433536)
        self.tool_rpy_camera = tuple(math.radians(v) for v in (98.60628626259471, 1.728528391568674, 87.84692388366312))
        self.board_translation = (0.568216245161778, 0.17172235141982198, 0.22930751482102782)
        self.board_rotation = self.load_board_rotation()
        self.current_ros = None

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_timer(0.05, self.publish_dynamic)
        self.publish_static()
        self.get_logger().info("hand-eye TF preview is publishing temporary frames for RViz")

    def on_joint_state(self, message):
        values = {}
        for name, position in zip(message.name, message.position):
            values[name] = float(position)
        names = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
        if all(name in values for name in names):
            self.current_ros = [values[name] for name in names]

    def ros_to_degrees(self, positions):
        result = []
        for index, position in enumerate(positions):
            if index == 4:
                result.append(position * 90.0 / 0.045)
            else:
                result.append(
                    self.zero_degrees[index]
                    + self.ros_directions[index] * math.degrees(position)
                )
        return result

    def fk_tool(self, positions):
        j1, j2, j3, j4, _ = self.ros_to_degrees(positions)
        yaw = math.radians(j1 - 90.0)
        th2 = math.radians(j2)
        th3 = math.radians(j2 + (135.0 - j3))
        th4 = math.radians(j2 + (135.0 - j3) + (135.0 - j4))
        radius = (
            self.j1_j2_axis_offset
            + self.upper_arm * math.sin(th2)
            + self.forearm * math.sin(th3)
            + self.tool * math.sin(th4)
        )
        z = (
            -self.upper_arm * math.cos(th2)
            - self.forearm * math.cos(th3)
            - self.tool * math.cos(th4)
        )
        translation = (radius * math.cos(yaw), radius * math.sin(yaw), z)
        x_axis = np.array(
            [math.cos(yaw) * math.sin(th4), math.sin(yaw) * math.sin(th4), -math.cos(th4)]
        )
        y_axis = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        z_axis = np.cross(x_axis, y_axis)
        z_axis = z_axis / np.linalg.norm(z_axis)
        rotation = np.column_stack([x_axis / np.linalg.norm(x_axis), y_axis, z_axis])
        return translation, rotation

    def publish_static(self):
        stamp = self.get_clock().now().to_msg()
        base_tf = make_transform(
            "base_link", "arm_base_link", self.arm_base_xyz, np.eye(3), stamp
        )
        board_tf = make_transform(
            "base_link",
            "charuco_board_frame",
            self.board_translation,
            self.board_rotation,
            stamp,
        )
        camera_tf = make_transform(
            "handeye_tool_frame",
            "camera_color_optical_frame",
            self.tool_t_camera,
            rpy_to_matrix(*self.tool_rpy_camera),
            stamp,
        )
        camera_alias_tf = make_transform(
            "camera_color_optical_frame",
            "gemini335_color_optical_frame",
            (0.0, 0.0, 0.0),
            np.eye(3),
            stamp,
        )
        depth_alias_tf = make_transform(
            "camera_color_optical_frame",
            "camera_depth_optical_frame",
            (0.0, 0.0, 0.0),
            np.eye(3),
            stamp,
        )
        gemini_depth_alias_tf = make_transform(
            "camera_depth_optical_frame",
            "gemini335_depth_optical_frame",
            (0.0, 0.0, 0.0),
            np.eye(3),
            stamp,
        )
        self.static_broadcaster.sendTransform(
            [base_tf, board_tf, camera_tf, camera_alias_tf, depth_alias_tf, gemini_depth_alias_tf]
        )

    def load_board_rotation(self):
        path = "generated/calibration/handeye_samples/20260831_223240/handeye_initial_solution.json"
        try:
            with open(path, "r", encoding="utf-8") as handle:
                report = json.load(handle)
        except OSError:
            return np.eye(3)
        for solution in report.get("solutions", []):
            if solution.get("label") == "filtered_plus_residual_trim":
                matrix = solution.get("tool_T_camera", {}).get("matrix_4x4")
                if matrix:
                    tool_t_camera = np.asarray(matrix, dtype=float)
                    break
        else:
            return np.eye(3)

        rows = []
        sample_path = "generated/calibration/handeye_samples/20260831_223240/samples.jsonl"
        try:
            with open(sample_path, "r", encoding="utf-8") as handle:
                rows = [json.loads(line) for line in handle if line.strip()]
        except OSError:
            return np.eye(3)

        robust_indices = set(
            next(
                solution.get("sample_indices", [])
                for solution in report.get("solutions", [])
                if solution.get("label") == "filtered_plus_residual_trim"
            )
        )
        rotations = []
        for row in rows:
            if int(row.get("index", -1)) not in robust_indices:
                continue
            base_t_tool = self.fk_matrix(row["joint_state"]["positions"])
            camera_t_board = np.eye(4)
            camera_t_board[:3, :3] = rotation_vector_to_matrix(
                row["camera_T_board"]["rotation_vector"]
            )
            camera_t_board[:3, 3] = row["camera_T_board"]["translation_xyz_m"]
            base_t_board = base_t_tool @ tool_t_camera @ camera_t_board
            rotations.append(base_t_board[:3, :3])
        if not rotations:
            return np.eye(3)
        mean_matrix = sum(rotations) / float(len(rotations))
        u, _, vt = np.linalg.svd(mean_matrix)
        rotation = u @ vt
        if np.linalg.det(rotation) < 0:
            u[:, -1] *= -1.0
            rotation = u @ vt
        return rotation

    def fk_matrix(self, positions):
        translation, rotation = self.fk_tool(positions)
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = translation
        return matrix

    def publish_dynamic(self):
        if self.current_ros is None:
            return
        stamp = self.get_clock().now().to_msg()
        translation, rotation = self.fk_tool(self.current_ros)
        self.broadcaster.sendTransform(
            make_transform(
                "arm_base_link", "handeye_tool_frame", translation, rotation, stamp
            )
        )


def main():
    rclpy.init()
    node = HandeyeTfPreview()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
