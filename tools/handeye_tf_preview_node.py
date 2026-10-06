#!/usr/bin/env python3
"""Publish the selected versioned hand-eye calibration and real-arm FK.

This utility publishes:

  base_link -> arm_base_link
  arm_base_link -> handeye_tool_frame
  handeye_tool_frame -> camera_color_optical_frame

The FK model matches the current ROS IK keyboard controller geometry.
Versioned parameters are loaded from config/handeye_calibrations.json.
No generated calibration files are required at runtime.

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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from handeye_calibration_registry import DEFAULT_CONFIG, load_registry, quaternion_to_matrix


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
        config_path = self.declare_parameter("calibration_config", str(DEFAULT_CONFIG)).value
        requested_id = self.declare_parameter("calibration_id", "").value
        self.registry, self.calibration_id, self.profile = load_registry(config_path, requested_id)
        geometry = self.registry["kinematics"]
        self.zero_degrees = geometry["zero_degrees"]
        self.ros_directions = geometry["ros_directions"]
        self.j1_j2_axis_offset = geometry["j1_j2_axis_offset_m"]
        self.upper_arm = geometry["upper_arm_m"]
        self.forearm = geometry["forearm_m"]
        self.tool = geometry["tool_m"]
        base_tf = self.registry["base_T_arm"]
        self.arm_base_xyz = base_tf["translation_xyz_m"]
        self.arm_base_rotation = quaternion_to_matrix(base_tf["quaternion_xyzw"])
        camera_tf = self.profile["tool_T_camera"]
        self.tool_t_camera = camera_tf["translation_xyz_m"]
        self.tool_rotation_camera = quaternion_to_matrix(camera_tf["quaternion_xyzw"])
        board_tf = self.profile["base_T_board_display"]
        self.board_translation = board_tf["translation_xyz_m"]
        self.board_rotation = quaternion_to_matrix(board_tf["quaternion_xyzw"])
        self.current_ros = None

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster = TransformBroadcaster(self)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.calibration_publisher = self.create_publisher(
            String, "/handeye/active_calibration", status_qos
        )
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_timer(0.05, self.publish_dynamic)
        self.publish_static()
        self.calibration_publisher.publish(String(data=json.dumps({
            "calibration_id": self.calibration_id,
            "calibration_config": config_path,
            "tool_T_camera": self.profile["tool_T_camera"],
        })))
        self.get_logger().info(f"hand-eye TF active calibration: {self.calibration_id}")

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
            "base_link", "arm_base_link", self.arm_base_xyz, self.arm_base_rotation, stamp
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
            self.tool_rotation_camera,
            stamp,
        )
        aliases = [
            make_transform(alias["parent"], alias["child"],
                           (0.0, 0.0, 0.0), np.eye(3), stamp)
            for alias in self.registry["camera_aliases"]
        ]
        self.static_broadcaster.sendTransform([base_tf, board_tf, camera_tf, *aliases])

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
