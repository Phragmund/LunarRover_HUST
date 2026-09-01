#!/usr/bin/env python3
"""Publish simple RViz geometry for real-arm grasping checks.

Geometry note (real arm, confirmed 2026-08-30):
  - J1 is a vertical yaw axis.
  - J2/J3/J4 are parallel pitch axes.
  - ``j1_j2_axis_offset`` is the shortest offset between the J1 and J2 axes.
    It belongs to the planar radial model and should not be interpreted as an
    old Rover_0-style vertical pillar height.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, Pose
from lunar_rover_msgs.msg import TargetObservation
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def color(r, g, b, a=1.0):
    msg = ColorRGBA()
    msg.r = float(r)
    msg.g = float(g)
    msg.b = float(b)
    msg.a = float(a)
    return msg


def point(x, y, z):
    msg = Point()
    msg.x = float(x)
    msg.y = float(y)
    msg.z = float(z)
    return msg


class ArmGraspRvizMarkers(Node):
    def __init__(self):
        super().__init__("arm_grasp_rviz_markers")
        self.zero_degrees = [90.0, 180.0, 135.0, 135.0, 0.0]
        self.ros_directions = [1.0, -1.0, 1.0, 1.0, 1.0]
        self.arm_base_xyz = (0.13, 0.0, 0.056)
        self.j1_j2_axis_offset = 0.07545
        self.upper_arm = 0.11391
        self.forearm = 0.11000
        self.tool = 0.15601
        self.workspace_center = self.arm_base_xyz
        self.workspace_radius = 0.45537
        self.latest_joint_positions = None
        self.latest_observation = None

        self.publisher = self.create_publisher(
            MarkerArray, "/visualization/arm_grasp_markers", 10
        )
        self.compat_publisher = self.create_publisher(
            MarkerArray, "/visualization_marker_array", 10
        )
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(
            TargetObservation, "/perception/target_observation", self.on_observation, 10
        )
        self.create_timer(0.1, self.publish_markers)
        self.get_logger().info("publishing /visualization/arm_grasp_markers")

    def on_joint_state(self, message):
        values = {}
        for name, position in zip(message.name, message.position):
            values[name] = float(position)
        names = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
        if all(name in values for name in names):
            self.latest_joint_positions = [values[name] for name in names]

    def on_observation(self, message):
        self.latest_observation = message

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

    def fk_points(self, positions):
        j1, j2, j3, j4, _ = self.ros_to_degrees(positions)
        yaw = math.radians(j1 - 90.0)
        th2 = math.radians(j2)
        th3 = math.radians(j2 + (135.0 - j3))
        th4 = math.radians(j2 + (135.0 - j3) + (135.0 - j4))

        radial_z = [(0.0, 0.0)]
        r = self.j1_j2_axis_offset
        z = 0.0
        radial_z.append((r, z))
        r += self.upper_arm * math.sin(th2)
        z -= self.upper_arm * math.cos(th2)
        radial_z.append((r, z))
        r += self.forearm * math.sin(th3)
        z -= self.forearm * math.cos(th3)
        radial_z.append((r, z))
        r += self.tool * math.sin(th4)
        z -= self.tool * math.cos(th4)
        radial_z.append((r, z))

        ox, oy, oz = self.arm_base_xyz
        points = []
        for radius, height in radial_z:
            points.append(
                point(
                    ox + radius * math.cos(yaw),
                    oy + radius * math.sin(yaw),
                    oz + height,
                )
            )
        return points

    def marker(self, marker_id, marker_type, ns):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def publish_markers(self):
        markers = MarkerArray()

        workspace = self.marker(0, Marker.SPHERE, "workspace")
        workspace.pose.position = point(*self.workspace_center)
        diameter = 2.0 * self.workspace_radius
        workspace.scale.x = diameter
        workspace.scale.y = diameter
        workspace.scale.z = diameter
        workspace.color = color(0.45, 0.9, 1.0, 0.05)
        markers.markers.append(workspace)

        if self.latest_joint_positions is not None:
            arm_points = self.fk_points(self.latest_joint_positions)
            line = self.marker(1, Marker.LINE_STRIP, "arm_links")
            line.scale.x = 0.012
            line.color = color(0.1, 0.55, 1.0, 1.0)
            line.points = arm_points
            markers.markers.append(line)

            labels = ["J1/base", "J2", "J3", "J4", "tool"]
            for idx, p in enumerate(arm_points):
                sphere = self.marker(10 + idx, Marker.SPHERE, "arm_joints")
                sphere.pose.position = p
                sphere.scale.x = 0.03 if idx < 4 else 0.025
                sphere.scale.y = sphere.scale.x
                sphere.scale.z = sphere.scale.x
                sphere.color = color(1.0, 0.85, 0.1, 1.0) if idx < 4 else color(0.1, 1.0, 0.3, 1.0)
                markers.markers.append(sphere)

                text = self.marker(30 + idx, Marker.TEXT_VIEW_FACING, "arm_labels")
                text.pose.position = point(p.x, p.y, p.z + 0.03)
                text.scale.z = 0.03
                text.color = color(1.0, 1.0, 1.0, 1.0)
                text.text = labels[idx]
                markers.markers.append(text)

            camera = self.marker(50, Marker.CUBE, "camera")
            camera.header.frame_id = "camera_color_optical_frame"
            camera.pose.position = point(0, 0, 0)
            camera.scale.x = 0.04
            camera.scale.y = 0.03
            camera.scale.z = 0.03
            camera.color = color(0.9, 0.1, 1.0, 0.8)
            markers.markers.append(camera)

            optical_axis = self.marker(51, Marker.ARROW, "camera")
            optical_axis.header.frame_id = "camera_color_optical_frame"
            optical_axis.points = [point(0, 0, 0), point(0, 0, 0.18)]
            optical_axis.scale.x = 0.015
            optical_axis.scale.y = 0.03
            optical_axis.scale.z = 0.05
            optical_axis.color = color(1.0, 0.1, 1.0, 1.0)
            markers.markers.append(optical_axis)

        if self.latest_observation is not None:
            obs = self.latest_observation
            target = self.marker(100, Marker.CUBE, "target_object")
            target.header.frame_id = obs.header.frame_id or "base_link"
            target.pose = obs.pose
            target.scale.x = max(0.015, float(obs.dimensions.x))
            target.scale.y = max(0.015, float(obs.dimensions.y))
            target.scale.z = max(0.015, float(obs.dimensions.z))
            target.color = color(0.0, 1.0, 0.15, 0.55)
            markers.markers.append(target)

            center = self.marker(101, Marker.SPHERE, "target_object")
            center.header.frame_id = target.header.frame_id
            center.pose = obs.pose
            center.scale.x = 0.03
            center.scale.y = 0.03
            center.scale.z = 0.03
            center.color = color(1.0, 0.0, 0.0, 1.0)
            markers.markers.append(center)

            label = self.marker(102, Marker.TEXT_VIEW_FACING, "target_object")
            label.header.frame_id = target.header.frame_id
            label.pose = Pose()
            label.pose.orientation.w = 1.0
            label.pose.position = point(
                obs.pose.position.x, obs.pose.position.y, obs.pose.position.z + 0.06
            )
            label.scale.z = 0.03
            label.color = color(1.0, 1.0, 1.0, 1.0)
            label.text = f"{obs.class_id} {obs.confidence:.2f}"
            markers.markers.append(label)

        self.publisher.publish(markers)
        self.compat_publisher.publish(markers)


def main():
    rclpy.init()
    node = ArmGraspRvizMarkers()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
