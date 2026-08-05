#!/usr/bin/env python3
"""Publish static SDF world geometry as RViz markers."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def parse_floats(text, expected=None, default=None):
    if text is None:
        return default
    values = [float(value) for value in text.split()]
    if expected is not None and len(values) != expected:
        return default
    return values


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


def q_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def rotate(q, v):
    result = q_multiply(q_multiply(q, (v[0], v[1], v[2], 0.0)), q_conjugate(q))
    return result[:3]


def pose_from_element(element):
    pose = element.find("pose")
    values = parse_floats(pose.text if pose is not None else None, default=[])
    values = (values + [0.0] * 6)[:6]
    return (values[0], values[1], values[2]), q_from_rpy(values[3], values[4], values[5])


def compose_pose(parent, child):
    parent_t, parent_q = parent
    child_t, child_q = child
    rotated_child = rotate(parent_q, child_t)
    return (
        (
            parent_t[0] + rotated_child[0],
            parent_t[1] + rotated_child[1],
            parent_t[2] + rotated_child[2],
        ),
        q_multiply(parent_q, child_q),
    )


def inverse_pose(pose):
    t, q = pose
    inv_q = q_conjugate(q)
    inv_t = rotate(inv_q, (-t[0], -t[1], -t[2]))
    return inv_t, inv_q


class WorldModelMarkerNode(Node):
    def __init__(self):
        super().__init__("world_model_marker")
        self.world_name = self.declare_parameter("world_name", "reachable_maze").value
        self.world_sdf = self.declare_parameter("world_sdf", "").value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.marker_topic = self.declare_parameter(
            "marker_topic", "/simulation/world_model_markers"
        ).value
        self.alpha = float(self.declare_parameter("alpha", 0.30).value)
        self.outline_alpha = float(self.declare_parameter("outline_alpha", 0.30).value)
        self.outline_width = float(self.declare_parameter("outline_width", 0.025).value)
        self.publish_period = float(self.declare_parameter("publish_period", 1.0).value)
        self.include_collision = bool(self.declare_parameter("include_collision", False).value)
        self.map_origin_world_x = float(
            self.declare_parameter("map_origin_world_x", 0.0).value
        )
        self.map_origin_world_y = float(
            self.declare_parameter("map_origin_world_y", 0.0).value
        )
        self.map_origin_world_z = float(
            self.declare_parameter("map_origin_world_z", 0.0).value
        )
        self.map_origin_world_yaw = float(
            self.declare_parameter("map_origin_world_yaw", 0.0).value
        )
        self.map_from_world = inverse_pose(
            (
                (
                    self.map_origin_world_x,
                    self.map_origin_world_y,
                    self.map_origin_world_z,
                ),
                q_from_rpy(0.0, 0.0, self.map_origin_world_yaw),
            )
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(MarkerArray, self.marker_topic, qos)
        self.markers = self.load_markers()
        self.create_timer(max(0.2, self.publish_period), self.publish)
        self.publish()
        self.get_logger().info(
            f"World model markers: {len(self.markers.markers)} markers -> {self.marker_topic}"
        )
        self.get_logger().info(
            "World model transform: Gazebo world origin shifted into "
            f"{self.frame_id} using robot start "
            f"({self.map_origin_world_x:.3f}, {self.map_origin_world_y:.3f}, "
            f"{self.map_origin_world_z:.3f}, yaw={self.map_origin_world_yaw:.3f})"
        )

    def world_path(self):
        if self.world_sdf:
            return Path(self.world_sdf)
        share = Path(get_package_share_directory("lunar_rover_simulation"))
        return share / "worlds" / f"{self.world_name}.sdf"

    def load_markers(self):
        path = self.world_path()
        root = ET.parse(path).getroot()
        markers = MarkerArray()
        marker_id = 0
        for model in root.findall("./world/model"):
            static_text = model.findtext("static", default="true").strip().lower()
            if static_text in ("false", "0"):
                continue
            model_pose = pose_from_element(model)
            for link in model.findall("link"):
                link_pose = compose_pose(model_pose, pose_from_element(link))
                tags = ["visual"]
                if self.include_collision:
                    tags.append("collision")
                for tag in tags:
                    for visual in link.findall(tag):
                        geometry = visual.find("geometry")
                        if geometry is None:
                            continue
                        world_pose = compose_pose(link_pose, pose_from_element(visual))
                        pose = compose_pose(self.map_from_world, world_pose)
                        marker = self.make_shape_marker(marker_id, model, visual, geometry, pose)
                        if marker is None:
                            continue
                        markers.markers.append(marker)
                        marker_id += 1
                        if marker.type == Marker.CUBE:
                            markers.markers.append(self.make_box_outline(marker_id, marker))
                            marker_id += 1
        if not markers.markers:
            self.get_logger().warn(f"No supported static visual geometry found in {path}")
        return markers

    def make_shape_marker(self, marker_id, model, visual, geometry, pose):
        box = geometry.find("box")
        sphere = geometry.find("sphere")
        cylinder = geometry.find("cylinder")
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "world_model"
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.position.x = pose[0][0]
        marker.pose.position.y = pose[0][1]
        marker.pose.position.z = pose[0][2]
        marker.pose.orientation.x = pose[1][0]
        marker.pose.orientation.y = pose[1][1]
        marker.pose.orientation.z = pose[1][2]
        marker.pose.orientation.w = pose[1][3]
        marker.color.r = 1.0
        marker.color.g = 0.62
        marker.color.b = 0.12
        marker.color.a = self.alpha
        marker.text = f"{model.get('name', 'model')}/{visual.get('name', 'visual')}"

        if box is not None:
            size = parse_floats(box.findtext("size"), expected=3)
            if size is None:
                return None
            marker.type = Marker.CUBE
            marker.scale.x, marker.scale.y, marker.scale.z = size
            return marker
        if sphere is not None:
            radius = float(sphere.findtext("radius", default="0"))
            if radius <= 0.0:
                return None
            marker.type = Marker.SPHERE
            marker.scale.x = marker.scale.y = marker.scale.z = radius * 2.0
            return marker
        if cylinder is not None:
            radius = float(cylinder.findtext("radius", default="0"))
            length = float(cylinder.findtext("length", default="0"))
            if radius <= 0.0 or length <= 0.0:
                return None
            marker.type = Marker.CYLINDER
            marker.scale.x = marker.scale.y = radius * 2.0
            marker.scale.z = length
            return marker
        return None

    def make_box_outline(self, marker_id, box_marker):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = "world_model_outline"
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.type = Marker.LINE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.outline_width
        marker.color.r = 1.0
        marker.color.g = 0.82
        marker.color.b = 0.20
        marker.color.a = self.outline_alpha

        q = (
            box_marker.pose.orientation.x,
            box_marker.pose.orientation.y,
            box_marker.pose.orientation.z,
            box_marker.pose.orientation.w,
        )
        center = (
            box_marker.pose.position.x,
            box_marker.pose.position.y,
            box_marker.pose.position.z,
        )
        sx = box_marker.scale.x * 0.5
        sy = box_marker.scale.y * 0.5
        sz = box_marker.scale.z * 0.5
        local_corners = [
            (-sx, -sy, -sz),
            (sx, -sy, -sz),
            (sx, sy, -sz),
            (-sx, sy, -sz),
            (-sx, -sy, sz),
            (sx, -sy, sz),
            (sx, sy, sz),
            (-sx, sy, sz),
        ]
        corners = []
        for corner in local_corners:
            rotated = rotate(q, corner)
            corners.append(
                Point(
                    x=center[0] + rotated[0],
                    y=center[1] + rotated[1],
                    z=center[2] + rotated[2],
                )
            )
        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )
        for start, end in edges:
            marker.points.append(corners[start])
            marker.points.append(corners[end])
        return marker

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.markers)


def main():
    rclpy.init()
    node = WorldModelMarkerNode()
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
