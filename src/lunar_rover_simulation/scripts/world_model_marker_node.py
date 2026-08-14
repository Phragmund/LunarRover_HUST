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
        self.marker_z_offset = float(
            self.declare_parameter("marker_z_offset", 0.0).value
        )
        self.include_visual = bool(self.declare_parameter("include_visual", True).value)
        self.include_collision = bool(self.declare_parameter("include_collision", False).value)
        self.include_pit_markers = bool(
            self.declare_parameter("include_pit_markers", True).value
        )
        self.marker_namespace = self.declare_parameter(
            "marker_namespace", "world_model"
        ).value
        self.outline_namespace = self.declare_parameter(
            "outline_namespace", f"{self.marker_namespace}_outline"
        ).value
        self.color_r = float(self.declare_parameter("color_r", 1.0).value)
        self.color_g = float(self.declare_parameter("color_g", 0.62).value)
        self.color_b = float(self.declare_parameter("color_b", 0.12).value)
        self.outline_color_r = float(
            self.declare_parameter("outline_color_r", 1.0).value
        )
        self.outline_color_g = float(
            self.declare_parameter("outline_color_g", 0.82).value
        )
        self.outline_color_b = float(
            self.declare_parameter("outline_color_b", 0.20).value
        )
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
            f"{self.map_origin_world_z:.3f}, yaw={self.map_origin_world_yaw:.3f}); "
            f"marker_z_offset={self.marker_z_offset:.3f}"
        )

    def world_path(self):
        if self.world_sdf:
            return Path(self.world_sdf)
        share = Path(get_package_share_directory("lunar_rover_simulation"))
        return share / "worlds" / f"{self.world_name}.sdf"

    def model_search_paths(self, world_path):
        paths = []
        if world_path.parent.name == "worlds":
            paths.append(world_path.parent.parent / "models")
        share = Path(get_package_share_directory("lunar_rover_simulation"))
        paths.append(share / "models")
        script_models = Path(__file__).resolve().parents[1] / "models"
        paths.append(script_models)
        return list(dict.fromkeys(paths))

    def resolve_model_sdf(self, uri, world_path):
        if not uri.startswith("model://"):
            return None
        model_name = uri.removeprefix("model://").split("/", 1)[0]
        for search_path in self.model_search_paths(world_path):
            model_sdf = search_path / model_name / "model.sdf"
            if model_sdf.exists():
                return model_sdf
        self.get_logger().warn(f"Could not resolve included model URI {uri}")
        return None

    def model_elements(self, world_root, world_path):
        world = world_root.find("./world")
        if world is None:
            return

        for model in world.findall("model"):
            yield model, ((0.0, 0.0, 0.0), q_from_rpy(0.0, 0.0, 0.0))

        for include in world.findall("include"):
            uri = include.findtext("uri", default="").strip()
            model_sdf = self.resolve_model_sdf(uri, world_path)
            if model_sdf is None:
                continue
            included_root = ET.parse(model_sdf).getroot()
            model = included_root.find("./model")
            if model is None:
                self.get_logger().warn(f"No <model> found in included SDF {model_sdf}")
                continue
            yield model, pose_from_element(include)

    def load_markers(self):
        path = self.world_path()
        root = ET.parse(path).getroot()
        markers = MarkerArray()
        marker_id = 0
        for model, parent_pose in self.model_elements(root, path):
            static_text = model.findtext("static", default="true").strip().lower()
            if static_text in ("false", "0"):
                continue
            model_pose = compose_pose(parent_pose, pose_from_element(model))
            for link in model.findall("link"):
                link_pose = compose_pose(model_pose, pose_from_element(link))
                tags = []
                if self.include_visual:
                    tags.append("visual")
                if self.include_collision:
                    tags.append("collision")
                for tag in tags:
                    for shape in link.findall(tag):
                        geometry = shape.find("geometry")
                        if geometry is None:
                            continue
                        world_pose = compose_pose(link_pose, pose_from_element(shape))
                        pose = compose_pose(self.map_from_world, world_pose)
                        marker = self.make_shape_marker(marker_id, model, shape, geometry, pose)
                        if marker is None:
                            continue
                        markers.markers.append(marker)
                        marker_id += 1
                        if marker.type == Marker.CUBE:
                            markers.markers.append(self.make_box_outline(marker_id, marker))
                            marker_id += 1
        if self.include_collision and self.include_pit_markers:
            for marker in self.load_pit_markers(root, path, marker_id):
                markers.markers.append(marker)
                marker_id += 1
        if not markers.markers:
            self.get_logger().warn(f"No supported static geometry found in {path}")
        return markers

    def world_includes_model(self, world_root, model_name):
        world = world_root.find("./world")
        if world is None:
            return False
        for include in world.findall("include"):
            uri = include.findtext("uri", default="").strip()
            if uri == f"model://{model_name}" or uri.startswith(f"model://{model_name}/"):
                return True
        return False

    def resolve_model_file(self, model_name, relative_path, world_path):
        for search_path in self.model_search_paths(world_path):
            candidate = search_path / model_name / relative_path
            if candidate.exists():
                return candidate
        return None

    def load_pit_markers(self, world_root, world_path, first_marker_id):
        model_name = "image_cave_heightmap_terrain"
        if not self.world_includes_model(world_root, model_name):
            return []
        pit_file = self.resolve_model_file(model_name, Path("metadata") / "pits.csv", world_path)
        if pit_file is None:
            return []

        markers = []
        rows = pit_file.read_text(encoding="ascii").splitlines()
        for index, row in enumerate(rows[1:]):
            if not row.strip():
                continue
            parts = [float(value) for value in row.split(",")]
            if len(parts) != 4:
                continue
            x, y, target_z, radius = parts
            pose = compose_pose(
                self.map_from_world,
                ((x, y, target_z + 0.020), q_from_rpy(0.0, 0.0, 0.0)),
            )
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.ns = "world_collision_pits"
            marker.id = first_marker_id + len(markers)
            marker.action = Marker.ADD
            marker.type = Marker.CYLINDER
            marker.pose.position.x = pose[0][0]
            marker.pose.position.y = pose[0][1]
            marker.pose.position.z = pose[0][2] + self.marker_z_offset
            marker.pose.orientation.x = pose[1][0]
            marker.pose.orientation.y = pose[1][1]
            marker.pose.orientation.z = pose[1][2]
            marker.pose.orientation.w = pose[1][3]
            marker.scale.x = radius * 2.0
            marker.scale.y = radius * 2.0
            marker.scale.z = 0.018
            marker.color.r = self.color_r
            marker.color.g = self.color_g
            marker.color.b = self.color_b
            marker.color.a = max(0.35, self.outline_alpha)
            marker.text = f"{model_name}/pit_{index:03d}"
            markers.append(marker)
        return markers

    def make_shape_marker(self, marker_id, model, shape, geometry, pose):
        box = geometry.find("box")
        sphere = geometry.find("sphere")
        cylinder = geometry.find("cylinder")
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = self.marker_namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.position.x = pose[0][0]
        marker.pose.position.y = pose[0][1]
        marker.pose.position.z = pose[0][2] + self.marker_z_offset
        marker.pose.orientation.x = pose[1][0]
        marker.pose.orientation.y = pose[1][1]
        marker.pose.orientation.z = pose[1][2]
        marker.pose.orientation.w = pose[1][3]
        marker.color.r = self.color_r
        marker.color.g = self.color_g
        marker.color.b = self.color_b
        marker.color.a = self.alpha
        marker.text = f"{model.get('name', 'model')}/{shape.get('name', 'shape')}"

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
        marker.ns = self.outline_namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.type = Marker.LINE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.outline_width
        marker.color.r = self.outline_color_r
        marker.color.g = self.outline_color_g
        marker.color.b = self.outline_color_b
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
