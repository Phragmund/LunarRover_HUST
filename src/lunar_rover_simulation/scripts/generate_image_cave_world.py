#!/usr/bin/env python3
"""Generate an SDF cave world from a red-point/black-wall reference image."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


DEFAULT_IMAGE = Path(
    "/home/phragmund/Downloads/ChatGPT Image Jul 29, 2026, 11_47_45 PM.png"
)
WORLD_NAME = "image_cave"
WALL_HEIGHT = 0.95
WALL_THICKNESS = 0.12
WALL_Z = WALL_HEIGHT * 0.5
SPAWN_X = -3.92
SPAWN_Y = 0.0


@dataclass(frozen=True)
class Wall:
    x0: float
    y0: float
    x1: float
    y1: float


def material_xml(name: str) -> str:
    colors = {
        "wall": ("0.18 0.17 0.16 1", "0.30 0.28 0.24 1"),
        "wall_dark": ("0.12 0.12 0.11 1", "0.22 0.21 0.19 1"),
        "spawn": ("0.08 0.32 0.12 0.45", "0.12 0.60 0.18 0.45"),
    }
    ambient, diffuse = colors[name]
    return (
        "<material>"
        f"<ambient>{ambient}</ambient>"
        f"<diffuse>{diffuse}</diffuse>"
        "<specular>0.02 0.02 0.02 1</specular>"
        "</material>"
    )


def red_points(image: np.ndarray) -> np.ndarray:
    blue, green, red = cv2.split(image)
    mask = ((red > 170) & (green < 90) & (blue < 90)).astype("uint8") * 255
    count, _labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
    points = []
    for index in range(1, count):
        area = stats[index, cv2.CC_STAT_AREA]
        if 8 <= area <= 300:
            points.append(tuple(centers[index]))
    if len(points) < 2:
        raise RuntimeError("Could not detect enough red endpoints in the source image")
    return np.array(points, dtype=float)


def line_mask(image: np.ndarray, red_mask: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    black = ((gray < 95) & (red_mask == 0)).astype("uint8") * 255
    black = cv2.dilate(black, np.ones((5, 5), np.uint8), iterations=1)
    red = cv2.dilate(red_mask, np.ones((5, 5), np.uint8), iterations=1)
    return (black > 0) | (red > 0)


def source_masks(image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    blue, green, red = cv2.split(image)
    red_mask = ((red > 170) & (green < 90) & (blue < 90)).astype("uint8") * 255
    return red_mask, line_mask(image, red_mask)


def extract_edges(image: np.ndarray, points: np.ndarray) -> list[tuple[int, int]]:
    red_mask, mask = source_masks(image)
    _ = red_mask
    edges: list[tuple[int, int]] = []
    height, width = image.shape[:2]
    for i, start in enumerate(points):
        for j in range(i + 1, len(points)):
            end = points[j]
            distance = float(np.linalg.norm(end - start))
            if distance < 8.0 or distance > 230.0:
                continue

            samples = max(12, int(distance / 2.0))
            xs = np.linspace(start[0], end[0], samples)
            ys = np.linspace(start[1], end[1], samples)
            xi = np.clip(np.rint(xs).astype(int), 0, width - 1)
            yi = np.clip(np.rint(ys).astype(int), 0, height - 1)
            if float(mask[yi, xi].mean()) <= 0.72:
                continue

            vector = end - start
            vv = float(np.dot(vector, vector))
            intermediate = False
            for k, point in enumerate(points):
                if k in (i, j):
                    continue
                t = float(np.dot(point - start, vector) / vv)
                if 0.08 < t < 0.92:
                    line_distance = abs(float(np.cross(vector, point - start))) / math.sqrt(vv)
                    if line_distance < 8.0:
                        intermediate = True
                        break
            if not intermediate:
                edges.append((i, j))
    if not edges:
        raise RuntimeError("Could not infer black wall segments from the source image")
    return edges


def estimate_channel_width_px(points: np.ndarray, edges: list[tuple[int, int]]) -> float:
    distances = []
    edge_points = [(points[i], points[j]) for i, j in edges]
    for index, (p0, p1) in enumerate(edge_points):
        vector = p1 - p0
        length = float(np.linalg.norm(vector))
        if length < 25.0:
            continue
        tangent = vector / length
        normal = np.array([-tangent[1], tangent[0]])
        center = (p0 + p1) * 0.5
        p_proj = sorted([float(np.dot(p0, tangent)), float(np.dot(p1, tangent))])
        for q0, q1 in edge_points[index + 1 :]:
            other = q1 - q0
            other_length = float(np.linalg.norm(other))
            if other_length < 25.0:
                continue
            other_tangent = other / other_length
            if abs(float(np.dot(tangent, other_tangent))) < math.cos(math.radians(12.0)):
                continue
            q_proj = sorted([float(np.dot(q0, tangent)), float(np.dot(q1, tangent))])
            overlap = min(p_proj[1], q_proj[1]) - max(p_proj[0], q_proj[0])
            if overlap < max(20.0, min(length, other_length) * 0.35):
                continue
            separation = abs(float(np.dot(((q0 + q1) * 0.5) - center, normal)))
            if 35.0 <= separation <= 100.0:
                distances.append(separation)
    if not distances:
        return 50.5
    return float(np.median(distances))


def entry_center(points: np.ndarray) -> np.ndarray:
    order = np.argsort(points[:, 0])
    return points[order[:2]].mean(axis=0)


def to_world(point: np.ndarray, origin: np.ndarray, meters_per_px: float) -> tuple[float, float]:
    x = SPAWN_X + (float(point[0]) - float(origin[0])) * meters_per_px
    y = SPAWN_Y - (float(point[1]) - float(origin[1])) * meters_per_px
    return x, y


def build_walls(points: np.ndarray, edges: list[tuple[int, int]], meters_per_px: float) -> list[Wall]:
    origin = entry_center(points)
    walls = []
    for i, j in edges:
        x0, y0 = to_world(points[i], origin, meters_per_px)
        x1, y1 = to_world(points[j], origin, meters_per_px)
        walls.append(Wall(x0, y0, x1, y1))
    return walls


def wall_xml(wall: Wall, index: int) -> str:
    dx = wall.x1 - wall.x0
    dy = wall.y1 - wall.y0
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    cx = (wall.x0 + wall.x1) * 0.5
    cy = (wall.y0 + wall.y1) * 0.5
    material = "wall" if index % 2 == 0 else "wall_dark"
    return f"""
        <collision name="image_cave_wall_{index:03d}_collision">
          <pose>{cx:.3f} {cy:.3f} {WALL_Z:.3f} 0 0 {yaw:.6f}</pose>
          <geometry><box><size>{length + WALL_THICKNESS:.3f} {WALL_THICKNESS:.3f} {WALL_HEIGHT:.3f}</size></box></geometry>
        </collision>
        <visual name="image_cave_wall_{index:03d}_visual">
          <pose>{cx:.3f} {cy:.3f} {WALL_Z:.3f} 0 0 {yaw:.6f}</pose>
          <geometry><box><size>{length + WALL_THICKNESS:.3f} {WALL_THICKNESS:.3f} {WALL_HEIGHT:.3f}</size></box></geometry>
          {material_xml(material)}
        </visual>"""


def spawn_marker_xml() -> str:
    return f"""
    <model name="image_cave_spawn_marker">
      <static>true</static>
      <link name="spawn_marker_link">
        <visual name="spawn_marker_visual">
          <pose>{SPAWN_X:.3f} {SPAWN_Y:.3f} 0.006 0 0 0</pose>
          <geometry><cylinder><radius>0.28</radius><length>0.012</length></cylinder></geometry>
          {material_xml("spawn")}
          <transparency>0.35</transparency>
        </visual>
      </link>
    </model>"""


def world_xml(walls: list[Wall]) -> str:
    walls_xml = "\n".join(wall_xml(wall, index) for index, wall in enumerate(walls))
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="{WORLD_NAME}">
    <gravity>0 0 -9.80665</gravity>
    <physics name="physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="libignition-gazebo-imu-system.so" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="libignition-gazebo-contact-system.so" name="ignition::gazebo::systems::Contact"/>
    <scene>
      <ambient>0.15 0.15 0.16 1</ambient>
      <background>0.035 0.038 0.043 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="cave_top_light" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.90 0.86 0.74 1</diffuse>
      <specular>0.16 0.14 0.10 1</specular>
      <direction>-0.35 0.30 -0.89</direction>
    </light>
    <include>
      <uri>model://lunar_ground</uri>
    </include>
    {spawn_marker_xml()}
    <model name="generated_image_cave_walls">
      <static>true</static>
      <link name="walls_link">{walls_xml}
      </link>
    </model>
  </world>
</sdf>
"""


def write_overlay(
    image: np.ndarray, points: np.ndarray, edges: list[tuple[int, int]], path: Path
) -> None:
    overlay = image.copy()
    for i, j in edges:
        start = tuple(np.rint(points[i]).astype(int))
        end = tuple(np.rint(points[j]).astype(int))
        cv2.line(overlay, start, end, (255, 0, 0), 1)
    for point in points:
        cv2.circle(overlay, tuple(np.rint(point).astype(int)), 5, (0, 255, 0), 1)
    cv2.imwrite(str(path), overlay)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=Path, default=DEFAULT_IMAGE)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "worlds" / f"{WORLD_NAME}.sdf",
    )
    parser.add_argument("--overlay", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(args.image)
    points = red_points(image)
    edges = extract_edges(image, points)
    channel_width_px = estimate_channel_width_px(points, edges)
    meters_per_px = 1.0 / channel_width_px
    walls = build_walls(points, edges, meters_per_px)
    args.output.write_text(world_xml(walls), encoding="utf-8")
    if args.overlay is not None:
        write_overlay(image, points, edges, args.overlay)
    print(
        f"wrote {args.output} from {len(points)} points and {len(edges)} wall segments; "
        f"channel_width={channel_width_px:.2f}px scale={meters_per_px:.5f}m/px"
    )


if __name__ == "__main__":
    main()
