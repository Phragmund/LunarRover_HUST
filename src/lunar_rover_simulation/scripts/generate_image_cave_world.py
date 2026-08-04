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
TERRAIN_MODEL_NAME = "image_cave_heightmap_terrain"
WALL_MODEL_NAME = "image_cave_walls"
BLOCK_MODEL_NAME = "image_cave_surface_blocks"
TERRAIN_TRIANGLE_EDGE = 0.2
TERRAIN_SIZE_X = 30.0
TERRAIN_SIZE_Y = 30.0
TERRAIN_CENTER_X = 4.0
TERRAIN_CENTER_Y = 2.24
TERRAIN_BASE_Z = 1.0
TERRAIN_BOTTOM_Z = 0.70
TERRAIN_Z_OFFSET = 0.90
TERRAIN_HEIGHT_RANGE = 0.20
TERRAIN_SEED = 360335
BLOCK_COUNT = 1200
BLOCK_SIZE_X = 0.10
BLOCK_SIZE_Y = 0.13
BLOCK_SIZE_Z = 0.05
BLOCK_MAX_TILT_DEG = 5.0
BLOCK_CLEARANCE_CAP = 0.01
BLOCK_TOP_Z_MIN = 1.00
BLOCK_TOP_Z_MAX = 1.01
BLOCK_TOP_BINS = 20
WALL_HEIGHT = 0.95
WALL_THICKNESS = 0.12
WALL_Z = TERRAIN_BASE_Z + WALL_HEIGHT * 0.5
SPAWN_X = -3.92
SPAWN_Y = 0.0
SPAWN_REFERENCE_PX = np.array([220.0, 750.0])
MANUAL_EDGES = (
    (96, 108),
    (127, 153),
    (150, 156),
    (52, 74),
    (41, 56),
    (27, 34),
    (59, 66),
    (69, 76),
)


@dataclass(frozen=True)
class Wall:
    x0: float
    y0: float
    x1: float
    y1: float


@dataclass(frozen=True)
class SurfaceBlock:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    top_z: float


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


def terrain_model_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "models" / TERRAIN_MODEL_NAME


def wall_model_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "models" / WALL_MODEL_NAME


def block_model_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "models" / BLOCK_MODEL_NAME


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
    return merge_manual_edges(edges, len(points))


def merge_manual_edges(
    edges: list[tuple[int, int]], point_count: int
) -> list[tuple[int, int]]:
    merged = {tuple(sorted(edge)) for edge in edges}
    for start, end in MANUAL_EDGES:
        if not (0 <= start < point_count and 0 <= end < point_count):
            raise RuntimeError(
                f"Manual wall segment {start}-{end} is outside the detected point range"
            )
        if start == end:
            raise RuntimeError(f"Manual wall segment {start}-{end} is degenerate")
        merged.add(tuple(sorted((start, end))))
    return sorted(merged)


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
    _ = points
    return SPAWN_REFERENCE_PX.copy()


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


def terrain_include_xml() -> str:
    return f"""
    <include>
      <uri>model://{TERRAIN_MODEL_NAME}</uri>
    </include>"""


def wall_include_xml() -> str:
    return f"""
    <include>
      <uri>model://{WALL_MODEL_NAME}</uri>
    </include>"""


def block_include_xml() -> str:
    return f"""
    <include>
      <uri>model://{BLOCK_MODEL_NAME}</uri>
    </include>"""


def spawn_marker_xml() -> str:
    return f"""
    <model name="image_cave_spawn_marker">
      <static>true</static>
      <link name="spawn_marker_link">
        <visual name="spawn_marker_visual">
          <pose>{SPAWN_X:.3f} {SPAWN_Y:.3f} {TERRAIN_BASE_Z + 0.006:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>0.28</radius><length>0.012</length></cylinder></geometry>
          {material_xml("spawn")}
          <transparency>0.35</transparency>
        </visual>
      </link>
    </model>"""


def world_xml(walls: list[Wall]) -> str:
    _ = walls
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
    {terrain_include_xml()}
    {wall_include_xml()}
    {block_include_xml()}
    {spawn_marker_xml()}
  </world>
</sdf>
"""


def terrain_preview_world_xml() -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="terrain_preview">
    <gravity>0 0 -9.80665</gravity>
    <physics name="physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <scene>
      <ambient>0.18 0.18 0.18 1</ambient>
      <background>0.030 0.032 0.036 1</background>
      <shadows>true</shadows>
    </scene>
    <gui fullscreen="0"/>
    <light name="terrain_key_light" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.95 0.89 0.76 1</diffuse>
      <specular>0.18 0.16 0.12 1</specular>
      <direction>-0.35 0.25 -0.90</direction>
    </light>
    {terrain_include_xml()}
    {wall_include_xml()}
    {block_include_xml()}
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


def generate_terrain_grid() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rng = np.random.default_rng(TERRAIN_SEED)
    x_min = TERRAIN_CENTER_X - TERRAIN_SIZE_X * 0.5
    y_min = TERRAIN_CENTER_Y - TERRAIN_SIZE_Y * 0.5
    row_spacing = TERRAIN_TRIANGLE_EDGE * math.sqrt(3.0) * 0.5
    rows = int(math.floor(TERRAIN_SIZE_Y / row_spacing)) + 1
    cols = int(math.floor(TERRAIN_SIZE_X / TERRAIN_TRIANGLE_EDGE)) + 1

    row_indices = np.arange(rows, dtype=np.float64)[:, None]
    col_indices = np.arange(cols, dtype=np.float64)[None, :]
    row_offsets = (np.arange(rows, dtype=np.float64) % 2.0)[:, None] * (
        TERRAIN_TRIANGLE_EDGE * 0.5
    )
    grid_x = x_min + col_indices * TERRAIN_TRIANGLE_EDGE + row_offsets
    grid_y = np.broadcast_to(y_min + row_indices * row_spacing, grid_x.shape).copy()

    low = rng.normal(0.0, 1.0, (rows, cols)).astype(np.float32)
    low = cv2.GaussianBlur(low, (0, 0), 1.6)
    low = (low - low.mean()) / (low.std() + 1e-6)

    mid = rng.normal(0.0, 1.0, (rows, cols)).astype(np.float32)
    mid = cv2.GaussianBlur(mid, (0, 0), 0.45)
    mid = (mid - mid.mean()) / (mid.std() + 1e-6)

    ridge_a = np.sin(grid_x * 2.7 + grid_y * 1.1)
    ridge_b = np.sin(grid_x * -1.3 + grid_y * 3.2 + 1.6)
    terraces = np.round((0.45 * low + 0.90 * mid + 0.65 * ridge_a) * 1.15) / 1.15
    ledges = np.sign(ridge_a + 0.75 * ridge_b) * 0.95

    height = 0.060 + 0.115 * low + 0.135 * mid + 0.165 * terraces + 0.095 * ledges

    for cx, cy, radius, amplitude in (
        (-1.6, 3.2, 1.9, 0.260),
        (2.8, -2.4, 1.7, -0.240),
        (7.8, 5.0, 2.2, 0.235),
        (9.7, -4.9, 1.8, 0.285),
        (-5.9, -1.1, 1.9, 0.220),
    ):
        distance = np.hypot(grid_x - cx, grid_y - cy)
        mask = np.clip(1.0 - distance / radius, 0.0, 1.0)
        height += amplitude * mask * mask

    # Keep the initial rover pose locally flat and low enough to avoid spawning on an edge.
    spawn_distance = np.hypot(grid_x - SPAWN_X, grid_y - SPAWN_Y)
    flat_weight = np.clip(1.0 - (spawn_distance - 0.45) / 0.65, 0.0, 1.0)
    height = height * (1.0 - flat_weight) + 0.0 * flat_weight

    height = height - float(height.mean())
    max_abs_height = float(np.max(np.abs(height)))
    if max_abs_height > 1e-6:
        height = height / max_abs_height * 0.10
    height = np.round(height / 0.010) * 0.010
    height = np.clip(height, -0.10, 0.10)
    height += TERRAIN_BASE_Z
    return grid_x, grid_y, height


def height_to_preview_pixel(height: np.ndarray) -> np.ndarray:
    normalized = np.clip(
        (height - TERRAIN_Z_OFFSET) / TERRAIN_HEIGHT_RANGE,
        0.0,
        1.0,
    )
    return np.rint(normalized * 255.0).astype(np.uint8)


def nearest_terrain_height(
    x: float, y: float, grid_x: np.ndarray, grid_y: np.ndarray, height: np.ndarray
) -> float:
    row = int(np.argmin(np.abs(grid_y[:, 0] - y)))
    col = int(np.argmin(np.abs(grid_x[row, :] - x)))
    return float(height[row, col])


def distance_to_wall(x: float, y: float, wall: Wall) -> float:
    ax = wall.x0
    ay = wall.y0
    bx = wall.x1
    by = wall.y1
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-9:
        return math.hypot(x - ax, y - ay)
    t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length_sq))
    px = ax + t * dx
    py = ay + t * dy
    return math.hypot(x - px, y - py)


def near_any_wall(x: float, y: float, walls: list[Wall], margin: float) -> bool:
    return any(distance_to_wall(x, y, wall) < margin for wall in walls)


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    rx = np.array(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    ry = np.array(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    rz = np.array(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
    return rz @ ry @ rx


def rotated_box_z_extents(
    size_x: float, size_y: float, size_z: float, roll: float, pitch: float, yaw: float
) -> tuple[float, float]:
    corners = np.array(
        [
            (sx * size_x * 0.5, sy * size_y * 0.5, sz * size_z * 0.5)
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ],
        dtype=np.float64,
    )
    rotated = corners @ rotation_matrix(roll, pitch, yaw).T
    return float(rotated[:, 2].min()), float(rotated[:, 2].max())


def generate_surface_blocks(
    grid_x: np.ndarray, grid_y: np.ndarray, height: np.ndarray, walls: list[Wall]
) -> tuple[list[SurfaceBlock], float]:
    rng = np.random.default_rng(TERRAIN_SEED + 37)
    height_cap = (float(height.min()) + float(height.max())) * 0.5 + BLOCK_CLEARANCE_CAP
    top_min = max(BLOCK_TOP_Z_MIN, float(height.min()))
    top_max = min(BLOCK_TOP_Z_MAX, height_cap)
    if top_min >= top_max:
        raise RuntimeError(
            f"Invalid surface block top range [{top_min:.3f}, {top_max:.3f}]"
        )
    x_min = float(grid_x.min()) + 0.35
    x_max = float(grid_x.max()) - 0.35
    y_min = float(grid_y.min()) + 0.35
    y_max = float(grid_y.max()) - 0.35
    max_tilt = math.radians(BLOCK_MAX_TILT_DEG)
    wall_margin = WALL_THICKNESS * 0.5 + max(BLOCK_SIZE_X, BLOCK_SIZE_Y) * 0.75
    spawn_margin = 0.85
    blocks: list[SurfaceBlock] = []
    bin_counts = [0] * BLOCK_TOP_BINS
    target_per_bin = math.ceil(BLOCK_COUNT / BLOCK_TOP_BINS)

    for _ in range(BLOCK_COUNT * 450):
        if len(blocks) >= BLOCK_COUNT:
            break
        available_bins = [
            index for index, count in enumerate(bin_counts) if count < target_per_bin
        ]
        if not available_bins:
            break
        bin_index = int(rng.choice(available_bins))
        bin_low = top_min + (top_max - top_min) * bin_index / BLOCK_TOP_BINS
        bin_high = top_min + (top_max - top_min) * (bin_index + 1) / BLOCK_TOP_BINS
        target_top_z = float(rng.uniform(bin_low, bin_high))

        x = float(rng.uniform(x_min, x_max))
        y = float(rng.uniform(y_min, y_max))
        if math.hypot(x - SPAWN_X, y - SPAWN_Y) < spawn_margin:
            continue
        if near_any_wall(x, y, walls, wall_margin):
            continue

        roll = float(rng.uniform(-max_tilt, max_tilt))
        pitch = float(rng.uniform(-max_tilt, max_tilt))
        yaw = float(rng.uniform(-math.pi, math.pi))
        min_offset, max_offset = rotated_box_z_extents(
            BLOCK_SIZE_X, BLOCK_SIZE_Y, BLOCK_SIZE_Z, roll, pitch, yaw
        )
        ground_z = nearest_terrain_height(x, y, grid_x, grid_y, height)
        center_z = target_top_z - max_offset
        bottom_z = center_z + min_offset
        if bottom_z < ground_z - 0.002:
            continue
        if target_top_z < BLOCK_TOP_Z_MIN or target_top_z > height_cap:
            continue
        blocks.append(SurfaceBlock(x, y, center_z, roll, pitch, yaw, target_top_z))
        bin_counts[bin_index] += 1

    if len(blocks) < BLOCK_COUNT:
        raise RuntimeError(
            f"Could only place {len(blocks)} surface blocks out of {BLOCK_COUNT}; "
            f"top-bin counts={bin_counts}"
        )
    return blocks, height_cap


def terrain_vertices(
    grid_x: np.ndarray, grid_y: np.ndarray, height: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    top = np.column_stack((grid_x.ravel(), grid_y.ravel(), height.ravel()))
    bottom = top.copy()
    bottom[:, 2] = TERRAIN_BOTTOM_Z
    return top, bottom


def terrain_top_faces(rows: int, cols: int) -> list[tuple[int, int, int]]:
    faces: list[tuple[int, int, int]] = []

    def vertex_index(row: int, col: int) -> int:
        return row * cols + col

    for row in range(rows - 1):
        next_row_shifted_right = row % 2 == 0
        for col in range(cols - 1):
            a = vertex_index(row, col)
            b = vertex_index(row, col + 1)
            c = vertex_index(row + 1, col)
            d = vertex_index(row + 1, col + 1)
            if next_row_shifted_right:
                faces.append((a, b, c))
                faces.append((b, d, c))
            else:
                faces.append((a, d, c))
                faces.append((a, b, d))
    return faces


def closed_terrain_mesh(
    grid_x: np.ndarray, grid_y: np.ndarray, height: np.ndarray
) -> tuple[np.ndarray, list[tuple[int, int, int]], int]:
    rows, cols = height.shape
    top, bottom = terrain_vertices(grid_x, grid_y, height)
    vertices = np.vstack((top, bottom))
    bottom_offset = top.shape[0]
    top_faces = terrain_top_faces(rows, cols)
    faces: list[tuple[int, int, int]] = []
    faces.extend(top_faces)
    faces.extend(
        (c + bottom_offset, b + bottom_offset, a + bottom_offset)
        for a, b, c in top_faces
    )

    def add_side_quad(t0: int, t1: int) -> None:
        b0 = t0 + bottom_offset
        b1 = t1 + bottom_offset
        faces.append((t0, t1, b1))
        faces.append((t0, b1, b0))

    for col in range(cols - 1):
        add_side_quad(col, col + 1)
        add_side_quad((rows - 1) * cols + col + 1, (rows - 1) * cols + col)
    for row in range(rows - 1):
        add_side_quad((row + 1) * cols, row * cols)
        add_side_quad(row * cols + cols - 1, (row + 1) * cols + cols - 1)

    return vertices, faces, len(top_faces)


def triangle_normal(
    a: np.ndarray, b: np.ndarray, c: np.ndarray
) -> tuple[float, float, float]:
    normal = np.cross(b - a, c - a)
    length = float(np.linalg.norm(normal))
    if length < 1e-9:
        return 0.0, 0.0, 1.0
    normal = normal / length
    return float(normal[0]), float(normal[1]), float(normal[2])


def write_ascii_stl(
    path: Path, vertices: np.ndarray, faces: list[tuple[int, int, int]]
) -> None:
    lines = ["solid image_cave_triangular_terrain"]
    for i0, i1, i2 in faces:
        a = vertices[i0]
        b = vertices[i1]
        c = vertices[i2]
        nx, ny, nz = triangle_normal(a, b, c)
        lines.append(f"  facet normal {nx:.8e} {ny:.8e} {nz:.8e}")
        lines.append("    outer loop")
        lines.append(f"      vertex {a[0]:.6f} {a[1]:.6f} {a[2]:.6f}")
        lines.append(f"      vertex {b[0]:.6f} {b[1]:.6f} {b[2]:.6f}")
        lines.append(f"      vertex {c[0]:.6f} {c[1]:.6f} {c[2]:.6f}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append("endsolid image_cave_triangular_terrain")
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def write_faceted_dae(
    path: Path, vertices: np.ndarray, faces: list[tuple[int, int, int]]
) -> None:
    positions: list[str] = []
    normals: list[str] = []
    indices: list[str] = []
    vertex_index = 0
    for i0, i1, i2 in faces:
        a = vertices[i0]
        b = vertices[i1]
        c = vertices[i2]
        nx, ny, nz = triangle_normal(a, b, c)
        for vertex in (a, b, c):
            positions.extend((f"{vertex[0]:.6f}", f"{vertex[1]:.6f}", f"{vertex[2]:.6f}"))
            normals.extend((f"{nx:.8f}", f"{ny:.8f}", f"{nz:.8f}"))
            indices.extend((str(vertex_index), str(vertex_index)))
            vertex_index += 1

    vertex_count = vertex_index
    triangle_count = len(faces)
    dae = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_effects>
    <effect id="sandstone_effect">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <diffuse><color>0.48 0.42 0.34 1</color></diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="sandstone_material" name="sandstone_material">
      <instance_effect url="#sandstone_effect"/>
    </material>
  </library_materials>
  <library_geometries>
    <geometry id="terrain_geometry" name="terrain_geometry">
      <mesh>
        <source id="terrain_positions">
          <float_array id="terrain_positions_array" count="{vertex_count * 3}">{" ".join(positions)}</float_array>
          <technique_common>
            <accessor source="#terrain_positions_array" count="{vertex_count}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="terrain_normals">
          <float_array id="terrain_normals_array" count="{vertex_count * 3}">{" ".join(normals)}</float_array>
          <technique_common>
            <accessor source="#terrain_normals_array" count="{vertex_count}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="terrain_vertices">
          <input semantic="POSITION" source="#terrain_positions"/>
        </vertices>
        <triangles count="{triangle_count}" material="sandstone_material">
          <input semantic="VERTEX" source="#terrain_vertices" offset="0"/>
          <input semantic="NORMAL" source="#terrain_normals" offset="1"/>
          <p>{" ".join(indices)}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="terrain_scene" name="terrain_scene">
      <node id="terrain_node" name="terrain_node">
        <instance_geometry url="#terrain_geometry">
          <bind_material>
            <technique_common>
              <instance_material symbol="sandstone_material" target="#sandstone_material"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#terrain_scene"/>
  </scene>
</COLLADA>
"""
    path.write_text(dae, encoding="utf-8")


def top_mesh_edges(faces: list[tuple[int, int, int]]) -> list[tuple[int, int]]:
    edges: set[tuple[int, int]] = set()
    for a, b, c in faces:
        for start, end in ((a, b), (b, c), (c, a)):
            edges.add(tuple(sorted((start, end))))
    return sorted(edges)


def terrain_edge_visuals_xml(
    vertices: np.ndarray, edges: list[tuple[int, int]]
) -> str:
    visuals = []
    for index, (start, end) in enumerate(edges):
        a = vertices[start]
        b = vertices[end]
        dx = float(b[0] - a[0])
        dy = float(b[1] - a[1])
        length = math.hypot(dx, dy)
        if length < 0.05:
            continue
        x = float((a[0] + b[0]) * 0.5)
        y = float((a[1] + b[1]) * 0.5)
        z = float(max(a[2], b[2]) + 0.018)
        yaw = math.atan2(dy, dx)
        visuals.append(
            f"""
      <visual name="terrain_edge_{index:04d}_visual">
        <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.6f}</pose>
        <geometry><box><size>{length:.4f} 0.0300 0.0140</size></box></geometry>
        <material>
          <ambient>0.035 0.030 0.026 1</ambient>
          <diffuse>0.070 0.060 0.050 1</diffuse>
          <specular>0.005 0.005 0.005 1</specular>
        </material>
      </visual>"""
        )
    return "\n".join(visuals)


def write_terrain_assets(model_dir: Path) -> tuple[float, float, np.ndarray, np.ndarray, np.ndarray]:
    texture_dir = model_dir / "materials" / "textures"
    texture_dir.mkdir(parents=True, exist_ok=True)
    mesh_dir = model_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    grid_x, grid_y, height = generate_terrain_grid()
    cv2.imwrite(
        str(texture_dir / "image_cave_triangular_height_preview.png"),
        height_to_preview_pixel(height),
    )
    height_range = (float(height.min()), float(height.max()))
    vertices, faces, top_face_count = closed_terrain_mesh(grid_x, grid_y, height)
    top_faces = faces[:top_face_count]
    top_vertices = vertices[: grid_x.size]
    write_ascii_stl(mesh_dir / "sandstone_terrain_closed.stl", vertices, faces)
    write_faceted_dae(mesh_dir / "sandstone_terrain_faceted.dae", top_vertices, top_faces)

    rng = np.random.default_rng(TERRAIN_SEED + 1)
    diffuse = np.zeros((64, 64, 3), dtype=np.uint8)
    base = np.array([86, 96, 116], dtype=np.int16)
    noise = rng.normal(0.0, 8.0, (64, 64, 1)).astype(np.int16)
    grain = (np.sin(np.linspace(0, 18, 64))[None, :, None] * 7).astype(np.int16)
    diffuse[:] = np.clip(base + noise + grain, 0, 255).astype(np.uint8)
    cv2.imwrite(str(texture_dir / "sandstone_diffuse.png"), diffuse)

    normal = np.full((8, 8, 3), (255, 128, 128), dtype=np.uint8)
    cv2.imwrite(str(texture_dir / "flat_normal.png"), normal)

    model_sdf = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{TERRAIN_MODEL_NAME}">
    <static>true</static>
    <link name="terrain_link">
      <visual name="terrain_visual">
        <geometry>
          <mesh>
            <uri>model://{TERRAIN_MODEL_NAME}/meshes/sandstone_terrain_faceted.dae</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.31 0.28 0.23 1</ambient>
          <diffuse>0.48 0.42 0.34 1</diffuse>
          <specular>0.035 0.030 0.025 1</specular>
        </material>
      </visual>
      <collision name="terrain_collision">
        <geometry>
          <mesh>
            <uri>model://{TERRAIN_MODEL_NAME}/meshes/sandstone_terrain_closed.stl</uri>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.25</mu>
              <mu2>1.05</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>10000000</kp>
              <kd>100</kd>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
"""
    (model_dir / "model.sdf").write_text(model_sdf, encoding="utf-8")

    model_config = f"""<?xml version="1.0"?>
<model>
  <name>Image Cave Heightmap Terrain</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>Lunar Rover Team</name>
    <email>maintainer@example.com</email>
  </author>
  <description>Deterministic watertight triangular sandstone-style terrain for the generated image cave world. Edge length {TERRAIN_TRIANGLE_EDGE:.2f} m, top faces {top_face_count}.</description>
</model>
"""
    (model_dir / "model.config").write_text(model_config, encoding="utf-8")
    return height_range[0], height_range[1], grid_x, grid_y, height


def write_wall_assets(model_dir: Path, walls: list[Wall]) -> None:
    model_dir.mkdir(parents=True, exist_ok=True)
    walls_xml = "\n".join(wall_xml(wall, index) for index, wall in enumerate(walls))
    model_sdf = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{WALL_MODEL_NAME}">
    <static>true</static>
    <link name="walls_link">{walls_xml}
    </link>
  </model>
</sdf>
"""
    (model_dir / "model.sdf").write_text(model_sdf, encoding="utf-8")

    model_config = f"""<?xml version="1.0"?>
<model>
  <name>Image Cave Walls</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>Lunar Rover Team</name>
    <email>maintainer@example.com</email>
  </author>
  <description>Generated cave wall segments from the red-point reference image. Contains {len(walls)} static wall segments.</description>
</model>
"""
    (model_dir / "model.config").write_text(model_config, encoding="utf-8")


def block_xml(block: SurfaceBlock, index: int) -> str:
    shade = 0.72 + 0.18 * ((index * 37) % 11) / 10.0
    ambient = (0.20 * shade, 0.18 * shade, 0.15 * shade)
    diffuse = (0.38 * shade, 0.33 * shade, 0.27 * shade)
    return f"""
        <collision name="surface_block_{index:04d}_collision">
          <pose>{block.x:.9f} {block.y:.9f} {block.z:.9f} {block.roll:.9f} {block.pitch:.9f} {block.yaw:.9f}</pose>
          <geometry><box><size>{BLOCK_SIZE_X:.4f} {BLOCK_SIZE_Y:.4f} {BLOCK_SIZE_Z:.4f}</size></box></geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.35</mu>
                <mu2>1.10</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="surface_block_{index:04d}_visual">
          <pose>{block.x:.9f} {block.y:.9f} {block.z:.9f} {block.roll:.9f} {block.pitch:.9f} {block.yaw:.9f}</pose>
          <geometry><box><size>{BLOCK_SIZE_X:.4f} {BLOCK_SIZE_Y:.4f} {BLOCK_SIZE_Z:.4f}</size></box></geometry>
          <material>
            <ambient>{ambient[0]:.3f} {ambient[1]:.3f} {ambient[2]:.3f} 1</ambient>
            <diffuse>{diffuse[0]:.3f} {diffuse[1]:.3f} {diffuse[2]:.3f} 1</diffuse>
            <specular>0.015 0.013 0.010 1</specular>
          </material>
        </visual>"""


def write_block_assets(
    model_dir: Path,
    grid_x: np.ndarray,
    grid_y: np.ndarray,
    height: np.ndarray,
    walls: list[Wall],
) -> tuple[int, float, float]:
    model_dir.mkdir(parents=True, exist_ok=True)
    blocks, height_cap = generate_surface_blocks(grid_x, grid_y, height, walls)
    blocks_xml = "\n".join(block_xml(block, index) for index, block in enumerate(blocks))
    top_max = max(block.top_z for block in blocks)
    model_sdf = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{BLOCK_MODEL_NAME}">
    <static>true</static>
    <link name="surface_blocks_link">{blocks_xml}
    </link>
  </model>
</sdf>
"""
    (model_dir / "model.sdf").write_text(model_sdf, encoding="utf-8")

    model_config = f"""<?xml version="1.0"?>
<model>
  <name>Image Cave Surface Blocks</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>Lunar Rover Team</name>
    <email>maintainer@example.com</email>
  </author>
  <description>{len(blocks)} randomized small sandstone blocks. Size {BLOCK_SIZE_X:.2f} x {BLOCK_SIZE_Y:.2f} x {BLOCK_SIZE_Z:.2f} m, yaw randomized, roll/pitch within {BLOCK_MAX_TILT_DEG:.1f} deg, max rotated top z {top_max:.3f} m, cap {height_cap:.3f} m.</description>
</model>
"""
    (model_dir / "model.config").write_text(model_config, encoding="utf-8")
    return len(blocks), height_cap, top_max


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=Path, default=DEFAULT_IMAGE)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "worlds" / f"{WORLD_NAME}.sdf",
    )
    parser.add_argument("--terrain-model-dir", type=Path, default=terrain_model_dir())
    parser.add_argument("--wall-model-dir", type=Path, default=wall_model_dir())
    parser.add_argument("--block-model-dir", type=Path, default=block_model_dir())
    parser.add_argument(
        "--preview-output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "worlds" / "terrain_preview.sdf",
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
    terrain_min, terrain_max, grid_x, grid_y, height = write_terrain_assets(
        args.terrain_model_dir
    )
    write_wall_assets(args.wall_model_dir, walls)
    block_count, block_cap, block_top_max = write_block_assets(
        args.block_model_dir, grid_x, grid_y, height, walls
    )
    args.output.write_text(world_xml(walls), encoding="utf-8")
    args.preview_output.write_text(terrain_preview_world_xml(), encoding="utf-8")
    if args.overlay is not None:
        write_overlay(image, points, edges, args.overlay)
    print(
        f"wrote {args.output} from {len(points)} points and {len(edges)} wall segments; "
        f"channel_width={channel_width_px:.2f}px scale={meters_per_px:.5f}m/px; "
        f"terrain_height=[{terrain_min:.3f},{terrain_max:.3f}]m; "
        f"surface_blocks={block_count} top_max={block_top_max:.3f}m cap={block_cap:.3f}m"
    )


if __name__ == "__main__":
    main()
