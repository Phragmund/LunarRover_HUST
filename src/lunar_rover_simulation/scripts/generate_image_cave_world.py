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
TERRAIN_GRID_SIZE = 33
TERRAIN_SIZE_X = 30.0
TERRAIN_SIZE_Y = 30.0
TERRAIN_CENTER_X = 4.0
TERRAIN_CENTER_Y = 2.24
TERRAIN_Z_OFFSET = -0.04
TERRAIN_HEIGHT_RANGE = 0.22
TERRAIN_SEED = 360335
WALL_HEIGHT = 0.95
WALL_THICKNESS = 0.12
WALL_Z = WALL_HEIGHT * 0.5
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
    {terrain_include_xml()}
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


def generate_terrain_grid() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rng = np.random.default_rng(TERRAIN_SEED)
    size = TERRAIN_GRID_SIZE
    xs = np.linspace(
        TERRAIN_CENTER_X - TERRAIN_SIZE_X * 0.5,
        TERRAIN_CENTER_X + TERRAIN_SIZE_X * 0.5,
        size,
    )
    ys = np.linspace(
        TERRAIN_CENTER_Y - TERRAIN_SIZE_Y * 0.5,
        TERRAIN_CENTER_Y + TERRAIN_SIZE_Y * 0.5,
        size,
    )
    grid_x, grid_y = np.meshgrid(xs, ys)

    low = rng.normal(0.0, 1.0, (size, size)).astype(np.float32)
    low = cv2.GaussianBlur(low, (0, 0), 15.0)
    low = (low - low.mean()) / (low.std() + 1e-6)

    mid = rng.normal(0.0, 1.0, (size, size)).astype(np.float32)
    mid = cv2.GaussianBlur(mid, (0, 0), 4.5)
    mid = (mid - mid.mean()) / (mid.std() + 1e-6)

    ridge_a = np.sin(grid_x * 2.7 + grid_y * 1.1)
    ridge_b = np.sin(grid_x * -1.3 + grid_y * 3.2 + 1.6)
    terraces = np.round((0.55 * low + 0.45 * mid + 0.35 * ridge_a) * 3.0) / 3.0
    ledges = np.sign(ridge_a + 0.55 * ridge_b) * 0.5

    height = 0.035 + 0.026 * low + 0.018 * mid + 0.022 * terraces + 0.012 * ledges

    for cx, cy, radius, amplitude in (
        (-1.6, 3.2, 1.4, 0.055),
        (2.8, -2.4, 1.1, -0.040),
        (7.8, 5.0, 1.7, 0.050),
        (9.7, -4.9, 1.2, 0.060),
        (-5.9, -1.1, 1.3, 0.045),
    ):
        distance = np.hypot(grid_x - cx, grid_y - cy)
        mask = np.clip(1.0 - distance / radius, 0.0, 1.0)
        height += amplitude * mask * mask

    # Keep the initial rover pose locally flat and low enough to avoid spawning on an edge.
    spawn_distance = np.hypot(grid_x - SPAWN_X, grid_y - SPAWN_Y)
    flat_weight = np.clip(1.0 - (spawn_distance - 0.75) / 0.85, 0.0, 1.0)
    height = height * (1.0 - flat_weight) + 0.0 * flat_weight

    height = np.clip(height, -0.03, 0.16)
    return grid_x, grid_y, height


def height_to_preview_pixel(height: np.ndarray) -> np.ndarray:
    normalized = np.clip(
        (height - TERRAIN_Z_OFFSET) / TERRAIN_HEIGHT_RANGE,
        0.0,
        1.0,
    )
    return np.rint(normalized * 255.0).astype(np.uint8)


def terrain_tile_xml(
    index: int,
    x: float,
    y: float,
    z: float,
    size_x: float,
    size_y: float,
    size_z: float,
    top: float,
) -> str:
    shade = float(np.clip((top + 0.03) / 0.19, 0.0, 1.0))
    ambient = (
        0.22 + 0.12 * shade,
        0.20 + 0.10 * shade,
        0.17 + 0.08 * shade,
    )
    diffuse = (
        0.36 + 0.16 * shade,
        0.32 + 0.14 * shade,
        0.26 + 0.12 * shade,
    )
    return f"""
        <collision name="terrain_tile_{index:04d}_collision">
          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
          <geometry><box><size>{size_x:.4f} {size_y:.4f} {size_z:.4f}</size></box></geometry>
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
        <visual name="terrain_tile_{index:04d}_visual">
          <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
          <geometry><box><size>{size_x:.4f} {size_y:.4f} {size_z:.4f}</size></box></geometry>
          <material>
            <ambient>{ambient[0]:.3f} {ambient[1]:.3f} {ambient[2]:.3f} 1</ambient>
            <diffuse>{diffuse[0]:.3f} {diffuse[1]:.3f} {diffuse[2]:.3f} 1</diffuse>
            <specular>0.025 0.023 0.020 1</specular>
          </material>
        </visual>"""


def terrain_tiles_xml(grid_x: np.ndarray, grid_y: np.ndarray, height: np.ndarray) -> str:
    bottom = -0.08
    overlap = 0.01
    tiles = []
    index = 0
    for row in range(height.shape[0] - 1):
        for col in range(height.shape[1] - 1):
            top = float(
                np.mean(
                    height[row : row + 2, col : col + 2],
                    dtype=np.float64,
                )
            )
            size_z = max(top - bottom, 0.02)
            x = float((grid_x[row, col] + grid_x[row, col + 1]) * 0.5)
            y = float((grid_y[row, col] + grid_y[row + 1, col]) * 0.5)
            size_x = abs(float(grid_x[row, col + 1] - grid_x[row, col])) + overlap
            size_y = abs(float(grid_y[row + 1, col] - grid_y[row, col])) + overlap
            z = bottom + size_z * 0.5
            tiles.append(terrain_tile_xml(index, x, y, z, size_x, size_y, size_z, top))
            index += 1
    return "\n".join(tiles)


def write_terrain_assets(model_dir: Path) -> tuple[float, float]:
    texture_dir = model_dir / "materials" / "textures"
    texture_dir.mkdir(parents=True, exist_ok=True)

    grid_x, grid_y, height = generate_terrain_grid()
    cv2.imwrite(
        str(texture_dir / "image_cave_tile_height_preview.png"),
        height_to_preview_pixel(height),
    )
    height_range = (float(height.min()), float(height.max()))

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
    <link name="terrain_link">{terrain_tiles_xml(grid_x, grid_y, height)}
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
  <description>Deterministic sharp sandstone-style tiled terrain for the generated image cave world.</description>
</model>
"""
    (model_dir / "model.config").write_text(model_config, encoding="utf-8")
    return height_range


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=Path, default=DEFAULT_IMAGE)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "worlds" / f"{WORLD_NAME}.sdf",
    )
    parser.add_argument("--terrain-model-dir", type=Path, default=terrain_model_dir())
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
    terrain_min, terrain_max = write_terrain_assets(args.terrain_model_dir)
    args.output.write_text(world_xml(walls), encoding="utf-8")
    if args.overlay is not None:
        write_overlay(image, points, edges, args.overlay)
    print(
        f"wrote {args.output} from {len(points)} points and {len(edges)} wall segments; "
        f"channel_width={channel_width_px:.2f}px scale={meters_per_px:.5f}m/px; "
        f"terrain_height=[{terrain_min:.3f},{terrain_max:.3f}]m"
    )


if __name__ == "__main__":
    main()
