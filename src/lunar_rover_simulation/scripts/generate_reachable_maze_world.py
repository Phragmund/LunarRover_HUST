#!/usr/bin/env python3
"""Generate a deterministic reachable maze world for sensor research."""

from __future__ import annotations

import random
from dataclasses import dataclass
from pathlib import Path


COLS = 8
ROWS = 5
CELL_PITCH = 1.12
WALL_THICKNESS = 0.12
WALL_HEIGHT = 1.0
WALL_TRANSPARENCY = 0.70
OBSTACLE_RADIUS = 0.10
OBSTACLE_COUNT = 180
SEED = 360335


@dataclass(frozen=True)
class WallSegment:
    orientation: str
    index: int
    start: int
    end: int


def generate_maze(cols: int, rows: int, rng: random.Random):
    visited = {(0, rows // 2)}
    stack = [(0, rows // 2)]
    removed_vertical = set()
    removed_horizontal = set()

    while stack:
        x, y = stack[-1]
        candidates = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < cols and 0 <= ny < rows and (nx, ny) not in visited:
                candidates.append((nx, ny))
        if not candidates:
            stack.pop()
            continue

        nx, ny = rng.choice(candidates)
        if nx != x:
            removed_vertical.add((max(x, nx), y))
        else:
            removed_horizontal.add((x, max(y, ny)))
        visited.add((nx, ny))
        stack.append((nx, ny))

    return removed_vertical, removed_horizontal


def merge_wall_segments(cols: int, rows: int, removed_vertical, removed_horizontal):
    segments = []

    for vx in range(cols + 1):
        y = 0
        while y < rows:
            is_outer = vx in (0, cols)
            has_wall = is_outer or (vx, y) not in removed_vertical
            if not has_wall:
                y += 1
                continue
            start = y
            while y < rows:
                is_outer = vx in (0, cols)
                has_wall = is_outer or (vx, y) not in removed_vertical
                if not has_wall:
                    break
                y += 1
            segments.append(WallSegment("vertical", vx, start, y))

    for hy in range(rows + 1):
        x = 0
        while x < cols:
            is_outer = hy in (0, rows)
            has_wall = is_outer or (x, hy) not in removed_horizontal
            if not has_wall:
                x += 1
                continue
            start = x
            while x < cols:
                is_outer = hy in (0, rows)
                has_wall = is_outer or (x, hy) not in removed_horizontal
                if not has_wall:
                    break
                x += 1
            segments.append(WallSegment("horizontal", hy, start, x))

    return segments


def wall_xml(segment: WallSegment, idx: int) -> str:
    min_x = -COLS * CELL_PITCH / 2.0
    min_y = -ROWS * CELL_PITCH / 2.0

    if segment.orientation == "vertical":
        x = min_x + segment.index * CELL_PITCH
        y0 = min_y + segment.start * CELL_PITCH
        y1 = min_y + segment.end * CELL_PITCH
        cx = x
        cy = (y0 + y1) / 2.0
        sx = WALL_THICKNESS
        sy = max(y1 - y0 + WALL_THICKNESS, WALL_THICKNESS)
    else:
        y = min_y + segment.index * CELL_PITCH
        x0 = min_x + segment.start * CELL_PITCH
        x1 = min_x + segment.end * CELL_PITCH
        cx = (x0 + x1) / 2.0
        cy = y
        sx = max(x1 - x0 + WALL_THICKNESS, WALL_THICKNESS)
        sy = WALL_THICKNESS

    return f"""
        <collision name="maze_wall_{idx:03d}_collision">
          <pose>{cx:.3f} {cy:.3f} {WALL_HEIGHT / 2.0:.3f} 0 0 0</pose>
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_HEIGHT:.3f}</size></box></geometry>
        </collision>
        <visual name="maze_wall_{idx:03d}_visual">
          <pose>{cx:.3f} {cy:.3f} {WALL_HEIGHT / 2.0:.3f} 0 0 0</pose>
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_HEIGHT:.3f}</size></box></geometry>
          <material>
            <ambient>0.30 0.31 0.30 0.30</ambient>
            <diffuse>0.44 0.45 0.42 0.30</diffuse>
            <specular>0.08 0.08 0.07 1</specular>
          </material>
          <transparency>{WALL_TRANSPARENCY:.2f}</transparency>
        </visual>"""


def obstacle_xml(x: float, y: float, idx: int) -> str:
    return f"""
        <collision name="hemisphere_obstacle_{idx:03d}_collision">
          <pose>{x:.3f} {y:.3f} 0.000 0 0 0</pose>
          <geometry><sphere><radius>{OBSTACLE_RADIUS:.3f}</radius></sphere></geometry>
        </collision>
        <visual name="hemisphere_obstacle_{idx:03d}_visual">
          <pose>{x:.3f} {y:.3f} 0.000 0 0 0</pose>
          <geometry><sphere><radius>{OBSTACLE_RADIUS:.3f}</radius></sphere></geometry>
          <material>
            <ambient>0.25 0.25 0.24 1</ambient>
            <diffuse>0.37 0.36 0.33 1</diffuse>
            <specular>0.05 0.05 0.04 1</specular>
          </material>
        </visual>"""


def generate_obstacles(rng: random.Random):
    min_x = -COLS * CELL_PITCH / 2.0 + 0.25
    max_x = COLS * CELL_PITCH / 2.0 - 0.25
    min_y = -ROWS * CELL_PITCH / 2.0 + 0.25
    max_y = ROWS * CELL_PITCH / 2.0 - 0.25
    obstacles = []
    start_x = -COLS * CELL_PITCH / 2.0 + CELL_PITCH / 2.0
    start_y = 0.0

    for _ in range(OBSTACLE_COUNT):
        for _attempt in range(50):
            x = rng.uniform(min_x, max_x)
            y = rng.uniform(min_y, max_y)
            if (x - start_x) ** 2 + (y - start_y) ** 2 > 0.75**2:
                obstacles.append((x, y))
                break

    return obstacles


def build_world() -> str:
    rng = random.Random(SEED)
    removed_vertical, removed_horizontal = generate_maze(COLS, ROWS, rng)
    walls = merge_wall_segments(COLS, ROWS, removed_vertical, removed_horizontal)
    obstacles = generate_obstacles(rng)

    walls_xml = "\n".join(wall_xml(segment, idx) for idx, segment in enumerate(walls))
    obstacles_xml = "\n".join(
        obstacle_xml(x, y, idx) for idx, (x, y) in enumerate(obstacles)
    )

    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="reachable_maze">
    <gravity>0 0 -9.80665</gravity>
    <physics name="physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="libignition-gazebo-imu-system.so" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="libignition-gazebo-contact-system.so" name="ignition::gazebo::systems::Contact"/>

    <scene>
      <ambient>0.36 0.36 0.35 1</ambient>
      <background>0.08 0.09 0.11 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.92 0.91 0.86 1</diffuse>
      <specular>0.20 0.20 0.18 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.45 0.25 -0.86</direction>
    </light>

    <include>
      <uri>model://lunar_ground</uri>
    </include>

    <model name="generated_reachable_maze">
      <static>true</static>
      <link name="maze_link">{walls_xml}
      </link>
    </model>

    <model name="maze_random_hemispheres">
      <static>true</static>
      <link name="obstacles_link">{obstacles_xml}
      </link>
    </model>
  </world>
</sdf>
"""


def main() -> None:
    out = Path(__file__).resolve().parents[1] / "worlds" / "reachable_maze.sdf"
    out.write_text(build_world(), encoding="utf-8")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
