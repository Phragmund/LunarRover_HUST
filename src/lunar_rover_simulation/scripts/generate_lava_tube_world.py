#!/usr/bin/env python3
"""Generate an open-top lava-tube world from 2D centerline corridors.

The geometry is intentionally procedural and deterministic so SLAM/navigation
regression tests can reuse the same cave while we iterate on algorithms.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from pathlib import Path


SEED = 20260728
WALL_HEIGHT = 0.95
WALL_THICKNESS = 0.16
FLOOR_Z = -0.13
FLOOR_THICKNESS = 0.05
BASE_X = -4.25
BASE_Y = 0.0


@dataclass(frozen=True)
class Segment:
    x0: float
    y0: float
    x1: float
    y1: float
    width: float
    name: str


@dataclass(frozen=True)
class PoseBox:
    name: str
    x: float
    y: float
    z: float
    yaw: float
    sx: float
    sy: float
    sz: float
    material: str


def material_xml(name: str) -> str:
    materials = {
        "floor": ("0.31 0.30 0.27 1", "0.47 0.44 0.36 1"),
        "base": ("0.42 0.41 0.38 1", "0.58 0.56 0.50 1"),
        "wall_a": ("0.22 0.21 0.20 1", "0.31 0.30 0.27 1"),
        "wall_b": ("0.18 0.18 0.17 1", "0.27 0.26 0.24 1"),
        "rock": ("0.18 0.17 0.16 1", "0.27 0.25 0.22 1"),
        "target_red": ("0.85 0.10 0.06 1", "1.00 0.14 0.08 1"),
        "target_blue": ("0.08 0.20 0.90 1", "0.12 0.30 1.00 1"),
        "target_yellow": ("0.90 0.72 0.10 1", "1.00 0.82 0.14 1"),
    }
    ambient, diffuse = materials[name]
    return (
        "<material>"
        f"<ambient>{ambient}</ambient>"
        f"<diffuse>{diffuse}</diffuse>"
        "<specular>0.02 0.02 0.02 1</specular>"
        "</material>"
    )


def box_model(box: PoseBox) -> str:
    return f"""
    <model name="{box.name}">
      <static>true</static>
      <link name="{box.name}_link">
        <collision name="{box.name}_collision">
          <pose>{box.x:.3f} {box.y:.3f} {box.z:.3f} 0 0 {box.yaw:.3f}</pose>
          <geometry><box><size>{box.sx:.3f} {box.sy:.3f} {box.sz:.3f}</size></box></geometry>
        </collision>
        <visual name="{box.name}_visual">
          <pose>{box.x:.3f} {box.y:.3f} {box.z:.3f} 0 0 {box.yaw:.3f}</pose>
          <geometry><box><size>{box.sx:.3f} {box.sy:.3f} {box.sz:.3f}</size></box></geometry>
          {material_xml(box.material)}
        </visual>
      </link>
    </model>"""


def segment_pose(segment: Segment) -> tuple[float, float, float, float]:
    dx = segment.x1 - segment.x0
    dy = segment.y1 - segment.y0
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    return (segment.x0 + segment.x1) * 0.5, (segment.y0 + segment.y1) * 0.5, length, yaw


def corridor_floor(segment: Segment, index: int) -> PoseBox:
    x, y, length, yaw = segment_pose(segment)
    return PoseBox(
        f"floor_{index:02d}_{segment.name}",
        x,
        y,
        FLOOR_Z,
        yaw,
        length + 0.18,
        segment.width + 0.12,
        FLOOR_THICKNESS,
        "floor",
    )


def corridor_walls(segment: Segment, index: int, rng: random.Random) -> list[PoseBox]:
    x, y, length, yaw = segment_pose(segment)
    nx = -math.sin(yaw)
    ny = math.cos(yaw)
    wall_z = FLOOR_Z + FLOOR_THICKNESS * 0.5 + WALL_HEIGHT * 0.5
    boxes: list[PoseBox] = []
    wall_count = max(2, math.ceil(length / 0.85))
    step = length / wall_count
    direction_x = math.cos(yaw)
    direction_y = math.sin(yaw)
    start_x = x - direction_x * length * 0.5
    start_y = y - direction_y * length * 0.5
    for side, sign in (("left", 1.0), ("right", -1.0)):
        for part in range(wall_count):
            center_along = (part + 0.5) * step
            jitter_offset = rng.uniform(-0.045, 0.045)
            jitter_yaw = rng.uniform(-0.07, 0.07)
            thickness = WALL_THICKNESS + rng.uniform(-0.025, 0.035)
            height = WALL_HEIGHT + rng.uniform(-0.10, 0.10)
            cx = start_x + direction_x * center_along + nx * sign * (segment.width * 0.5 + thickness * 0.5 + jitter_offset)
            cy = start_y + direction_y * center_along + ny * sign * (segment.width * 0.5 + thickness * 0.5 + jitter_offset)
            boxes.append(
                PoseBox(
                    f"wall_{index:02d}_{side}_{part:02d}_{segment.name}",
                    cx,
                    cy,
                    FLOOR_Z + FLOOR_THICKNESS * 0.5 + height * 0.5,
                    yaw + jitter_yaw,
                    step + 0.10,
                    thickness,
                    height,
                    "wall_a" if (index + part) % 2 == 0 else "wall_b",
                )
            )
    return boxes


def sphere_model(name: str, x: float, y: float, z: float, radius: float) -> str:
    return f"""
    <model name="{name}">
      <static>true</static>
      <link name="{name}_link">
        <collision name="{name}_collision">
          <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
          <geometry><sphere><radius>{radius:.3f}</radius></sphere></geometry>
        </collision>
        <visual name="{name}_visual">
          <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
          <geometry><sphere><radius>{radius:.3f}</radius></sphere></geometry>
          {material_xml("rock")}
        </visual>
      </link>
    </model>"""


def world_xml() -> str:
    rng = random.Random(SEED)
    segments = [
        Segment(-3.35, 0.00, -2.35, 0.00, 1.08, "entry_ramp"),
        Segment(-2.35, 0.00, -1.15, 0.10, 1.05, "main_a"),
        Segment(-1.15, 0.10, 0.05, -0.18, 1.02, "main_b"),
        Segment(0.05, -0.18, 1.30, -0.05, 0.95, "narrow"),
        Segment(1.30, -0.05, 2.55, 0.28, 1.05, "main_c"),
        Segment(2.55, 0.28, 3.80, 0.12, 1.12, "main_d"),
        Segment(3.80, 0.12, 4.75, 0.40, 1.30, "terminal_chamber"),
        Segment(-0.60, 0.02, -0.85, 1.05, 0.92, "left_branch"),
        Segment(-0.85, 1.05, 0.20, 1.35, 0.90, "left_deadend"),
        Segment(1.65, 0.02, 1.85, -1.00, 0.95, "right_branch"),
        Segment(1.85, -1.00, 3.05, -0.92, 0.92, "right_loop_a"),
        Segment(3.05, -0.92, 3.80, 0.12, 0.90, "right_loop_b"),
    ]

    boxes: list[PoseBox] = [
        PoseBox("base_pad", BASE_X, BASE_Y, -0.025, 0.0, 1.50, 1.50, 0.05, "base"),
        PoseBox("dropoff_zone", BASE_X, -0.45, 0.006, 0.0, 0.50, 0.50, 0.012, "target_yellow"),
    ]
    for index, segment in enumerate(segments):
        boxes.append(corridor_floor(segment, index))
        boxes.extend(corridor_walls(segment, index, rng))

    rocks = [
        (-1.80, 0.30, 0.050),
        (-0.30, -0.35, 0.055),
        (0.85, 0.22, 0.060),
        (1.45, -0.42, 0.045),
        (2.30, 0.55, 0.055),
        (2.55, -0.74, 0.060),
        (3.50, 0.22, 0.050),
        (4.25, 0.60, 0.055),
    ]
    rock_xml = "\n".join(
        sphere_model(f"rock_{i:02d}", x, y, FLOOR_Z + FLOOR_THICKNESS * 0.5 + r * 0.62, r)
        for i, (x, y, r) in enumerate(rocks)
    )
    targets = [
        PoseBox("science_target_floor_red", 0.05, 1.10, FLOOR_Z + 0.055, 0.25, 0.05, 0.05, 0.05, "target_red"),
        PoseBox("science_target_floor_blue", 4.30, 0.22, FLOOR_Z + 0.055, -0.45, 0.05, 0.05, 0.05, "target_blue"),
        PoseBox("science_target_wall_yellow", 2.00, -0.49, FLOOR_Z + 0.30, 0.10, 0.05, 0.05, 0.05, "target_yellow"),
    ]
    all_boxes_xml = "\n".join(box_model(box) for box in boxes + targets)
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="lava_tube">
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
      <ambient>0.12 0.12 0.13 1</ambient>
      <background>0.03 0.035 0.04 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="low_sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>-6 -3 5 0 0 0</pose>
      <diffuse>0.95 0.88 0.72 1</diffuse>
      <specular>0.18 0.16 0.12 1</specular>
      <direction>0.95 0.15 -0.22</direction>
    </light>
    <light name="observer_fill" type="point">
      <cast_shadows>false</cast_shadows>
      <pose>-4.2 0 2.2 0 0 0</pose>
      <diffuse>0.25 0.26 0.28 1</diffuse>
      <specular>0.02 0.02 0.02 1</specular>
      <attenuation><range>6</range><constant>0.7</constant><linear>0.08</linear><quadratic>0.02</quadratic></attenuation>
    </light>
{all_boxes_xml}
{rock_xml}
  </world>
</sdf>
"""


def main() -> None:
    out = Path(__file__).resolve().parents[1] / "worlds" / "lava_tube.sdf"
    out.write_text(world_xml(), encoding="utf-8")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
