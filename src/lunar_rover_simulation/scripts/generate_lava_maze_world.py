#!/usr/bin/env python3
"""Generate a lava-tube world from a 2D grid topology.

This is method 4: grid maze + cave styling. The generator first defines clear
traversable cells, then only emits wall segments along free/blocked boundaries.
That prevents the branch-overlap artifacts seen in centerline extrusion.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


CELL = 1.05
WALL_THICKNESS = 0.16
WALL_HEIGHT = 0.92
FLOOR_Z = -0.13
FLOOR_THICKNESS = 0.05
ORIGIN_X = -2.0 * CELL
ORIGIN_Y = -2.0 * CELL


FREE = {
    (0, 2), (1, 2), (2, 2), (3, 2), (4, 2), (5, 2), (6, 2),
    (2, 3), (2, 4), (3, 4), (4, 4),
    (4, 3),
    (4, 1), (5, 1), (6, 1), (6, 2),
    (6, 3), (6, 4),
    (1, 1), (2, 1),
    (5, 3),
}
COLS = 7
ROWS = 5


@dataclass(frozen=True)
class Box:
    name: str
    x: float
    y: float
    z: float
    yaw: float
    sx: float
    sy: float
    sz: float
    material: str
    transparency: float | None = None


def cell_center(cell: tuple[int, int]) -> tuple[float, float]:
    col, row = cell
    return ORIGIN_X + col * CELL, ORIGIN_Y + row * CELL


def material_xml(name: str) -> str:
    colors = {
        "base": ("0.42 0.41 0.38 1", "0.58 0.56 0.50 1"),
        "floor": ("0.31 0.30 0.27 1", "0.47 0.44 0.36 1"),
        "wall": ("0.22 0.21 0.20 0.82", "0.32 0.30 0.27 0.82"),
        "wall_dark": ("0.17 0.17 0.16 0.82", "0.26 0.25 0.23 0.82"),
        "rock": ("0.18 0.17 0.16 1", "0.27 0.25 0.22 1"),
        "target_red": ("0.85 0.10 0.06 1", "1.00 0.14 0.08 1"),
        "target_blue": ("0.08 0.20 0.90 1", "0.12 0.30 1.00 1"),
        "target_yellow": ("0.90 0.72 0.10 1", "1.00 0.82 0.14 1"),
        "zone": ("0.10 0.28 0.12 0.65", "0.16 0.60 0.18 0.65"),
    }
    ambient, diffuse = colors[name]
    return (
        "<material>"
        f"<ambient>{ambient}</ambient>"
        f"<diffuse>{diffuse}</diffuse>"
        "<specular>0.02 0.02 0.02 1</specular>"
        "</material>"
    )


def box_model(box: Box) -> str:
    transparency = (
        f"\n          <transparency>{box.transparency:.2f}</transparency>"
        if box.transparency is not None
        else ""
    )
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
          {material_xml(box.material)}{transparency}
        </visual>
      </link>
    </model>"""


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


def merge_intervals(values: list[int]) -> list[tuple[int, int]]:
    if not values:
        return []
    values = sorted(values)
    intervals = []
    start = prev = values[0]
    for value in values[1:]:
        if value == prev + 1:
            prev = value
        else:
            intervals.append((start, prev))
            start = prev = value
    intervals.append((start, prev))
    return intervals


def wall_boxes() -> list[Box]:
    vertical_edges: dict[int, list[int]] = {}
    horizontal_edges: dict[int, list[int]] = {}
    for col, row in FREE:
        for edge_col, neighbor in ((col, (col - 1, row)), (col + 1, (col + 1, row))):
            if neighbor not in FREE:
                vertical_edges.setdefault(edge_col, []).append(row)
        for edge_row, neighbor in ((row, (col, row - 1)), (row + 1, (col, row + 1))):
            if neighbor not in FREE:
                horizontal_edges.setdefault(edge_row, []).append(col)

    boxes: list[Box] = []
    wall_z = FLOOR_Z + FLOOR_THICKNESS * 0.5 + WALL_HEIGHT * 0.5
    index = 0
    for edge_col, rows in sorted(vertical_edges.items()):
        for row0, row1 in merge_intervals(rows):
            length = (row1 - row0 + 1) * CELL + 0.04
            x = ORIGIN_X + (edge_col - 0.5) * CELL
            y = ORIGIN_Y + ((row0 + row1) * 0.5) * CELL
            boxes.append(
                Box(
                    f"lava_wall_v_{index:02d}",
                    x,
                    y,
                    wall_z,
                    0.0,
                    WALL_THICKNESS,
                    length,
                    WALL_HEIGHT,
                    "wall" if index % 2 == 0 else "wall_dark",
                    0.18,
                )
            )
            index += 1
    for edge_row, cols in sorted(horizontal_edges.items()):
        for col0, col1 in merge_intervals(cols):
            length = (col1 - col0 + 1) * CELL + 0.04
            x = ORIGIN_X + ((col0 + col1) * 0.5) * CELL
            y = ORIGIN_Y + (edge_row - 0.5) * CELL
            boxes.append(
                Box(
                    f"lava_wall_h_{index:02d}",
                    x,
                    y,
                    wall_z,
                    0.0,
                    length,
                    WALL_THICKNESS,
                    WALL_HEIGHT,
                    "wall" if index % 2 == 0 else "wall_dark",
                    0.18,
                )
            )
            index += 1
    return boxes


def world_xml() -> str:
    floor_boxes = [
        Box("base_pad", -3.85, 0.0, -0.025, 0.0, 1.50, 1.50, 0.05, "base"),
        Box("dropoff_zone", -3.85, -0.45, 0.006, 0.0, 0.50, 0.50, 0.012, "zone", 0.35),
        Box("entry_ramp", -2.90, 0.0, -0.077, 0.0, 1.00, 1.10, 0.05, "floor"),
    ]
    for col, row in FREE:
        x, y = cell_center((col, row))
        floor_boxes.append(
            Box(f"floor_{col}_{row}", x, y, FLOOR_Z, 0.0, CELL + 0.04, CELL + 0.04, FLOOR_THICKNESS, "floor")
        )

    rocks = [
        (-0.95, 0.15, 0.052),
        (0.15, -0.26, 0.056),
        (1.15, 0.28, 0.060),
        (2.20, -0.45, 0.050),
        (3.20, 0.36, 0.058),
        (4.25, 0.18, 0.054),
        (2.25, 1.30, 0.050),
    ]
    targets = [
        Box("science_target_left_deadend", -0.95, 2.10, FLOOR_Z + 0.055, 0.25, 0.05, 0.05, 0.05, "target_red"),
        Box("science_target_loop", 4.25, -1.05, FLOOR_Z + 0.055, -0.45, 0.05, 0.05, 0.05, "target_blue"),
        Box("science_target_wall_platform", 2.25, -0.48, FLOOR_Z + 0.32, 0.10, 0.05, 0.05, 0.05, "target_yellow"),
    ]
    models = "\n".join(box_model(box) for box in floor_boxes + wall_boxes() + targets)
    rock_models = "\n".join(
        sphere_model(f"rock_{i:02d}", x, y, FLOOR_Z + FLOOR_THICKNESS * 0.5 + radius * 0.65, radius)
        for i, (x, y, radius) in enumerate(rocks)
    )
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
      <ambient>0.14 0.14 0.15 1</ambient>
      <background>0.035 0.038 0.043 1</background>
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
      <pose>-3.5 0 2.4 0 0 0</pose>
      <diffuse>0.30 0.31 0.32 1</diffuse>
      <specular>0.02 0.02 0.02 1</specular>
      <attenuation><range>7</range><constant>0.7</constant><linear>0.08</linear><quadratic>0.02</quadratic></attenuation>
    </light>
{models}
{rock_models}
  </world>
</sdf>
"""


def main() -> None:
    out = Path(__file__).resolve().parents[1] / "worlds" / "lava_tube.sdf"
    out.write_text(world_xml(), encoding="utf-8")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
