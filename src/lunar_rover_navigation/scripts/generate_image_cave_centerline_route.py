#!/usr/bin/env python3
"""Generate a closed image_cave centerline waypoint route for Nav2."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import sys
from pathlib import Path

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ROUTE = (
    REPO_ROOT
    / "src"
    / "lunar_rover_navigation"
    / "routes"
    / "image_cave_centerline_loop.csv"
)
DEFAULT_PREVIEW = REPO_ROOT / "tmp" / "image_cave_centerline_loop_preview.png"


def load_cave_generator():
    script = (
        REPO_ROOT
        / "src"
        / "lunar_rover_simulation"
        / "scripts"
        / "generate_image_cave_world.py"
    )
    spec = importlib.util.spec_from_file_location("image_cave_generator", script)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load {script}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def world_to_map(
    x_world: float,
    y_world: float,
    yaw_world: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
) -> tuple[float, float, float]:
    dx = x_world - origin_x
    dy = y_world - origin_y
    c = math.cos(-origin_yaw)
    s = math.sin(-origin_yaw)
    return (
        c * dx - s * dy,
        s * dx + c * dy,
        normalize_angle(yaw_world - origin_yaw),
    )


def skeletonize_corridor(gen):
    image = cv2.imread(str(gen.DEFAULT_IMAGE), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(gen.DEFAULT_IMAGE)
    points = gen.red_points(image)
    edges = gen.extract_edges(image, points)
    channel_width_px = gen.estimate_channel_width_px(points, edges)
    meters_per_px = gen.WALL_CENTER_WIDTH_SCALE / channel_width_px
    walls = gen.build_walls(points, edges, meters_per_px)
    grid_x, grid_y, _height = gen.generate_terrain_grid()
    mask = gen.build_corridor_mask(walls, grid_x, grid_y)
    accessible = (mask.accessible.astype(np.uint8) * 255)
    accessible = cv2.morphologyEx(
        accessible, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1
    )
    skeleton = cv2.ximgproc.thinning(
        accessible, thinningType=cv2.ximgproc.THINNING_ZHANGSUEN
    )
    distance = (
        cv2.distanceTransform((accessible > 0).astype(np.uint8), cv2.DIST_L2, 5)
        * mask.resolution
    )
    return walls, mask, accessible, skeleton, distance


def skeleton_adjacency(skeleton: np.ndarray):
    rows, cols = skeleton.shape
    pixels = [tuple(item) for item in np.argwhere(skeleton > 0)]
    pixel_set = set(pixels)
    adjacency: dict[tuple[int, int], list[tuple[int, int]]] = {}
    for row, col in pixels:
        neighbors = []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr = row + dr
                nc = col + dc
                if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) in pixel_set:
                    neighbors.append((nr, nc))
        adjacency[(row, col)] = neighbors
    return adjacency


def nearest_skeleton_pixel(
    skeleton: np.ndarray, mask, x_world: float, y_world: float
) -> tuple[int, int]:
    rows, cols = np.nonzero(skeleton > 0)
    if len(rows) == 0:
        raise RuntimeError("Empty cave skeleton")
    x = mask.x_min + cols.astype(np.float64) * mask.resolution
    y = mask.y_min + rows.astype(np.float64) * mask.resolution
    index = int(np.argmin((x - x_world) ** 2 + (y - y_world) ** 2))
    return int(rows[index]), int(cols[index])


def closed_dfs_walk(
    adjacency: dict[tuple[int, int], list[tuple[int, int]]],
    start: tuple[int, int],
) -> list[tuple[int, int]]:
    sys.setrecursionlimit(max(10000, len(adjacency) * 4))
    visited_edges: set[tuple[tuple[int, int], tuple[int, int]]] = set()
    route = [start]

    def edge_key(a: tuple[int, int], b: tuple[int, int]):
        return (a, b) if a <= b else (b, a)

    def visit(node: tuple[int, int], previous: tuple[int, int] | None = None) -> None:
        neighbors = adjacency[node]
        if previous is not None:
            base_angle = math.atan2(node[0] - previous[0], node[1] - previous[1])
        else:
            base_angle = 0.0

        def sort_key(neighbor: tuple[int, int]) -> float:
            angle = math.atan2(neighbor[0] - node[0], neighbor[1] - node[1])
            # Prefer locally smooth continuations before side branches.
            return abs(normalize_angle(angle - base_angle))

        for neighbor in sorted(neighbors, key=sort_key):
            key = edge_key(node, neighbor)
            if key in visited_edges:
                continue
            visited_edges.add(key)
            route.append(neighbor)
            visit(neighbor, node)
            route.append(node)

    visit(start)
    return route


def pixel_to_world(point: tuple[int, int], mask) -> tuple[float, float]:
    row, col = point
    return (
        mask.x_min + float(col) * mask.resolution,
        mask.y_min + float(row) * mask.resolution,
    )


def resample_polyline(points: list[tuple[float, float]], spacing: float):
    if len(points) < 2:
        return points
    result = [points[0]]
    distance_since_last = 0.0
    previous = np.array(points[0], dtype=np.float64)
    for current_tuple in points[1:]:
        current = np.array(current_tuple, dtype=np.float64)
        segment = current - previous
        segment_length = float(np.linalg.norm(segment))
        if segment_length < 1e-9:
            previous = current
            continue
        direction = segment / segment_length
        remaining = segment_length
        cursor = previous.copy()
        while distance_since_last + remaining >= spacing:
            step = spacing - distance_since_last
            cursor = cursor + direction * step
            result.append((float(cursor[0]), float(cursor[1])))
            remaining -= step
            distance_since_last = 0.0
        distance_since_last += remaining
        previous = current
    if math.hypot(result[0][0] - result[-1][0], result[0][1] - result[-1][1]) > 1e-6:
        result.append(result[0])
    return result


def yaw_along(points: list[tuple[float, float]], index: int) -> float:
    if len(points) < 2:
        return 0.0
    current = points[index]
    if index + 1 < len(points):
        other = points[index + 1]
    else:
        other = points[0]
    dx = other[0] - current[0]
    dy = other[1] - current[1]
    if math.hypot(dx, dy) < 1e-6 and index > 0:
        previous = points[index - 1]
        dx = current[0] - previous[0]
        dy = current[1] - previous[1]
    return math.atan2(dy, dx)


def nearest_clearance(x: float, y: float, mask, distance: np.ndarray) -> float:
    col = int(round((x - mask.x_min) / mask.resolution))
    row = int(round((y - mask.y_min) / mask.resolution))
    row = max(0, min(distance.shape[0] - 1, row))
    col = max(0, min(distance.shape[1] - 1, col))
    return float(distance[row, col])


def draw_preview(
    path: Path,
    walls,
    mask,
    accessible: np.ndarray,
    skeleton: np.ndarray,
    route_world: list[tuple[float, float]],
    waypoint_world: list[tuple[float, float]],
    spawn_x: float,
    spawn_y: float,
) -> None:
    image = np.zeros((*accessible.shape, 3), dtype=np.uint8)
    image[accessible > 0] = (52, 52, 52)
    image[skeleton > 0] = (0, 0, 150)

    def pix(x: float, y: float) -> tuple[int, int]:
        return (
            int(round((x - mask.x_min) / mask.resolution)),
            int(round((y - mask.y_min) / mask.resolution)),
        )

    for wall in walls:
        cv2.line(image, pix(wall.x0, wall.y0), pix(wall.x1, wall.y1), (0, 0, 0), 2)

    route_pixels = np.array([pix(x, y) for x, y in route_world], dtype=np.int32)
    if len(route_pixels) > 1:
        cv2.polylines(image, [route_pixels], False, (255, 90, 0), 1, lineType=cv2.LINE_AA)

    for index, (x, y) in enumerate(waypoint_world):
        color = (0, 255, 255) if index % 5 == 0 else (0, 180, 255)
        cv2.circle(image, pix(x, y), 2, color, -1)
    cv2.circle(image, pix(spawn_x, spawn_y), 5, (0, 255, 0), -1)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), cv2.flip(image, 0))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_ROUTE)
    parser.add_argument("--preview", type=Path, default=DEFAULT_PREVIEW)
    parser.add_argument("--spacing", type=float, default=0.8)
    parser.add_argument("--map-origin-x", type=float, default=None)
    parser.add_argument("--map-origin-y", type=float, default=None)
    parser.add_argument("--map-origin-yaw", type=float, default=0.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.spacing <= 0.05:
        raise ValueError("--spacing must be greater than 0.05 m")
    gen = load_cave_generator()
    origin_x = gen.SPAWN_X if args.map_origin_x is None else args.map_origin_x
    origin_y = gen.SPAWN_Y if args.map_origin_y is None else args.map_origin_y
    walls, mask, accessible, skeleton, distance = skeletonize_corridor(gen)
    adjacency = skeleton_adjacency(skeleton)
    start = nearest_skeleton_pixel(skeleton, mask, origin_x, origin_y)
    dense_pixels = closed_dfs_walk(adjacency, start)
    dense_world = [pixel_to_world(point, mask) for point in dense_pixels]
    waypoints_world = resample_polyline(dense_world, args.spacing)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="ascii") as output:
        writer = csv.writer(output)
        writer.writerow(
            [
                "index",
                "x",
                "y",
                "yaw",
                "x_world",
                "y_world",
                "yaw_world",
                "clearance_m",
            ]
        )
        for index, (x_world, y_world) in enumerate(waypoints_world):
            yaw_world = yaw_along(waypoints_world, index)
            x_map, y_map, yaw_map = world_to_map(
                x_world, y_world, yaw_world, origin_x, origin_y, args.map_origin_yaw
            )
            writer.writerow(
                [
                    index,
                    f"{x_map:.4f}",
                    f"{y_map:.4f}",
                    f"{yaw_map:.6f}",
                    f"{x_world:.4f}",
                    f"{y_world:.4f}",
                    f"{yaw_world:.6f}",
                    f"{nearest_clearance(x_world, y_world, mask, distance):.4f}",
                ]
            )

    draw_preview(
        args.preview,
        walls,
        mask,
        accessible,
        skeleton,
        dense_world,
        waypoints_world,
        origin_x,
        origin_y,
    )
    dense_length = sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(dense_world, dense_world[1:])
    )
    print(
        f"wrote {args.output}; waypoints={len(waypoints_world)} "
        f"spacing={args.spacing:.2f}m dense_walk={dense_length:.1f}m "
        f"skeleton_pixels={len(adjacency)} preview={args.preview}"
    )


if __name__ == "__main__":
    main()
