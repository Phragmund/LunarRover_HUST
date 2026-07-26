# Lunar Rover Perception

This package fuses the standard Mid-360 and Gemini 335 `PointCloud2`
streams into a robot-centered 2.5D terrain map.

## Outputs

| Topic | Type | Meaning |
| --- | --- | --- |
| `/terrain/grid_map` | `grid_map_msgs/msg/GridMap` | Elevation, variance, slope, roughness, step, confidence and traversability layers |
| `/terrain/traversability` | `nav_msgs/msg/OccupancyGrid` | `-1` unknown, `0..99` increasing terrain cost, `100` impassable |
| `/terrain/ground` | `sensor_msgs/msg/PointCloud2` | Representative centers of traversable cells |
| `/terrain/obstacles` | `sensor_msgs/msg/PointCloud2` | Representative centers of lethal cells |

The default caution/lethal step thresholds are 0.05/0.07 m. The lethal
slope threshold is 22 degrees; the `slope` grid-map layer is in radians.
Observations are stored in world-indexed
cells, age out after 1.5 seconds, and are cropped to the rolling map before
each publication.
