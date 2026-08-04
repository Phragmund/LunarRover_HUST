# Simulation Worlds

The default simulation test world is `image_cave`.

`image_cave` is the main navigation and driving test map. It contains the
generated cave wall topology from the red-point reference image, plus the
current triangular pseudo-surface terrain:

- 186 static wall segments generated from the annotated cave graph.
- A watertight triangular terrain mesh raised around 1 m above the world origin.
- Terrain triangle edge length: 0.2 m.
- Terrain height range: 0.9 m to 1.1 m.
- Terrain visual mesh: `image_cave_heightmap_terrain/meshes/sandstone_terrain_faceted.dae`.
- Terrain collision mesh: `image_cave_heightmap_terrain/meshes/sandstone_terrain_closed.stl`.
- Walls model: `image_cave_walls`.
- Surface block model: `image_cave_surface_blocks`.
- Surface block count: 1200.
- Surface block size: 0.10 m x 0.13 m x 0.05 m.
- Surface block rotation: random yaw, random roll/pitch within 5 deg.
- Surface block height constraint: each accepted block is placed on sampled
  terrain only if its rotated top vertex stays below the terrain height range
  midpoint plus 0.01 m. With the current terrain this cap is about 1.01 m.
  The generated top vertices are evenly distributed across 20 bins in the
  1.00 m to 1.01 m interval.
- Default rover spawn: `x=-3.92`, `y=0.0`, `z=1.45`, `yaw=0.0`.

Use the full default bringup with:

```bash
ros2 launch lunar_rover_bringup bringup.launch.py \
  mode:=sim gui:=true rviz:=true sensor_noise:=false
```

Equivalent explicit world selection:

```bash
ros2 launch lunar_rover_bringup bringup.launch.py \
  mode:=sim world:=image_cave gui:=true rviz:=true sensor_noise:=false
```

`terrain_preview` is a lightweight Gazebo-only preview world for inspecting the
same terrain and wall models without spawning the rover or ROS controllers:

```bash
export IGN_GAZEBO_RESOURCE_PATH=/home/phragmund/LunarRover/install/lunar_rover_simulation/share/lunar_rover_simulation/models:/home/phragmund/LunarRover/src/lunar_rover_simulation/models
export GZ_SIM_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH
ign gazebo -r -v 2 /home/phragmund/LunarRover/src/lunar_rover_simulation/worlds/terrain_preview.sdf
```

Other available worlds:

| World | Contents | Typical use |
| --- | --- | --- |
| `flat` | Flat lunar ground plane. | Sensor, controller, and spawn smoke tests. |
| `steps` | Flat ground plus step-course geometry. | Suspension and step traversal tests. |
| `rocks` | Flat ground plus scattered small and medium rocks. | Obstacle contact and perception checks. |
| `mixed` | Combined basic terrain elements. | General low-risk integration testing. |
| `stairs_maze` | Maze-like walls plus stair/step features. | Structured navigation and obstacle handling. |
| `reachable_maze` | Generated maze with guaranteed reachable corridors. | Mapping and path-planning regression tests. |
| `lava_tube` | Procedural lava-tube corridor world. | Cave-like sensor and exploration tests. |
| `image_cave` | Reference-image cave topology with triangular terrain. | Default full-system test map. |
| `terrain_preview` | Same cave terrain and wall models, no rover required. | Fast visual inspection in Gazebo. |

Regenerate the image cave world and its reusable terrain/wall models with:

```bash
python3 src/lunar_rover_simulation/scripts/generate_image_cave_world.py
colcon build --symlink-install --packages-select lunar_rover_simulation
```
