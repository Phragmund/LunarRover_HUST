# Lunar Rover Navigation

This package adds `/terrain/traversability` to both Nav2 costmaps through
`lunar_rover_navigation::TerrainLayer`. Unknown terrain is lethal by default,
values 0 through 99 are scaled into graded Nav2 costs, and value 100 is
lethal.

The supplied Humble configuration uses SmacPlanner2D, Regulated Pure Pursuit,
VoxelLayer, InflationLayer, velocity smoothing, and collision monitoring.
Nominal speed is 0.25 m/s. Terrain and proximity costs reduce commanded speed
toward 0.08 m/s.

The velocity chain terminates at `/cmd_vel_safe` as
`geometry_msgs/msg/Twist`. The rover safety supervisor owns the final
conversion and 20 Hz publication of `/chassis/cmd_vel` as
`geometry_msgs/msg/TwistStamped`.
