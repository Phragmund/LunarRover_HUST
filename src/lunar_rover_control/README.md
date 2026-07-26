# lunar_rover_control

ROS 2 control and wheel-odometry package for a four-wheel-steering,
six-wheel-drive rocker-bogie rover.

## Runtime contract

- Input: `/chassis/cmd_vel` (`geometry_msgs/msg/TwistStamped`), received at 20 Hz.
- Hardware commands: four steering `position` interfaces followed by six wheel
  `velocity` interfaces.
- Feedback: `/joint_states`, including all steering positions and wheel velocities.
- Wheel odometry: `/wheel/odometry` (`nav_msgs/msg/Odometry`). It does not publish
  TF by default because the localization filter owns `odom -> base_footprint`.
- Simulated lower-controller state: `/chassis/state`
  (`lunar_rover_msgs/msg/ChassisState`).

The rover controller independently stops all drive interfaces after 250 ms
without a valid command. The simulated state node reports the same watchdog
condition. The physical lower controller must implement the same timeout.

Array order is fixed:

- Steering: front-left, front-right, rear-left, rear-right.
- Wheels: front-left, front-right, middle-left, middle-right, rear-left,
  rear-right.

## Controllers

`lunar_rover_control/RoverController` computes each wheel contact velocity from
the commanded planar twist. It minimizes steering travel by reversing a wheel
when appropriate, enforces steering, wheel-speed, and acceleration limits, and
holds drive speed at zero until steering is aligned.

`lunar_rover_control/RockerDifferentialController` applies a bounded spring and
damper effort to enforce:

```text
left_rocker + coupling_sign * right_rocker = equilibrium_offset
```

Set `coupling_sign` from the joint-axis convention in the robot description.
