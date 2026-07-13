# Quad-X Upstream Trajectory-Follower Angle-Mode Experiment

This sibling experiment keeps the upstream ROScopter trajectory follower and
controller while adapting their boundary to ROSflight firmware angle mode.
It does not modify upstream ROScopter or `rosflight_io`.

```text
path_planner
    | /waypoints
path_manager
    | /trajectory_command
trajectory_velocity_adapter.py
    | /trajectory_command_compensated
upstream trajectory_follower
    | /high_level_command_thrust
    | mode 10: roll, pitch, yaw-rate, thrust N
thrust_to_throttle_adapter.py
    | /high_level_command
    | mode 6: roll, pitch, yaw-rate, normalized throttle
upstream controller
    | /command
    | ROSflight mode 2
rosflight_io -> Veloxity
```

The velocity adapter compensates for the upstream follower not consuming the
velocity field in `TrajectoryCommand`. It advances the follower-only position
reference by `kd / kp * commanded_velocity` on the north, east, and down axes,
making its translational PID equivalent to a position-and-velocity tracking
loop. The original `/trajectory_command` remains unchanged for plotting and
analysis.

The thrust adapter applies:

```text
throttle = thrust_N / (mass * gravity) * equilibrium_throttle
```

The `mass` and `gravity` values come from the same upstream `multirotor.yaml`
used by the trajectory follower and controller. The experiment overrides the
adapter's equilibrium throttle with the measured Veloxity simulation hover
value, 0.686, so hover thrust maps to the actual normalized hover command.
`upstream_angle_baseline.yaml` supplies experiment-local trajectory and command
limits. It reduces path-manager velocity/acceleration, adds overdamped
lateral damping and velocity feed-forward, and clamps the converted command to
0.30 rad roll/pitch, 0.70 rad/s yaw rate, and 0.40--0.85 throttle.

## Run

From the Veloxity repository root, first source ROS 2 and the caller-provided
ROSflight workspace. Then build and source the Veloxity shim in that same
shell:

```bash
source scripts/build_and_source_ros2_shim.zsh

./examples/quadx_upstream_angle_waypoints/clean_slate.zsh

./examples/quadx_upstream_angle_waypoints/run_upstream_angle_experiment.zsh \
  --firmware rust \
  --use-rviz true \
  --duration 120 \
  --bag-name takeoff_logs/quadx_upstream_angle_mode_rust
```

Pass `--record-all true` for backend-comparison captures that must retain every
discovered ROS topic. The default remains the smaller curated topic set.

## Compare Veloxity and C backends

The checked-in comparison plotter opens interactive Matplotlib figures with
time-range sliders. It does not save image files:

```bash
python3 tools/plot_quadx_upstream_firmware_compare.py
```

Its default inputs are the all-topic comparison bags
`takeoff_logs/quadx_upstream_backend_compare_veloxity_accel_quat_fix_repeat`
and
`takeoff_logs/quadx_upstream_backend_compare_c`. Override them with
`--veloxity-bag` and `--c-bag` when comparing another capture pair.

Use `--firmware c` for a comparison against upstream C SIL firmware.

This remains a simulation experiment. Its mass, equilibrium throttle, mixer,
mission, and controller gains are not automatically valid for hardware.
