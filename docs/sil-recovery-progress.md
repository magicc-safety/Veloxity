# SIL Recovery Progress

This document records the current ROSflight/ROScopter SIL state for Voloxide. It is a recovery log,
not the operator tutorial. For runnable instructions, use:

- `tutorial/voloxide-firmware-bridge.md`
- `tutorial/voloxide-roscopter-waypoints.md`
- `tutorial/sil-findings.md`

## Current Boundary

- Repo root: `/run/host/home/skink/projects/voloxide_proj/Voloxide`
- ROS workspace root: `/run/host/home/skink/projects/voloxide_proj/workspace`
- ROSflight C firmware and Voloxide/Rust firmware should not be edited for the current waypoint
  drift/dip investigation. The behavior reproduced across both firmware paths, so the active scope
  is middleware, launch order, calibration, ROScopter parameters, and simulator sensor settings.

## Committed Firmware-Bridge Fixes

Commit `f0786e3` on `main` fixed the Voloxide SIL firmware endpoint:

- `ros2/voloxide_sil_board_shim/src/voloxide_sil_board.cpp` runs two Voloxide firmware iterations
  per SIL manager tick to match upstream C SIL behavior.
- `sim/src/ffi.rs` consumes sensor snapshots edge-triggered by timestamp so the second firmware
  iteration does not reconsume the same IMU sample.
- `sim/src/ffi.rs`, `sim/src/board.rs`, and `voloxide_core/src/packets.rs` keep GNSS latitude and
  longitude in degrees, matching ROSflight C behavior.

## Current Working Waypoint Configuration

The best validated Rust-backed GUI waypoint run used:

- `RMW_IMPLEMENTATION=rmw_zenoh_cpp` with `ros2 run rmw_zenoh_cpp rmw_zenohd`.
- RViz enabled from launch.
- Waypoint markers from `ros2 run roscopter_gcs rviz_waypoint_publisher`.
- Stock `standalone_sensors` settings, including simulated barometer bias and random walk.
- Standard firmware init from `rosflight_sim multirotor_init_firmware.launch.py`.
- Explicit `/calibrate_imu`.
- ROScopter estimator started first with:

  ```bash
  ros2 run roscopter estimator \
    --ros-args \
    --params-file workspace/install/roscopter/share/roscopter/params/estimator.yaml \
    -p hotstart_estimator:=false \
    -p rho:=-1000000.0
  ```

- Arm, then wait about four seconds on the ground so ROScopter's barometer calibration happens
  while the vehicle is stationary.
- Start controller, trajectory follower, path manager, path planner, and waypoint marker publisher.
- Set `/path_manager hold_last true`.
- Load `workspace/src/roscopter/roscopter/params/multirotor_mission.yaml`.
- Toggle RC override off to enter computer control.

This flow is captured in `scripts/run_voloxide_waypoint_demo.zsh`.

## Validated Results

With GUI and waypoint markers active:

- `/command`: about `390 Hz`, max gap around `6 ms` in the validated window.
- `/sim/pwm_output`: about `400 Hz`, max gap around `6 ms` in the validated window.
- Mid-route around the `z=-20` leg:
  - trajectory `z=-20.0`
  - estimator `p_d=-20.671`
  - truth `z=-20.836`
- Final hold:
  - trajectory `z=-40.0`
  - estimator `p_d=-40.340`
  - truth `z=-41.086`
  - status clean: `armed=true`, `offboard=true`, `error_code=0`

The previous final-hold vertical mismatch was roughly `6-9 m`; with estimator auto-density it was
about `0.75 m`.

## Key Findings

- Zenoh is materially more reliable than Fast DDS on this machine under GUI load. Earlier Fast DDS
  runs showed `/command` and `/sim/pwm_output` stalls in the hundreds of milliseconds.
- The temporary zero-baro-bias overlay was removed. Current intended tests use stock simulated
  barometer bias and random walk.
- `hold_last=true` prevents the mission from wrapping from the final `z=-40` waypoint back toward
  the first `z=-10` waypoint.
- ROScopter's GNSS update path uses north/east position and velocity plus vertical velocity; it does
  not use GNSS altitude as a continuous vertical-position correction in the inspected code.
- ROScopter height tracking is primarily barometer-driven. The estimator uses GNSS altitude for
  initialization and air-density calculation when `rho` is left as `NOT_IN_USE`.
- The local installed estimator YAML sets `rho: 1.225`, which forces sea-level air density. The sim
  origin is around `1387 m` MSL, so that value creates a pressure-to-height scale error.
- Passing `-p rho:=-1000000.0` leaves `rho` as `NOT_IN_USE`, which lets ROScopter compute density
  from GNSS altitude. That is the most important configuration fix found so far.

## Remaining Investigation Targets

- Tune waypoint transition behavior above firmware: `waypoint_tolerance`, `max_velocity`,
  `max_acceleration`, and trajectory follower/controller gains.
- Decide whether the `rho` override belongs in a Voloxide helper launch/script or should remain a
  documented runtime parameter.
- Keep comparing C and Rust only at the firmware endpoint boundary. Shared ROScopter drift/dip
  behavior should remain a ROSflight/ROScopter configuration investigation.
