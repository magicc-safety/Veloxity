# SIL Findings

This document summarizes the findings from recovering Voloxide's ROSflight/ROScopter SIL waypoint
workflow.

## Firmware Scope

Do not modify ROSflight C firmware or Voloxide/Rust firmware for the current waypoint drift/dip
investigation. The behavior reproduced across both firmware paths, so the active problem space is:

- ROS 2 middleware and graph timing.
- Launch order.
- Firmware and estimator calibration sequence.
- ROScopter estimator parameters.
- ROScopter path-manager and trajectory-follower tuning.
- Simulator sensor configuration.

## Middleware

Fast DDS showed intermittent local graph stalls under load on this machine. In earlier GUI runs,
`/command` and `/sim/pwm_output` had gaps in the hundreds of milliseconds.

Zenoh RMW was much more stable. In the validated GUI run:

- `/command`: about 390 Hz, max gap about 6 ms.
- `/sim/pwm_output`: about 400 Hz, max gap about 6 ms.

Use:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rmw_zenoh_cpp rmw_zenohd
```

## Barometer Simulation

Use stock standalone sensor simulation for realistic tests. The stock standalone barometer settings
include random bias and random walk:

- `baro_bias_range=500.0`
- `baro_bias_walk_stdev=0.1`
- `baro_stdev=4.0`

A zero-bias overlay was briefly tested as a diagnostic control, then removed. The current expectation
is that the estimator/configuration should handle the stock simulator behavior.

## Height Estimation

The ROSflight docs describe `rho` as normally calculated from GNSS altitude when left as
`NOT_IN_USE`.

The local ROScopter estimator code reads GNSS altitude and computes `gps_h`, but the GNSS measurement
update uses:

```text
gps_n, gps_e, gps_vn, gps_ve, gps_vd
```

It does not include GNSS altitude as a continuous vertical-position measurement. Height tracking is
therefore primarily barometer driven, and `rho` must be correct for the pressure-to-height scale.

The local estimator YAML currently sets:

```yaml
rho: 1.225
```

That forces sea-level density. The standalone sim origin is around 1387 m MSL, so the intended
configuration is to leave `rho` as `NOT_IN_USE`, e.g. by launching the estimator with:

```bash
-p rho:=-1000000.0
```

## Validated GUI Result

Validated run configuration:

- Voloxide/Rust firmware backend.
- RViz enabled.
- Waypoint marker publisher enabled.
- Stock standalone barometer bias/walk.
- Explicit firmware IMU calibration.
- Estimator-first ROScopter startup.
- `rho` left as `NOT_IN_USE`.
- `/path_manager hold_last=true`.

Final hold sample:

- trajectory `z=-40.0`
- estimator `p_d=-40.34`
- truth `z=-41.09`
- `/status`: `armed=true`, `offboard=true`, `error_code=0`

The previous sea-level-density configuration showed roughly 6-9 m of final vertical mismatch. The
validated configuration reduced that to about 0.75 m.

## Waypoint Transition Behavior

ROScopter's path manager interpolates between waypoints in 3D and uses feed-forward velocity and
acceleration terms. The docs warn that if `max_velocity` or `max_acceleration` is too aggressive, the
vehicle can fall behind the trajectory; when the setpoint reaches the end of a leg and slows down,
visual performance can look poor.

Next tuning targets:

- `max_velocity`
- `max_acceleration`
- `waypoint_tolerance`
- `hold_last`

`hold_last=true` prevents the final waypoint from wrapping back to the first waypoint, which
previously looked like vertical drift after mission completion.

