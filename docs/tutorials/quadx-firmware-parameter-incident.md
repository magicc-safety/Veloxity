# Quad-X Firmware-Parameter Incident Report

**Date:** 2026-07-16 through 2026-07-17

This report records the diagnosis of an immediate offboard flyaway, the
firmware and ROScopter configuration corrections made during the investigation,
and a later altitude excursion caused by partial RC override. It applies to the
`3dquad` airframe using the ROSflight 2.0 firmware interface, Veloxity SIL, the
upstream ROScopter trajectory follower, and the canned Quad-X mixer.

## Initial symptoms

The first manual waypoint attempt appeared controllable under RC override. On
release to offboard control, it immediately saturated motor outputs, developed
large attitude errors, and departed rapidly. Reasserting RC override restored a
controllable path. A previously recorded automatic waypoint experiment using
the simulation firmware profile did not show this behavior.

The failed run recorded approximately 29,000 saturated PWM samples and departed
by tens of metres within seconds. The successful reference kept roll and pitch
commands near 0.1 rad, used throttle between approximately 0.66 and 0.82, and
had only isolated saturation samples.

## Root configuration problem

The failing startup loaded a hardware firmware snapshot into simulation. The
snapshot selected `PRIMARY_MIXER=2` (canned Quad-X), but also wrote all of the
following:

- the complete `PRI_MIXER_*_*` matrix;
- all `PRI_MIXER_OUT_*` output mappings;
- all `PRI_MIXER_PWM_*` rates;
- `SECONDARY_MIXER=255`; and
- the complete `SEC_MIXER_*_*` matrix.

The custom matrices contained physical-model coefficients on the order of
`-5814`, `+/-25303`, and `+/-267065`, while `USE_MOTOR_PARAM=0` selected the
normalized canned-mixer path. These two representations are incompatible.

Selecting canned Quad-X did not make the later individual matrix writes
harmless. ROSflight's parameter callback immediately copies every
`PRI_MIXER_*_*` or `SEC_MIXER_*_*` write into the live mixer. The order in the
old file produced a particularly deceptive final state:

1. `PRIMARY_MIXER=2` constructed the canned primary matrix.
2. Later primary-matrix and output writes modified the live primary data.
3. `SECONDARY_MIXER=255` caused mixer initialization to reconstruct the canned
   primary and initially mirror it into the secondary.
4. Subsequent `SEC_MIXER_*_*` writes replaced that mirrored secondary with the
   large custom matrix.

ROSflight selects primary matrix rows for RC-overridden axes and secondary
matrix rows for offboard axes. The resulting state explains why RC override
could be controllable while offboard control saturated immediately. This was a
configuration error around the canned Quad-X mixer, not evidence that the
canned Quad-X coefficients themselves are defective.

## Corrected firmware profile

The reviewed profile now requests only the mixer choices needed for the canned
path:

```yaml
- { name: PRIMARY_MIXER, type: 6, value: 2 }
- { name: USE_MOTOR_PARAM, type: 6, value: 0 }
```

It does not load primary or secondary matrix elements, output mappings, PWM
rates, or an explicit secondary mixer. With the default invalid secondary
selection, firmware mirrors the selected canned primary matrix and no later
matrix writes alter it.

Additional simulation corrections were:

- roll and pitch angle PID `P=7.5`, `D=1.5`;
- roll and pitch rate `P=1.5`;
- yaw-rate `P=4.0`;
- `MAG_PITCH=0`;
- identity magnetometer calibration matrix with zero biases;
- `RC_MAX_THR=1.0`, leaving the ROS throttle adapter as the operational limit;
- explicit IMU and barometer calibration after every simulator restart.

The magnetometer identity calibration is simulation-specific. Hardware must use
a calibration measured on the installed sensor, not this identity matrix.

`RC_MAX_THR=1.0` was not the original flyaway fix. It removes a second firmware
scaling stage after the ROS adapter clamps throttle to its validated range. The
mixer cleanup is the strongest causal correction; PID and sensor changes remove
important additional mismatches but were changed in the same trial, so the run
does not isolate them experimentally.

## ROScopter configuration corrections

The experiment overlay was corrected to use the actual node name
`/trajectory_velocity_adapter`, not `/trajectory_veloxity_adapter`. The
trajectory follower and velocity adapter retain different parameter names, but
their matching numerical gains are intentional:

```yaml
u_n_kd: 3.5
north_kd: 3.5
```

The simulation throttle adapter and ROScopter controller use the same limits:

```yaml
equilibrium_throttle: 0.686
min_throttle: 0.40
max_throttle: 0.85
```

The adapter chain is:

```text
/trajectory_command
  -> trajectory_velocity_adapter
  -> /trajectory_command_compensated
  -> trajectory_follower
  -> /high_level_command_thrust
  -> thrust_to_throttle_adapter
  -> /high_level_command
  -> controller
  -> /command
```

## Successful retry and remaining RC-override trap

After the corrections, the vehicle performed stable offboard waypoint flight.
Throttle settled near 0.48 in the recorded portion, attitude requests remained
small, and the original immediate flyaway did not recur.

A later deliberate roll-stick override produced a separate vertical excursion.
The recorded status changed from `rc_override=0` to `rc_override=4`, which is
the X/roll-stick bit, not the full override switch. RC throttle remained near
its low value. Firmware therefore substituted RC roll while continuing to use
offboard throttle. The offboard throttle command rose from about 0.478 to 0.715
and then 0.777; truth altitude moved from about -4 m to -23 m before the
controller commanded minimum throttle and returned toward the -5 m target.
Transient unhealthy-estimator status accompanied the event.

This event was not caused by the current canned Quad-X matrix. It demonstrates
that per-axis RC muxing is unsafe for a controller whose desired attitude and
thrust are coupled. Use the mapped attitude-and-throttle override switch for a
complete handoff. Do not treat a single deviated stick as equivalent to full RC
takeover.

## Local startup and go/no-go checklist

1. Start the simulator firmware and RViz.
2. While disarmed, load the reviewed firmware profile.
3. Calibrate IMU and barometer after every simulator restart.
4. Start estimator, path manager, path planner, velocity adapter, trajectory
   follower, throttle adapter, and controller in that order.
5. Confirm the velocity adapter reports lead values `2.333/2.333/0.875 s`.
6. Confirm the controller receives estimated state.
7. Load and print the mission from both planner and manager.
8. Start a bag before arming.
9. Arm with the full override switch held and sticks centered.
10. Require `failsafe=false`, `error_code=0`, and a live offboard stream.
11. Release the full switch, then require `rc_override=0`.
12. Reassert the full switch immediately for unexpected attitude, altitude, or
    estimator behavior.

Never reload firmware parameters while armed. Do not copy the simulation
magnetometer calibration or hover-throttle value onto hardware without a
hardware-specific measurement and review.

## Diagnostic recordings

- `flight-logs/hardware_exp2_20260716-220642`: original failed offboard run.
- `flight-logs/hardware_exp2_20260716-223039`: mission loaded while disarmed;
  captured the expected saturated takeoff request but no offboard release.
- `flight-logs/hardware_exp2_20260716-223416`: successful retry plus the
  roll-only override altitude excursion.

These paths are local experiment artifacts and are intentionally not committed.
