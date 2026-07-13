# Quad-X Upstream Angle-Mode Experiment Log

This journal records trajectory-following experiments using a scientific-method
workflow. Keep GUI/RViz enabled for every run. Change one independent variable
at a time when possible, record any confound explicitly, and retain failed runs.

## Fixed constraints

- Firmware endpoint: Veloxity Rust SIL unless the run says otherwise.
- Simulation: upstream `rosflight_sim` standalone multirotor with unmodified
  `rosflight_io` and ROScopter sources.
- Mission: upstream `multirotor_mission.yaml`.
- GUI: `--use-rviz true` is mandatory.
- Primary first-corner metrics: maximum estimated-state north overshoot and
  maximum simulator-truth north overshoot after aligning the simulator origin
  to the estimator frame at mission release.
- Supporting metrics: state at handoff, trajectory-reference discontinuity,
  closest truth pass, estimator error, and time after mission release.

## Reconstructed history (2026-07-13)

Parameters that cannot be proven from a bag are marked unknown. Velocity and
acceleration maxima and velocity-lead ratios were recovered directly from the
recorded command topics.

| Run | Max speed | Velocity lead | Aligned-truth overshoot | Observation |
| --- | ---: | ---: | ---: | --- |
| `rust` | 3.0 m/s | none | 2.13 m | Fast baseline; estimated-state overshoot was 1.66 m. |
| `tuned1` | 2.0 m/s | none | 2.09 m | Slowing alone did not materially reduce overshoot; estimated overshoot was 1.20 m. |
| `tuned2` | unknown | none | unavailable | Mission was never released; retain as a failed preflight/run. |
| `tuned2b` | 2.0 m/s | 1.667 s | 1.09 m | Estimated overshoot was 0.98 m. Path handoff jumped the original north reference 1.21 m at an estimated waypoint distance of 0.99 m. |
| `tuned3` | 2.0 m/s | 1.667 s | 0.56 m | Estimated overshoot was 0.92 m. Original north reference jumped 1.29 m at handoff. |
| `tuned4` | 2.0 m/s | 1.667 s | 1.89 m | Estimated overshoot was 1.18 m. No greater-than-0.1 m reference jump was detected. Other parameters are not recoverable. |
| `tuned5` | 1.5 m/s | 1.667 s | 0.98 m | Estimated overshoot was 0.77 m. Original north reference jumped 1.66 m at a 0.98 m estimated waypoint distance. |
| `tuned6` | 1.5 m/s | 2.333 s | 0.85 m | Estimated overshoot was 0.57 m. Original north reference jumped 1.60 m at a 0.99 m estimated waypoint distance. Bag shutdown was interrupted, but corner data is intact. |

### Current interpretation

The path manager changes legs as soon as estimated 3-D distance enters its
capture radius, even when the time-parameterized reference has not reached the
waypoint. In `tuned6`, the original north reference changed from about 18.40 m
to 20.00 m while estimated north velocity was about 0.82 m/s. This premature
handoff is now the leading hypothesis for poor corner tracking. Higher lateral
derivative gain alone produced only a marginal improvement.

## `tuned7`: moderate waypoint capture radius

**Status:** partial pass.

**Question:** Will reducing capture radius from 1.0 m to 0.5 m delay the leg
handoff enough to reduce the reference jump and first-corner overshoot without
causing the long dwell observed with a very tight tolerance?

**Hypothesis:** A 0.5 m radius will reduce both the handoff discontinuity and
overshoot relative to `tuned6`, while still permitting timely waypoint capture.

**Independent variable:** `waypoint_tolerance: 1.0 -> 0.5` m.

**Controlled parameters:** max velocity 1.5 m/s, max acceleration 1.0 m/s^2,
lateral `kp=1.5`, `ki=0.01`, `kd=3.5`, velocity lead 2.333 s, hover throttle
0.686, angle/throttle limits unchanged.

**Procedure:** Rust SIL, GUI/RViz enabled, 120 s flight, bag
`takeoff_logs/quadx_upstream_angle_mode_rust_tuned7`.

**Results:** Aligned-truth first-corner overshoot fell from 0.85 m in `tuned6`
to 0.64 m; estimated-state overshoot was 0.82 m. The reference still jumped
1.10 m at the handoff, but the capture radius was reduced as intended. Visual
assessment was good through the early waypoints, then became poor at/after
waypoint 3. Later in the mission, estimated/reference error remained below
about 1 m while estimator/truth separation grew and was predominantly vertical.

**Decision:** Retain the 0.5 m tolerance. The result passes the first-corner
criterion but not the full mission. Test the missing down-axis commanded-
velocity compensation next, then investigate residual estimator/truth vertical
bias separately if it remains.

## `tuned8`: down-axis velocity compensation

**Status:** mixed result; replication required.

**Question:** Does restoring the down-axis velocity reference improve the
descending legs that begin at waypoint 3?

**Hypothesis:** Adding `down_kd / down_kp * commanded_down_velocity` to the
follower-only down-position reference will reduce estimated vertical trajectory
lag without changing the successful first-corner behavior.

**Independent variable:** down-axis velocity lead `0.0 -> 0.875` s.

**Controlled parameters:** Same as `tuned7`, including 0.5 m waypoint
tolerance, 1.5 m/s maximum velocity, lateral 2.333 s velocity lead, and GUI.

**Procedure:** Rust SIL, GUI/RViz enabled, 120 s flight, bag
`takeoff_logs/quadx_upstream_angle_mode_rust_tuned8`.

**Results:** Down-axis estimated/reference RMS error improved from 0.317 m to
0.141 m on the first descending leg and from 0.527 m to 0.179 m on the final
descending leg. Final-descent truth/reference RMS improved from 2.187 m to
1.438 m. Estimated-state first-corner overshoot improved slightly from 0.82 m
to 0.77 m. Aligned-truth overshoot was 1.30 m, showing estimator/truth variation
that the controller cannot observe.

**Decision:** The vertical hypothesis is supported. Repeat the identical
configuration once to quantify estimator/run variability before accepting or
rejecting the full configuration.

## `tuned8_repeat`: replication

**Status:** complete; hypothesis supported.

**Question:** Are the `tuned8` vertical improvement and horizontal regression
repeatable under identical configuration?

**Hypothesis:** Vertical estimated/reference improvement will repeat. The
first-corner result will vary with estimator/truth initialization error.

**Independent variable:** None; exact replication of `tuned8`.

**Procedure:** Rust SIL, GUI/RViz enabled, 120 s flight, bag
`takeoff_logs/quadx_upstream_angle_mode_rust_tuned8_repeat`.

**Results:** Vertical estimated/reference RMS was 0.129 m on descent 1 and
0.190 m on descent 2, closely reproducing `tuned8`. Estimated-state first-corner
overshoot improved again to 0.65 m. Aligned-truth first-corner overshoot was
1.09 m, with a 0.48 m horizontal estimator/truth discrepancy at capture.

**Decision:** Retain down-axis velocity compensation. It reproducibly improves
vertical command tracking without degrading estimated-state lateral tracking.
Treat remaining simulator-truth deviation as a separate estimator/frame issue,
not as trajectory-controller tuning error.

## Veloxity versus upstream-C backend comparison

**Status:** captures complete; interactive plotter available.

**Question:** With the upstream ROScopter stack, mission, adapters, and tuning
held constant, how closely does the Veloxity firmware endpoint reproduce the
upstream C SIL endpoint?

**Hypothesis:** Both backends will follow the same overall trajectory and show
similar attitude/PWM profiles, with measurable differences in waypoint settling.

**Independent variable:** firmware backend (`rust`/Veloxity versus upstream C).

**Controlled parameters:** GUI/RViz enabled, 120 s mission duration, identical
mission and experiment YAML, fresh firmware and barometer initialization, and
all discovered ROS topics recorded.

**Procedure:** Captured
`takeoff_logs/quadx_upstream_backend_compare_veloxity` followed by
`takeoff_logs/quadx_upstream_backend_compare_c`, using `--record-all true`.
The Veloxity bag contains 1,166,555 messages (591.4 MiB); the C bag contains
1,140,767 messages (579.7 MiB). Both span approximately 130 s and contain 48
topics. Compare interactively with
`tools/plot_quadx_upstream_firmware_compare.py`.

**Results:** Overall 3-D paths, motor PWM, and attitude profiles are similar.
At the first `(20, 0, -10)` corner, Veloxity captured at 28.79 s and had 0.73 m
maximum estimated along-track overshoot. C captured at 29.91 s and had 0.01 m
maximum estimated overshoot. Aligned-truth overshoot was 1.05 m for Veloxity;
C remained 0.25 m short during the 3.2 s corner-analysis window.

**Follow-up question:** Is C's smaller corner overshoot caused by different or
random controller parameters, or by a backend implementation difference?

**Follow-up analysis:** Both backends loaded the same firmware parameter file.
The relevant rate and angle gains match (`ROLL_RATE_P=1.5`, `PITCH_RATE_P=1.5`,
`YAW_RATE_P=4.0`, `ROLL_ANGLE_P=7.5`, `PITCH_ANGLE_P=7.5`, and angle
`D=1.5`). The Rust and C PID equations are also structurally equivalent.
Veloxity tracked its own commanded roll/pitch more tightly than C (RMS errors
0.0040/0.0025 rad versus 0.0060/0.0044 rad), so the evidence does not support
an inferior Veloxity angle PID.

On the first north leg, however, Veloxity's firmware attitude estimate differed
from simulator truth by 0.0649 rad RMS in roll and 0.0601 rad RMS in pitch. The
C estimate differed by only 0.0103 and 0.0075 rad. Consequently the external
trajectory follower generated roughly twice the angle-command RMS for Veloxity
(0.1034 rad versus 0.0484 rad), and its position-error RMS was 0.572 m versus
0.171 m for C. Source inspection found matching estimator/filter parameter
defaults, which points to estimator execution, sensor timing, or mathematical
parity rather than parameter selection. Run-to-run simulated sensor noise still
exists, but it is too small an explanation for the observed 6--8x attitude
estimate-error ratio.

**Decision:** Preserve both all-topic bags. Do not retune the shared trajectory
controller or Veloxity PID to conceal this result. Treat Veloxity attitude
estimator/sensor-timing parity with upstream C as the next experimental
variable, and validate it directly before changing gains.

## Estimator root-cause isolation

**Status:** root cause isolated in replay; flight validation of a correction is
not yet performed.

**Question:** At which boundary does Veloxity first diverge from upstream C:
SIL sample delivery, IMU calibration, estimator timing, quaternion propagation,
or accelerometer correction?

**Hypotheses:**

1. The Veloxity SIL shim reuses cached IMU samples.
2. The backends apply different IMU calibration values.
3. Quaternion integration differs materially.
4. The accelerometer correction uses a different quaternion convention.

**Procedure:** Audited the Veloxity and upstream-C SIL/estimator source paths,
then analyzed both all-topic backend bags with
`diagnose_estimator_parity.py`. Replayed each bag's processed `/imu/data`
through four estimator variants, independently selecting the Rust or C
accelerometer quaternion product and simultaneous or upstream-C sequential
matrix-exponential integration.

**Results:** Simulator IMU, firmware IMU, truth, and attitude counts were all
approximately 52,000 in each 130 s bag, falsifying repeated cached samples for
these captures. Both firmware IMU streams showed the same calibrated offsets
to within normal run variation (approximately `+0.181`, `+0.143`, `-0.218`
rad/s gyro), falsifying calibration as the source.

On the Veloxity input stream, replay with the pre-fix Hamilton quaternion product
produced roll/pitch truth RMS of `0.0998/0.0544` rad. Changing only to the C
`turbomath` product convention reduced this to `0.0171/0.0081` rad. Changing only the
matrix-exponential integration style altered error by less than `0.00003` rad.
The C bag independently reproduced the result: `0.0909/0.0408` rad with the
pre-fix Hamilton product and `0.0150/0.00737` rad with the corrected
`turbomath` product.

**Root cause:** Upstream `turbomath::Quaternion::operator*` uses the opposite
cross-term convention from `nalgebra::Quaternion` multiplication. Veloxity's
`accel_correction()` computes `q_acc_inv * attitude` using nalgebra/Hamilton
cross-term signs, while the C estimator performs that expression with
turbomath signs. The mismatch entered during commit `a96af03` ("Complete
ROSflight compatibility migration", 2026-05-13); commit `aef5d21` later
expanded the same nalgebra signs manually while trimming the hot path. The
error is most visible when yaw and lateral tilt are both nonzero, where the
incorrect cross terms couple roll and pitch correction.

**Decision:** Accept hypothesis 4 and reject hypotheses 1--3 as primary causes.
The next controlled change should alter only the two `q_tilde_i/j` cross-term
signs in Veloxity, add a C-convention regression test, then repeat the paired
GUI/RViz mission and compare fresh all-topic bags.

## Accelerometer quaternion-convention correction

**Status:** attitude hypothesis confirmed; first flight has a confounding
ROScopter barometer-calibration failure and is not a clean full-mission pass.

**Change:** Reversed the two cross terms in each of `q_tilde_i` and
`q_tilde_j` to reproduce upstream turbomath multiplication. Added a coupled
yaw/tilt regression test with the upstream-C expected correction
`(-0.15601143, +0.47869038, 0)`.

**Verification:** All seven focused estimator tests passed and the ROS 2 shim
rebuilt successfully. Ran a 120 s GUI/RViz mission with all topics recorded in
`takeoff_logs/quadx_upstream_backend_compare_veloxity_accel_quat_fix`.

**Attitude results:** Firmware attitude-minus-truth RMS improved from
`0.0649/0.0601` rad roll/pitch before the fix to `0.0109/0.00764` rad. This is
effectively the upstream-C baseline of `0.0105/0.00752` rad. First-corner
capture was 29.90 s, estimated overshoot was 0.02 m, and aligned-truth
overshoot was 0.17 m, versus C's 29.91 s, 0.01 m, and -0.25 m respectively.
The root-cause hypothesis is therefore supported in flight.

**Vertical confound:** The firmware barometer calibration was requested twice
and logged complete, so it was not omitted. Separately, ROScopter begins its
own 100-pressure-sample calibration only after first arm. In this run it logged
`Bad baro calibration. Recalibrating` at arm +1.01 s; the C baseline logged no
such failure. The corrected run's estimated-minus-aligned-truth down RMS was
1.372 m and it finished at estimate/truth/command
`-40.29/-42.45/-40.00` m. C's down RMS was 0.215 m and final values were
`-40.28/-40.35/-40.00` m. Thus the controller followed its biased altitude
estimate while physical truth drifted 2.16 m beyond it.

**Decision:** Accept the attitude correction. Mark this flight unsuitable for
vertical backend comparison. Repeat the identical corrected GUI/all-topic run;
only use a capture without a ROScopter `Bad baro calibration` warning as the
clean validation bag. Preserve the contaminated bag as evidence of the
calibration failure mode.

## Corrected clean replication

**Status:** pass.

**Procedure:** Repeated the identical corrected 120 s mission with GUI/RViz and
all-topic recording in
`takeoff_logs/quadx_upstream_backend_compare_veloxity_accel_quat_fix_repeat`.

**Results:** No ROScopter bad-baro-calibration event occurred. Down-axis
estimate-minus-aligned-truth RMS was 0.290 m versus 0.215 m for C; final
estimate/truth/command was `-40.27/-40.99/-40.00` m versus C's
`-40.28/-40.35/-40.00` m. First-corner capture was 30.17 s with 0.08 m
estimated and 0.11 m aligned-truth overshoot. Attitude error remained at C-like
levels. This clean replication supports the quaternion correction and shows
that the prior 2.16 m Z deviation was dominated by the rejected ROScopter baro
calibration, not the firmware attitude change.

**Decision:** Use this corrected repeat as the definitive Veloxity comparison
bag. Point the interactive C/Veloxity plotter to it and retire older Rust and
pre-fix Veloxity bags at the operator's request.

## Fresh paired replication (2026-07-13)

**Question:** Does a fresh run of each backend reproduce the corrected
Veloxity/C estimator parity and comparable trajectory following?

**Hypothesis:** With the accelerometer quaternion correction retained and all
other experiment inputs controlled, Veloxity and C will again have nearly
identical attitude error. Horizontal and vertical path metrics will still show
normal trial-to-trial variation, including any independent ROScopter barometer
calibration failures.

**Procedure:** Deleted the prior definitive Veloxity and C comparison bags,
then ran each backend once for 120 s with GUI/RViz enabled and all ROS topics
recorded. The replacement bags are
`takeoff_logs/quadx_upstream_backend_compare_veloxity_accel_quat_fix_repeat`
(1,166,524 messages, 591.6 MiB) and
`takeoff_logs/quadx_upstream_backend_compare_c` (1,139,801 messages,
579.5 MiB).

**Results:** Veloxity attitude-minus-truth RMS was
`0.01074/0.00726/0.00931` rad roll/pitch/yaw. C was
`0.01036/0.00752/0.01126` rad. The roll/pitch differences are only
`+0.00037/-0.00026` rad, again confirming estimator parity.

At the first corner, Veloxity had 0.11 m maximum aligned-truth along-track
overshoot and remained 0.02 m short in its estimate. C had 0.67 m
aligned-truth overshoot and remained 0.06 m short in its estimate. This
reversal from some earlier trials is evidence that a single run's corner
ranking is not a stable backend-performance result.

Veloxity logged no rejected ROScopter barometer calibration. Its down-axis
estimate-minus-aligned-truth RMS was 0.658 m and its final
estimate/truth/command was `-40.26/-38.89/-40.00` m. C logged
`Bad baro calibration. Recalibrating` at arm +1.00 s and again at +2.01 s.
Its down-axis RMS was 0.276 m and final values were
`-40.28/-39.94/-40.00` m. Because ROScopter rejected and restarted C's
post-arm calibration twice, this pair is not a clean controlled comparison of
vertical backend behavior even though the numerical C error happened to be
smaller.

**Decision:** Accept the estimator-parity hypothesis. Preserve both fresh
bags as the requested replication and label the C Z-axis result as
barometer-calibration-contaminated. Do not infer that one backend has better
trajectory following from this single pair; the corrected firmware attitude
results are essentially equivalent, while waypoint and altitude outcomes
retain meaningful stochastic run-to-run variation.

### Terminal waypoint-tolerance check

**Question:** Did the two backends stop visibly converging on waypoint 4
because the path manager considered the waypoint reached?

**Method:** Compared `/estimated_state`, `/sim/truth_state`, and
`/trajectory_command` against final HOLD waypoint 4 at `(0, 0, -40)` m. The
path manager uses the estimated full 3-D Euclidean distance and the configured
`waypoint_tolerance: 0.5` m.

**Results:** Veloxity first entered the tolerance sphere at bag time 120.78 s
and ended 0.366 m from the waypoint in estimated coordinates. C entered at
121.50 s and ended 0.323 m away. Their minimum estimated distances were 0.245
and 0.238 m. Both therefore satisfied the waypoint acceptance test. At bag
shutdown, however, `/trajectory_command` still requested essentially exactly
`(0, 0, -40)` with nearly zero reference velocity. The estimated terminal
velocities were also small: approximately 0.050 m/s for Veloxity and 0.035 m/s
for C horizontally.

**Interpretation:** Entering tolerance started waypoint 4's 10 s HOLD timer;
it did not disable position feedback or replace the exact waypoint command.
Neither capture lasted a full 10 s after its first tolerance entry, so neither
advanced beyond the final hold. The visible residual offset is therefore a
small closed-loop/estimator residual inside the accepted region, not evidence
that the controller stopped issuing corrections at the tolerance boundary.
