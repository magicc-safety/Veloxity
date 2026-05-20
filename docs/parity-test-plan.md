# ROSflight C Parity Test Plan

This plan tracks the remaining parity areas that need direct tests before moving
to hardware. The first pass should be Rust unit tests and SIL/ROS sim tests. The
embedded hardware checks stay listed here until sim behavior is clean enough to
justify board time.

## Fixed-Wing Paths

These tests are not a claim that the quadrotor controller, quadrotor mixer, or
quadrotor estimator are fixed-wing implementations. They cover shared command
plumbing, ROSflight canned fixed-wing mixer branches, and `PARAM_FIXED_WING`
branches that currently live in shared/quad-named modules. A dedicated
fixed-wing vehicle pipeline still needs its own coverage.

- RC command interpretation:
  - `PARAM_FIXED_WING=1` maps all primary RC control axes to passthrough.
  - Fixed-wing failsafe uses passthrough channel types and does not apply
    multirotor throttle range validation.
- Mixer selection:
  - Canned fixed-wing and inverted-V-tail primary mixers produce the same servo
    and throttle channel mapping as C.
  - Secondary mixer selection preserves the ROSflight C row-selection behavior:
    attitude override selects primary torque rows, throttle override selects
    primary force rows, and otherwise secondary rows are used.
  - Fixed-wing output types and default PWM rates remain servo/motor/aux at
    50 Hz.
- Estimator health:
  - Fixed-wing mode does not set unhealthy-estimator solely because the
    accelerometer correction window timed out in the current attitude-estimator
    implementation.

## Numeric Sensor Compensation

- IMU:
  - Orientation is applied before bias correction.
  - Gyro and accel biases match ROSflight C sign conventions, including the
    gravity sign used by accelerometer calibration.
- Magnetometer:
  - Orientation is applied before hard-iron subtraction.
  - The 3x3 soft-iron matrix is applied row-wise to the corrected vector.
- Barometer:
  - Pressure-to-altitude uses the ROSflight correction formula.
  - Calibration stores the observed pressure bias, but processed pressure is not
    pre-subtracted before altitude conversion.
- Pitot:
  - Differential-pressure calibration stores the static bias.
  - Airspeed uses the ROSflight `sqrt(2 * dp / rho)` relationship after bias
    correction.
- Battery:
  - First and subsequent low-pass outputs match ROSflight initialization and
    alpha usage.

## Controller/PID Numeric Trace

- PID derivative low-pass and integrator update match a C trace for the first
  two samples.
- Integrator gates match C for disarmed, long-dt, and low-throttle cases.
- Saturation anti-windup holds the integrator in the same cases as C.
- Quad rate-mode output matches a C trace using equilibrium torque offsets.
- Angle-mode control uses current body-rate feedback the same way as C.

## Telemetry Stream Rates

- Heartbeat streams at 1 Hz.
- Status streams at 10 Hz.
- IMU and attitude stream on every IMU update.
- Raw outputs stream every 8 IMU updates, matching the C 400 Hz / 8 = 50 Hz
  default.
- Non-IMU sensor streams are emitted when their fresh flags are present and do
  not depend on an IMU control update.
- RC raw, parameter streaming, command ACK, version, statustext, and backup
  hard-error reporting are emitted from the same events as C.

## Embedded Board Paths

Run these as sim or dummy-board proxies first, then repeat on hardware.

- PWM:
  - Motor, servo, GPIO, and aux output classification matches mixer ownership.
  - Pulse ranges and default update rates match ROSflight C for quad and
    fixed-wing canned mixers.
  - Disarmed outputs and spin-when-armed behavior match C.
- LEDs:
  - Armed, failsafe, error, and normal states drive the same LED intent.
- Persistent params:
  - Read, write, defaults, and rejected-while-armed command paths match C.
- Backup memory:
  - Valid backup data is reported after companion heartbeat.
  - Rearm intent is routed through the explicit state-machine transition.
  - Backup memory is cleared once after boot handling.
- RC parsing:
  - Good, short, stale, and lost-link frames produce the same RC state and
    error flags as C.
- Sensor init failure behavior:
  - Each board sensor init/read failure increments board-owned sensor errors and
    is visible in status telemetry.
