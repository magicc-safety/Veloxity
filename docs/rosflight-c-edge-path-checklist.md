# ROSflight C Firmware Edge-Path Checklist

This checklist tracks ROSflight 2.0 C firmware behavior that Voloxide must preserve. It is intentionally scoped to the active core firmware modules under:

- `workspace/src/rosflight_ros_pkgs/rosflight_firmware/src/*.cpp`
- `workspace/src/rosflight_ros_pkgs/rosflight_firmware/include/*.h`

Vendored libraries, STM32 HAL code, USB stacks, and board-driver implementation internals are not treated as firmware semantics here. Board interfaces are included as required backend behavior and hardware validation surfaces.

Legend:

- `[x]` Reviewed and currently represented in Voloxide.
- `[~]` Implemented or partially represented, but still needs targeted evidence.
- `[ ]` Open parity item.
- `[hw]` Board/backend-specific behavior that must be validated on target hardware or SIL backend.

## Core Scheduler

Source: `rosflight.cpp`, `rosflight.h`

- [x] Initialization order: params, state manager, comms, sensors, estimator, controller, mixer, command manager, RC.
- [x] Main loop receives comms before board/sensor/control work.
- [x] Sensor read is the scheduler gate for control updates.
- [x] IMU sample is the only path that runs estimator, controller, mixer, and command update.
- [x] First IMU timestamp initializes time and skips control.
- [x] Backward time sets `ERROR_TIME_GOING_BACKWARDS` and skips control.
- [x] Advancing time clears `ERROR_TIME_GOING_BACKWARDS`.
- [x] RC receive path runs after control-stage work and can publish raw RC.
- [~] Loop-time telemetry should match the C status path on all backends, including embedded targets.
- [~] Non-IMU telemetry scheduling should remain tied to the world/scheduler stage, not individual sensor processors.

## State Manager

Source: `state_manager.cpp`, `state_manager.h`

### States

- [x] `INIT`
- [x] `PREFLIGHT`
- [x] `CALIBRATING`
- [x] `ARMED`
- [x] `FAILSAFE`
- [x] `ERROR`

### Events

- [x] `EVENT_INITIALIZED`
- [x] `EVENT_REQUEST_ARM`
- [x] `EVENT_REQUEST_DISARM`
- [x] `EVENT_RC_LOST`
- [x] `EVENT_RC_FOUND`
- [x] `EVENT_ERROR`
- [x] `EVENT_NO_ERROR`
- [x] `EVENT_CALIBRATION_COMPLETE`
- [x] `EVENT_CALIBRATION_FAILED`

### Error Flags

- [x] `ERROR_INVALID_MIXER`
- [x] `ERROR_IMU_NOT_RESPONDING`
- [x] `ERROR_RC_LOST`
- [x] `ERROR_UNHEALTHY_ESTIMATOR`
- [x] `ERROR_TIME_GOING_BACKWARDS`
- [x] `ERROR_UNCALIBRATED_IMU`
- [~] `ERROR_BUFFER_OVERRUN`: represented as a flag; targeted trigger coverage still needed.
- [x] `ERROR_INVALID_FAILSAFE`
- [x] Calibration-specific failure flags used by Voloxide for sensor calibration diagnostics.

### Transition Branches

- [x] `INIT + INITIALIZED -> PREFLIGHT`.
- [x] `PREFLIGHT + RC_FOUND`: clear RC lost, clear failsafe.
- [x] `PREFLIGHT + RC_LOST`: set RC lost error.
- [x] `PREFLIGHT + ERROR`: enter `ERROR`.
- [x] `PREFLIGHT + REQUEST_ARM`: reject if throttle is above arm threshold.
- [x] `PREFLIGHT + REQUEST_ARM`: reject if throttle override switch is not active and take-min-throttle is not enabled.
- [x] `PREFLIGHT + REQUEST_ARM`: if gyro-on-arm is enabled, enter `CALIBRATING` and start gyro calibration.
- [x] `PREFLIGHT + REQUEST_ARM`: otherwise arm and enter `ARMED`.
- [x] `ERROR + RC_LOST`: set RC lost error even if already in error.
- [x] `ERROR + RC_FOUND`: clear RC lost, clear failsafe.
- [x] `ERROR + NO_ERROR -> PREFLIGHT`.
- [x] `ERROR + REQUEST_ARM`: reject and log active arming blockers at 1 Hz.
- [x] `CALIBRATING + CALIBRATION_COMPLETE`: arm and enter `ARMED`.
- [x] `CALIBRATING + CALIBRATION_FAILED -> PREFLIGHT`.
- [x] `CALIBRATING + RC_LOST`: set RC lost error.
- [x] `CALIBRATING + ERROR -> ERROR`.
- [x] `CALIBRATING + NO_ERROR`: clear error state.
- [x] `ARMED + RC_LOST`: enter failsafe, set RC lost error, update status.
- [x] `ARMED + REQUEST_DISARM`: disarm to `PREFLIGHT` when no error.
- [x] `ARMED + REQUEST_DISARM`: disarm to `ERROR` when error is active.
- [x] `ARMED + ERROR`: mark error while staying armed.
- [x] `ARMED + NO_ERROR`: clear error state.
- [x] `FAILSAFE + ERROR`: mark error.
- [x] `FAILSAFE + REQUEST_DISARM`: disarm and enter `ERROR`.
- [x] `FAILSAFE + RC_FOUND`: clear failsafe, enter `ARMED`, clear RC lost.
- [x] Status updates on state or error-code changes.

### Backup Memory

- [x] Hardfault backup data contains reset count, error code, arm flag, debug registers, and checksum.
- [x] Backup memory initializes before read/write.
- [x] Invalid checksum is ignored except memory is cleared.
- [x] Valid backup data is sent over comms after boot.
- [x] Armed hardfault intent is routed through explicit Voloxide state-machine transition rather than a raw setter.
- [x] Rearm succeeds only from a good preflight state.
- [x] Rearm failure and success are logged as critical messages.
- [x] Backup memory is cleared after read.

### LEDs

- [x] Failsafe blinks LED1 at 100 ms.
- [x] Error blinks LED1 at 500 ms.
- [x] Disarmed turns LED1 off.
- [x] Armed turns LED1 on.
- [x] RC override drives LED0 through command manager.
- [hw] LED polarity and board-specific output routing still require hardware validation.

## Communication Manager

Source: `comm_manager.cpp`, `comm_manager.h`, `comm_link.h`

- [x] Comm link listener covers parameter list/read/set, commands, timesync, offboard, aux, external attitude, heartbeat.
- [x] Parameter requests are filtered by target system.
- [x] Parameter set rejects wrong type.
- [x] Parameter list streams all params by index.
- [x] Parameter read supports name and index paths.
- [x] Unknown parameter read/set is rejected.
- [x] Command handling sends command ACK for accepted/rejected commands.
- [x] Calibration commands ACK when accepted/started, not when calibration finishes.
- [x] Calibration completion and failure are reported through state/log/error paths.
- [x] Arm/disarm command path routes into the state manager, not direct state mutation.
- [x] Reboot command uses delayed reboot path.
- [x] Timesync responds only to request messages where `tc1 == 0`.
- [x] Timesync response uses local time in `tc1` and echoed remote timestamp in `ts1`.
- [x] Offboard control maps active/inactive channel flags into command manager muxes.
- [x] Offboard control supports rate and angle modes.
- [x] Offboard control ignore bits leave channels inactive.
- [x] Aux command supports GPIO, servo, motor, and invalid/no-op branch.
- [x] External attitude updates estimator input.
- [x] Heartbeat marks companion link as connected.
- [x] Log/statustext sending is gated on companion heartbeat.
- [x] Version string reports `Voloxide 1.0`.
- [x] Status publishes armed, failsafe, RC override, offboard, error code, control mode, and loop time.
- [x] Stream scheduling covers heartbeat, status, IMU, attitude, RC raw, outputs, baro, diff pressure, mag, range, GNSS, battery, and logs.
- [~] Stream rates should be checked against C defaults after final warning cleanup.

## Command Manager

Source: `command_manager.cpp`, `command_manager.h`

- [x] Failsafe command is recomputed when fixed-wing/failsafe params change.
- [x] Multirotor failsafe throttle outside `[0, 1]` sets invalid failsafe.
- [x] Fixed-wing failsafe uses configured attitude/throttle behavior.
- [x] RC F-axis switch supports X, Y, Z, and invalid/default warning paths.
- [x] RC command maps sticks into command channels.
- [x] RC attitude type switch selects rate or angle command mode.
- [x] Missing attitude type switch uses configured default mode.
- [x] Invalid attitude mode falls back safely.
- [x] Stick override lag branch preserves temporary RC ownership after stick movement.
- [x] Attitude override switch can override X/Y/Z.
- [x] Throttle override switch can override F/T.
- [x] Offboard inactive channels are treated as overridden by RC.
- [x] Throttle override take-min-throttle takes the lower of RC and onboard throttle.
- [x] Failsafe path outputs failsafe command.
- [x] Offboard timeout deactivates onboard command channels.
- [x] LED0 reflects RC override state.
- [x] Status updates when RC override bitmask changes.
- [~] Fixed-wing command paths are structurally represented but need targeted fixed-wing regression tests.

## RC

Source: `rc.cpp`, `rc.h`

- [x] RC backend type change reinitializes board RC input.
- [x] Stick channel mapping reloads on parameter change.
- [x] Switch channel/direction mapping reloads on parameter change.
- [x] Switch mapping logs mapped/unmapped switch state.
- [x] Frame lost or receiver failsafe sets RC lost.
- [x] Too few channels is treated as RC lost in Voloxide.
- [x] Out-of-range channel values set RC lost.
- [x] Healthy RC clears RC lost.
- [x] Stick arming path requires low throttle plus yaw/right stick hold for 1 second when arm switch is unmapped.
- [x] Stick disarm path requires low throttle plus yaw/left stick hold for 1 second when arm switch is unmapped.
- [x] Arm switch path requests arm/disarm from state manager.
- [x] One-sided throttle stick scaling is preserved.
- [x] Two-sided stick scaling is preserved.
- [x] Switch direction inversion is preserved.
- [x] `new_command()` reports only fresh RC frames.
- [hw] Receiver frame parsing and exact channel scaling need backend-specific hardware/SIL validation.

## Sensors

Source: `sensors.cpp`, `sensors.h`

- [x] Missing IMU calibration parameters set uncalibrated IMU error.
- [x] IMU orientation params recompute board-to-FCU rotation.
- [x] Mag orientation params recompute mag-to-FCU rotation.
- [x] IMU read success publishes calibrated/rotated accel and gyro.
- [x] IMU read failure sets IMU-not-responding error.
- [x] Gyro calibration can be started and reports accepted start.
- [x] Gyro calibration success stores bias params and clears failure.
- [x] Gyro calibration failure sets calibration failure.
- [x] Accel calibration can be started and reports accepted start.
- [x] Accel calibration success stores bias params and clears failure.
- [x] Accel calibration failure sets calibration failure.
- [x] Baro calibration can be started and reports accepted start.
- [x] Baro calibration success stores bias params and clears failure.
- [x] Baro calibration failure sets calibration failure.
- [x] Differential pressure calibration can be started and reports accepted start.
- [x] Differential pressure calibration success stores bias params and clears failure.
- [x] Differential pressure calibration failure sets calibration failure.
- [x] Baro, mag, diff pressure, range, GNSS, and battery reads feed telemetry resources.
- [x] GNSS latitude and longitude stay in degrees in the SIL path.
- [x] Sensor processors do not ACK completion directly.
- [~] Exact C sensor compensation math needs separate numeric tests against recorded C samples.
- [hw] Physical sensor driver initialization/failure paths require embedded board validation.

## Estimator

Source: `estimator.cpp`, `estimator.h`

- [x] Estimator reset/init path.
- [x] First IMU sample initializes estimator time.
- [x] IMU dt comes from IMU timestamps.
- [x] Non-advancing dt is handled without corrupting estimator state.
- [x] External attitude input is accepted.
- [x] Estimator unhealthy timeout sets unhealthy estimator error.
- [x] Healthy estimator clears unhealthy estimator error.
- [~] Fixed-wing estimator exceptions need targeted fixed-wing tests.
- [~] Vertical baro/altitude behavior remains a ROScopter configuration issue, not firmware parity; keep documented separately.

## Controller

Source: `controller.cpp`, `controller.h`

- [x] Controller parameters update PID gains and trims.
- [x] Multirotor controller path maps commands into roll/pitch/yaw/throttle outputs.
- [x] Integrators update only when armed and when C-style gates permit.
- [x] Disarmed controller does not accumulate integrator state.
- [x] Output saturation and anti-windup paths are represented.
- [~] Fixed-wing controller path needs targeted fixed-wing tests.
- [~] Numeric PID parity should be checked with a captured C trace.

## Mixer

Source: `mixer.cpp`, `mixer.h`

- [x] Primary mixer parameter selects canned/custom mixer.
- [x] Invalid primary mixer sets invalid mixer error.
- [x] Custom mixer loads output types, rates, and matrix params.
- [x] Full 10x10 SVD pseudoinverse path is represented.
- [x] Secondary mixer is represented, including fixed-wing/vtail fallback behavior.
- [x] PWM init uses selected mixer rates.
- [x] ESC calibration mixer path is represented.
- [x] Motor-parameter thrust conversion path is represented.
- [x] `K_Q` and battery-voltage guards log errors and avoid invalid math.
- [x] Idle-throttle yaw suppression is represented.
- [x] Mixer output scaling and over-2.0 warning are represented.
- [x] AUX output handling is represented.
- [x] GPIO, servo, motor, and invalid aux output handling are represented.
- [x] Motor outputs are forced low when disarmed.
- [~] Exact PWM pulse output ranges need hardware/backend validation.

## Parameters

Source: `param.cpp`, `param.h`, `param_listener.h`

- [x] Default parameter table is represented.
- [x] Parameter checksum/update path is represented.
- [x] Read-by-index and read-by-name branches are represented.
- [x] Invalid parameter name/index rejects gracefully.
- [x] Type-specific int/float set path is represented.
- [x] Param listeners receive change notifications.
- [~] Persistent storage behavior needs backend-specific validation.

## Board Interface

Source: `interface/board.h` plus board-specific backends.

- [x] Clock, delay, reboot, serial, backup memory, LEDs, PWM, RC, sensors, and comm-link surfaces are identified.
- [x] SIL board routes sensor snapshots and PWM outputs through the firmware-compatible interface.
- [~] Loop timing, serial buffering, and ignored-result behavior require warning cleanup before final board parity review.
- [hw] PixRacer Pro and Nucleo board paths need physical target validation once warning cleanup is complete.

## Current Open Parity Evidence

- [ ] Add targeted fixed-wing tests for command manager, estimator exceptions, controller, and secondary mixer.
- [ ] Add numeric sensor compensation tests from C-recorded samples.
- [ ] Add numeric controller/PID trace comparison against C.
- [ ] Add stream-rate snapshot tests against C defaults.
- [ ] Add embedded board validation for PWM pulse ranges, LED polarity, persistent params, backup memory, RC parsing, and sensor init failure paths.
