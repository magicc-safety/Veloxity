# Feature Flags

Cargo features are compile-time switches. They can include optional dependencies, select alternate
hardware paths, or enable diagnostics that should not be present in normal flight builds.

Use features explicitly with `--features 'feature-a feature-b'`.

## Core Features

These live in `crates/veloxity_core/Cargo.toml`.

| Feature | Purpose | Normal flight build? |
| --- | --- | --- |
| `timing-diagnostics` | Enables core-side timing counters and status text used by board/sim diagnostics. | No; diagnostic-only. |
| `scope-timing-pins` | Enables compile-time paths that let board crates drive logic-analyzer timing pins around selected core stages. | No; measurement-only. |
| `pre-control-scope` | Marks the pre-control portion of a control tick when paired with board scope pins. | No. |
| `rc-command-scope` | Marks RC command/state handling timing when paired with board scope pins. | No. |
| `control-scope-estimator` | Selects estimator timing as the scoped control substage. | No. |
| `control-scope-controller` | Selects controller timing as the scoped control substage. | No. |
| `control-scope-mixer` | Selects mixer timing as the scoped control substage. | No. |
| `control-scope-pwm` | Selects PWM output composition/write timing as the scoped control substage. | No. |

## Pico 2 W Features

These live in `boards/pico2w/Cargo.toml`.

The default Pico feature set is `ism330dhcx-driver` plus `imu-producer-interrupt-executor`.

| Feature | Purpose | Normal flight build? |
| --- | --- | --- |
| `ism330dhcx-driver` | Enables the real ISM330DHCX SPI/data-ready IMU driver dependency. | Yes; default. |
| `imu-producer-interrupt-executor` | Runs the IMU producer on the core 1 Embassy interrupt executor. | Yes; default. |
| `imu-odr-1666hz` | Selects the lower `1.666 kHz` ISM330DHCX output data rate. ODR means output data rate: the hardware sample production rate. | No; comparison/bring-up. |
| `ism330dhcx-1k666` | Backward-compatible alias for `imu-odr-1666hz`. | No; prefer `imu-odr-1666hz`. |
| `imu-400hz` | Legacy GY-91 MPU sample throttle used by old probe paths. It is not the current ISM330DHCX flight IMU path. | No. |
| `timing-diagnostics` | Enables `veloxity_core/timing-diagnostics`. | No; diagnostic-only. |
| `scope-timing-pins` | Enables board GPIO timing pulses for Saleae/logic-analyzer captures and `veloxity_core/scope-timing-pins`. | No; measurement-only. |
| `imu-producer-scope` | Uses the scope pin for IMU producer timing. | No. |
| `pre-control-scope` | Uses the scope pin for pre-control timing and enables the matching core feature. | No. |
| `rc-command-scope` | Uses the scope pin for RC command service timing and enables the matching core feature. | No. |
| `control-scope-estimator` | Selects estimator timing inside the control pipeline. | No. |
| `control-scope-controller` | Selects controller timing inside the control pipeline. | No. |
| `control-scope-mixer` | Selects mixer timing inside the control pipeline. | No. |
| `control-scope-pwm` | Selects PWM output timing inside the control pipeline. | No. |
| `release-loop-bench` | Legacy release-mode loop timing summaries. | No. |
| `release-loop-classifier` | Adds classification around the release loop benchmark path. | No. |
| `core1-disable-heartbeat` | Disables core 1 heartbeat work to isolate timing effects. | No; isolation diagnostic. |
| `core1-disable-mavlink-tx` | Disables core 1 MAVLink transmit work to isolate timing effects. | No. |
| `core1-disable-mavlink-rx` | Disables core 1 MAVLink receive work to isolate timing effects. | No. |
| `core1-disable-crsf` | Disables core 1 CRSF receiver work to isolate timing effects. | No. |
| `core1-disable-gps` | Disables core 1 GPS work to isolate timing effects. | No. |

## Pixracer Pro Features

These live in `boards/pixracerpro/Cargo.toml`.

| Feature | Purpose | Normal flight build? |
| --- | --- | --- |
| `legacy-run-once` | Uses the ordinary `World::run_once()` loop for A/B checks against the realtime scheduler. | No; comparison mode. |
| `timing-diagnostics` | Enables STM32 and core timing diagnostics. | No; diagnostic-only. |
| `scope-timing-pins` | Enables Pixracer Pro logic-analyzer timing outputs. | No; measurement-only. |
| `sensor-poll-diagnostics` | Enables board sensor-poll diagnostics. | No; diagnostic-only. |

## Sim Firmware Features

These live in `sim/firmware/Cargo.toml`.

| Feature | Purpose | Normal simulator run? |
| --- | --- | --- |
| `timing-diagnostics` | Enables core timing diagnostics in the host-side simulator firmware. | No; use only when investigating timing behavior. |

## STM32 Platform Features

These live in `platforms/stm_32/Cargo.toml`.

| Feature | Purpose | Normal flight build? |
| --- | --- | --- |
| `timing-diagnostics` | Enables shared STM32 peripheral timing counters used by Pixracer Pro diagnostics. | No; diagnostic-only. |

## Why Features Matter

Feature flags change the compiled firmware image. When reporting a timing result, always record the
exact feature set, target, optimization mode, and board. A build with timing pins or diagnostic text
is not identical to a clean flight build.
