# Repository Map

This repository is a Rust workspace plus a ROS 2 shim package. Source code lives in the workspace
members. Generated build outputs live in `target/` and `workspace/` and should not be committed.

## Top-Level Folders

| Path | Keep? | Purpose |
| --- | --- | --- |
| `assets/` | Yes | Static project assets used by documentation. |
| `boards/` | Yes | Board applications. Each board crate chooses pins, peripherals, board I/O, PWM driver, and the concrete `World` type. |
| `comms/` | Yes | Communication adapters. `voloxide_mavlink` implements ROSflight MAVLink transport for the core firmware. |
| `crates/` | Yes | Firmware libraries. `voloxide_core` is the board-independent flight stack. |
| `docs/` | Yes | Current branch documentation. This should describe tested workflows and retained implementation paths. |
| `platforms/` | Yes | Reusable platform support shared by board crates. |
| `scripts/` | Yes | Repository helper scripts that orchestrate local builds. |
| `sim/` | Yes | Host simulator firmware crate and ROS 2 shim package. |
| `third_party/` | Yes | Vendored dependency patches needed by the workspace. |
| `tools/` | Yes | Standalone tools that are not part of the flight core, including the ESP32C5 ESP-NOW UART bridge. |
| `xtask/` | Yes | Repository command wrapper used for common build, test, flash, and cleanup commands. |
| `target/` | Generated | Cargo build output. Remove with `cargo xtask clean-generated`. |
| `workspace/` | Generated | Local colcon workspace used to build the ROS 2 shim overlay. Remove with `cargo xtask clean-generated`. |
| `rosflight_memory/` | Generated | Local runtime parameter/state artifact path from older runs. Remove with `cargo xtask clean-generated`. |

## Rust Workspace Members

The workspace is declared in `Cargo.toml`.

| Member | Role |
| --- | --- |
| `crates/voloxide_core` | `no_std` flight stack: params, state machine, sensors, estimator, controller, mixer, PWM, telemetry scheduling, and the `World` scheduler. |
| `comms/voloxide_mavlink` | MAVLink parser and ROSflight MAVLink adapter implementing `CommInterface`. |
| `sim/firmware` | Host-side Rust firmware static library and FFI boundary for the ROS 2 shim. |
| `boards/pico2w` | RP2350/Pico 2 W board firmware and hardware probes. |
| `boards/nucleo` | Nucleo-H753ZI board firmware. |
| `boards/pixracerpro` | Pixracer Pro board firmware. |
| `platforms/rp2350` | Reusable RP2350 metadata for multicore and PIO allocation. |
| `platforms/stm_32` | Shared STM32/Embassy peripheral drivers. |
| `xtask` | Local command runner invoked as `cargo xtask ...`. |

Root `cargo build` uses workspace `default-members`, which are host-compatible:

```text
comms/voloxide_mavlink
crates/voloxide_core
sim/firmware
xtask
```

Embedded board crates are built explicitly with `cargo xtask check-board ...` or
`cargo xtask build-board ...` because they target Cortex-M.

## Source Boundaries

### Core

`crates/voloxide_core` must stay board-independent. It defines traits such as `BoardIo`,
`CommInterface`, `Estimator`, `Controller`, `Mixer`, and `PwmDriver`. Board crates and sim crates
provide concrete implementations.

### Communication

`comms/voloxide_mavlink` is the protocol adapter. Core owns protocol-neutral message structs and
command handling; `voloxide_mavlink` parses and emits MAVLink frames.

### Simulation

`sim/firmware` exposes the Rust firmware through C ABI functions such as `voloxide_sim_create`,
`voloxide_sim_set_sensors`, `voloxide_sim_run_once`, and `voloxide_sim_get_pwm`.

`sim/ros2/voloxide_sil_board_shim` is a ROS 2 package. It links `target/debug/libsim.a` and presents
the Rust firmware as the SIL board endpoint expected by ROSflight.

### Board Crates

Board crates own physical integration:

- pin assignments
- peripheral initialization
- sensor queues
- serial or mailbox transport
- PWM output driver
- `World` instantiation

They should not reimplement flight logic that belongs in `voloxide_core`.

Current RP2350/Pico 2 W work is concentrated in:

| Path | Purpose |
| --- | --- |
| `boards/pico2w/src/bin/voloxide.rs` | Main RP2350 firmware entry point, Embassy task setup, core split, and IMU ODR feature selection. |
| `boards/pico2w/src/board.rs` | `BoardIo` implementation, sensor queue drains, serial flush budget, and service hooks. |
| `boards/pico2w/src/ism330dhcx.rs` | ISM330DHCX data-ready/SPI packet producer path. |
| `boards/pico2w/src/rc_receiver.rs` | CRSF receiver path feeding service-side RC state. |
| `boards/pico2w/src/comms_core.rs` | Core-to-transport MAVLink mailbox used by UART/ESP-NOW testing. |
| `platforms/rp2350/src/multicore.rs` | RP2350 core-role metadata. |
| `platforms/rp2350/src/pio.rs` | Shared PIO allocation metadata. |

Current retained STM32 work is concentrated in:

| Path | Purpose |
| --- | --- |
| `boards/nucleo/src/bin/voloxide.rs` | Nucleo-H753ZI firmware entry point and `World` construction. |
| `boards/nucleo/src/board.rs` | Nucleo `BoardIo` and board setup. |
| `boards/pixracerpro/src/bin/voloxide.rs` | Pixracer Pro firmware entry point and `World` construction. |
| `boards/pixracerpro/src/board.rs` | Pixracer Pro `BoardIo` and board setup. |
| `boards/pixracerpro/src/pwm.rs` | Pixracer Pro PWM driver. |
| `platforms/stm_32/src/peripherals/` | Shared STM32 sensor, serial, and signal-task drivers. |
| `platforms/stm_32/stm32h7x3_common.rs` | Shared STM32H7 configuration. |

### Platform Crates

Platform crates own reusable chip-family code. Board crates depend on them when a concept applies
to more than one board.

## Generated Files

These are disposable:

```text
target/
workspace/build/
workspace/install/
workspace/log/
rosflight_memory/
tools/__pycache__/
tools/espnow_uart_bridge/build*/
tools/espnow_uart_bridge/sdkconfig
tools/espnow_uart_bridge/dependencies.lock
```

Clean them with:

```bash
cargo xtask clean-generated
```

Do not delete source defaults such as `tools/espnow_uart_bridge/sdkconfig.*.defaults`; those are the
checked-in build configurations for the bridge roles.
