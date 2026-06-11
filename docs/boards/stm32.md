# STM32 Boards

This page covers the retained STM32 board paths:

- Nucleo-H753ZI: `boards/nucleo`
- Pixracer Pro / STM32H7: `boards/pixracerpro`
- shared platform code: `platforms/stm_32`

These boards are kept in the repository because they are part of the intended hardware support
matrix. They are behind the Pico 2 W path in validation on this branch, so a successful compile
check should be treated as the starting point for renewed sensor bring-up, not proof of flight
readiness.

## Source Layout

| Path | Purpose |
| --- | --- |
| `boards/nucleo/src/bin/voloxide.rs` | Nucleo firmware entry point and `World` construction. |
| `boards/nucleo/src/board.rs` | Nucleo implementation of board setup and `BoardIo`. |
| `boards/pixracerpro/src/bin/voloxide.rs` | Pixracer Pro firmware entry point and `World` construction. |
| `boards/pixracerpro/src/board.rs` | Pixracer Pro implementation of board setup and `BoardIo`. |
| `boards/pixracerpro/src/pwm.rs` | Pixracer Pro PWM driver. |
| `platforms/stm_32/stm32h7x3_common.rs` | Shared STM32H7 configuration. |
| `platforms/stm_32/src/peripherals/` | Shared STM32 peripheral drivers and Embassy signal tasks. |

## Firmware Model

The STM32 boards follow the generic embedded firmware shape:

- the board crate initializes chip clocks, pins, serial transports, sensor peripherals, and PWM;
- Embassy peripheral tasks produce packets or signal new sensor data to board-owned queues;
- the board `BoardIo` implementation drains those queues into `voloxide_core` sensor resources;
- the board constructs a `World` with STM32-specific board, PWM, and MAVLink transport types;
- the firmware loop uses the ordinary `World::run_once()` scheduler shape until renewed hardware
  work proves a need for a board-specific realtime split.

That differs from the active Pico 2 W path. Pico 2 W uses a measured high-rate IMU/control loop with
`realtime_scheduler_step()`, `run_imu_control_tick()`, and service phases. STM32 has richer retained
sensor-driver coverage in `platforms/stm_32`, but those paths have not yet gone through the same
current-branch timing and end-to-end telemetry validation.

## Install

```bash
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools
```

## Check

```bash
cargo xtask check-board nucleo
cargo xtask check-board pixracerpro
```

Direct equivalents:

```bash
cargo check -p nucleo --target thumbv7em-none-eabihf
cargo check -p pixracerpro --target thumbv7em-none-eabihf
```

## Build

```bash
cargo xtask build-board nucleo
cargo xtask build-board pixracerpro
```

Direct equivalents:

```bash
cargo build -p nucleo --target thumbv7em-none-eabihf --bin voloxide
cargo build -p pixracerpro --target thumbv7em-none-eabihf --bin voloxide
```

## Flash Or Run

Prefer the repository wrapper when the local runner is configured:

```bash
cargo xtask flash-board nucleo
cargo xtask flash-board pixracerpro
```

Direct `cargo run` is also valid when the board crate runner and probe selection match the connected
hardware:

```bash
cargo run -p nucleo --target thumbv7em-none-eabihf --bin voloxide
cargo run -p pixracerpro --target thumbv7em-none-eabihf --bin voloxide
```

Treat flashing as the start of renewed validation, not proof of readiness. The exact probe
selection may need to be supplied by your local `probe-rs` setup. Check attached probes with:

```bash
probe-rs list
probe-rs info --chip STM32H743ZI
```

Use the chip name that matches the connected board.

## Shared Peripheral Drivers

The STM32 platform exposes peripheral tasks that signal packet results to board code. Important
driver files include:

| File | Device/path |
| --- | --- |
| `platforms/stm_32/src/peripherals/adis16500.rs` | ADIS16500 IMU |
| `platforms/stm_32/src/peripherals/bmi08x.rs` | BMI08x IMU |
| `platforms/stm_32/src/peripherals/dps310.rs` | DPS310 barometer |
| `platforms/stm_32/src/peripherals/iis2mdc.rs` | IIS2MDC magnetometer |
| `platforms/stm_32/src/peripherals/ist8308.rs` | IST8308 magnetometer |
| `platforms/stm_32/src/peripherals/ms4525.rs` | MS4525 airspeed |
| `platforms/stm_32/src/peripherals/sbus.rs` | SBUS RC |
| `platforms/stm_32/src/peripherals/telem.rs` | Telemetry serial path |
| `platforms/stm_32/src/peripherals/ublox.rs` | u-blox GNSS |
| `platforms/stm_32/src/peripherals/vcp.rs` | USB virtual COM port |

The current compatibility update makes the ADIS16500 and BMI08x IMU packet signals explicit as
`ImuPacket<f64>`, matching their existing `f64` sensor math and the current generic packet type in
`voloxide_core`.

## Bring-Up Order

For renewed STM32 validation, use this order:

1. Confirm both board crates compile.
2. Flash the board with `probe-rs`.
3. Confirm the firmware reaches the main loop.
4. Bring up serial/MAVLink communication.
5. Validate IMU packet production.
6. Validate barometer and magnetometer packet production.
7. Validate RC input.
8. Confirm PWM output behavior with props removed.
9. Only then compare behavior against the simulator and ROSflight C firmware.

Do not treat stale STM32 notes from older branches as authoritative. Update this page and the
hardware runbook as each sensor path is revalidated.
