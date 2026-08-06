# STM32 Boards

Veloxity currently has firmware crates for two STM32H7 boards:

- Nucleo-H753ZI: `boards/nucleo`
- Pixracer Pro: `boards/pixracerpro`

Pixracer Pro is the active STM32 hardware-validation target. Nucleo-H753ZI is retained and checked
for compilation, but its sensors and hardware behavior need renewed validation.

For the meaning of the repository's check, build, and flash commands, see
[Build and Tool Commands](../build-and-tools.md). For the shared `World`, `BoardIo`, and scheduler
architecture, see [Veloxity Core Architecture](../architecture-guide.md).

## Board Differences

Both crates initialize STM32 peripherals, implement `BoardIo`, construct a concrete `World`, and
run Embassy tasks that collect sensor and communication data. The important differences are:

| Area | Nucleo-H753ZI | Pixracer Pro |
| --- | --- | --- |
| Current status | Compiles; renewed hardware validation needed. | Active STM32 hardware-validation target. |
| Firmware loop | Calls `World::run_once()`. | Uses the realtime scheduler with a fixed `400 Hz` control rate. |
| IMU | BMI088 at `400 Hz`. | BMI085 at `400 Hz`.|
| Magnetometer | IIS2MDC. | IST8308. |
| Air-data and range sensors | DLHRL20G is constructed, but its task is currently disabled. | MS4525 airspeed and LLV3HP range sensor.|
| MAVLink transport | Telemetry UART. | Telemetry UART by default; USB VCP is an optional build feature. |
| Outputs | Twelve standard PWM outputs. | Seven standard PWM outputs. |

The Pixracer Pro service policy polls service work repeatedly while enough time remains before the
next control deadline. This lets MAVLink and slower sensors make progress without moving them into
the IMU/control path. The detailed scheduler behavior can be found in the
[architecture guide](../architecture-guide.md#realtime-scheduler).

## Source Layout

| Path | Purpose |
| --- | --- |
| `boards/nucleo/src/bin/veloxity.rs` | Nucleo firmware entry point and `World` construction. |
| `boards/nucleo/src/board.rs` | Nucleo peripheral setup and `BoardIo` implementation. |
| `boards/pixracerpro/src/bin/veloxity.rs` | Pixracer Pro firmware entry point and `World` construction. |
| `boards/pixracerpro/src/board.rs` | Pixracer Pro peripheral setup and `BoardIo` implementation. |
| `boards/pixracerpro/src/pwm.rs` | Pixracer Pro PWM adapter. |
| `platforms/stm_32/stm32h7x3_common.rs` | Shared STM32H7 clock and interrupt configuration. |
| `platforms/stm_32/src/peripherals/` | Shared STM32 peripheral drivers and Embassy tasks. |

## Quick Start

Install the Rust target and probe-rs tooling:

```bash
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools
```

Check or build the board you intend to use:

```bash
cargo xtask check-board nucleo
cargo xtask build-board nucleo

cargo xtask check-board pixracerpro
cargo xtask build-board pixracerpro
```

`check-board` type-checks the selected firmware. `build-board` produces a development-profile
firmware binary under `target/`; it does not connect to or modify a board.

### Flashing warning

The repository currently configures one runner for every `thumbv7em-none-eabihf` crate:

```toml
runner = "probe-rs run --chip STM32H743IIKx"
```

That chip selection is intended for Pixracer Pro. The `flash-board` command does not currently
replace it with a board-specific chip, so the nucleo cannot be flashed without a full probe-rs command (to be included here after our team revalidates the nucleo)

List connected probes before flashing:

```bash
probe-rs list
```

For the currently configured Pixracer Pro path:

```bash
cargo xtask flash-board pixracerpro
```

This flashes an optimized release build with UART MAVLink transport and no optional features. The
Pixracer Pro diagnostic and USB options are documented in [Feature Flags](../features.md).
Use the [BMI08x identity probe](../pixracerpro-bmi08x-id-probe.md) to distinguish the onboard
BMI085 and BMI088 variants before selecting an accelerometer range configuration.
For the external 3DR NEO-M9N/IST8308 module, see the
[magnetometer orientation and validation guide](../pixracerpro-3dr-m9n-ist8308-orientation.md).

## Hardware And Peripheral Map

The shared STM32 platform contains more drivers than either board currently uses. This table
distinguishes active board tasks from code that is only available in the platform:

| Driver or path | Nucleo-H753ZI | Pixracer Pro |
| --- | --- | --- |
| BMI08x IMU | Active as BMI088. | Active as BMI085. |
| DPS310 barometer | Active. | Active. |
| IIS2MDC magnetometer | Active. | Not used. |
| IST8308 magnetometer | Not used. | Active. |
| DLHRL20G airspeed | Constructed, but task disabled. | Not used. |
| MS4525 airspeed | Not used. | Active. |
| LLV3HP range sensor | Not used. | Active. |
| u-blox GNSS | Active. | Active. |
| PPS input | Active. | Active. |
| SBUS RC input | Active. | Active. |
| Telemetry UART | Active MAVLink transport. | Active by default. |
| USB VCP | USB task is started, but `BoardIo` uses the telemetry UART. | Optional MAVLink transport selected by `usb-vcp-serial`. |
| SD card | Task is started. | Task is started. |
| ADIS16500 IMU | Shared driver available; not initialized by either board. | Shared driver available; not initialized by either board. |

## Pixracer Pro Validation Record

The repository offers a bench test for recording a 10-second Pixracer Pro HAL UART run at `921600` baud.
The following results demonstrate the rates tested by our team using this benchmarking:

| Check | Recorded result |
| --- | --- |
| Valid MAVLink frames | `6420` |
| Packet integrity | Zero CRC errors, sequence gaps, estimated missing frames, or duplicates. |
| IMU | `399.5 Hz` host rate and `399.4 Hz` board-timestamp rate. |
| Attitude | `50.0 Hz`. |
| Barometer | `25.0 Hz`. |
| RC | `100.0 Hz`. |
| Output raw | `50.0 Hz`. |
| Status | `10.0 Hz`. |
| Heartbeat | `1.0 Hz`. |
| TIMESYNC | `5.0 Hz`. |
| Request/response paths | Command acknowledgement, version response, and `334` parameter-response frames were observed. |
| Firmware loop timing | Approximately `406.2 us` average and `468 us` maximum within the `2.5 ms` control period. |
| Scope timing | BMI08x production and foreground IMU consumption remained at `400 Hz`; measured producer-to-consumer latency was below `100 us`. |

This run also injected ground-station heartbeat and TIMESYNC messages plus version and parameter
requests.

### Repeating the acceptance test

Use the repository tester with the serial device for your adapter:

```bash
python3 tools/mavlink_tester.py \
  --transport uart \
  --device /dev/serial/by-id/YOUR_ADAPTER \
  --baud 921600 \
  --duration-s 10 \
  --warmup-s 0.5 \
  --diagnostics \
  --acceptance \
  --expect-imu-hz 400 \
  --expect-rc-hz 100 \
  --expect-attitude-hz 50 \
  --expect-output-raw-hz 50
```

The acceptance mode injects heartbeat, TIMESYNC, version, and parameter requests and fails when
required telemetry or replies are missing. It does not enforce every expected rate, so inspect the
reported rates as well as the final pass/fail result.

For timing captures, flash the diagnostic build:

```bash
cargo xtask flash-board pixracerpro --scope-timing-pins
```

The feature does not permanently assign meanings to the test pins. Add temporary pin transitions
around the code being measured, record the assignment with the capture, and remove the temporary
instrumentation afterward.
