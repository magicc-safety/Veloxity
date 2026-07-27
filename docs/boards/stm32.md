# STM32 Boards

This page covers the STM32 board paths:

- Nucleo-H753ZI: `boards/nucleo`
- Pixracer Pro / STM32H7: `boards/pixracerpro`
- shared platform code: `platforms/stm_32`

These boards are kept in the repository because they are part of the intended hardware support
matrix. Pixracer Pro is now the active STM32 validation path. Nucleo-H753ZI remains compile-current
and should still be treated as awaiting renewed hardware validation.

## Source Layout

| Path                                     | Purpose                                                     |
| ---------------------------------------- | ----------------------------------------------------------- |
| `boards/nucleo/src/bin/veloxity.rs`      | Nucleo firmware entry point and `World` construction.       |
| `boards/nucleo/src/board.rs`             | Nucleo implementation of board setup and `BoardIo`.         |
| `boards/pixracerpro/src/bin/veloxity.rs` | Pixracer Pro firmware entry point and `World` construction. |
| `boards/pixracerpro/src/board.rs`        | Pixracer Pro implementation of board setup and `BoardIo`.   |
| `boards/pixracerpro/src/pwm.rs`          | Pixracer Pro PWM driver.                                    |
| `platforms/stm_32/stm32h7x3_common.rs`   | Shared STM32H7 configuration.                               |
| `platforms/stm_32/src/peripherals/`      | Shared STM32 peripheral drivers and Embassy signal tasks.   |

## Firmware Model

The STM32 boards follow the generic embedded firmware shape:

- the board crate initializes chip clocks, pins, serial transports, sensor peripherals, and PWM;
- Embassy peripheral tasks produce packets or signal new sensor data to board-owned queues;
- the board `BoardIo` implementation drains those queues into `veloxity_core` sensor resources;
- the board constructs a `World` with STM32-specific board, PWM, and MAVLink transport types;
- Pixracer Pro uses the realtime `World` scheduler entrypoints with a fixed `400 Hz` control update
  baseline and board-specific continuous service polling;
- Nucleo keeps the ordinary `World::run_once()` firmware loop for now, while its `BoardIo` adapter
  exposes the same IMU/service sensor split so it stays compile-current.

<!-- That still differs from the active Pico 2 W path. Pico 2 W uses a dual-core board runtime around the -->
<!-- same core scheduler. -->

STM32 has an interrupt executor model: peripheral tasks produce IMU, RC, and
other sensor packets, and one high-level firmware loop owns `World`. The first Pixracer Pro port
does not rewrite driver tasks; it changes only how board-owned packets are presented to the core
fast path and service path.

The realtime Pixracer Pro entrypoint uses a board-specific service policy. It keeps the shared
core defaults intact, but uses a Pixracer-owned continuous polling policy that attempts prioritized
service work back-to-back while the control-slack guard remains satisfied. Fresh RC gets handled
immediately after the service sensor drain instead of waiting for a later circular service phase.
The control-slack guard intentionally allows service when a fixed-rate control deadline is overdue
but no accumulated IMU sample is available; otherwise stale deadline bookkeeping can block service
for hundreds of milliseconds to protect a control update that cannot run. Hardware diagnostics
showed that UART baud, TX pipe drain, and final send gating were not the limiter; telemetry needed
regular service opportunities in the measured control slack. The continuous service policy is
Pixracer Pro-specific.
 <!-- until RP2350/Pico 2 W is retested for consistency. -->

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
cargo check -p nucleo --target thumbv7em-none-eabihf --bin veloxity
cargo check -p pixracerpro --target thumbv7em-none-eabihf --bin veloxity
```

## Build

```bash
cargo xtask build-board nucleo
cargo xtask build-board pixracerpro
```

Direct equivalents:

```bash
cargo build -p nucleo --target thumbv7em-none-eabihf --bin veloxity
cargo build -p pixracerpro --target thumbv7em-none-eabihf --bin veloxity
```

## Flash Or Run

Prefer the repository wrapper when the local runner is configured:

```bash
cargo xtask flash-board nucleo
cargo xtask flash-board pixracerpro
```

The Pixracer Pro wrapper command flashes a release build with UART transport and no optional
features by default. Use `--vcp` to select USB VCP; `--scope-timing-pins` and
`--sensor-poll-diagnostics` are separate explicit opt-ins and can be combined with it.

Direct `cargo run` is also valid when the board crate runner and probe selection match the connected
hardware:

```bash
cargo run -p nucleo --target thumbv7em-none-eabihf --bin veloxity
cargo run -p pixracerpro --target thumbv7em-none-eabihf --bin veloxity
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

| File                                            | Device/path           |
| ----------------------------------------------- | --------------------- |
| `platforms/stm_32/src/peripherals/adis16500.rs` | ADIS16500 IMU         |
| `platforms/stm_32/src/peripherals/bmi08x.rs`    | BMI08x IMU            |
| `platforms/stm_32/src/peripherals/dps310.rs`    | DPS310 barometer      |
| `platforms/stm_32/src/peripherals/iis2mdc.rs`   | IIS2MDC magnetometer  |
| `platforms/stm_32/src/peripherals/ist8308.rs`   | IST8308 magnetometer  |
| `platforms/stm_32/src/peripherals/ms4525.rs`    | MS4525 airspeed       |
| `platforms/stm_32/src/peripherals/sbus.rs`      | SBUS RC               |
| `platforms/stm_32/src/peripherals/telem.rs`     | Telemetry serial path |
| `platforms/stm_32/src/peripherals/ublox.rs`     | u-blox GNSS           |
| `platforms/stm_32/src/peripherals/vcp.rs`       | USB virtual COM port  |

The current compatibility update makes the ADIS16500 and BMI08x IMU packet signals explicit as
`ImuPacket<f64>`, matching their existing `f64` sensor math and the current generic packet type in
`veloxity_core`.

## Pixracer Pro Timing And Telemetry Validation

Pixracer Pro has been validated on hardware with the bounded high-rate MAVLink profile:

| Stream            | Configured rate             | Observed result                                        |
| ----------------- | --------------------------- | ------------------------------------------------------ |
| Attitude          | `50 Hz`                     | `50.0 Hz`.                                             |
| Barometer         | `25 Hz`                     | `25.0 Hz`.                                             |
| Command ACK       | On demand                   | Single response frame.                                 |
| GNSS              | No fix expected in this run | `0.0 Hz`, no frames.                                   |
| Heartbeat         | `1 Hz`                      | `1.0 Hz`.                                              |
| IMU               | `400 Hz`                    | `399.5 Hz` host rate, `399.4 Hz` board timestamp rate. |
| Output raw        | `50 Hz`                     | `50.0 Hz`.                                             |
| Parameter traffic | Request/response burst      | `2563.1 Hz` during the parameter burst, `334` frames.  |
| RC                | `100 Hz`                    | `100.0 Hz`.                                            |
| Status            | `10 Hz`                     | `10.0 Hz`.                                             |
| TIMESYNC          | `5 Hz`                      | `5.0 Hz`.                                              |
| Version response  | On demand                   | Single response frame.                                 |

For messages with board timestamps, the latest run measured IMU at `399.4 Hz`, attitude at
`50.0 Hz`, output raw at `50.0 Hz`, RC at `100.0 Hz`, and TIMESYNC at `5.0 Hz` on the board side.

The latest 10-second UART MAVLink acceptance run at `921600` baud passed with `6420` valid MAVLink
frames, zero CRC errors, zero MAVLink sequence gaps, zero estimated missing frames, and zero
duplicates. The run injected ground-station heartbeat and TIMESYNC frames plus version and parameter
requests, so it exercises telemetry, parser, and response paths. It is not a substitute for a full
flight-command profile; offboard/setpoint/RC override traffic should be tested separately if those
are part of the mission.

Control timing stayed well inside the `2.5 ms` period. The latest acceptance run reported firmware
loop timing around `406 us` average and `468 us` max. Saleae captures showed the BMI08x producer,
foreground IMU consumption, and control pipeline cadence all staying inside the `400 Hz` timing
budget.

Use this diagnostic firmware when validating the current MAVLink throughput issue:

```bash
cargo build -p pixracerpro --target thumbv7em-none-eabihf --bin veloxity --release \
  --features 'scope-timing-pins'
```

The `scope-timing-pins` feature keeps the GPIO timing path buildable for targeted Saleae captures,
but the branch does not assign a permanent meaning to Pixracer Pro test pins. Place short-lived
test-pin calls around the producer, consumer, service, or control section being measured, then use
the MAVLink tester to assess emitted stream rates and packet health.

Diagnostic decision record:

- Initial Pixracer Pro runs produced only about 75% of the requested high-rate streams even though
  control timing had substantial slack.
- Increasing UART baud and increasing the per-service stream budget did not materially improve the
  rates.
- Historical TX queue/drain counters matched exactly with zero partial writes or errors, ruling
  out UART baud, TX pipe capacity, and async drain as the limiter.
- Scope captures showed the foreground scheduler could stop entering service for about `500 ms`
  while control remained healthy. The root cause was the service slack guard using stale fixed-rate
  deadline state when no accumulated IMU sample was available, so it blocked service to protect a
  control update that could not run.
- The scheduler now allows service in that stale-deadline/no-IMU case. Post-fix service-off gaps
  stayed below `1 ms` in the validation capture.
- Producer/consumer scope validation showed a clean `400 Hz` BMI08x producer cadence, matching
  foreground IMU consumption count, and producer-to-consumer latency below `100 us`.
- The accepted UART validation run held IMU telemetry at `399.5 Hz` host rate and `399.4 Hz` board
  timestamp rate for 10 seconds at `921600` baud, with zero CRC errors, zero MAVLink sequence gaps,
  injected heartbeat/TIMESYNC/version/parameter traffic, and firmware loop timing avg `406.2 us`,
  max `468 us`.

Those historical onboard counters were useful for identifying the scheduling problem, but current
timing validation should use scope pins plus external MAVLink rate and packet-health tooling.

## Bring-Up Order

For renewed STM32 validation, use this order:

1. Confirm both board crates compile.
2. Flash the board with `probe-rs`.
3. Confirm the firmware reaches the main loop.
4. Bring up serial/MAVLink communication.
5. Validate IMU packet production.
6. Validate barometer and magnetometer packet production.
7. Validate GNSS when hardware lock/data is available.
8. Validate RC input.
9. Confirm PWM output behavior with props removed.
10. Run the loaded MAVLink acceptance test with any expected real-flight command streams.
11. Only then compare behavior against the simulator and ROSflight C firmware.

Do not treat stale STM32 notes from older branches as authoritative. Update this page and the
hardware runbook as each sensor path is revalidated.
