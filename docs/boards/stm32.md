# STM32 Boards

This page covers the STM32 board paths:

- Nucleo-H753ZI: `boards/nucleo`
- Pixracer Pro / STM32H7: `boards/pixracerpro`
- shared platform code: `platforms/stm_32`

These boards are kept in the repository because they are part of the intended hardware support
matrix. Pixracer Pro is now the active STM32 validation path. Nucleo-H753ZI remains compile-current
and should still be treated as awaiting renewed hardware validation.

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
- Pixracer Pro uses the realtime `World` scheduler entrypoints with a fixed `400 Hz` control update
  baseline and board-specific post-control telemetry scheduling;
- Nucleo keeps the ordinary `World::run_once()` firmware loop for now, while its `BoardIo` adapter
  exposes the same IMU/service sensor split so it stays compile-current.

That still differs from the active Pico 2 W path. Pico 2 W uses a dual-core board runtime around the
same core scheduler. STM32 keeps its interrupt executor model: peripheral tasks produce IMU, RC, and
other sensor packets, and one high-level firmware loop owns `World`. The first Pixracer Pro port
does not rewrite driver tasks; it changes only how board-owned packets are presented to the core
fast path and service path.

Pixracer Pro has a `legacy-run-once` feature for A/B testing against the ordinary `World::run_once()`
loop:

```bash
cargo check -p pixracerpro --target thumbv7em-none-eabihf --features legacy-run-once
```

The realtime Pixracer Pro entrypoint uses a board-specific telemetry policy. It keeps the shared
core defaults intact, but asks the realtime service step to send more named telemetry streams per
service opportunity and sends up to four telemetry streams immediately after each completed control
update. Hardware diagnostics showed that UART baud, TX pipe drain, and final send gating were not
the limiter; telemetry needed more scheduling opportunities in the measured post-control slack.
The post-control burst is intentionally Pixracer Pro-specific until RP2350/Pico 2 W is retested for
consistency.

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

## Pixracer Pro Timing And Telemetry Validation

Pixracer Pro has been validated on hardware with the bounded high-rate MAVLink profile:

| Stream | Configured rate | Observed result |
| --- | --- | --- |
| IMU | `400 Hz` | About `398 Hz` in the latest 120-second burst-4 run. |
| RC | `100 Hz` | `100.0 Hz`. |
| Attitude | `50 Hz` | `50.0 Hz`. |
| Output raw | `50 Hz` | `50.0 Hz`. |
| Status | `10 Hz` | `10.0 Hz`. |
| Heartbeat | `1 Hz` | `1.0 Hz`. |

The latest 120-second bidirectional MAVLink load at `921600` baud passed acceptance with zero CRC
errors, zero MAVLink sequence gaps, and about `29.7 kB/s` RX throughput. TX enqueue and UART drain
matched exactly with no partial writes or errors. The run injected ground-station MAVLink heartbeat
and TIMESYNC frames plus version and parameter requests, so it exercises telemetry, parser, and
response paths. It is not a substitute for a full flight-command profile; offboard/setpoint/RC
override traffic should be tested separately if those are part of the mission.

Control timing stayed well inside the `2.5 ms` period. The latest burst-4 run reported firmware
loop timing around `406 us` average, `471 us` p99, and `534 us` max, with control perf max around
`598 us`. That leaves roughly `1.9 ms` of slack in the worst observed control pass. Saleae captures
show PD12 control-active pulses completing well before the next PD11 `400 Hz` deadline marker.

Use this diagnostic firmware when validating the current MAVLink throughput issue:

```bash
cargo build -p pixracerpro --target thumbv7em-none-eabihf --bin voloxide --release \
  --features 'scope-timing-pins timing-diagnostics'
```

The `scope-timing-pins` feature maps the Pixracer Pro timing signals as follows:

| Pin | Meaning |
| --- | --- |
| PD11 | 400 Hz control-deadline marker |
| PD12 | Control pipeline active time |

PD11 should pulse every `2.5 ms`. PD12 should remain well shorter than that period and should not
overlap the next PD11 marker. The post-control telemetry burst is outside the measured control
pipeline active pulse; use MAVLink timing diagnostics and TXQ/TXD counters to assess telemetry load.

The `timing-diagnostics` feature emits STATUSTEXT diagnostics, including:

| Prefix | Meaning |
| --- | --- |
| `TXQ` | Firmware writes into the telemetry TX pipe: attempts, full-frame successes, attempted bytes, accepted bytes, partial errors, and total errors. |
| `TXD` | Async UART TX task drain/write counters: pipe reads, read bytes, UART writes, written bytes, and UART errors. |
| `TMS` | Rotating telemetry scheduler counters per stream: eligible during actual send attempts, selected, sent, and selected-but-failed-final-gate counts. |

Diagnostic decision record:

- Initial Pixracer Pro runs produced only about 75% of the requested high-rate streams even though
  control timing had substantial slack.
- Increasing UART baud and increasing the per-service stream budget did not materially improve the
  rates.
- `TXQ` and `TXD` matched exactly with zero partial writes or errors, ruling out UART baud, TX pipe
  capacity, and async drain as the limiter.
- `TMS` showed selected streams sent successfully (`f0`), so the missing frames were caused by too
  few scheduling opportunities, not final send gating.
- Adding a small post-control telemetry burst fixed the rates. Burst `3` hit the target streams;
  burst `4` is the current Pixracer Pro value for extra scheduling margin and still keeps the
  `400 Hz` control loop comfortably inside budget.

`TMS` emits one stream per diagnostic interval so scheduler visibility does not flood the shared
STATUSTEXT response queue. Readiness probes such as `named_telemetry_due()` do not update TMS
counters; the counters describe actual realtime send attempts.

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
