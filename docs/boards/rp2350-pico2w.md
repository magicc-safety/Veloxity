# RP2350 / Pico 2 W

The Pico 2 W is the active hardware bring-up board for this branch. The firmware crate is
`boards/pico2w`; reusable RP2350 metadata lives in `platforms/rp2350`.

## Source Layout

| Path | Purpose |
| --- | --- |
| `boards/pico2w/src/bin/voloxide.rs` | Main firmware entry point, Embassy tasks, core split, and `World` construction. |
| `boards/pico2w/src/board.rs` | Pico implementation of the `BoardIo` contract. |
| `boards/pico2w/src/comms_core.rs` | Shared MAVLink mailbox between transport tasks and the flight core. |
| `boards/pico2w/src/pwm.rs` | PIO PWM/DShot-facing driver implementation. |
| `boards/pico2w/src/ism330dhcx.rs` | ISM330DHCX IMU packet path. |
| `boards/pico2w/src/barometer.rs` | Barometer packet path. |
| `boards/pico2w/src/gy91.rs` | Legacy GY-91/BMP280 support used as low-rate pressure path. |
| `boards/pico2w/src/gps.rs` | GPS and magnetometer path. |
| `boards/pico2w/src/rc_receiver.rs` | CRSF RC receiver path. |
| `boards/pico2w/src/bin/*probe.rs` | Hardware probes for individual buses and sensors. |
| `platforms/rp2350/src/multicore.rs` | Shared RP2350 core-role metadata. |
| `platforms/rp2350/src/pio.rs` | Shared RP2350 PIO allocation metadata. |

## Firmware Model

The intended runtime split is:

- core 0 runs the Voloxide flight-control `World`
- core 1 owns communication services that can jitter without blocking the flight loop
- PIO owns timing-sensitive output/input work
- `BoardIo::update_sensor_bus()` drains the newest board-local sensor packets into core resources

The control loop is IMU-driven: the control pipeline runs only when a processed IMU packet has a new
timestamp.

## Install

```bash
rustup target add thumbv8m.main-none-eabihf
cargo install probe-rs-tools
```

## Check And Build

```bash
cargo xtask check-board pico2w
cargo xtask build-board pico2w
```

Release build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release
```

Useful feature build for the current high-rate IMU path:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'ism330dhcx-driver ism330dhcx-3k333 imu-producer-interrupt-executor'
```

Logic-analyzer timing build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'ism330dhcx-driver ism330dhcx-3k333 scope-timing-pins control-scope-controller imu-producer-interrupt-executor'
```

Timing diagnostics build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'timing-diagnostics ism330dhcx-driver ism330dhcx-3k333 imu-producer-interrupt-executor'
```

Use the logic-analyzer timing build when measuring loop timing with GPIO instead of MAVLink
statustext diagnostics. Do not enable `timing-diagnostics`, `release-loop-bench`,
`release-loop-classifier`, or `release-loop-spike-counter` for the cleanest timing measurement.

## Flash

With a debug probe attached:

```bash
probe-rs download --chip RP235x --protocol swd \
  target/thumbv8m.main-none-eabihf/release/voloxide

probe-rs reset --chip RP235x
```

If multiple probes are visible, add `--probe <VID:PID:SERIAL>`.

Example from current bring-up notes:

```bash
probe-rs download --probe 2e8a:000c-0:E6647C7403301534 \
  --chip RP235x \
  --protocol swd \
  target/thumbv8m.main-none-eabihf/release/voloxide

probe-rs reset --chip RP235x
```

## Companion UART Wiring

The XIAO ESP32C5 bridge is electrically just a UART peer from the Pico point of view.

| Pico 2 W | XIAO ESP32C5 | Meaning |
| --- | --- | --- |
| GP0 / UART0 TX | D7 / RX / GPIO12 | Pico MAVLink TX to ESP32C5 RX |
| GP1 / UART0 RX | D6 / TX / GPIO11 | ESP32C5 TX to Pico MAVLink RX |
| GND | GND | Common ground |

Current UART speed:

```text
2_000_000 baud
```

For the full physical pinout, see [Pico 2 W flight hardware pinout](../pico2w-esc-imu-pinout.md).

## Realtime Scheduler And Scope Timing

The normal `World::run_once()` path is still used by simulation and broad host tests. The Pico 2 W
firmware uses the realtime scheduler path in `World` so high-rate IMU closure is separated from
slower service work:

```text
main loop on core 0
├── realtime_scheduler_step
│   ├── ImuControl when BoardIo::imu_pending() is true
│   ├── Service when one deferred service phase is due and still early in the frame
│   └── Idle otherwise
├── run_imu_control_tick
│   ├── drain latest IMU packet only
│   ├── process IMU packet
│   ├── update IMU health/calibration state
│   └── run estimator/controller/mixer/PWM only for a new IMU timestamp
└── run_service_step_with_deferral
    ├── Input: MAVLink ingress, events, params, commands
    ├── SensorsRc: non-IMU sensors and RC/state/LED/PWM state sync
    ├── Responses: at most one queued response per service call
    ├── Telemetry: one realtime telemetry group
    ├── Flush: one budgeted serial flush step
    └── DeferredBoard: board-specific deferred actions
```

The critical split is that RC command/state work is not run inside `run_imu_control_tick`. CRSF
frames are still received on core 1 and queued, but the RC packet is drained and interpreted in the
`SensorsRc` service phase. The control pipeline reads the latest already-computed command state.
This preserves the expected ROSflight command/state behavior while keeping variable RC muxing and
arming logic out of the IMU close-loop path.

The service window is intentionally tight. A service phase can run only when:

- no IMU sample is pending,
- the service deferral deadline has elapsed,
- the last control closure completed no more than `120 us` ago, and
- this control sample has not already received a service phase.

That gives each fresh IMU sample priority and prevents a late service phase from starting just
before the next data-ready event.

### Scope Timing Pins

The `scope-timing-pins` feature drives easy-to-probe Pico 2 W pins for logic-analyzer timing.
Connect analyzer ground to Pico ground and probe GP18, GP19, and GP22 at the Pico header.

| Pico 2 W GPIO | Signal | What the pulse means |
| --- | --- | --- |
| GP18 | Realtime pass boundary | Toggles at the start of each top-level realtime scheduler pass on core 0. Edge-to-edge time is a scheduler pass, not necessarily one control closure. |
| GP19 | Control pipeline body | High only after a fresh IMU timestamp is accepted and while estimator, controller, mixer, PWM composition, and PWM write run. |
| GP22 | Selected diagnostic window | Depends on the enabled scope feature. See below. |

GP22 has several mutually-exclusive diagnostic modes:

| Feature | GP22 meaning | Use |
| --- | --- | --- |
| `scope-timing-pins` only | Service phase duration | Measures non-control service work on core 0. |
| `imu-producer-scope` | Core 1 IMU producer duration | Measures data-ready handling, SPI read, and IMU queue push. |
| `pre-control-scope` | Pre-control work inside `run_imu_control_tick` | Measures IMU drain/process/health before GP19 rises. |
| `rc-command-scope` | `run_rc_command_state_stages()` duration | Measures RC receive/interpreting, command muxing, state machine, PWM state sync, and LED update where that stage is called. |
| `control-scope-estimator` | Estimator substage | Measures one control substage on GP22. |
| `control-scope-controller` | Controller substage | Measures one control substage on GP22. |
| `control-scope-mixer` | Mixer substage | Measures one control substage on GP22. |
| `control-scope-pwm` | PWM composition/write substage | Measures one control substage on GP22. |

Use only one GP22 mode at a time. The firmware has compile-time guards for the known conflicting
scope modes.

RP2350 interrupt-executor experiments are intentionally feature-gated. Use exactly one of:
`raw-swi-smoke`, `interrupt-executor-smoke`, or `imu-producer-interrupt-executor`. The raw smoke
build proves that the selected RP2350 core 1 interrupt vector can fire by toggling GP22 from the
interrupt handler. The current experiment uses `SIO_IRQ_BELL`, because `SIO_IRQ_FIFO` is owned by
Embassy multicore and `SWI_IRQ_5` did not deliver on core 1 in bring-up captures. The executor smoke
build proves that an Embassy interrupt executor task can poll from that IRQ. The real producer build
moves only the ISM330DHCX producer task to that interrupt executor.

Flash the logic-analyzer build:

```bash
probe-rs download --chip RP235x --protocol swd \
  target/thumbv8m.main-none-eabihf/release/voloxide

probe-rs reset --chip RP235x
```

If using the current debug probe from bring-up:

```bash
probe-rs download --probe 2e8a:000c-0:E6647C7403301534 \
  --chip RP235x \
  --protocol swd \
  target/thumbv8m.main-none-eabihf/release/voloxide

probe-rs reset --probe 2e8a:000c-0:E6647C7403301534 --chip RP235x
```

### Logic-Analyzer Builds

Common full-system producer timing build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'ism330dhcx-driver ism330dhcx-3k333 scope-timing-pins imu-producer-scope imu-producer-interrupt-executor'
```

Pre-control timing build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'ism330dhcx-driver ism330dhcx-3k333 scope-timing-pins pre-control-scope imu-producer-interrupt-executor'
```

RC command/state timing build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'ism330dhcx-driver ism330dhcx-3k333 scope-timing-pins rc-command-scope imu-producer-interrupt-executor'
```

Do not enable `timing-diagnostics`, `release-loop-bench`, `release-loop-classifier`, or
`release-loop-spike-counter` when taking clean GPIO timing captures. Those features are useful for
coarse telemetry, but they add work and can obscure the exact scope-edge timing.

### Current Timing Measurements

The current validated capture set used the real ISM330DHCX at its native `3.333 kHz` ODR with full
core 1 transport enabled and loaded MAVLink telemetry over the ESP32C5 bridge. Relevant budgets:

- `300 us`: nominal 3.333 kHz IMU/control period.
- `312.5 us`: older 3.2 kHz comparison budget.
- `333.333 us`: looser 3.0 kHz comparison budget.

The measurements below are from Saleae CSV exports. Saleae channel mapping in the current exports
is Channel 0 = GP19 full control body, Channel 1 = GP18 realtime pass boundary, and Channel 3 =
GP22 selected diagnostic window.

Latest optimized build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'ism330dhcx-driver ism330dhcx-3k333 scope-timing-pins control-scope-controller imu-producer-interrupt-executor'
```

| Measurement | Samples | Mean | p99 | p99.9 | Worst | Over 300 us | Over 312.5 us | Over 333.333 us |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| GP19 full control body | `214524` | `125.119 us` | `229.092 us` | `257.906 us` | `308.382 us` | `3` | `0` | `0` |
| GP22 controller substage | n/a | `37.595 us` | `73.820 us` | n/a | `117.886 us` | `0` | `0` | `0` |

The three GP19 pulses over `300 us` were `308.382 us`, `301.340 us`, and `300.110 us`. There were
no full-control pulses over either the old `312.5 us` comparison budget or the `333.333 us` period.
The mid-capture GP19 rate measured about `3527.9 Hz`; use the pulse-width statistics for processing
headroom and producer-period captures when checking the exact IMU cadence.

Loaded telemetry during the same timing campaign:

| Stream | Expected rate | Current result |
| --- | ---: | --- |
| IMU telemetry | `400 Hz` | Hit expected rate during high-rate link validation. |
| RC raw telemetry | `100 Hz` | Hit expected rate after RC input was restored. |
| Attitude quaternion | `50 Hz` | Hit expected rate. |
| Output raw | `50 Hz` | Hit expected rate. |
| GNSS | `10 Hz` | Decoded in the current stream. |
| Heartbeat | `1 Hz` | Decoded in the current stream. |

The loaded receiver pass showed about `29.15 kB/s` over the UART/ESP-NOW path, `0` invalid CRCs,
and no estimated missing valid MAVLink sequence frames. Status telemetry reported average loop time
about `128.1 us`, p99 `224 us`, and max `241 us` after the realtime-path optimization.

Interpretation:

- The current firmware is viable at the native `3.333 kHz` IMU/control update rate under the tested
  telemetry load.
- There is useful average headroom, and the p99 path is well under one 3.333 kHz period.
- The remaining tail is still in the control body, so future estimator/controller/mixer/PWM
  optimizations are the right place to find more margin.
- Barometer and magnetometer work should stay in service-side low-rate paths and feed latest-value
  state into the estimator. They should not be polled synchronously inside `run_imu_control_tick`.

Historical note: earlier 1.666 kHz captures found that RC/state work inside the IMU tick was the
major avoidable tail. Moving RC/state into the service phase cut pre-control p99 from about
`143.5 us` to about `62.1 us` before the current 3.333 kHz validation.

### Core 1 Transport Findings

Several A/B feature gates are kept for future isolation:

- `core1-disable-heartbeat`
- `core1-disable-mavlink-tx`
- `core1-disable-mavlink-rx`
- `core1-disable-crsf`
- `core1-disable-gps`

Disabling CRSF plus MAVLink TX made the timing look close to an isolated IMU producer. CRSF alone
was the largest single contributor to the old pre-control tail because completed CRSF packets were
causing RC/state work to run inside the IMU tick. MAVLink TX also contributed by adding core 1
transport pressure and mailbox activity. The current firmware mitigates this by:

- moving RC/state out of `run_imu_control_tick`,
- bounding response work to one response per realtime service call,
- limiting service phases to the early post-control window,
- using a bounded UART TX batch path for high-rate telemetry,
- increasing CRSF UART read chunk size to `32` bytes, and
- lowering UART/PIO/DMA transport interrupt priority relative to the IMU producer interrupt.

## ESP32C5 ESP-NOW Bridge

The bridge project is:

```text
tools/espnow_uart_bridge/
```

It was tested independently as a UART-over-air link before connecting it to the RP2350 firmware.
Use the bridge README for role-specific ESP-IDF commands:

[ESP32C5 ESP-NOW UART bridge](../../tools/espnow_uart_bridge/README.md)

Operational rule from bring-up: put the XIAO boards into boot mode for flashing, then reset them
after flashing so they leave download mode and run the flashed image.

### Runtime Telemetry Test

Use the ground XIAO USB Serial/JTAG endpoint as the host serial device. The current bridge UART rate
is `2_000_000` baud.

Example with the currently tested ground XIAO:

```bash
python3 tools/mavlink_tester.py \
  --transport uart \
  --device /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_38:44:BE:A4:15:B8-if00 \
  --baud 2000000 \
  --samples 20000 \
  --duration-s 63 \
  --warmup-s 3 \
  --show 6 \
  --diagnostics
```

The `63` second duration with a `3` second warmup produces a 60 second measured window.

Historical 60 second result with RP2350 release firmware built as:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'ism330dhcx-driver ism330dhcx-1k666 release-loop-bench'
```

| Stream | Measured rate | Notes |
| --- | ---: | --- |
| IMU telemetry | `50.0 Hz` | Board timestamp interval avg `20.000 ms`, p99 `20.546 ms`. |
| RC telemetry | `50.0 Hz` | Board timestamp interval avg `20.000 ms`, p99 `28.000 ms`. |
| Barometer telemetry | `5.0 Hz` | Host interval avg `199.986 ms`. |
| Heartbeat | `1.0 Hz` | 60 frames in the 60 second window. |
| PERF statustext | `1.0 Hz` | Loop bench avg `65.2 us`, p90 max `230 us`, p99 max `460 us`, max `859 us`. |

Transport throughput in that run was about `5251 B/s`. The parser rejected `707` candidate frames by
CRC over the 60 second run. Treat that as an ESP-NOW/USB serial transport-quality issue to track
separately from RP2350 loop timing.

Older bench and classifier reports counted broad scheduler passes rather than the exact IMU
close-loop path. They were useful for finding the architecture problem, but GPIO captures are now
the authoritative timing source for the realtime loop. To separate broad scheduler passes from
passes that did not run control, use the classifier feature:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'ism330dhcx-driver ism330dhcx-1k666 release-loop-classifier'
```

Historical 60 second classifier result before the realtime RC/state split:

| Pass class | Samples | Average | p90 max | p99 max | Max | Over 600 us |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Closure/control pass | `103863` | `393.3 us` | `510 us` | `710 us` | `971 us` | `2149` |
| No-control pass | `254588` | `66.1 us` | `130 us` | `450 us` | `652 us` | `22` |
| All classifier passes | `358451` | `160.9 us` | `430 us` | `610 us` | `971 us` | `2171` |

In this report, a closure/control pass meant `World` received a new processed IMU timestamp and ran
estimator, controller, mixer, and PWM output. A no-control pass meant the scheduler still serviced
communication, sensors, RC/state, telemetry, and board actions, but did not close the control loop.
Do not use those historical max values to evaluate the current native 3.333 kHz close-loop budget.

Current high-rate receiver validation command:

```bash
python3 tools/mavlink_tester.py \
  --transport uart \
  --device /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_38:44:BE:A4:15:B8-if00 \
  --baud 2000000 \
  --duration-s 45 \
  --warmup-s 3 \
  --show 4 \
  --diagnostics \
  --acceptance \
  --expect-imu-hz 400 \
  --expect-rc-hz 100 \
  --expect-attitude-hz 50 \
  --expect-output-raw-hz 50
```

## Sensor Bring-Up

Hardware probes live in `boards/pico2w/src/bin/`. Use them to isolate buses before debugging the
full firmware:

```bash
cargo run -p pico2w --target thumbv8m.main-none-eabihf --bin imu_spi_probe --release
cargo run -p pico2w --target thumbv8m.main-none-eabihf --bin imu_spi_bench --release
cargo run -p pico2w --target thumbv8m.main-none-eabihf --bin sensor_stack_probe --release
cargo run -p pico2w --target thumbv8m.main-none-eabihf --bin crsf_probe --release
```

The high-rate IMU path is the ISM330DHCX over SPI with a data-ready interrupt. The barometer is a
low-rate path and can be polled outside the critical control pass.

## Current Hardware Findings

- ESP32C5 bridge can pass UART data bidirectionally over ESP-NOW in isolation.
- The RP2350 firmware path is designed to keep communication work out of the measured control pass.
- The control pass is IMU-driven and validated at the native ISM330DHCX `3.333 kHz` ODR.
- Runtime telemetry and diagnostics should be tested in release mode when evaluating timing.

Use [hardware bring-up notes](../hardware-bringup-notes.md) for the concise latest runbook.
