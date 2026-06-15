# RP2350 / Pico 2 W

The Pico 2 W is the active hardware bring-up board for this branch. The firmware crate is
`boards/pico2w`. The `platforms/rp2350` crate is intentionally thin: Pico code imports the Embassy
RP HAL through `rp2350_platform::hal`, and the crate also holds early shared metadata for core roles
and PIO allocation.

## Source Layout

| Path | Purpose |
| --- | --- |
| `boards/pico2w/src/bin/veloxity.rs` | Main firmware entry point, Embassy tasks, core split, and `World` construction. |
| `boards/pico2w/src/board.rs` | Pico implementation of the `BoardIo` contract. |
| `boards/pico2w/src/comms_core.rs` | Shared MAVLink mailbox between transport tasks and the flight core. |
| `boards/pico2w/src/pwm.rs` | PIO PWM/DShot-facing driver implementation. |
| `boards/pico2w/src/ism330dhcx.rs` | ISM330DHCX IMU packet path. |
| `boards/pico2w/src/barometer.rs` | Barometer packet path. |
| `boards/pico2w/src/gy91.rs` | Legacy GY-91/BMP280 support used as low-rate pressure path. |
| `boards/pico2w/src/gps.rs` | GPS and magnetometer path. |
| `boards/pico2w/src/rc_receiver.rs` | CRSF RC receiver path. |
| `boards/pico2w/src/bin/*probe.rs` | Hardware probes for individual buses and sensors. |
| `platforms/rp2350/src/lib.rs` | Re-exports Embassy RP as `rp2350_platform::hal`. |
| `platforms/rp2350/src/multicore.rs` | Shared RP2350 core-role metadata. |
| `platforms/rp2350/src/pio.rs` | Shared RP2350 PIO allocation metadata. |

## Firmware Model

The intended runtime split is:

- core 0 runs the Veloxity flight-control `World`
- core 1 owns communication services that can jitter without blocking the flight loop
- PIO owns timing-sensitive output/input work
- `BoardIo::update_sensor_bus()` drains the newest board-local sensor packets into core resources

The IMU intake path is data-ready driven, while the full control pipeline runs from an independent
fixed-rate control deadline. The current stable baseline samples the ISM330DHCX at the high-rate
output data rate (ODR) and runs estimator/controller/mixer/PWM at 1.5 kHz using the accumulated IMU
samples since the previous control update.

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
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin veloxity --release
```

The default `pico2w` feature set is the current hardware baseline: real ISM330DHCX data-ready
input, native high-rate IMU ODR, a fixed `1.5 kHz` control update rate, the core 1
interrupt-executor IMU producer, bounded high-rate telemetry, CRSF RC input, GPS PIO service, and
the UART MAVLink bridge.

The default feature set is `ism330dhcx-driver` plus `imu-producer-interrupt-executor`. Remaining
opt-in features should be treated as measurement, fallback, or bring-up tools:

| Feature | Purpose | Normal flight build? |
| --- | --- | --- |
| `ism330dhcx-driver` | Enables the real ISM330DHCX hardware IMU driver. This is part of the default baseline. | Yes; default. |
| `imu-producer-interrupt-executor` | Runs the IMU producer on the core 1 Embassy interrupt executor driven by `SIO_IRQ_BELL`. This is part of the default baseline. | Yes; default. |
| `imu-odr-1666hz` | Runs the ISM330DHCX at the lower `1.666 kHz` ODR for timing-margin comparisons. | No; default is the high-rate ODR. |
| `ism330dhcx-1k666` | Compatibility alias for `imu-odr-1666hz`. | No; prefer the clearer `imu-odr-1666hz` name. |
| `imu-400hz` | Legacy GY-91 MPU sample throttle for old probe paths. It does not change the current ISM330DHCX flight IMU or 400 Hz MAVLink IMU telemetry. | No. |
| `scope-timing-pins` | Enables GP18/GP19/GP22 Saleae timing outputs. | No; use only while measuring. |
| `control-scope-estimator`, `control-scope-controller`, `control-scope-mixer`, `control-scope-pwm` | Selects which control substage GP22 marks. | No; combine one with `scope-timing-pins` during timing captures. |
| `imu-producer-scope`, `pre-control-scope`, `rc-command-scope` | Uses GP22 for producer, pre-control, or RC service timing. | No; targeted timing captures only. |
| `timing-diagnostics` | Emits coarse MAVLink STATUSTEXT timing diagnostics from measured world paths. | No; useful when a logic analyzer is unavailable. |
| `release-loop-bench`, `release-loop-classifier` | Legacy onboard release-mode loop timing summaries. | No; prefer Saleae captures for final timing claims. |
| `core1-disable-heartbeat`, `core1-disable-mavlink-tx`, `core1-disable-mavlink-rx`, `core1-disable-crsf`, `core1-disable-gps` | Core 1 transport isolation gates used to identify interference from individual producer/transport tasks. | No; diagnostic-only. |

Logic-analyzer timing build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin veloxity --release \
  --features 'scope-timing-pins control-scope-controller'
```

Timing diagnostics build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin veloxity --release \
  --features 'timing-diagnostics'
```

Use the logic-analyzer timing build when measuring loop timing with GPIO instead of MAVLink
statustext diagnostics. Do not enable `timing-diagnostics` for clean Saleae timing captures.

## Flash

With a debug probe attached:

```bash
probe-rs download --chip RP235x --protocol swd \
  target/thumbv8m.main-none-eabihf/release/veloxity

probe-rs reset --chip RP235x
```

If multiple probes are visible, add `--probe <VID:PID:SERIAL>`.

Example from current bring-up notes:

```bash
probe-rs download --probe 2e8a:000c-0:E6647C7403301534 \
  --chip RP235x \
  --protocol swd \
  target/thumbv8m.main-none-eabihf/release/veloxity

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
│   ├── ControlUpdate when the fixed control deadline is due and accumulated IMU exists
│   ├── Service when one deferred service phase is due and still early in the frame
│   └── Idle otherwise
├── run_imu_control_tick
│   ├── drain latest IMU packet only
│   ├── process IMU packet
│   ├── update IMU health/calibration state
│   └── add the processed IMU sample to the control accumulator
├── run_control_update_tick
│   ├── consume all accumulated IMU samples as one averaged control sample
│   ├── skip missed fixed-rate intervals without burst catch-up
│   └── run estimator/controller/mixer/PWM once
└── run_service_step_with_deferral
    ├── Input: MAVLink ingress, events, params, commands
    ├── Sensors: non-IMU sensors, including latest queued RC packet drain
    ├── RcCommand: RC interpretation, state/LED/PWM state sync
    ├── Responses: at most one queued response per service call
    ├── Telemetry0/1/2: bounded deadline-ordered realtime telemetry
    ├── Flush: one budgeted serial flush step
    └── DeferredBoard: board-specific deferred actions
```

The fixed-rate catch-up policy is explicit: if the scheduler wakes late, it advances past missed
control intervals and runs at most one control update. It does not burst multiple control updates
back-to-back, and it does not consume a control deadline when no fresh accumulated IMU sample
exists.

The critical split is that RC command/state work is not run inside `run_imu_control_tick` or
`run_control_update_tick`. CRSF frames are still received on core 1 and queued with latest-sample
replacement. Core 0 drains the newest RC packet in the `Sensors` service phase and interprets it in
the `RcCommand` service phase. The control pipeline reads the latest already-computed command state.
This preserves the expected ROSflight command/state behavior while keeping variable RC muxing and
arming logic out of the IMU close-loop path.

The service window is intentionally tight. A service phase can run only when:

- no IMU sample is pending,
- the service deferral deadline has elapsed,
- the last control closure completed no more than `120 us` ago, and
- this control sample has not already received a service phase.

That gives each fresh IMU sample priority and prevents a late service phase from starting just
before the next data-ready event.

The IMU sample rate, control update rate, and telemetry rates are intentionally separate:

- IMU ODR is a board hardware choice in `boards/pico2w/src/bin/veloxity.rs`. The default high-rate
  ODR uses the ISM330DHCX `0x9*` ODR register settings; `imu-odr-1666hz` selects the lower `0x8*`
  settings.
- Control cadence is configured through `World::set_control_loop_rates`. The current Pico 2 W
  flight image sets `ControlLoopRates::fixed_rate_hz(1_500)`, so core 0 still ingests high-rate IMU
  samples but runs the full estimator/controller/mixer/PWM pipeline at 1.5 kHz. Other closure rates
  such as `1_000` or `400` Hz use the same path; no scheduler rewrite is needed.
- Telemetry cadence is configured through `World::set_telemetry_rates`. `TelemetryRates` covers
  heartbeat, status, IMU, attitude, output raw, diff pressure, baro, mag, range, battery, GNSS, and
  RC stream rates.

This split is deliberate. Sampling faster than the control loop keeps the newest IMU data fresh
without requiring the full control pipeline to meet every raw data-ready period. Because the
high-rate IMU ODR is not an integer multiple of the `1.5 kHz` control rate, the realtime scheduler
does not simply run control on every Nth data-ready edge. IMU data-ready events ingest and process
samples as they arrive, while an independent control deadline runs the control pipeline at the
configured cadence. All processed IMU samples accumulated since the previous control deadline are
averaged into the control sample. That software boxcar stage is the current anti-aliasing bridge
between raw IMU ODR and the lower control update rate. Lower control rates naturally average more
IMU samples per control update; higher control rates average fewer. A fixed-rate control deadline
does not rerun on stale IMU data if no new sample has arrived.

### Scope Timing Pins

The `scope-timing-pins` feature drives easy-to-probe Pico 2 W pins for logic-analyzer timing.
Connect analyzer ground to Pico ground. The current full timing capture uses GP19, GP14, GP18, and
GP22.

| Pico 2 W GPIO | Signal | What the pulse means |
| --- | --- | --- |
| GP14 | Raw IMU data-ready | ISM330DHCX data-ready signal. Use rising edges to measure raw IMU ODR. |
| GP18 | Scheduled control deadline marker | Short pulse when core 0 consumes a fixed-rate control deadline. Use rising edges to measure scheduler cadence. |
| GP19 | Control pipeline execution | High while the full estimator/controller/mixer/PWM pipeline runs. Use pulse width for control execution time. |
| GP22 | Selected diagnostic window | Depends on the enabled scope feature. See below. |

GP22 has several mutually-exclusive diagnostic modes:

| Feature | GP22 meaning | Use |
| --- | --- | --- |
| `scope-timing-pins` only | Service phase duration | Measures non-control service work on core 0. |
| `imu-producer-scope` | Core 1 IMU producer duration | Measures data-ready handling, SPI read, and IMU queue push. |
| `pre-control-scope` | Pre-control work inside `run_imu_control_tick` | Measures IMU drain/process/health on GP22. |
| `rc-command-scope` | `run_rc_command_state_stages()` duration | Measures RC receive/interpreting, command muxing, state machine, PWM state sync, and LED update where that stage is called. |
| `control-scope-estimator` | Estimator substage | Measures one control substage on GP22. |
| `control-scope-controller` | Controller substage | Measures one control substage on GP22. |
| `control-scope-mixer` | Mixer substage | Measures one control substage on GP22. |
| `control-scope-pwm` | PWM composition/write substage | Measures one control substage on GP22. |

Use only one GP22 mode at a time. The firmware has compile-time guards for the known conflicting
scope modes.

The RP2350 interrupt-executor IMU producer is now the default firmware path. Core 1 still owns UART
MAVLink transport, CRSF receive, GPS PIO service, and the ISM330DHCX producer, but the IMU producer
itself runs on an Embassy `InterruptExecutor` driven by `SIO_IRQ_BELL`. That avoids waiting for the
normal core 1 executor poll cadence before servicing a data-ready edge. `SIO_IRQ_FIFO` remains owned
by Embassy multicore, and `SWI_IRQ_5` did not deliver reliably on core 1 during bring-up, so
`SIO_IRQ_BELL` is the selected interrupt vector. The old raw interrupt and synthetic-IMU bring-up
feature surface has been removed from the documented baseline because the real interrupt-executor
producer has been validated on hardware.

Flash the logic-analyzer build:

```bash
probe-rs download --chip RP235x --protocol swd \
  target/thumbv8m.main-none-eabihf/release/veloxity

probe-rs reset --chip RP235x
```

If using the current debug probe from bring-up:

```bash
probe-rs download --probe 2e8a:000c-0:E6647C7403301534 \
  --chip RP235x \
  --protocol swd \
  target/thumbv8m.main-none-eabihf/release/veloxity

probe-rs reset --probe 2e8a:000c-0:E6647C7403301534 --chip RP235x
```

### Logic-Analyzer Builds

Common full-system producer timing build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin veloxity --release \
  --features 'scope-timing-pins imu-producer-scope'
```

Pre-control timing build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin veloxity --release \
  --features 'scope-timing-pins pre-control-scope'
```

RC command/state timing build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin veloxity --release \
  --features 'scope-timing-pins rc-command-scope'
```

Do not enable `timing-diagnostics`, `release-loop-bench`, or `release-loop-classifier` when taking
clean GPIO timing captures. Those features are useful for coarse telemetry, but they add work and
can obscure the exact scope-edge timing.

### Current Timing Measurements

The current validated capture set uses the real ISM330DHCX with full core 1 transport enabled and
loaded MAVLink telemetry over the ESP32C5 bridge. The stable default baseline is:

- raw IMU data-ready: about `3.55 kHz` (`281.67 us` period)
- fixed control update: `1.5 kHz` (`666.67 us` budget)
- IMU telemetry: `400 Hz`
- RC telemetry/input: `100 Hz`

Latest 120-second loaded Saleae capture:

| Measurement | Samples | Mean | p95 | p99 | Worst |
| --- | ---: | ---: | ---: | ---: | ---: |
| Raw IMU data-ready interval | `439380` | `281.67 us` | `281.67 us` | `281.67 us` | `281.68 us` |
| Scheduled control deadline interval | `185823` | `666.00 us` | `676.68 us` | `689.91 us` | `903.69 us` |
| Actual control update start interval | `185823` | `666.00 us` | `693.11 us` | `710.72 us` | `909.39 us` |
| Control pipeline execution time | `185824` | `186.20 us` | `252.56 us` | `279.23 us` | `367.09 us` |
| Control deadline to pipeline start | `185824` | `29.07 us` | `45.82 us` | `59.37 us` | `106.92 us` |
| Control deadline to pipeline complete | `185824` | `215.28 us` | `285.04 us` | `324.03 us` | `411.52 us` |
| Service-slice execution time | `185729` | `102.17 us` | `207.72 us` | `258.44 us` | `493.83 us` |

At 1.5 kHz, the exact control budget is about `666.67 us`. The worst measured
control-deadline-to-pipeline-complete latency was `411.52 us`, leaving about `255 us` of margin in
this run. The actual control-start cadence still has occasional long periods followed by shorter
catch-up periods, but the catch-up policy runs at most one control update and skips missed logical
intervals rather than bursting back-to-back control outputs.

Configured bounded high-rate telemetry profile:

| Stream | Configured rate |
| --- | ---: |
| IMU telemetry | `400 Hz` |
| RC raw telemetry | `100 Hz` |
| Attitude quaternion | `50 Hz` |
| Output raw | `50 Hz` |
| Differential pressure | `50 Hz` |
| Range | `50 Hz` |
| Barometer | `25 Hz` |
| Magnetometer | `25 Hz` |
| Battery | `25 Hz` |
| GNSS | `10 Hz` |
| Status | `10 Hz` |
| Heartbeat | `1 Hz` |

Loaded telemetry validated during the same timing campaign:

| Stream | Acceptance expectation | Current result |
| --- | ---: | --- |
| IMU telemetry | `400 Hz` | `400.1 Hz` host, `400.0 Hz` board timestamp rate. |
| RC raw telemetry | `100 Hz` | `100.0 Hz` host, `100.0 Hz` board timestamp rate. |
| Attitude quaternion | `50 Hz` | `50.0 Hz`. |
| Output raw | `50 Hz` | `50.0 Hz`. |
| GNSS | `10 Hz` | `10.0 Hz`. |
| Status | `10 Hz` | `10.0 Hz`. |
| Heartbeat | `1 Hz` | `1.0 Hz`. |

The 120-second loaded receiver pass moved about `29.35 kB/s` over the UART/ESP-NOW path, had
`0` invalid CRC candidates, and had no valid MAVLink sequence gaps, reordering, or duplicates.
Status telemetry reported firmware loop time average `190.9 us`, p99 `280 us`, and max `328 us`.

RC command freshness is bounded primarily by the external RC packet rate. CRSF packets are queued as
latest-value packets on the board side; core 0 drains the newest packet in the service sensor phase
and applies it in the RC command service phase. With a 100 Hz RC source, new stick data arrives
about every `10 ms` and the service scheduler adds a few control periods of phase latency. The
1.5 kHz control loop therefore runs many control updates per RC frame using the latest applied
command, which is the expected behavior for normal RC stick flying.

Interpretation:

- The meaningful fixed-rate control claim is scheduled control deadline to control-complete, not raw
  IMU data-ready to control-complete. The raw IMU ODR is intentionally faster than the control loop.
- The native high-rate ODR did not pass when the control pipeline ran every IMU sample, and 2 kHz
  worked with much tighter margin.
- The current default architecture therefore samples the IMU at the high-rate ODR but runs the full
  control update at `1.5 kHz`.
- Barometer and magnetometer work should stay in service-side low-rate paths and feed latest-value
  state into the estimator. They should not be polled synchronously inside `run_imu_control_tick`.

Historical note: earlier 1.666 kHz captures found that RC/state work inside the IMU tick was the
major avoidable tail. Moving RC/state into the service phase cut pre-control p99 from about
`143.5 us` to about `62.1 us` before the fixed-rate control baseline was selected.

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

Older bench and classifier reports counted broad scheduler passes rather than the exact IMU
close-loop path. They were useful for finding the architecture problem, but they are intentionally
not reproduced here because the GPIO captures above are now the authoritative timing source for the
realtime loop. Use Git history for those obsolete 1.666 kHz/release-loop diagnostic runs.

Current loaded receiver validation command:

```bash
python3 tools/mavlink_tester.py \
  --transport uart \
  --device /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_38:44:BE:A4:15:B8-if00 \
  --baud 2000000 \
  --duration-s 120 \
  --warmup-s 3 \
  --show 4 \
  --diagnostics \
  --bidirectional \
  --timesync-probe \
  --timesync-period-s 0.5 \
  --expect-imu-hz 400 \
  --expect-rc-hz 100 \
  --expect-attitude-hz 50 \
  --expect-output-raw-hz 50
```

## Sensor Bring-Up

Hardware probes live in `boards/pico2w/src/bin/`. Use them to isolate buses before debugging the
full firmware:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin imu_spi_probe --release
probe-rs download --chip RP235x --protocol swd target/thumbv8m.main-none-eabihf/release/imu_spi_probe
probe-rs reset --chip RP235x

cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin imu_spi_bench --release
probe-rs download --chip RP235x --protocol swd target/thumbv8m.main-none-eabihf/release/imu_spi_bench
probe-rs reset --chip RP235x

cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin sensor_stack_probe --release
probe-rs download --chip RP235x --protocol swd target/thumbv8m.main-none-eabihf/release/sensor_stack_probe
probe-rs reset --chip RP235x

cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin crsf_probe --release
probe-rs download --chip RP235x --protocol swd target/thumbv8m.main-none-eabihf/release/crsf_probe
probe-rs reset --chip RP235x
```

The high-rate IMU path is the ISM330DHCX over SPI with a data-ready interrupt. The barometer is a
low-rate path and can be polled outside the critical control pass.

## Current Hardware Findings

- ESP32C5 bridge can pass UART data bidirectionally over ESP-NOW in isolation.
- The RP2350 firmware path is designed to keep communication work out of the measured control pass.
- The default firmware samples the ISM330DHCX at high-rate ODR and runs the full control pipeline at
  a fixed 1.5 kHz.
- The latest 120-second loaded timing run kept every measured control-deadline-to-complete latency
  inside the 1.5 kHz budget while maintaining expected MAVLink telemetry rates.
- Runtime telemetry and diagnostics should be tested in release mode when evaluating timing.

Use [hardware bring-up notes](../hardware-bringup-notes.md) for the concise latest runbook.
