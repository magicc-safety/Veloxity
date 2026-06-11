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

The default `pico2w` feature set is the current hardware baseline: real ISM330DHCX data-ready
input, native high-rate IMU ODR, a fixed `2 kHz` control update rate, the core 1
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
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'scope-timing-pins control-scope-controller'
```

Timing diagnostics build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'timing-diagnostics'
```

Use the logic-analyzer timing build when measuring loop timing with GPIO instead of MAVLink
statustext diagnostics. Do not enable `timing-diagnostics` for clean Saleae timing captures.

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
│   └── run estimator/controller/mixer/PWM only when the configured control rate is due
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

The IMU sample rate, control update rate, and telemetry rates are intentionally separate:

- IMU ODR is a board hardware choice in `boards/pico2w/src/bin/voloxide.rs`. The default high-rate
  ODR uses the ISM330DHCX `0x9*` ODR register settings; `imu-odr-1666hz` selects the lower `0x8*`
  settings.
- Control cadence is configured through `World::set_control_loop_rates`. The current Pico 2 W
  flight image sets `ControlLoopRates::fixed_rate_hz(2_000)`, so core 0 still ingests high-rate IMU
  samples but runs the full estimator/controller/mixer/PWM pipeline at 2 kHz. Other closure rates
  such as `1_000` or `400` Hz use the same path; no scheduler rewrite is needed.
- Telemetry cadence is configured through `World::set_telemetry_rates`. `TelemetryRates` covers
  heartbeat, status, IMU, attitude, output raw, diff pressure, baro, mag, range, battery, GNSS, and
  RC stream rates.

This split is deliberate. Sampling faster than the control loop keeps the newest IMU data fresh
without requiring the full control pipeline to meet every raw data-ready period. Because the
high-rate IMU ODR is not an integer multiple of the `2 kHz` control rate, the realtime scheduler
does not simply run control on every Nth data-ready edge. IMU data-ready events ingest and process
samples as they arrive, while an independent control deadline runs the control pipeline at the
configured cadence. All processed IMU samples accumulated since the previous control deadline are
averaged into the control sample. That software boxcar stage is the current anti-aliasing bridge
between raw IMU ODR and the lower control update rate. Lower control rates naturally average more
IMU samples per control update; higher control rates average fewer. A fixed-rate control deadline
does not rerun on stale IMU data if no new sample has arrived.

### Scope Timing Pins

The `scope-timing-pins` feature drives easy-to-probe Pico 2 W pins for logic-analyzer timing.
Connect analyzer ground to Pico ground and probe GP18, GP19, and GP22 at the Pico header.

| Pico 2 W GPIO | Signal | What the pulse means |
| --- | --- | --- |
| GP18 | New IMU available strobe | Short pulse when core 0 enters an `ImuControl` scheduler step. Use rising edges to measure available-IMU cadence. |
| GP19 | Complete fast loop | High for the full `run_imu_control_tick()` call, including IMU drain/process/health and any estimator/controller/mixer/PWM work for the accepted timestamp. |
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
  --features 'scope-timing-pins imu-producer-scope'
```

Pre-control timing build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'scope-timing-pins pre-control-scope'
```

RC command/state timing build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'scope-timing-pins rc-command-scope'
```

Do not enable `timing-diagnostics`, `release-loop-bench`, or `release-loop-classifier` when taking
clean GPIO timing captures. Those features are useful for coarse telemetry, but they add work and
can obscure the exact scope-edge timing.

### Current Timing Measurements

The current validated capture set uses the real ISM330DHCX with full core 1 transport enabled and
loaded MAVLink telemetry over the ESP32C5 bridge. Relevant budgets:

- `300 us`: nominal 3.333 kHz raw IMU period.
- `312.5 us`: older 3.2 kHz comparison budget.
- `333.333 us`: looser 3.0 kHz comparison budget.
- `500 us`: configured 2 kHz control update period.
- `600 us`: lower-rate 1.666 kHz ODR timing-margin period.

The measurements below are from Saleae CSV exports. Saleae channel mapping in the current exports
is Channel 0 = GP19 full control body, Channel 1 = GP18 IMU-control step marker, and Channel 3 =
GP22 selected diagnostic window.

Latest optimized build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'scope-timing-pins control-scope-controller'
```

| Measurement | Samples | Mean | p99 | p99.9 | Worst | Over 300 us | Over 312.5 us | Over 333.333 us |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| GP19 full control body | `214524` | `125.119 us` | `229.092 us` | `257.906 us` | `308.382 us` | `3` | `0` | `0` |
| GP22 controller substage | n/a | `37.595 us` | `73.820 us` | n/a | `117.886 us` | `0` | `0` | `0` |

The three GP19 pulses over `300 us` were `308.382 us`, `301.340 us`, and `300.110 us`. There were
no full-control pulses over either the old `312.5 us` comparison budget or the `333.333 us` period.
The mid-capture GP19 rate measured about `3527.9 Hz`; use the pulse-width statistics for processing
headroom and producer-period captures when checking the exact IMU cadence.

120-second confirmation run of the same reverted baseline:

| Measurement | Samples | Mean | p99 | p99.9 | Worst | Over 300 us | Over 312.5 us | Over 333.333 us |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| GP19 full control body | `458796` | `127.588 us` | `231.520 us` | `265.120 us` | `328.160 us` | `15` | `5` | `0` |
| GP22 controller substage | `458787` | `38.209 us` | `75.040 us` | `85.600 us` | `125.920 us` | `0` | `0` | `0` |

The 120-second run confirmed no full-control pulse exceeded `333.333 us`, the `3.0 kHz` comparison
period. It did show rare strict-`300 us` misses against the `3.333 kHz` period, with `15 / 458796`
GP19 pulses over `300 us`. The worst matched frame was `328.160 us` total, with `80.960 us` in
controller and `247.200 us` outside controller.

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
| IMU telemetry | `400 Hz` | Hit expected rate during high-rate link validation. |
| RC raw telemetry | `100 Hz` | Hit expected rate after RC input was restored. |
| Attitude quaternion | `50 Hz` | Hit expected rate. |
| Output raw | `50 Hz` | Hit expected rate. |
| GNSS | `10 Hz` | Decoded in the current stream. |
| Heartbeat | `1 Hz` | Decoded in the current stream. |

The loaded receiver pass showed about `29.15 kB/s` over the UART/ESP-NOW path, `0` invalid CRCs,
and no estimated missing valid MAVLink sequence frames. Status telemetry reported average loop time
about `128.1 us`, p99 `224 us`, and max `241 us` after the realtime-path optimization.

A later 120-second timing confirmation received about `29.42 kB/s`, with RC at about `100 Hz`,
attitude/output raw at about `50 Hz`, GNSS at `10 Hz`, status at `10 Hz`, and heartbeat at `1 Hz`.
Its host-side IMU rate print was about `401 Hz`; use board timestamp deltas rather than host
inter-arrival timestamps for precise scheduler-rate claims because one UART read can contain
multiple MAVLink frames with the same host timestamp. That pass reported `24` invalid CRC candidates
and sequence-gap accounting reported estimated missing valid MAVLink frames, so keep the earlier
clean receiver pass as the link-integrity reference and use the 120-second Saleae data as the longer
timing reference.

A short post-cleanup 24.4-second Saleae check of the simplified default feature surface produced
`84107` GP19 control pulses: mean `148.784 us`, p99 `248.960 us`, p99.9 `274.560 us`, worst
`312.800 us`, with `2` pulses over `300 us`, `1` over `312.5 us`, and `0` over `333.333 us`. This
is a smoke check of the simplified feature surface; keep the 120-second Saleae run as the stronger
timing reference. The GP22 controller substage remained bounded: mean `38.192 us`, p99
`78.240 us`, worst `107.360 us`, and `0` pulses over `300 us`.

Interpretation:

- The meaningful timing claim is GP14 IMU data-ready to GP19 control-complete, not GP19 width alone.
- The lower `1.666 kHz` ODR timing-margin configuration passed that end-to-end check with zero
  overruns against a `600 us` budget in the measured run.
- The native high-rate ODR did not pass when the control pipeline ran every IMU sample: GP14 to GP19
  complete exceeded both `300 us` and `333.333 us` budgets frequently.
- The current default architecture therefore samples the IMU at the high-rate ODR but runs the full
  control update at `2 kHz`.
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

Older bench and classifier reports counted broad scheduler passes rather than the exact IMU
close-loop path. They were useful for finding the architecture problem, but they are intentionally
not reproduced here because the GPIO captures above are now the authoritative timing source for the
realtime loop. Use Git history for those obsolete 1.666 kHz/release-loop diagnostic runs.

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
- The control pass is IMU-driven and validated at the native ISM330DHCX `3.333 kHz` ODR.
- Runtime telemetry and diagnostics should be tested in release mode when evaluating timing.

Use [hardware bring-up notes](../hardware-bringup-notes.md) for the concise latest runbook.
