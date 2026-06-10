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
  --features 'ism330dhcx-driver ism330dhcx-1k666 release-loop-bench'
```

Timing diagnostics build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'timing-diagnostics ism330dhcx-driver ism330dhcx-1k666 release-loop-bench'
```

Logic-analyzer timing build:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'ism330dhcx-driver ism330dhcx-1k666 scope-timing-pins'
```

Use this build when measuring loop timing with GPIO instead of MAVLink statustext diagnostics. Do
not enable `timing-diagnostics`, `release-loop-bench`, `release-loop-classifier`, or
`release-loop-spike-counter` for the cleanest timing measurement.

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

## Scope Timing Pins

The `scope-timing-pins` feature drives easy-to-probe Pico 2 W pins for logic-analyzer timing:

| Pico 2 W GPIO | Signal | What the pulse means |
| --- | --- | --- |
| GP18 | Loop pass boundary | Toggles at the start of each top-level `World::run_once()` call on core 0; edge-to-edge time is one full pass. |
| GP19 | Control closure | High only after a fresh IMU timestamp is accepted and while estimator, controller, mixer, and PWM composition/write run. |
| GP22 | Non-control work | High during non-control work. On no-fresh-IMU passes this stays high for the full GP18 pass; on control passes it drops low while GP19 is high. |

Connect the analyzer ground to Pico ground. Probe GP18, GP19, and GP22 at the Pico header. GP19
should pulse at the IMU-driven control rate; GP18 will pulse more often because the firmware also
services communication, sensors, RC/state, telemetry, and board actions between control closures.
Use GP18 edge-to-edge intervals that overlap GP19 as full control-pass timing, GP18 intervals that
do not overlap GP19 as full no-control-pass timing, and GP19 pulse width as the inner control body
timing.

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

Latest 60 second result with RP2350 release firmware built as:

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
separately from RP2350 loop timing; the firmware loop timing stayed comfortably below the 600 us
1.66 kHz budget except for `284` reported over-budget samples out of `884263` loop samples.

To separate control-loop closure passes from passes that did not run control, use the classifier
feature:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'ism330dhcx-driver ism330dhcx-1k666 release-loop-classifier'
```

Latest 60 second classifier result:

| Pass class | Samples | Average | p90 max | p99 max | Max | Over 600 us |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Closure/control pass | `103863` | `393.3 us` | `510 us` | `710 us` | `971 us` | `2149` |
| No-control pass | `254588` | `66.1 us` | `130 us` | `450 us` | `652 us` | `22` |
| All classifier passes | `358451` | `160.9 us` | `430 us` | `610 us` | `971 us` | `2171` |

In this report, a closure/control pass means `World` received a new processed IMU timestamp and ran
estimator, controller, mixer, and PWM output. A no-control pass means the scheduler still serviced
communication, sensors, RC/state, telemetry, and board actions, but did not close the control loop.

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
- The control pass is IMU-driven and intended to run at the closest natural ISM330DHCX rate to
  1.66 kHz.
- Runtime telemetry and diagnostics should be tested in release mode when evaluating timing.

Use [hardware bring-up notes](../hardware-bringup-notes.md) for the concise latest runbook.
