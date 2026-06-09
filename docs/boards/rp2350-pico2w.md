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
