# Hardware Bring-Up Runbook

This file records the current hardware state that is useful for cloning, wiring, flashing, and
repeating the latest tested Voloxide hardware workflows. Historical diagnostic branches, failed
build-directory names, and one-off experiments live in Git history rather than in current docs.

## Current Hardware

- RP2350/Pico 2 W target is flashed through the Raspberry Pi Debug Probe.
  - Debug probe USB ID used in testing: `2e8a:000c:E6647C7403301534`.
  - Prefer `/dev/serial/by-id/...` symlinks over `/dev/ttyACM*` names.
- ESP-NOW bridge uses two Seeed Studio XIAO ESP32-C5 boards.
  - Air/drone XIAO base MAC: `38:44:be:a4:06:bc`.
  - Ground/receiver XIAO base MAC: `38:44:be:a4:15:b8`.
  - Ground XIAO USB-C is the host serial endpoint.
  - Air XIAO UART connects to RP2350 UART0.

## Wiring

Air-side XIAO to RP2350/Pico 2 W:

| XIAO ESP32-C5 | RP2350/Pico 2 W |
| --- | --- |
| D6 / TX / GPIO11 | GP1 / UART0 RX |
| D7 / RX / GPIO12 | GP0 / UART0 TX |
| GND | GND |

Use `2_000_000` baud for the RP2350 telemetry UART.

For isolated ESP-NOW bridge testing, disconnect the air XIAO from RP2350 UART and jumper air XIAO
`D6 / TX / GPIO11` to air XIAO `D7 / RX / GPIO12`. Bytes written to the ground USB endpoint should
echo through:

```text
ground USB -> ESP-NOW -> air UART TX -> air UART RX -> ESP-NOW -> ground USB
```

## RP2350 Firmware

Current useful build checks:

```bash
rustup target add thumbv8m.main-none-eabihf
cargo xtask check-board pico2w
cargo xtask build-board pico2w
```

Release flashing through the debug probe:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release
probe-rs download --probe 2e8a:000c-0:E6647C7403301534 \
  --chip RP235x \
  --protocol swd \
  target/thumbv8m.main-none-eabihf/release/voloxide
```

Useful feature sets from current testing:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'ism330dhcx-driver ism330dhcx-1k666 release-loop-bench'

cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'timing-diagnostics ism330dhcx-driver ism330dhcx-1k666 release-loop-bench'
```

Current RP2350 status:

- Main firmware builds and flashes with `probe-rs download`.
- ISM330DHCX accelerometer/gyro path is validated through interrupt-driven firmware.
- IMU is configured to the natural `1666 Hz` ODR for the 1.66 kHz control-loop target.
- GPS over PIO UART and ELRS/CRSF receiver have produced parsed MAVLink through the ESP-NOW bridge.
- Barometer passthrough worked in earlier probes and is integrated through the board sensor path,
  but still deserves a fresh post-flash validation run after hardware wiring changes.

## ESP-NOW Bridge

The current bridge source lives in `tools/espnow_uart_bridge`. It is a transparent serial link:

- Ground role: USB Serial/JTAG local endpoint.
- Air role: UART1 local endpoint on XIAO `D6/TX/GPIO11` and `D7/RX/GPIO12`.
- ESP-NOW: fixed peer MACs, channel 1, unicast packets.
- Logs/console output are disabled on the clean bridge images so the byte stream stays clean.

Build clean air and ground images:

```bash
idf.py -C tools/espnow_uart_bridge \
  -B tools/espnow_uart_bridge/build-air-unicast \
  -D SDKCONFIG=build-air-unicast/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.air.defaults" \
  build

idf.py -C tools/espnow_uart_bridge \
  -B tools/espnow_uart_bridge/build-ground-unicast \
  -D SDKCONFIG=build-ground-unicast/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.ground.defaults" \
  build
```

Reliable XIAO ESP32-C5 flash sequence:

1. Hold `BOOT`.
2. Tap `RESET`.
3. Release `BOOT`.
4. Flash with esptool using `--before no-reset --after hard-reset`.
5. If the board remains in `waiting for download`, tap `RESET` once with `BOOT` released.

The latest clean isolated ESP-NOW UART bridge test passed bidirectionally for 120 seconds:

- Frames: `5826/5826` exact echoes.
- Timeouts: `0`.
- Mismatches: `0`.
- Verified goodput: about `8.7 kB/s` / `69.9 kbps`.
- RTT median: about `20 ms`.

## Current End-To-End Findings

- 2026-06-09 Saleae logic-analyzer capture in progress on RP2350/Pico 2 W loop timing.
  - Flashed release firmware with
    `ism330dhcx-driver ism330dhcx-1k666 scope-timing-pins`.
  - Deliberately did not enable `timing-diagnostics`, `release-loop-bench`,
    `release-loop-classifier`, or `release-loop-spike-counter`; the firmware should be running the
    normal release loop with telemetry plus GPIO strobes only.
  - `GP18` is the loop-boundary toggle: it changes state at the start of each top-level
    `World::run_once()` pass on core 0, so edge-to-edge time is one full pass.
  - `GP19` is the control-closure strobe: high only during the fresh-IMU estimator, controller,
    mixer, and PWM composition/write path.
  - `GP22` is the non-control-work strobe: high during non-control work, high for an entire
    no-fresh-IMU pass, and low during the `GP19` control closure on control passes.
  - Flash command used the Raspberry Pi Debug Probe
    `2e8a:000c-0:E6647C7403301534`; `probe-rs reset` returned success with warnings that the core
    was already running and breakpoint cleanup timed out.
- RP2350 telemetry over ESP-NOW has carried MAVLink heartbeat, RC, TIMESYNC, STATUSTEXT, PERF, IMU,
  and barometer traffic in current branch testing.
- Latest real-IMU release-loop run through the ESP32C5 bridge used a 60 second measured window after
  a 3 second warmup. The firmware was built with
  `ism330dhcx-driver ism330dhcx-1k666 release-loop-bench`.
  - IMU telemetry: `50.0 Hz`, board timestamp p99 `20.546 ms`.
  - RC telemetry: `50.0 Hz`, board timestamp p99 `28.000 ms`.
  - Barometer telemetry: `5.0 Hz`.
  - Heartbeat and PERF statustext: `1.0 Hz`.
  - Loop bench: `884263` loop samples, average `65.2 us`, p90 max `230 us`, p99 max `460 us`, max
    `859 us`, `284` samples over the `600 us` budget.
  - Transport throughput: about `5251 B/s`.
  - MAVLink parser rejected `707` candidate frames by CRC; track this as ESP-NOW/USB serial
    transport quality, not as a flight-loop timing failure.
- Latest low-overhead classifier run split scheduler passes into closure/control and no-control
  timing classes. The firmware was built with
  `ism330dhcx-driver ism330dhcx-1k666 release-loop-classifier`.
  - Closure/control pass: `103863` samples, average `393.3 us`, p90 max `510 us`, p99 max `710 us`,
    max `971 us`, `2149` samples over `600 us`.
  - No-control pass: `254588` samples, average `66.1 us`, p90 max `130 us`, p99 max `450 us`, max
    `652 us`, `22` samples over `600 us`.
  - All classifier passes: `358451` samples, average `160.9 us`, p90 max `430 us`, p99 max
    `610 us`, max `971 us`, `2171` samples over `600 us`.
- TIMESYNC request/response rate through the ESP-NOW bridge is not yet full parity with a wired
  link, but bidirectional MAVLink command/response routing is proven.

## Generated Artifacts

These paths are generated locally and can be removed when cleaning a clone:

```bash
cargo xtask clean-generated
```

That removes Rust build output, the generated ROS 2 shim workspace, runtime parameter memory,
Python caches, and ESP-IDF bridge build directories. Source files and default configs remain.
