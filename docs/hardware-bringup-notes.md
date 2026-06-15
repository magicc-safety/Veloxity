# Hardware Bring-Up Runbook

This file records the current hardware state that is useful for cloning, wiring, flashing, and
repeating the latest tested Veloxity hardware workflows. Historical diagnostic branches, failed
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
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin veloxity --release
probe-rs download --probe 2e8a:000c-0:E6647C7403301534 \
  --chip RP235x \
  --protocol swd \
  target/thumbv8m.main-none-eabihf/release/veloxity
```

Useful feature sets from current testing:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin veloxity --release

cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin veloxity --release \
  --features 'scope-timing-pins control-scope-controller'
```

Current RP2350 status:

- Main firmware builds and flashes with `probe-rs download`.
- ISM330DHCX accelerometer/gyro path is validated through interrupt-driven firmware.
- IMU is configured at the high-rate output data rate (ODR) and feeds a fixed 1.5 kHz control loop.
- GPS over PIO UART and ELRS/CRSF receiver have produced parsed MAVLink through the ESP-NOW bridge.
- Barometer passthrough worked in earlier probes and is integrated through the board sensor path.
  The current production hardware still needs a fresh validation run after the dedicated barometer
  wiring is installed.

## ESP-NOW Bridge

The current bridge source lives in `tools/espnow_uart_bridge`. It is MAVLink-frame-aware rather than
a transparent byte pipe: local serial input is scanned for complete MAVLink v1 frames, and ESP-NOW
packets are packed without splitting a MAVLink frame. Radio loss should therefore appear as whole
MAVLink frame gaps, not partial-frame CRC corruption.

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

- 2026-06-12 Saleae logic-analyzer and MAVLink captures validated the current RP2350/Pico 2 W
  realtime loop at a fixed 1.5 kHz control update rate. GPIO timing captures are the authoritative
  loop-timing source; MAVLink status loop-time is useful but broader and coarser.
- Current realtime firmware shape:
  - core 1 owns ISM330DHCX data-ready handling, SPI read, IMU queue push, UART0 MAVLink transport,
    UART1 CRSF receive, and GPS PIO service;
  - core 0 owns `World`, the realtime scheduler, service phases, and the control pipeline;
  - high-rate IMU samples are accumulated and averaged for the fixed-rate control deadline;
  - catch-up skips missed logical control intervals and never bursts back-to-back control updates;
  - RC packet drain and RC command/state work run in service phases, not in the control update;
  - service work can run only in the early post-control window.
- Current timing-pin meanings:
  - `GP14` is the raw ISM330DHCX data-ready signal;
  - `GP18` emits a short pulse when core 0 consumes a scheduled control deadline;
  - `GP19` is high while the control pipeline executes;
  - `GP22` is selected by the scope feature: service, IMU producer, pre-control, RC command/state,
    or one control substage.
- The latest 120-second loaded Saleae capture at 1.5 kHz produced:
  - raw IMU data-ready interval: mean `281.67 us`, p99 `281.67 us`, worst `281.68 us`;
  - scheduled control deadline interval: mean `666.00 us`, p99 `689.91 us`, worst `903.69 us`;
  - actual control update start interval: mean `666.00 us`, p99 `710.72 us`, worst `909.39 us`;
  - control pipeline execution time: mean `186.20 us`, p99 `279.23 us`, worst `367.09 us`;
  - control deadline to pipeline complete: mean `215.28 us`, p99 `324.03 us`, worst `411.52 us`;
  - service-slice execution time: mean `102.17 us`, p99 `258.44 us`, worst `493.83 us`.
- At 1.5 kHz, the control budget is about `666.67 us`. The worst measured
  control-deadline-to-pipeline-complete latency left about `255 us` margin in the latest 120-second
  run.
- Current configured bounded telemetry profile:
  - IMU telemetry: `400 Hz`;
  - RC telemetry: `100 Hz`;
  - attitude quaternion, output raw, differential pressure, and range: `50 Hz`;
  - barometer, magnetometer, and battery: `25 Hz`;
  - GNSS: `10 Hz`;
  - status: `10 Hz`;
  - heartbeat: `1 Hz`.
- Current loaded telemetry validation over the ESP32C5 link checked the streams present in the
  current hardware setup:
  - IMU telemetry: `400.1 Hz` host, `400.0 Hz` board timestamp rate;
  - RC telemetry: `100.0 Hz` host, `100.0 Hz` board timestamp rate;
  - attitude and output raw: `50.0 Hz`;
  - GNSS and status: `10.0 Hz`;
  - heartbeat: `1.0 Hz`;
  - receiver throughput: about `29.35 kB/s`;
  - invalid CRC candidates: `0`;
  - valid MAVLink sequence gaps, reordering, and duplicates: `0`.
- Earlier measurement series remain useful historically: the IMU producer cadence was clean, CRSF
  and MAVLink TX pressure explained much of the old long tail, and moving RC/state work out of the
  IMU tick removed a major avoidable source of control jitter.
- RP2350 telemetry over ESP-NOW has carried MAVLink heartbeat, RC, TIMESYNC, STATUSTEXT, PERF, IMU,
  attitude, output raw, GNSS, and pressure traffic in current branch testing. The current board has
  no flight barometer installed for production use; pressure telemetry was from the earlier
  GY-91/BMP280 path.
- TIMESYNC request/response rate through the ESP-NOW bridge is not yet full parity with a wired
  link, but bidirectional MAVLink command/response routing is proven.
- See [RP2350 / Pico 2 W](boards/rp2350-pico2w.md) for the detailed scheduler, scope feature, and
  timing table.

## Generated Artifacts

These paths are generated locally and can be removed when cleaning a clone:

```bash
cargo xtask clean-generated
```

That removes Rust build output, the generated ROS 2 shim workspace, runtime parameter memory,
Python caches, and ESP-IDF bridge build directories. Source files and default configs remain.
