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
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release

cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'scope-timing-pins control-scope-controller'
```

Current RP2350 status:

- Main firmware builds and flashes with `probe-rs download`.
- ISM330DHCX accelerometer/gyro path is validated through interrupt-driven firmware.
- IMU is configured to the native `3.333 kHz` ODR for the current high-rate control-loop target.
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

- 2026-06-10 Saleae logic-analyzer captures validated the current RP2350/Pico 2 W realtime loop.
  GPIO timing captures are now the authoritative loop-timing source; older MAVLink PERF and
  classifier runs were useful for finding the issue but included broader scheduler work.
- Current realtime firmware shape:
  - core 1 owns ISM330DHCX data-ready handling, SPI read, IMU queue push, UART0 MAVLink transport,
    UART1 CRSF receive, and GPS PIO service;
  - core 0 owns `World`, the realtime scheduler, service phases, and the control pipeline;
  - RC/state work is deferred to the `SensorsRc` service phase and is no longer run inside
    `run_imu_control_tick`;
  - one queued response is sent per realtime service response phase;
  - service work can run only in the early post-control window.
- Current timing-pin meanings:
  - `GP18` toggles at each core 0 realtime scheduler pass boundary.
  - `GP19` is high during the accepted-new-IMU estimator/controller/mixer/PWM body.
  - `GP22` is selected by the scope feature: service, IMU producer, pre-control, RC command/state,
    or one control substage.
- Current validated 3.333 kHz close-loop timing with loaded MAVLink telemetry:
  - GP19 full control body: `214524` samples, mean `125.119 us`, p99 `229.092 us`, p99.9
    `257.906 us`, worst `308.382 us`;
  - GP19 full control body over `300 us`: `3` pulses; over `312.5 us`: `0`; over `333.333 us`:
    `0`;
  - GP22 controller substage: mean `37.595 us`, p99 `73.820 us`, worst `117.886 us`;
  - the three GP19 pulses over `300 us` were `308.382 us`, `301.340 us`, and `300.110 us`.
- A longer 120-second confirmation run of the same reverted baseline with controller scope and
  loaded telemetry produced `458796` GP19 control pulses: mean `127.588 us`, p99 `231.520 us`,
  p99.9 `265.120 us`, worst `328.160 us`. It had `15` pulses over `300 us`, `5` over
  `312.5 us`, and `0` over `333.333 us`. GP22 controller remained bounded: mean `38.209 us`,
  p99 `75.040 us`, p99.9 `85.600 us`, worst `125.920 us`, and `0` pulses over `300 us`.
- A post-cleanup 24.4-second Saleae smoke capture after making the RP2350 baseline the default
  produced `84107` GP19 control pulses: mean `148.784 us`, p99 `248.960 us`, p99.9 `274.560 us`,
  worst `312.800 us`, `2` over `300 us`, `1` over `312.5 us`, and `0` over `333.333 us`. GP22
  controller remained below `300 us`, with p99 `78.240 us` and worst `107.360 us`.
- Current loaded telemetry validation over the ESP32C5 link:
  - IMU telemetry: `400 Hz`;
  - RC telemetry: `100 Hz`;
  - attitude quaternion: `50 Hz`;
  - output raw: `50 Hz`;
  - GNSS: `10 Hz`;
  - heartbeat: `1 Hz`;
  - receiver throughput: about `29.15 kB/s`;
  - invalid CRCs: `0`;
  - estimated missing valid MAVLink sequence frames: `0`.
- The 120-second timing confirmation received IMU telemetry at about `401 Hz`, RC telemetry at
  about `100 Hz`, attitude and output raw at about `50 Hz`, GNSS at `10 Hz`, status at `10 Hz`,
  and heartbeat at `1 Hz`, with about `29.42 kB/s` received. That run saw `24` invalid CRC
  candidates and sequence-gap accounting reported estimated missing frames, so it is a timing
  confirmation rather than the cleanest link-integrity reference.
- The measurement series found the main problem:
  - the IMU producer cadence was clean and not the source of the long tail;
  - disabling CRSF and MAVLink TX made timing close to the isolated IMU case;
  - direct RC-stage scope captures showed `run_rc_command_state_stages()` could take p99 about
    `98.8 us` and worst about `146.9 us`;
  - moving RC/state to the service phase cut pre-control p99 from `143.5 us` to `62.1 us`.
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
