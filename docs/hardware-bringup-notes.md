# Hardware Bring-Up Notes

This file records the current hardware bring-up state so future sessions do not have to rediscover
device identities, wiring choices, and known failures.

## Current USB Identities

- Raspberry Pi Debug Probe / RP2350 SWD: `2e8a:000c:E6647C7403301534`
  - Stable symlink: `usb-Raspberry_Pi_Debug_Probe__CMSIS-DAP__E6647C7403301534-if01`
  - Usually `/dev/ttyACM0` for debug-probe UART.
- Drone/air XIAO ESP32-C5: base MAC `38:44:be:a4:06:bc`
  - Stable symlink: `usb-Espressif_USB_JTAG_serial_debug_unit_38:44:BE:A4:06:BC-if00`
  - Usually `/dev/ttyACM1`.
- Ground/receiver XIAO ESP32-C5: base MAC `38:44:be:a4:15:b8`
  - Stable symlink: `usb-Espressif_USB_JTAG_serial_debug_unit_38:44:BE:A4:15:B8-if00`
  - Usually `/dev/ttyACM2`.

Prefer `/dev/serial/by-id/...` symlinks when possible because `/dev/ttyACM*` numbers can change.

## Intended Wiring

- RP2350/Pico UART0 telemetry:
  - GP0 TX -> drone XIAO D5 / SCL / GPIO24 RX.
  - GP1 RX <- drone XIAO D6 / TX / GPIO11 TX.
  - Common GND required.
  - UART baud: `2_000_000`.
- Ground XIAO uses USB-C as its output. It is not expected to output over external UART pins.

## Confirmed Working

- RP2350 SWD/debug probe can identify and flash the RP2350.
- Main `voloxide` firmware builds and can be flashed with `probe-rs download`.
- Main `voloxide` firmware with `timing-diagnostics ism330dhcx-driver` builds, flashes, and emits
  MAVLink over RP2350 UART0 at `2_000_000` baud.
- Sensor bring-up status in the main firmware:
  - IMU accelerometer/gyro works through the ISM330DHCX driver and produced parsed MAVLink IMU
    frames through the ESP-NOW bridge at about 833 Hz in the latest bench test.
  - GPS over the PIO UART path works and produced parsed GNSS MAVLink frames through the ESP-NOW
    bridge at about 7.4 Hz indoors with no fix.
  - Barometer worked in earlier standalone probes, but is not yet integrated into the main
    `voloxide` binary.
  - ELRS/CRSF receiver worked in earlier standalone testing, but the latest main-firmware bridge
    test did not show parsed RC frames.
- ESP32-C5 images build for air and ground roles.
- Manual BOOT-mode flashing works on both ESP32-C5 boards using esptool with `--before no-reset`.
- The ESP-NOW path was previously proven with an air test-pattern image and ground stats image.
- Pico UART0 GP0 -> drone XIAO GPIO24/D5 -> ESP-NOW -> ground XIAO USB is confirmed working with
  the `uart0_text_probe` image. Ground received many `PICO_UART_TEST N` lines.
- Clean non-diagnostic bridge images are confirmed working after manually resetting both XIAOs with
  `BOOT` released:
  - Air image: `tools/espnow_uart_bridge/build-air-uart-clean`.
  - Ground image: `tools/espnow_uart_bridge/build-ground-clean`.
  - Latest parsed output included IMU, GNSS, status, STATUSTEXT diagnostics, and perf frames.

## Not Yet Proven

- Main-firmware barometer telemetry. The `Gy91`/BMP280 helper exists and earlier baro probes worked,
  but `boards/pico2w/src/bin/voloxide.rs` currently calls `Board::new_uart(config, None)`, so no
  baro object is passed into the main board sensor producer.
- Main-firmware ELRS/CRSF telemetry through the ESP-NOW bridge. The receiver worked in earlier
  standalone testing, but the latest test reported `rc: no frames`.
- Full bidirectional companion traffic through the ESP-NOW bridge. The latest test primarily proved
  RP2350 TX -> air XIAO -> ESP-NOW -> ground XIAO USB.

## Current ESP32-C5 Boot/Flash Behavior

- Automatic esptool reset into bootloader is unreliable on both XIAO ESP32-C5 boards.
- Reliable flashing sequence:
  1. Hold `BOOT`.
  2. Tap `RESET`.
  3. Release `BOOT`.
  4. Flash with esptool `--before no-reset --after hard-reset`.
  5. If USB output shows `waiting for download`, tap `RESET` once with `BOOT` released.
- Seeing `boot:0x8 (DOWNLOAD(UART0/USB))` and `waiting for download` means the board is still in
  ROM download mode and the bridge application is not running.

## Current Bridge Diagnostics Status

- `sdkconfig.stats.defaults` enables diagnostics.
- `sdkconfig.test-pattern.defaults` enables the air-side periodic test sender.
- Diagnostics and ground USB payload output were changed to write through
  `usb_serial_jtag_write_bytes()`.
- Direct USB serial/JTAG writes require `usb_serial_jtag_driver_install()` before use; this is now
  initialized in `bridge.c` for diagnostics and ground output paths.
- Diagnostics now also call `usb_serial_jtag_wait_tx_done()` after direct writes and mirror stats to
  stdout. This was added because boards could be running app firmware but remain silent on USB.
- Ground diagnostics are confirmed working after the flush/stdout-mirror change. A successful ground
  read prints the bridge banner and repeated stats lines.
- Latest ESP-NOW-only state:
  - Confirmed working after pre-init diagnostics.
  - Air app prints boot diagnostics and reports rising `send_ok`.
  - Ground app receives forwarded `[espnow-uart test] air_count=N` payloads and reports rising `rx`.
- Diagnostics now print `ESP-NOW UART bridge boot` before WiFi/ESP-NOW init and report init errors
  instead of aborting silently. Reflash the air diagnostic image after this change before drawing any
  further conclusions about the air sender.

## Latest End-to-End Test

Clean bridge images and real-IMU Voloxide firmware were tested through the ground XIAO USB endpoint:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'timing-diagnostics ism330dhcx-driver'
probe-rs download --probe 2e8a:000c:E6647C7403301534 --chip RP235x --protocol swd \
  target/thumbv8m.main-none-eabihf/release/voloxide
python3 tools/mavlink_tester.py --transport uart --device /dev/ttyACM2 --baud 2000000 \
  --samples 400 --duration-s 14 --warmup-s 1 --show 8
```

Observed result:

- `imu`: 10835 frames, about 833 Hz.
- `gnss`: 96 frames, about 7.4 Hz, indoor no-fix data.
- `status`: 87 frames.
- `text`: 38 STATUSTEXT diagnostics.
- `perf`: 37 timing diagnostic frames.
- `baro`: no frames.
- `rc`: no frames.

After flashing clean bridge images, tap `RESET` on both XIAOs with `BOOT` released. Otherwise one of
the boards may remain in ROM download mode and the ground USB stream can be zero bytes.

## Next Bring-Up Steps

1. Integrate the barometer into the main `voloxide` firmware. Avoid giving two owners independent
   control of SPI1; the existing `Gy91` helper currently owns the whole blocking SPI bus.
2. Re-test ELRS/CRSF on UART1 GP8/GP9 in the main firmware and compare with the standalone
   `crsf_probe` result if no RC frames appear.
3. Prove bidirectional companion traffic through ESP-NOW by sending a MAVLink request from the
   ground XIAO USB endpoint and confirming the RP2350 receives/parses it.
4. Once baro and RC are fixed, rerun `mavlink_tester.py` and record expected rates for all sensors.

## Known Build Trap

- ESP-IDF build directories keep their generated `sdkconfig`. Do not reuse a build directory that
  previously enabled `CONFIG_BRIDGE_TEST_PATTERN=y` for a UART-forwarding test.
- Use `tools/espnow_uart_bridge/build-air-uart-stats` for the air UART-forwarding diagnostic image:
  `sdkconfig.defaults;sdkconfig.air.defaults;sdkconfig.stats.defaults`.
- Use `tools/espnow_uart_bridge/build-air-test` only for the air ESP-NOW test-pattern image:
  `sdkconfig.defaults;sdkconfig.air.defaults;sdkconfig.stats.defaults;sdkconfig.test-pattern.defaults`.

## Useful Commands

Build air test-pattern diagnostics:

```bash
source /tmp/esp-idf/export.sh
IDF_SKIP_CHECK_SUBMODULES=1 idf.py -C tools/espnow_uart_bridge \
  -B tools/espnow_uart_bridge/build-air-test \
  -D SDKCONFIG=build-air-test/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.air.defaults;sdkconfig.stats.defaults;sdkconfig.test-pattern.defaults" \
  build
```

Build ground stats diagnostics:

```bash
source /tmp/esp-idf/export.sh
IDF_SKIP_CHECK_SUBMODULES=1 idf.py -C tools/espnow_uart_bridge \
  -B tools/espnow_uart_bridge/build-ground-stats \
  -D SDKCONFIG=build-ground-stats/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.ground.defaults;sdkconfig.stats.defaults" \
  build
```

Build clean bridge images:

```bash
source /tmp/esp-idf/export.sh
IDF_SKIP_CHECK_SUBMODULES=1 idf.py -C tools/espnow_uart_bridge \
  -B tools/espnow_uart_bridge/build-air-uart-clean \
  -D SDKCONFIG=build-air-uart-clean/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.air.defaults" \
  build
IDF_SKIP_CHECK_SUBMODULES=1 idf.py -C tools/espnow_uart_bridge \
  -B tools/espnow_uart_bridge/build-ground-clean \
  -D SDKCONFIG=build-ground-clean/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.ground.defaults" \
  build
```

Read ground output:

```bash
python3 -c 'import serial,time; s=serial.Serial("/dev/ttyACM2",115200,timeout=0.2); s.dtr=False; s.rts=False; end=time.time()+8; data=bytearray()
while time.time()<end:
    data.extend(s.read(4096))
s.close(); print(data.decode("utf-8","replace"))'
```
