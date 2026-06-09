# Hardware Bring-Up Notes

This file records the current hardware bring-up state so future sessions do not have to rediscover
device identities, wiring choices, and known failures.

## Current USB Identities

- Raspberry Pi Debug Probe / RP2350 SWD: `2e8a:000c:E6647C7403301534`
  - Stable symlink: `usb-Raspberry_Pi_Debug_Probe__CMSIS-DAP__E6647C7403301534-if01`
  - Usually `/dev/ttyACM0` for debug-probe UART.
- Drone/air XIAO ESP32-C5: base MAC `38:44:be:a4:06:bc`
  - Stable symlink: `usb-Espressif_USB_JTAG_serial_debug_unit_38:44:BE:A4:06:BC-if00`
  - Device number changes; in the 2026-06-09 session it was `/dev/ttyACM2`.
- Ground/receiver XIAO ESP32-C5: base MAC `38:44:be:a4:15:b8`
  - Stable symlink: `usb-Espressif_USB_JTAG_serial_debug_unit_38:44:BE:A4:15:B8-if00`
  - Device number changes; in the 2026-06-09 session it was `/dev/ttyACM1`.

Prefer `/dev/serial/by-id/...` symlinks when possible because `/dev/ttyACM*` numbers can change.

## Intended Wiring

- RP2350/Pico UART0 telemetry:
  - Preferred/documented XIAO UART pair: GP0 TX -> drone XIAO D7 / RX / GPIO12 RX.
  - GP1 RX <- drone XIAO D6 / TX / GPIO11 TX.
  - Earlier unidirectional testing used GP0 TX -> drone XIAO D5 / SCL / GPIO24 RX; this is not the
    documented XIAO `Serial1` RX pin and should not be the default for bidirectional validation.
  - Common GND required.
  - UART baud: `2_000_000`.
- Ground XIAO uses USB-C as the host companion endpoint. With clean bridge builds, host writes to
  ground USB are forwarded to RP2350 GP1, and RP2350 GP0 telemetry is forwarded back to ground USB.

## Confirmed Working

- RP2350 SWD/debug probe can identify and flash the RP2350.
- Main `voloxide` firmware builds and can be flashed with `probe-rs download`.
- Main `voloxide` firmware with `timing-diagnostics ism330dhcx-driver` builds, flashes, and emits
  MAVLink over RP2350 UART0 at `2_000_000` baud.
- Main `voloxide` firmware also builds with
  `timing-diagnostics ism330dhcx-driver ism330dhcx-1k666 release-loop-bench` for the 300 MHz,
  1.666 kHz IMU budget test.
- Sensor bring-up status in the main firmware:
  - IMU accelerometer/gyro works through the ISM330DHCX driver and produced parsed MAVLink IMU
    frames through the ESP-NOW bridge at about 833 Hz in the latest bench test.
  - GPS over the PIO UART path works and produced parsed GNSS MAVLink frames through the ESP-NOW
    bridge at about 7.4 Hz indoors with no fix.
  - Barometer worked in earlier standalone probes. Main firmware now has a BMP280-on-SPI1 producer
    sharing the IMU SPI owner and publishing through the board sensor bus, but this still needs a
    post-flash hardware validation run.
  - ELRS/CRSF receiver worked in earlier standalone testing, but the latest main-firmware bridge
    test did not show parsed RC frames.
- ESP32-C5 images build for air and ground roles.
- Manual BOOT-mode flashing works on both ESP32-C5 boards using esptool with `--before no-reset`.
- 2026-06-09: high-rate RP2350 firmware flashed successfully after unplug/replugging the Raspberry
  Pi Debug Probe. The working command used the exact CMSIS-DAP selector:
  `probe-rs download --probe 2e8a:000c-0:E6647C7403301534 --chip RP235x --protocol swd`.
- 2026-06-09: patched bidirectional ESP-NOW bridge images build successfully:
  - Air: `tools/espnow_uart_bridge/build-air-bidir-v12`.
  - Ground: `tools/espnow_uart_bridge/build-ground-bidir-v12`.
- 2026-06-09: patched ground bridge image flashed successfully to
  `38:44:BE:A4:15:B8` with esptool `--before no-reset` after manual BOOT/RESET entry.
- 2026-06-09: the ESP-NOW bridge implementation was replaced with a simpler fixed-peer unicast
  bridge:
  - Ground local endpoint: USB Serial/JTAG driver API only.
  - Air local endpoint: UART1 on XIAO `D6/TX/GPIO11` and `D7/RX/GPIO12`.
  - ESP-NOW: fixed channel 1, explicit peer MACs, no broadcast, no VFS/stdout/stdin use in the byte
    path, clean binary payload stream.
- 2026-06-09: clean unicast bridge images build successfully:
  - Air: `tools/espnow_uart_bridge/build-air-unicast`.
  - Ground: `tools/espnow_uart_bridge/build-ground-unicast`.
  - Generated configs verified: air peer `38:44:be:a4:15:b8`, ground peer `38:44:be:a4:06:bc`,
    payload max 200 bytes, D6/GPIO11 TX, D7/GPIO12 RX, primary/secondary USB console disabled.
- 2026-06-09: ground flash attempt while not in manual bootloader mode failed with
  `Failed to connect to ESP32-C5: No serial data received`. This confirms the next action is manual
  BOOT/RESET entry before flashing, not another code change.
- 2026-06-09: after manual BOOT/RESET entry, both unicast bridge images flashed and verified:
  - Ground `38:44:BE:A4:15:B8`: `tools/espnow_uart_bridge/build-ground-unicast`.
  - Air `38:44:BE:A4:06:BC`: `tools/espnow_uart_bridge/build-air-unicast`.
- 2026-06-09: first isolated loopback attempt immediately after esptool `--after hard-reset` sent
  808 bytes to the ground USB endpoint and received 0 bytes. Treat this as inconclusive until both
  XIAOs are manually reset once with `BOOT` released; automatic post-flash reset is not reliable
  enough to prove the bridge app is running.
- 2026-06-09: fixed-peer ESP-NOW itself is proven bidirectional with diagnostic images:
  - Air-to-ground: air `build-air-unicast-test-stats` sent periodic test packets; ground
    `build-ground-unicast-stats` received them with rising `rx_packets`, `rx_bad_crc=0`, and
    `rx_bad_peer=0`.
  - Ground-to-air: ground `build-ground-unicast-test-stats` sent periodic test packets; air
    `build-air-unicast-test-stats` received them with rising `rx_packets`, `rx_bad_crc=0`, and
    `rx_bad_peer=0`.
  - Runtime STA MACs are the base MACs: air `38:44:be:a4:06:bc`, ground `38:44:be:a4:15:b8`.
  - Diagnostic stats builds intentionally skip the local USB RX forwarding task so diagnostics are
    readable; clean builds still run the transparent USB/UART forwarding tasks.
- 2026-06-09: clean transparent ESP-NOW UART bridge is proven bidirectional in isolation:
  - Images: `tools/espnow_uart_bridge/build-ground-unicast` and
    `tools/espnow_uart_bridge/build-air-unicast`.
  - Required fix: `local_rx_task` must call `vTaskDelay(1)` when the local read returns no bytes.
    Without that yield, the high-priority local RX task can spin and starve the ESP-NOW TX path.
  - Test wiring: air XIAO `D6 / TX / GPIO11` jumpered to air XIAO `D7 / RX / GPIO12`, air
    disconnected from RP2350 UART.
  - Test result after flashing clean images and resetting with `BOOT` released:
    `sent=936 received=936 exact_payload_found=True offset=0 elapsed_s=0.053`.
  - This proves the full path:
    ground USB -> ESP-NOW -> air UART TX -> air UART RX -> ESP-NOW -> ground USB.
- 2026-06-09: 120-second isolated loopback consistency test passed after reflashing both clean
  working images and resetting with `BOOT` released:
  - Frames: `5826/5826` exact echoes.
  - Timeouts: `0`.
  - Mismatches: `0`.
  - Bytes sent: `1048680`.
  - Bytes echoed and verified: `1048680`.
  - Residual RX bytes: `0`.
  - Verified goodput: `8737.6 B/s` / `69.90 kbps`.
  - RTT: min `15.166 ms`, mean `20.588 ms`, median `20.006 ms`, p95 `25.293 ms`, p99 `31.605 ms`,
    max `50.252 ms`.
- The ESP-NOW path was previously proven with an air test-pattern image and ground stats image.
- Pico UART0 GP0 -> drone XIAO GPIO24/D5 -> ESP-NOW -> ground XIAO USB is confirmed working with
  the `uart0_text_probe` image. Ground received many `PICO_UART_TEST N` lines.
- Clean non-diagnostic bridge images are confirmed working after manually resetting both XIAOs with
  `BOOT` released:
  - Air image: `tools/espnow_uart_bridge/build-air-uart-clean`.
  - Ground image: `tools/espnow_uart_bridge/build-ground-clean`.
  - Latest parsed output included IMU, GNSS, status, STATUSTEXT diagnostics, and perf frames.

## Not Yet Proven

- Main-firmware barometer telemetry after the new BMP280 shared-SPI producer is flashed.
- Main-firmware ELRS/CRSF telemetry through the ESP-NOW bridge. The receiver worked in earlier
  standalone testing, but the latest test reported `rc: no frames`. If the receiver is not powered,
  connected, bound, and emitting CRSF, no RC frames are expected.
- Full bidirectional MAVLink companion traffic through the ESP-NOW bridge after reconnecting the air
  XIAO to RP2350 UART0. `tools/mavlink_tester.py --timesync-probe` sends MAVLink TIMESYNC requests
  over the ground XIAO USB endpoint and counts responses.
- 1.666 kHz real-IMU budget test on hardware.
- 2026-06-09 superseded failure: isolated loopback with air `D6/TX/GPIO11 -> D5/GPIO24` failed on
  v14 and v15, returning ESP32 backtrace text instead of the test payload. Seeed documents hardware
  `Serial1` as `D6/TX/GPIO11` and `D7/RX/GPIO12`; the current bridge now uses D6/D7 and must be
  validated with a D6 -> D7 isolated loopback before reconnecting to the RP2350.

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

- The current bridge target is a clean unicast serial bridge, not the earlier broadcast/diagnostic
  bridge. Treat old `build-air-bidir-v12`, `build-ground-bidir-v12`, `build-air-uart-clean`, and
  `build-ground-clean` images as historical unless deliberately comparing regressions.
- `sdkconfig.stats.defaults` enables diagnostics.
- `sdkconfig.test-pattern.defaults` enables the air-side periodic test sender.
- Diagnostics and ground USB payload output were changed to write through
  `usb_serial_jtag_write_bytes()`.
- Ground USB payload input is read with `usb_serial_jtag_read_bytes()` in clean builds and forwarded
  to the air bridge over ESP-NOW.
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
- 2026-06-09 ground crash: the ground direct-USB build printed an ESP32 backtrace on USB instead of
  MAVLink. `addr2line` decoded the backtrace into
  `/tmp/esp-idf/components/vfs/vfs.c` and `usb_serial_jtag_vfs.c`. The fix in `bridge.c` is to keep
  the low-level `usb_serial_jtag_driver_install/read_bytes/write_bytes` path but stop touching
  `stdin`, `stdout`, `fcntl`, or `usb_serial_jtag_vfs` on the ground image.

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

1. Reset both XIAOs with `BOOT` released so both bridge applications run.
2. Validate the ESP-NOW UART bridge in isolation before trusting Voloxide telemetry:
   - Temporarily disconnect the air XIAO UART pins from Pico GP0/GP1, or otherwise ensure the Pico
     UART is not driving the same lines.
   - Jumper air XIAO `D6 / TX / GPIO11` to air XIAO `D7 / RX / GPIO12`.
   - Send a known byte pattern into the ground XIAO USB endpoint and verify the exact bytes echo
     back. This proves ground USB -> ESP-NOW -> air UART TX -> air UART RX -> ESP-NOW -> ground USB.
   - Remove the loopback jumper and reconnect air XIAO UART pins to Pico GP0/GP1 before Voloxide
     validation.
3. Run `mavlink_tester.py --timesync-probe` and confirm IMU, baro, GNSS, status, perf, and TIMESYNC
   response frames.
4. Re-test ELRS/CRSF on UART1 GP8/GP9 in the main firmware and compare with the standalone
   `crsf_probe` result if no RC frames appear.
5. Once RC is connected and verified, record expected rates for all sensors.

## Known Build Trap

- ESP-IDF build directories keep their generated `sdkconfig`. Do not reuse a build directory that
  previously enabled `CONFIG_BRIDGE_TEST_PATTERN=y` for a UART-forwarding test.
- Use `tools/espnow_uart_bridge/build-air-uart-stats` for the air UART-forwarding diagnostic image:
  `sdkconfig.defaults;sdkconfig.air.defaults;sdkconfig.stats.defaults`.
- Use `tools/espnow_uart_bridge/build-air-test` only for the air ESP-NOW test-pattern image:
  `sdkconfig.defaults;sdkconfig.air.defaults;sdkconfig.stats.defaults;sdkconfig.test-pattern.defaults`.
- If `/tmp/esp-idf` is missing but `.espressif` still exists, restore ESP-IDF with
  `git clone --depth 1 --branch release/v6.0 https://github.com/espressif/esp-idf.git /tmp/esp-idf`
  and then run `git submodule update --init --recursive --depth 1` in `/tmp/esp-idf`.

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

Build current patched bidirectional bridge images without sourcing `export.sh`:

```bash
PATH=/home/skink/projects/ROSflight/.distrobox-home/ROSflight/.espressif/tools/riscv32-esp-elf/esp-15.2.0_20251204/riscv32-esp-elf/bin:$PATH \
IDF_PATH=/tmp/esp-idf ESP_IDF_VERSION=6.0.0 \
IDF_PYTHON_ENV_PATH=/home/skink/projects/ROSflight/.distrobox-home/ROSflight/.espressif/python_env/idf6.0_py3.12_env \
IDF_TOOLS_PATH=/home/skink/projects/ROSflight/.distrobox-home/ROSflight/.espressif \
IDF_SKIP_CHECK_SUBMODULES=1 \
/home/skink/projects/ROSflight/.distrobox-home/ROSflight/.espressif/python_env/idf6.0_py3.12_env/bin/python \
  /tmp/esp-idf/tools/idf.py -C tools/espnow_uart_bridge \
  -B tools/espnow_uart_bridge/build-air-unicast \
  -D SDKCONFIG=build-air-unicast/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.air.defaults" build

PATH=/home/skink/projects/ROSflight/.distrobox-home/ROSflight/.espressif/tools/riscv32-esp-elf/esp-15.2.0_20251204/riscv32-esp-elf/bin:$PATH \
IDF_PATH=/tmp/esp-idf ESP_IDF_VERSION=6.0.0 \
IDF_PYTHON_ENV_PATH=/home/skink/projects/ROSflight/.distrobox-home/ROSflight/.espressif/python_env/idf6.0_py3.12_env \
IDF_TOOLS_PATH=/home/skink/projects/ROSflight/.distrobox-home/ROSflight/.espressif \
IDF_SKIP_CHECK_SUBMODULES=1 \
/home/skink/projects/ROSflight/.distrobox-home/ROSflight/.espressif/python_env/idf6.0_py3.12_env/bin/python \
  /tmp/esp-idf/tools/idf.py -C tools/espnow_uart_bridge \
  -B tools/espnow_uart_bridge/build-ground-unicast \
  -D SDKCONFIG=build-ground-unicast/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.ground.defaults" build
```

Flash the current patched bridge images after manually entering BOOT mode:

```bash
/tmp/voloxide-esptool-venv/bin/python -m esptool --chip esp32c5 \
  -p /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_38:44:BE:A4:15:B8-if00 \
  -b 460800 --before no-reset --after hard-reset write-flash \
  --flash-mode dio --flash-size 2MB --flash-freq 80m \
  0x2000 tools/espnow_uart_bridge/build-ground-unicast/bootloader/bootloader.bin \
  0x8000 tools/espnow_uart_bridge/build-ground-unicast/partition_table/partition-table.bin \
  0x10000 tools/espnow_uart_bridge/build-ground-unicast/espnow_uart_bridge.bin

/tmp/voloxide-esptool-venv/bin/python -m esptool --chip esp32c5 \
  -p /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_38:44:BE:A4:06:BC-if00 \
  -b 460800 --before no-reset --after hard-reset write-flash \
  --flash-mode dio --flash-size 2MB --flash-freq 80m \
  0x2000 tools/espnow_uart_bridge/build-air-unicast/bootloader/bootloader.bin \
  0x8000 tools/espnow_uart_bridge/build-air-unicast/partition_table/partition-table.bin \
  0x10000 tools/espnow_uart_bridge/build-air-unicast/espnow_uart_bridge.bin
```

Run the isolated bidirectional bridge loopback test after both XIAOs are reset with `BOOT` released
and the air XIAO UART pins are looped back:

```bash
python3 - <<'PY'
import os
import select
import termios
import time

device = "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_38:44:BE:A4:15:B8-if00"
baud = termios.B2000000
fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
attrs = termios.tcgetattr(fd)
attrs[0] = 0
attrs[1] = 0
attrs[2] = baud | termios.CS8 | termios.CREAD | termios.CLOCAL
attrs[3] = 0
attrs[4] = baud
attrs[5] = baud
termios.tcsetattr(fd, termios.TCSANOW, attrs)
termios.tcflush(fd, termios.TCIOFLUSH)

payload = (b"VOL_ESPNOW_LOOPBACK_20260609_" + bytes(range(64))) * 8
os.write(fd, payload)

deadline = time.monotonic() + 5.0
rx = bytearray()
while time.monotonic() < deadline and len(rx) < len(payload):
    readable, _, _ = select.select([fd], [], [], 0.25)
    if readable:
        rx.extend(os.read(fd, 4096))
os.close(fd)

print(f"sent={len(payload)} received={len(rx)}")
if bytes(rx[:len(payload)]) != payload:
    raise SystemExit("loopback: FAIL")
print("loopback: PASS")
PY
```

Run the 1.666 kHz IMU, baro, and bidirectional TIMESYNC validation after flashing:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --release \
  --features 'timing-diagnostics ism330dhcx-driver ism330dhcx-1k666 release-loop-bench'
python3 tools/mavlink_tester.py --transport uart --device /dev/ttyACM2 --baud 2000000 \
  --samples 400 --duration-s 20 --warmup-s 1 --show 8 --timesync-probe --diagnostics
```

Read ground output:

```bash
python3 -c 'import serial,time; s=serial.Serial("/dev/ttyACM2",115200,timeout=0.2); s.dtr=False; s.rts=False; end=time.time()+8; data=bytearray()
while time.time()<end:
    data.extend(s.read(4096))
s.close(); print(data.decode("utf-8","replace"))'
```
