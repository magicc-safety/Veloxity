# ESP-NOW UART Bridge

This is a test bridge for two Seeed Studio XIAO ESP32-C5 boards.

- `air` firmware: XIAO UART pins connect to Pico UART0.
- `ground` firmware: XIAO USB-C appears as the host serial endpoint.
- MAVLink v1 frames are forwarded in both directions over ESP-NOW using unicast packets on a fixed
  channel.

The bridge is MAVLink-frame-aware rather than a fully transparent byte pipe. It scans local serial
input for MAVLink v1 frames and packs only complete frames into ESP-NOW packets. If an ESP-NOW packet
is lost, the host should see whole MAVLink frame loss/sequence gaps instead of partial-frame CRC
corruption. The ground XIAO uses USB Serial/JTAG as the local serial endpoint, and the air XIAO uses
UART1. ESP-IDF logs and console output are disabled so the ground USB stream stays clean.

The bridge keeps one ESP-NOW packet in flight at a time and waits up to
`CONFIG_BRIDGE_SEND_TIMEOUT_MS` for the send callback. That backpressure keeps packet ordering
simple without allowing a missed callback to stop the transmit task forever.

## Wiring

Air-side XIAO to Pico:

| XIAO ESP32-C5 | Pico 2 W |
| --- | --- |
| D6 / TX / GPIO11 | GP1 / UART0 RX |
| D7 / RX / GPIO12 | GP0 / UART0 TX |
| GND | GND |

Power the air-side XIAO from USB-C for bench testing. Do not connect XIAO `5V` to the Pico/BEC rail while USB-C is also connected unless we deliberately verify that power path first.

For isolated bidirectional bridge testing, disconnect the air XIAO from the Pico UART and jumper air
XIAO `D6 / TX / GPIO11` to air XIAO `D7 / RX / GPIO12`. A MAVLink frame written to the ground USB
endpoint should echo back through:

```text
ground USB MAVLink frame -> ESP-NOW -> air UART TX -> air UART RX -> ESP-NOW -> ground USB
```

Arbitrary non-MAVLink byte patterns are intentionally discarded by the framed bridge.

## Peers

The current boards are configured as fixed ESP-NOW peers:

| Role | Base MAC | Peer MAC |
| --- | --- | --- |
| air | `38:44:be:a4:06:bc` | `38:44:be:a4:15:b8` |
| ground | `38:44:be:a4:15:b8` | `38:44:be:a4:06:bc` |

Both images use Wi-Fi channel 1 and a `2000000` baud local serial rate.

## MAVLink Runtime Test

With the air XIAO connected to Pico UART0 and the ground XIAO connected to the host over USB-C, test
the bridge as a MAVLink serial endpoint:

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

The command measures 60 seconds after a 3 second warmup.

## Build

Install and source ESP-IDF with ESP32-C5 support, then:

```bash
idf.py -C tools/espnow_uart_bridge \
  -B tools/espnow_uart_bridge/build-air-unicast \
  -D SDKCONFIG=build-air-unicast/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.air.defaults" \
  build
```

For the ground side:

```bash
idf.py -C tools/espnow_uart_bridge \
  -B tools/espnow_uart_bridge/build-ground-unicast \
  -D SDKCONFIG=build-ground-unicast/sdkconfig \
  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.ground.defaults" \
  build
```

Automatic reset into the ESP32-C5 ROM loader is unreliable on these XIAO boards. The reliable flash
sequence is:

1. Hold `BOOT`.
2. Tap `RESET`.
3. Release `BOOT`.
4. Flash with esptool `--before no-reset --after hard-reset`.
5. If the board remains in `waiting for download`, tap `RESET` once with `BOOT` released.
