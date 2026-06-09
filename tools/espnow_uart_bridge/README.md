# ESP-NOW UART Bridge

This is a test bridge for two Seeed Studio XIAO ESP32-C5 boards.

- `air` firmware: XIAO UART pins connect to Pico UART0.
- `ground` firmware: XIAO USB-C appears as the host serial endpoint.
- Bytes are forwarded in both directions over ESP-NOW using unicast packets on a fixed channel.

The bridge intentionally acts as a transparent serial link. The ground XIAO uses USB Serial/JTAG as
the local serial endpoint, and the air XIAO uses UART1. ESP-IDF logs and console output are disabled
so the ground USB stream stays clean.

## Wiring

Air-side XIAO to Pico:

| XIAO ESP32-C5 | Pico 2 W |
| --- | --- |
| D6 / TX / GPIO11 | GP1 / UART0 RX |
| D7 / RX / GPIO12 | GP0 / UART0 TX |
| GND | GND |

Power the air-side XIAO from USB-C for bench testing. Do not connect XIAO `5V` to the Pico/BEC rail while USB-C is also connected unless we deliberately verify that power path first.

For isolated bidirectional bridge testing, disconnect the air XIAO from the Pico UART and jumper air
XIAO `D6 / TX / GPIO11` to air XIAO `D7 / RX / GPIO12`. A byte pattern written to the ground USB
endpoint should echo back exactly through:

```text
ground USB -> ESP-NOW -> air UART TX -> air UART RX -> ESP-NOW -> ground USB
```

## Peers

The current boards are configured as fixed ESP-NOW peers:

| Role | Base MAC | Peer MAC |
| --- | --- | --- |
| air | `38:44:be:a4:06:bc` | `38:44:be:a4:15:b8` |
| ground | `38:44:be:a4:15:b8` | `38:44:be:a4:06:bc` |

Both images use Wi-Fi channel 1 and a `2000000` baud local serial rate.

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
