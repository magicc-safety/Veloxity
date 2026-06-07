# ESP-NOW UART Bridge

This is a test bridge for two Seeed Studio XIAO ESP32-C5 boards.

- `air` firmware: XIAO UART pins connect to Pico UART0.
- `ground` firmware: XIAO USB-C appears as the host serial endpoint.
- Bytes are forwarded over ESP-NOW using broadcast packets on a fixed channel.

## Wiring

Air-side XIAO to Pico:

| XIAO ESP32-C5 | Pico 2 W |
| --- | --- |
| D6 / TX / GPIO11 | GP1 / UART0 RX |
| D7 / RX / GPIO12 | GP0 / UART0 TX |
| GND | GND |

Power the air-side XIAO from USB-C for bench testing. Do not connect XIAO `5V` to the Pico/BEC rail while USB-C is also connected unless we deliberately verify that power path first.

## Build

Install and source ESP-IDF with ESP32-C5 support, then:

```bash
idf.py -C tools/espnow_uart_bridge set-target esp32c5
idf.py -C tools/espnow_uart_bridge -B tools/espnow_uart_bridge/build-air -D SDKCONFIG=build-air/sdkconfig -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.air.defaults" build
idf.py -C tools/espnow_uart_bridge -B tools/espnow_uart_bridge/build-air -p /dev/ttyACM1 flash
```

For the ground side:

```bash
idf.py -C tools/espnow_uart_bridge -B tools/espnow_uart_bridge/build-ground -D SDKCONFIG=build-ground/sdkconfig -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.ground.defaults" build
idf.py -C tools/espnow_uart_bridge -B tools/espnow_uart_bridge/build-ground -p /dev/ttyACM2 flash
```

The current UART baud is `2000000` to match the Voloxide companion UART.
Diagnostics are disabled by default so the ground USB serial remains a clean byte stream.
