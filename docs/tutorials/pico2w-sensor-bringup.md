# Pico 2 W Sensor Bring-Up

This guide documents the Pico 2 W GY-91-style SPI sensor bring-up used on the RP2350 branch.

## Hardware

Tested wiring:

| Pico 2 W | GY-91 label | Function |
| --- | --- | --- |
| 3V3 | 3V3 | 3.3 V power |
| GND | GND | Ground |
| GP10 | SCL | SPI1 SCK |
| GP11 | SDA | SPI1 MOSI |
| GP12 | SDO/SA0 | SPI1 MISO |
| GP13 | NCS | MPU chip select |
| GP14 | CSB | BMP280 chip select |

Leave `VIN` unconnected when using Pico 2 W `3V3`.

The tested board did not expose an MPU data-ready interrupt pin on the visible header, so the current
driver polls over SPI. The MPU accel/gyro path is a direct SPI burst. BMP280 and optional
magnetometer reads are throttled separately.

## Build The Probe

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin imu_spi_probe
```

## Flash And Verify The Probe

```bash
probe-rs download --chip RP235x target/thumbv8m.main-none-eabihf/debug/imu_spi_probe
probe-rs verify --chip RP235x target/thumbv8m.main-none-eabihf/debug/imu_spi_probe
probe-rs reset --chip RP235x
```

`probe-rs reset` may warn that the core is already running while clearing breakpoints. If the UART
probe output continues, that warning is not a sensor failure.

## Read UART Output

With the debug probe UART connected to Pico GP0/GP1 and ground:

```bash
cat /dev/ttyACM0
```

Expected output on the tested module:

```text
mpu9250 whoami 0x70
bmp280 chipid 0x58
ak8963 not detected
imu seq=4 accel=(0.169,-0.129,10.185) gyro=(0.002,0.026,0.003) temp=28.66
baro pressure=85626.7 temp=26.52
```

The `0x70` MPU identity behaves like an MPU6500-class accel/gyro. The BMP280 ID `0x58` confirms the
barometer. AK8963 magnetometer ID should be `0x48` when present; the tested module returned `0x00`
through the MPU auxiliary I2C path, so the driver treats magnetometer as absent.

## Build Voloxide Firmware

UART MAVLink firmware:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide
```

Wi-Fi MAVLink firmware:

```bash
VOLOXIDE_WIFI_SSID=MAGICC VOLOXIDE_WIFI_PASSWORD=magiccwifi \
  cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin voloxide --features wifi
```

The board code wires the GY-91 driver into `SensorBus` without changing `voloxide_core`. Accel/gyro
samples populate `sensors.imu`, BMP280 samples populate `sensors.baro`, and magnetometer samples
populate `sensors.mag` only if AK8963 is detected.

## Current Limitations

- Magnetometer is optional and was not detected on the tested module.
- The visible module header did not expose data-ready interrupt, so sampling is polled.
- Sensor calibration is not complete. Use the current output for bring-up, not final flight tuning.
- BMP280 altitude is currently left as `0.0`; pressure and temperature are populated.
