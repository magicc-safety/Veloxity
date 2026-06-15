# Pico 2 W Sensor Bring-Up

This tutorial is for validating sensors on the active RP2350/Pico 2 W hardware path. It is not a
record of old GY-91-only experiments.

For full wiring, read [Pico 2 W flight hardware pinout](../pico2w-esc-imu-pinout.md). For board
build and flashing context, read [RP2350 / Pico 2 W](../boards/rp2350-pico2w.md).

## Sensor Roles

| Sensor path | Role in current branch |
| --- | --- |
| ISM330DHCX over SPI + data-ready interrupt | Primary flight IMU. This is the source that should drive the control loop. |
| BMP280/GY-91 pressure path | Low-rate barometer. Poll in quiet moments; do not use it as the flight IMU. |
| GPS PIO UART | GNSS input path. |
| QMC5883L/I2C magnetometer | Slow magnetometer path when wired. |
| CRSF UART receiver | RC input path. |

## Install

```bash
rustup target add thumbv8m.main-none-eabihf
cargo install probe-rs-tools
```

## Check The Board Crate

```bash
cargo xtask check-board pico2w
```

## Validate The IMU Path

Build and flash the IMU probe:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin imu_spi_probe --release
probe-rs download --chip RP235x --protocol swd \
  target/thumbv8m.main-none-eabihf/release/imu_spi_probe
probe-rs reset --chip RP235x
```

Run the IMU timing bench:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin imu_spi_bench --release
probe-rs download --chip RP235x --protocol swd \
  target/thumbv8m.main-none-eabihf/release/imu_spi_bench
probe-rs reset --chip RP235x
```

The default flight target for this branch is the real ISM330DHCX at the high-rate output data rate
(ODR) feeding a
fixed 1.5 kHz control loop. Timing results should be collected from release builds, not debug
builds.

## Validate The Full Sensor Stack

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin sensor_stack_probe --release
probe-rs download --chip RP235x --protocol swd \
  target/thumbv8m.main-none-eabihf/release/sensor_stack_probe
probe-rs reset --chip RP235x
```

Use this after the individual IMU, barometer, GPS/mag, and RC paths have been isolated. If the full
stack fails, return to the individual probe matching the failing bus.

## Validate RC Input

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin crsf_probe --release
probe-rs download --chip RP235x --protocol swd \
  target/thumbv8m.main-none-eabihf/release/crsf_probe
probe-rs reset --chip RP235x
```

The RP4TD-M receiver uses UART1 on the current pinout. Confirm power and logic levels before
connecting it to RP2350 GPIO.

## Validate GPS And Magnetometer

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin gps_pio_probe --release
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin gps_mag_probe --release
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin qmc5883l_probe --release
```

Flash the probe matching the bus you are testing:

```bash
probe-rs download --chip RP235x --protocol swd \
  target/thumbv8m.main-none-eabihf/release/<probe-name>
probe-rs reset --chip RP235x
```

## Validate Full Firmware Sensor Flow

Build a release firmware image with the current IMU feature set:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin veloxity --release
```

Flash:

```bash
probe-rs download --chip RP235x --protocol swd \
  target/thumbv8m.main-none-eabihf/release/veloxity
probe-rs reset --chip RP235x
```

For GPIO timing capture:

```bash
cargo build -p pico2w --target thumbv8m.main-none-eabihf --bin veloxity --release \
  --features 'scope-timing-pins control-scope-controller'
```

With `scope-timing-pins`, capture GP14 for raw IMU data-ready, GP18 for scheduled control deadline,
GP19 for control pipeline execution, and GP22 for the selected diagnostic window. A good current
loaded timing run looks like the 120-second 1.5 kHz baseline in the RP2350 guide: every measured
control-deadline-to-pipeline-complete latency remains inside the 1.5 kHz budget while telemetry
stays at the configured rates. The bounded high-rate telemetry profile configures IMU at `400 Hz`,
RC at `100 Hz`, attitude/output/differential-pressure and range at `50 Hz`,
barometer/magnetometer/battery at `25 Hz`, GNSS at `10 Hz`, status at `10 Hz`, and heartbeat at
`1 Hz`. The current high-rate link acceptance command checks the streams present in the current
hardware setup: IMU, RC, attitude, and output raw.

```bash
python3 tools/mavlink_tester.py \
  --transport uart \
  --device /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_38:44:BE:A4:15:B8-if00 \
  --baud 2000000 \
  --duration-s 45 \
  --warmup-s 3 \
  --show 4 \
  --diagnostics \
  --acceptance \
  --expect-imu-hz 400 \
  --expect-rc-hz 100 \
  --expect-attitude-hz 50 \
  --expect-output-raw-hz 50
```

## Debugging Rule

Do not debug the full firmware first when a bus is unknown. Prove the individual sensor with its
probe binary, then prove the combined sensor stack, then prove the full firmware loop.
