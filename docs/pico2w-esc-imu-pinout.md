# Pico 2 W ESC, IMU, and Barometer Pinout

This wiring plan keeps the high-rate flight IMU deterministic while preserving the slower GY-91/BMP280
path as barometer-only. The new fast path is the Adafruit ISM330DHCX 6 DoF IMU on SPI with a
data-ready interrupt. The current GY-91 board should no longer feed accel/gyro data into the flight
loop.

## Electrical Assumptions

- ESC: SEQURE Blueson A2 4-in-1 AM32 ESC.
- Motor protocol: DShot600 first, with DShot300 and DShot150 available by changing PIO timing.
- ESC signal voltage: Pico 2 W drives 3.3 V GPIO. Confirm the ESC input accepts 3.3 V logic before
  flight hardware testing.
- Power: the ESC has no BEC. Do not power the Pico 2 W from the ESC signal harness unless an
  external regulator is added. The Pico 2 W and ESC must share ground.
- IMU: Adafruit ISM330DHCX breakout over SPI. The intended driver is ST's `ism330dhcx-rs`
  embedded-hal async driver with Embassy/RP SPI and GPIO interrupt plumbing. The repository patches
  version 2.0.0 locally under `third_party/ism330dhcx-rs` only to disable `half`'s default `std`
  feature; the driver source is unchanged and remains `no_std` for the Pico target.
- Barometer: the existing GY-91/BMP280 path is retained as a low-rate pressure/temperature source.
  The board code treats it as barometer-only; MPU accel/gyro samples from that board are ignored by
  the flight path.

## Proposed Header GPIO Allocation

| Function | Pico 2 W GPIO | Notes |
| --- | ---: | --- |
| ESC motor 1 signal | GP2 | DShot PIO output bit 0 |
| ESC motor 2 signal | GP3 | DShot PIO output bit 1 |
| ESC motor 3 signal | GP4 | DShot PIO output bit 2 |
| ESC motor 4 signal | GP5 | DShot PIO output bit 3 |
| ESC telemetry reserve | GP6 | Reserved for AM32 telemetry or bidirectional DShot later |
| ISM330DHCX SPI1 SCK | GP10 | Hardware SPI fast IMU bus |
| ISM330DHCX SPI1 MOSI | GP11 | Hardware SPI fast IMU bus |
| ISM330DHCX SPI1 MISO | GP12 | Hardware SPI fast IMU bus |
| ISM330DHCX chip select | GP13 | Reuses the old GY-91 MPU chip-select position |
| ISM330DHCX INT1 / DRDY | GP14 | Reuses the old BMP280 SPI chip-select position |
| ISM330DHCX INT2 reserve | GP15 | Optional FIFO/wakeup interrupt if needed |
| Flight status LED | GP16 | Discrete LED |
| Comms status LED | GP17 | Discrete LED |
| Fault status LED | GP18 | Discrete LED |
| Addressable LED reserve | GP19 | PIO-driven WS2812-style status strip if needed |
| Barometer I2C SDA | GP20 | Slow sensor bus for GY-91/BMP280 pressure path |
| Barometer I2C SCL | GP21 | Slow sensor bus for GY-91/BMP280 pressure path |
| Future aux / capture | GP22 | Reserved |
| Future ADC0 | GP26 | Battery voltage/current path later |
| Future ADC1 | GP27 | Battery voltage/current path later |
| Future ADC2 | GP28 | Battery voltage/current path later |

GP0/GP1 are the default UART MAVLink transport in UART-only firmware. In Wi-Fi MAVLink firmware they
are available for debug UART output during bring-up. Keep the Pico 2 W wireless pins owned by the
CYW43 driver.

## ISM330DHCX SPI Wiring

The Adafruit STEMMA QT/Qwiic connector exposes I2C, but the deterministic IMU path should use the
breakout's SPI pads plus the data-ready interrupt pin.

| Pico 2 W | Adafruit ISM330DHCX label | Function |
| --- | --- | --- |
| 3V3 | VIN | Breakout power from Pico 3.3 V |
| GND | GND | Common ground |
| GP10 | SCK | SPI SCK |
| GP11 | SDI / MOSI | SPI MOSI |
| GP12 | SDO / MISO | SPI MISO |
| GP13 | CS | SPI chip select |
| GP14 | INT1 | Data-ready interrupt |
| GP15 | INT2 | Optional, leave disconnected initially |

This keeps the high-rate sensor wiring short and grouped on GP10-GP15. GP14 is intentionally
repurposed from the old BMP280 SPI chip select because the barometer no longer needs to live on the
fast SPI bus.

## GY-91/BMP280 Barometer Wiring

Move the current GY-91-style board toward the slow I2C header and use it only as a barometer source.
The firmware branch still has the legacy SPI BMP280 implementation for bring-up compatibility, but
the intended wiring for the cleaned-up board is:

| Pico 2 W | GY-91/BMP280 label | Function |
| --- | --- | --- |
| 3V3 | 3V3 | 3.3 V module power |
| GND | GND | Common ground |
| GP20 | SDA | I2C data |
| GP21 | SCL | I2C clock |

If the physical GY-91 board keeps the MPU and BMP280 on shared I2C lines, leave the MPU unused in
firmware. The flight loop should receive IMU samples only from the ISM330DHCX queue.

## PIO and DMA Allocation

| Block | State machine | Purpose | GPIOs |
| --- | --- | --- | --- |
| PIO0 | SM0 | CYW43 Wi-Fi transport | internal Pico 2 W radio pins |
| PIO1 | SM0 | 4-lane DShot motor output | GP2-GP5 |
| PIO1 | SM1 | ESC telemetry reserve | GP6 |
| PIO2 | SM0 | addressable LED reserve | GP19 |

The DShot output should be a single PIO program that emits four motor lines in parallel from packed
DMA words. That keeps all motors frame-synchronous and consumes one state machine instead of four.

## Firmware Shape

Core 0 remains the deterministic flight side:

- receives ISM330DHCX data-ready interrupts,
- samples the ISM330DHCX over SPI through the Embassy-supported `ism330dhcx-rs` driver,
- pushes completed IMU packets into a board-local queue,
- runs the Voloxide control loop from queued sensor samples,
- polls slow barometer data separately.

Core 1 remains the communications side:

- owns CYW43 Wi-Fi,
- owns UDP MAVLink,
- passes MAVLink bytes to core 0 through the mailbox.

The IMU path should not cross the core boundary. Sensor samples should enter `voloxide_core` on core
0 so the control loop is not gated by Wi-Fi, UDP, or mailbox scheduling.

## Branch Status

This branch prepares the architecture for the ISM330DHCX but does not yet complete the physical
driver task. `BoardIo::update_sensor_bus()` drains IMU samples from the new ISM330DHCX queue and
drains barometer samples from the GY-91/BMP280 path. The remaining hardware step is to add the
Embassy SPI/interrupt task that configures the ISM330DHCX and pushes packets into that queue.
