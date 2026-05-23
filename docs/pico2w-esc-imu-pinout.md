# Pico 2 W ESC, IMU, Barometer, and RC Pinout

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
- RC receiver: RadioMaster RP4TD-M ExpressLRS over CRSF. The receiver is a 5 V device with a CRSF
  UART bus interface and up to 500 Hz / F1000Hz refresh capability, so the default wiring uses Pico
  UART1 rather than PIO soft serial.

## Proposed Header GPIO Allocation

| Function | Pico 2 W GPIO | Notes |
| --- | ---: | --- |
| ESC motor 1 signal | GP2 | DShot PIO output bit 0 |
| ESC motor 2 signal | GP3 | DShot PIO output bit 1 |
| ESC motor 3 signal | GP4 | DShot PIO output bit 2 |
| ESC motor 4 signal | GP5 | DShot PIO output bit 3 |
| ESC telemetry reserve | GP6 | Reserved for AM32 telemetry or bidirectional DShot later |
| RC receiver UART1 TX | GP8 | Pico TX to receiver RX for CRSF telemetry/config |
| RC receiver UART1 RX | GP9 | Receiver TX to Pico RX for CRSF channel frames |
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

## RadioMaster RP4TD-M CRSF Wiring

Use the RP4TD-M as a CRSF serial receiver. Cross the UART data lines in the usual flight-controller
style:

| Pico 2 W | RP4TD-M label | Function |
| --- | --- | --- |
| 5V / VBUS-regulated 5 V rail | 5V | Receiver power |
| GND | GND | Common ground |
| GP8 / UART1 TX | RX | CRSF telemetry/config from Pico to receiver |
| GP9 / UART1 RX | TX | CRSF RC frames from receiver to Pico |

Do not power the RP4TD-M from Pico `3V3`; RadioMaster specifies DC 5.0 V for the RP4TD-M. The Pico
GPIO side is still 3.3 V logic, so verify the receiver TX/RX pads are 3.3 V UART-level before
connecting directly. If measurement or vendor data shows 5 V UART levels, add a level shifter on the
receiver TX line before connecting it to GP9.

The default firmware allocation is UART1 at 420000 baud, 8N1. Embassy UART RX interrupt/DMA service
feeds bytes into the `crsf` crate's `no_std` `PacketParser`; completed RC channel packets are mapped
into `RcPacket` and pushed into the board-local RC queue. `BoardIo::update_sensor_bus()` then drains
the latest RC packet into `SensorBus` without blocking the flight loop. PIO is reserved as a fallback
only if a later board revision needs soft-serial because UART1 pins are unavailable.

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
- receives CRSF frames on UART1 and pushes completed RC packets into a board-local queue,
- runs the Voloxide control loop from queued sensor samples,
- polls slow barometer data separately.

Core 1 remains the communications side:

- owns CYW43 Wi-Fi,
- owns UDP MAVLink,
- passes MAVLink bytes to core 0 through the mailbox.

The IMU path should not cross the core boundary. Sensor samples should enter `voloxide_core` on core
0 so the control loop is not gated by Wi-Fi, UDP, or mailbox scheduling.

## Branch Status

This branch prepares the architecture for the ISM330DHCX and RP4TD-M but does not yet complete the
physical driver tasks. `BoardIo::update_sensor_bus()` drains IMU samples from the new ISM330DHCX
queue, RC samples from the CRSF receiver queue, and barometer samples from the GY-91/BMP280 path. The
remaining hardware step is to add the Embassy SPI/interrupt task that configures the ISM330DHCX.
The RP4TD-M path already uses the `crsf` parser crate and has a UART1 receiver task shape; hardware
validation still needs the receiver wired and bound.
