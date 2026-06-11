# Pico 2 W Flight Hardware Pinout

This wiring plan keeps the high-rate flight IMU deterministic while preserving the slower GY-91/BMP280
path as barometer-only. The fast path is the Adafruit ISM330DHCX 6 DoF IMU on SPI with a data-ready
interrupt. The GPS module provides both GNSS serial data and the co-located QMC5883L magnetometer.
The current GY-91 board should no longer feed accel/gyro data into the flight loop.

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
- GPS and magnetometer: the HGLRC M100 Pro GPS uses a PIO UART for GNSS data and the shared slow I2C
  bus for the co-located QMC5883L magnetometer. GPS PPS/timepulse gets a GPIO interrupt input when
  the pad is available. If the module exposes QMC5883L `DRDY`, route it to the optional GP22 input.
- RC receiver: RadioMaster RP4TD-M ExpressLRS over CRSF. The receiver is a 5 V device with a CRSF
  UART bus interface and up to 500 Hz / F1000Hz refresh capability, so the default wiring uses Pico
  UART1 rather than PIO soft serial.
- Companion link: Seeed Studio XIAO ESP32C5 is treated as the companion-computer UART endpoint. Its
  ESP-NOW/UDP bridge behavior is outside the Pico pinout; electrically it is just the UART0 peer.

## Power Rails and Logic Levels

Do not power 5 V peripherals from the Pico 2 W `3V3` pin. Use a regulated 5 V rail sized for the
receiver, GPS, ESP32 bridge, LEDs, and any future payloads. The Pico and every external module must
share ground.

| Device | Planned power rail | Reasoning and notes |
| --- | --- | --- |
| Pico 2 W / RP2350 | Regulated 5 V into the Pico power input, or USB for bench only | The Pico creates its local 3.3 V rail for the RP2350 and low-current 3.3 V peripherals. |
| HGLRC M100 Pro GPS + QMC5883L | 5 V rail | Vendor spec lists 3.6-5.5 V input, so Pico `3V3` is below the stated input range. Module logic is expected to be 3.3 V: GPS UART TX/RX is 3.3 V logic, and QMC5883L I2C should follow the Pico-side 3.3 V pullups. |
| RadioMaster RP4TD-M ELRS receiver | 5 V rail | Vendor spec lists DC 5.0 V working voltage. Verify CRSF TX/RX are 3.3 V logic before direct connection to GP8/GP9. |
| Seeed Studio XIAO ESP32C5 companion bridge | 5 V/VBUS input or its battery input | Use 5 V input for shared aircraft power unless intentionally running it from its own battery path. XIAO GPIO remains 3.3 V logic. |
| Adafruit ISM330DHCX breakout | Pico `3V3` preferred | The breakout accepts 3-5 V on `VIN`; powering from 3.3 V keeps SPI and interrupt wiring in the Pico logic domain. |
| GY-91/BMP280 barometer module | Pico `3V3` preferred | Many GY-91 variants accept 3-5 V, but 3.3 V avoids 5 V I2C pullups and keeps the BMP280 path safe for RP2350 GPIO. |
| Discrete/status LEDs | 3.3 V GPIO through resistors | Size resistors for low current. Do not load GPIO directly. |
| Addressable LED strip | Usually 5 V rail | Use level shifting or a 3.3 V-compatible LED/short single-pixel path if the LED data input is not reliable at 3.3 V. |

The RP2350 GPIO pins are not 5 V tolerant. If a 5 V-powered receiver, GY-91 clone, LED strip, or
unexpected GPS board variant pulls any signal line to 5 V, add level shifting before connecting it to
the Pico. The M100 Pro GPS/magnetometer path is expected to be direct-wired because its UART is 3.3 V
logic and its open-drain I2C lines should use the Pico-side 3.3 V pullups. Still confirm idle voltage
on GPS `TX`, `SDA`, `SCL`, `PPS`, and any `DRDY` pad during bring-up.

## Proposed Header GPIO Allocation

| Function | Pico 2 W GPIO | Notes |
| --- | ---: | --- |
| Companion UART0 TX | GP0 | Pico TX to ESP32C5 RX |
| Companion UART0 RX | GP1 | ESP32C5 TX to Pico RX |
| ESC motor 1 signal | GP2 | DShot PIO output bit 0 |
| ESC motor 2 signal | GP3 | DShot PIO output bit 1 |
| ESC motor 3 signal | GP4 | DShot PIO output bit 2 |
| ESC motor 4 signal | GP5 | DShot PIO output bit 3 |
| GPS PIO UART RX | GP6 | GPS TX to Pico RX |
| GPS PIO UART TX | GP7 | Pico TX to GPS RX for configuration |
| RC receiver UART1 TX | GP8 | Pico TX to receiver RX for CRSF telemetry/config |
| RC receiver UART1 RX | GP9 | Receiver TX to Pico RX for CRSF channel frames |
| ISM330DHCX SPI1 SCK | GP10 | Hardware SPI fast IMU bus |
| ISM330DHCX SPI1 MOSI | GP11 | Hardware SPI fast IMU bus |
| ISM330DHCX SPI1 MISO | GP12 | Hardware SPI fast IMU bus |
| ISM330DHCX chip select | GP13 | Hardware SPI fast IMU bus |
| ISM330DHCX INT1 / DRDY | GP14 | Primary data-ready interrupt |
| ISM330DHCX INT2 reserve | GP15 | Optional FIFO/wakeup interrupt if needed |
| GPS PPS / timepulse | GP16 | Optional GPIO interrupt input from GPS |
| ESC telemetry reserve | GP17 | Reserved for AM32 telemetry or bidirectional DShot later |
| Flight status LED / scope realtime-pass toggle | GP18 | Discrete GPIO LED by default; `scope-timing-pins` toggles this at each core 0 realtime scheduler pass boundary |
| Addressable LED reserve / scope control strobe | GP19 | PIO-driven WS2812-style status strip by default; `scope-timing-pins` drives this high only during an IMU-triggered control closure |
| Slow I2C SDA | GP20 | QMC5883L magnetometer plus GY-91/BMP280 pressure path |
| Slow I2C SCL | GP21 | QMC5883L magnetometer plus GY-91/BMP280 pressure path |
| Mag DRDY / aux interrupt / scope diagnostic strobe | GP22 | Optional QMC5883L data-ready input if exposed; otherwise spare GPIO. `scope-timing-pins` uses this for the selected GP22 diagnostic mode. |
| Future ADC0 | GP26 | Battery voltage/current path later |
| Future ADC1 | GP27 | Battery voltage/current path later |
| Future ADC2 | GP28 | Battery voltage/current path later |

Keep the Pico 2 W wireless pins owned by the CYW43 driver. The companion path remains UART0 even
when the ESP32C5 peer forwards bytes over ESP-NOW or UDP on its side.

## Layout Visual

```text
                 Raspberry Pi Pico 2 W / RP2350

  companion/comms edge                         sensor/control edge

  GP0  UART0 TX  --------------------------->  ESP32C5 RX
  GP1  UART0 RX  <---------------------------  ESP32C5 TX

  GP2  DShot M1  --------------------------->  4-in-1 ESC M1
  GP3  DShot M2  --------------------------->  4-in-1 ESC M2
  GP4  DShot M3  --------------------------->  4-in-1 ESC M3
  GP5  DShot M4  --------------------------->  4-in-1 ESC M4

  GP6  GPS RX    <---------------------------  HGLRC M100 GPS TX
  GP7  GPS TX    --------------------------->  HGLRC M100 GPS RX
  GP16 GPS PPS   <---------------------------  HGLRC M100 PPS/timepulse

  GP8  UART1 TX  --------------------------->  RP4TD-M RX
  GP9  UART1 RX  <---------------------------  RP4TD-M TX

  GP10 SPI1 SCK  --------------------------->  ISM330DHCX SCK
  GP11 SPI1 MOSI --------------------------->  ISM330DHCX SDI/MOSI
  GP12 SPI1 MISO <---------------------------  ISM330DHCX SDO/MISO
  GP13 GPIO CS   --------------------------->  ISM330DHCX CS
  GP14 GPIO IRQ  <---------------------------  ISM330DHCX INT1/DRDY
  GP15 GPIO IRQ  <---------------------------  ISM330DHCX INT2 optional

  GP20 I2C SDA   <-------------------------->  QMC5883L SDA + BMP280 SDA
  GP21 I2C SCL   --------------------------->  QMC5883L SCL + BMP280 SCL
  GP22 GPIO IRQ  <---------------------------  QMC5883L DRDY optional

  GP17 ESC TEL   <---------------------------  AM32 telemetry optional
  GP18 GPIO LED  --------------------------->  discrete status LED, or scope whole-loop strobe
  GP19 PIO LED   --------------------------->  addressable status LED optional, or scope control strobe
```

All external modules must share ground with the Pico. Keep the GP10-GP15 IMU bundle physically short
and away from motor phase wiring.

## Companion ESP32C5 Wiring

Use the XIAO ESP32C5 as a UART-attached companion bridge. Cross the UART data lines:

| Pico 2 W | ESP32C5 label | Function |
| --- | --- | --- |
| Regulated 5 V or battery input | power input | Board power, confirm the exact XIAO power pin used |
| GND | GND | Common ground |
| GP0 / UART0 TX | RX | MAVLink bytes from Pico to companion bridge |
| GP1 / UART0 RX | TX | MAVLink bytes from companion bridge to Pico |

The default firmware allocation is UART0 at 2 Mbaud. UART interrupt/DMA service keeps the physical
serial path outside the measured flight loop as much as possible.

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

## GPS and Magnetometer Wiring

The HGLRC M100 Pro GPS carries both the GNSS receiver and the QMC5883L magnetometer. GNSS data uses
the GP6/GP7 PIO UART pair because the two hardware UARTs are reserved for companion traffic and CRSF.
The QMC5883L uses the shared slow I2C bus.

| Pico 2 W | HGLRC M100 Pro label | Function |
| --- | --- | --- |
| Regulated 5 V | VCC | GPS module power; vendor input range is 3.6-5.5 V |
| GND | GND | Common ground |
| GP6 | TX | GPS serial data to Pico |
| GP7 | RX | Optional GPS configuration from Pico |
| GP16 | PPS / timepulse | Optional timepulse interrupt |
| GP20 | SDA | QMC5883L I2C data |
| GP21 | SCL | QMC5883L I2C clock |
| GP22 | DRDY | Optional magnetometer data-ready input if exposed |

Treat GP16 and GP22 as GPIO interrupt inputs in firmware when the corresponding pads are wired.
If the GPS board does not expose PPS or magnetometer DRDY, leave those pins disconnected.
The GPS UART is expected to be 3.3 V logic. The QMC5883L I2C lines are open-drain and should be
pulled up to Pico `3V3`, not 5 V. With those conditions met, no level shifter is needed on GP6, GP7,
GP20, or GP21.

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

The BMP280 path is expected to be polled at low rate. Do not spend an interrupt-capable GPIO on it
unless the barometer module is changed to a part with a useful data-ready interrupt.

## Status LEDs

This layout preserves LED options without stealing pins from sensors:

| Pico 2 W | LED type | Notes |
| --- | --- | --- |
| GP18 | Discrete GPIO LED / scope realtime-pass toggle | Default flight/status LED; `scope-timing-pins` overrides this as a logic analyzer output |
| GP19 | Addressable LED / scope control strobe | Optional PIO-driven WS2812-style status LED; `scope-timing-pins` overrides this as a logic analyzer output |
| GP22 | Optional aux LED / scope diagnostic strobe | Optional spare status output; `scope-timing-pins` overrides this as the selected GP22 diagnostic output |

If GP22 is not needed for magnetometer data-ready, it can be used as a second discrete status LED.

## PIO and DMA Allocation

| Block | State machine | Purpose | GPIOs |
| --- | --- | --- | --- |
| PIO0 | SM0 | CYW43 Wi-Fi transport | internal Pico 2 W radio pins |
| PIO1 | SM0 | 4-lane DShot motor output | GP2-GP5 |
| PIO1 | SM1 | GPS serial input/output | GP6-GP7 |
| PIO2 | SM0 | addressable LED reserve | GP19 |

The DShot output should be a single PIO program that emits four motor lines in parallel from packed
DMA words. That keeps all motors frame-synchronous and consumes one state machine instead of four.
AM32 telemetry remains reserved on GP17; final implementation can use a normal GPIO/PIO input path
depending on whether the ESC telemetry wire or bidirectional DShot turnaround is used.

## Firmware Shape

Core 0 is the deterministic `World` side:

- runs the realtime scheduler,
- gives pending IMU samples priority over service work,
- drains the latest queued IMU packet and closes the estimator/controller/mixer/PWM loop,
- slices slower work into service phases,
- runs RC command/state handling from the service `SensorsRc` phase,
- runs telemetry enqueue, response drain, serial flush, and deferred board actions outside the hot
  IMU tick.

Core 1 owns hardware producers and transports that can jitter without directly blocking the control
pipeline:

- receives ISM330DHCX data-ready notifications,
- samples the ISM330DHCX over SPI through the Embassy-supported `ism330dhcx-rs` driver,
- pushes completed IMU packets into a board-local queue,
- receives CRSF frames on UART1 and pushes completed RC packets into a board-local queue,
- receives GPS serial data through the PIO UART and GPS PPS through a GPIO interrupt when wired,
- runs UART0 MAVLink transport to the ESP32C5 bridge,
- polls or services low-rate board-side sensor paths as they are enabled.

The IMU producer does cross from core 1 to core 0 through a small board-local queue, but the control
pipeline does not wait on Wi-Fi, UDP, UART TX, CRSF parsing, GPS parsing, or mailbox scheduling.
Core 0 closes the loop only from already-queued sensor and command state.

## Current Firmware Status

The current RP2350 firmware path is designed around the ISM330DHCX as the flight IMU. The board
code drains IMU samples from the ISM330DHCX queue and RC samples from the CRSF receiver queue. The
current board does not have a production barometer installed; the earlier GY-91/BMP280 pressure path
remains a low-rate service-side reference path until the dedicated barometer hardware is added. The
IMU path is interrupt-driven and the default firmware uses the native ISM330DHCX `3.333 kHz` ODR.
Use `ism330dhcx-1k666` only when deliberately testing the lower-rate timing-margin mode.

Core 0 closes the control loop only from the latest queued IMU packet plus already-processed command
state. RC interpretation, barometer, magnetometer, GPS, telemetry, and parameter work run in bounded
service phases so those lower-rate paths do not add synchronous work to every IMU control tick.

Treat wiring changes as hardware changes that need fresh probe validation. Use
`docs/tutorials/pico2w-sensor-bringup.md` to validate individual buses before debugging the full
firmware image.
