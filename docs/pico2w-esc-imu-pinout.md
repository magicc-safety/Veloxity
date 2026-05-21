# Pico 2 W ESC and IMU Pinout Draft

This is the first concrete wiring plan for the RP2350 / Pico 2 W board target. It keeps the first
hardware pass focused on four DShot motors, LEDs, Wi-Fi MAVLink, and one 9-DOF IMU.

## Electrical Assumptions

- ESC: SEQURE Blueson A2 4-in-1 AM32 ESC.
- Motor protocol: DShot600 first, with DShot300 and DShot150 available by changing PIO timing.
- ESC signal voltage: Pico 2 W drives 3.3 V GPIO. Confirm the ESC input accepts 3.3 V logic before
  flight hardware testing.
- Power: the ESC has no BEC. Do not power the Pico 2 W from the ESC signal harness unless an
  external regulator is added. The Pico 2 W and ESC must share ground.
- IMU candidate: GY-91 MPU9250/BMP280 10DOF module. Exact module spec, supported voltage, exposed
  pins, and preferred bus mode still need to be confirmed before adding driver dependencies.

## Proposed Header GPIO Allocation

| Function | Pico 2 W GPIO | Notes |
| --- | ---: | --- |
| ESC motor 1 signal | GP2 | DShot PIO output bit 0 |
| ESC motor 2 signal | GP3 | DShot PIO output bit 1 |
| ESC motor 3 signal | GP4 | DShot PIO output bit 2 |
| ESC motor 4 signal | GP5 | DShot PIO output bit 3 |
| ESC telemetry reserve | GP6 | Reserved for AM32 telemetry or bidirectional DShot later |
| IMU SPI1 SCK | GP10 | Hardware SPI, DMA-backed |
| IMU SPI1 MOSI | GP11 | Hardware SPI, DMA-backed |
| IMU SPI1 MISO | GP12 | Hardware SPI, DMA-backed |
| IMU chip select | GP13 | Normal GPIO output |
| IMU data-ready interrupt | GP14 | GPIO interrupt into the flight core |
| Flight status LED | GP16 | Discrete LED |
| Comms status LED | GP17 | Discrete LED |
| Fault status LED | GP18 | Discrete LED |
| Addressable LED reserve | GP19 | PIO-driven WS2812-style status strip if needed |
| Future I2C SDA | GP20 | Reserved for later sensors |
| Future I2C SCL | GP21 | Reserved for later sensors |
| Future aux / capture | GP22 | Reserved |
| Future ADC0 | GP26 | Battery voltage/current path later |
| Future ADC1 | GP27 | Battery voltage/current path later |
| Future ADC2 | GP28 | Battery voltage/current path later |

Keep GP0/GP1 open for a debug UART during bring-up. Keep the Pico 2 W wireless pins owned by the
CYW43 driver.

## PIO and DMA Allocation

| Block | State machine | Purpose | GPIOs |
| --- | --- | --- | --- |
| PIO0 | SM0 | CYW43 Wi-Fi transport | internal Pico 2 W radio pins |
| PIO1 | SM0 | 4-lane DShot motor output | GP2-GP5 |
| PIO1 | SM1 | ESC telemetry reserve | GP6 |
| PIO2 | SM0 | addressable LED reserve | GP19 |

The DShot output should be a single PIO program that emits four motor lines in parallel from packed
DMA words. That keeps all motors frame-synchronous and consumes one state machine instead of four.

The IMU should start on hardware SPI1 with DMA. PIO SPI is only worth adding if the selected IMU
driver or board routing forces timing that the hardware SPI peripheral cannot satisfy.

## Firmware Shape

Core 0 remains the deterministic flight side:

- owns IMU SPI sampling and data-ready interrupt handling,
- owns PIO DShot output frame preparation,
- runs the Voloxide control loop,
- publishes motor command frames into the PIO/DMA output queue.

Core 1 remains the communications side:

- owns CYW43 Wi-Fi,
- owns UDP MAVLink,
- passes MAVLink bytes to core 0 through the mailbox.

The IMU path should not cross the core boundary. Sensor samples should enter `voloxide_core` on
core 0 so the control loop is not gated by Wi-Fi, UDP, or mailbox scheduling.

## DShot Notes

DShot600 has a bit time of about 1.67 us and a 16-bit frame time of about 26.7 us before inter-frame
spacing. A 1 kHz control loop is well within that budget. The PIO program should be parameterized so
DShot300 and DShot150 are clock-divider changes rather than separate implementations.

Bidirectional DShot is a later feature. It requires line turnaround and input capture after the
transmit frame. Keep GP6 reserved until we decide whether AM32 telemetry will use a separate TLM wire
or true bidirectional DShot on each motor signal.

## Next Step: Confirm GY-91 Details

The next hardware-specific step is to confirm the exact GY-91 module details and choose the driver
stack for MPU9250 plus BMP280. Capture this before implementation:

- product page or datasheet link for the exact GY-91 board,
- silkscreen pin labels,
- whether the board exposes SPI pins for the MPU9250 and BMP280 or only I2C,
- voltage requirements for `VCC` and logic pins,
- interrupt/data-ready pin availability,
- whether the BMP280 should be ignored for the first bring-up or wired on the same bus now.

If SPI is available, keep the proposed SPI1 wiring. If the module is I2C-only, move the IMU path to
the reserved GP20/GP21 I2C pins and leave GP10-GP14 open for a future high-rate SPI IMU.
