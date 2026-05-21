# RP2350 / Pico 2 W Platform Plan

## Reference Stack

- `embassy-rp` is the RP2040/RP235x HAL. It exposes RP2350 peripherals, PIO, multicore support,
  and the `rp235xa` feature for the Pico 2 W class of boards.
- `cyw43` supports the CYW43439 radio used by Raspberry Pi Pico W and Pico 2 W.
- `cyw43-pio` is the Embassy PIO transport for the nonstandard half-duplex SPI path to the Wi-Fi
  radio. Its `RM2_CLOCK_DIVIDER` is intended for the Pico 2 W RM2 radio module clocking.
- `embassy-net` is the natural UDP/IP layer once the CYW43 driver is wired in.

## Initial Voloxide Shape

- `platforms/rp2350` owns reusable RP2350 concepts:
  - core-role assignment,
  - multicore mailbox sizing,
  - PIO state-machine allocation metadata,
  - Wi-Fi UDP MAVLink configuration metadata.
- `boards/pico2w` owns the Pico 2 W implementation:
  - Voloxide `World` entry point,
  - `BoardIo` implementation backed by a MAVLink mailbox,
  - PIO PWM driver skeleton,
  - default PIO state-machine allocation placeholders.

## Intended Core Split

- Core 0: flight-control loop running `voloxide_core::world::World`.
- Core 1: Wi-Fi/CYW43/UDP MAVLink transport.
- Cross-core boundary: byte-oriented MAVLink mailbox. From `voloxide_core`'s perspective, the board
  still looks like serial MAVLink I/O.

This keeps UDP and Wi-Fi latency away from the stabilization loop. The core-1 task can absorb Wi-Fi
timing jitter while core 0 only consumes complete byte streams from the mailbox.

## PIO Direction

PIO is the right place for deterministic signal timing on this board:

- DShot motor output,
- SBUS or other receiver input,
- timing-sensitive sensor buses if normal I2C/SPI timing is not sufficient,
- CYW43 PIO SPI for Wi-Fi.

The current `DEFAULT_PIO_ALLOCATIONS` reserves PIO0 SM0 for CYW43, PIO1 SM0 for a four-lane DShot
bus, PIO1 SM1 for later ESC telemetry, and PIO2 SM0 for an addressable status LED. The proposed
pinout is documented in `docs/pico2w-esc-imu-pinout.md`.

## Open Decisions

- The exact IMU part number and matching Rust driver.
- Whether AM32 telemetry will use a separate telemetry wire or bidirectional DShot line turnaround.
- Whether future low-rate sensors use the reserved I2C pins or their own bus.
- Mailbox sizing and backpressure behavior between the Wi-Fi core and flight-control core.
- How Wi-Fi credentials and UDP peer settings are supplied without baking private network data into
  firmware.
