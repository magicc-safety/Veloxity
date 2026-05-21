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

- motor/servo output,
- SBUS or other receiver input,
- timing-sensitive sensor buses if normal I2C/SPI timing is not sufficient,
- CYW43 PIO SPI for Wi-Fi.

The current `DEFAULT_PIO_ALLOCATIONS` is deliberately provisional. It exists to make the ownership
model explicit before pin and state-machine assignments are finalized.

## Open Decisions

- Which GPIOs are assigned to motors, servos, receiver input, and each sensor bus.
- Which PIO block/state machine is reserved for CYW43 versus flight-control I/O.
- Whether sensor I/O is all PIO-backed immediately or whether low-rate sensors start on hardware
  I2C/SPI and move to PIO only where timing demands it.
- Mailbox sizing and backpressure behavior between the Wi-Fi core and flight-control core.
- How Wi-Fi credentials and UDP peer settings are supplied without baking private network data into
  firmware.
