# Veloxity Documentation

## Reader Paths

### I Want To Understand The Repository

1. [Repository map](repository-map.md)
2. [Build and tool commands](build-and-tools.md)
3. [Feature flags](features.md)
4. [Core architecture](architecture-guide.md)

### I Want To Run The Simulator

1. [Build and tool commands](build-and-tools.md)
2. [Veloxity ROScopter sim end-to-end](tutorials/veloxity-roscopter-sim-end-to-end.md)

### I Want To Work On Hardware

1. [Board bring-up guide](boards/README.md)
2. [RP2350 / Pico 2 W guide](boards/rp2350-pico2w.md)
3. [Pico 2 W wiring](pico2w-esc-imu-pinout.md)
4. [ESP32C5 ESP-NOW UART bridge](../tools/espnow_uart_bridge/README.md)
5. [STM32 board guide](boards/stm32.md)

The current RP2350/Pico 2 W firmware target uses high-rate ISM330DHCX data-ready intake with a
fixed 1.5 kHz control loop and bounded service phases. The June 12 loaded Saleae and MAVLink run
recorded the timing table used by the current docs, while newer IMU delay observations are tracked
against the Pico board-local SPI register path. Hardware status, Saleae scope-pin meanings, and
exact build/test commands live in the RP2350 guide and hardware bring-up notes.

The current Pixracer Pro / STM32H7 firmware target uses a fixed 400 Hz control loop with
board-specific post-control telemetry scheduling. The latest 120-second hardware validation held
the configured high-rate MAVLink streams with zero CRC errors and zero sequence gaps while keeping
control timing comfortably inside the 2.5 ms period. Pixracer Pro setup, scope timing, and
validation notes live in the STM32 board guide.

### I Want To Modify Firmware Logic

1. [Core architecture](architecture-guide.md)
2. [Feature flags](features.md)
3. `crates/veloxity_core/src/world.rs`
4. `crates/veloxity_core/src/control.rs`
5. `crates/veloxity_core/src/estimator/quad.rs`
6. `crates/veloxity_core/src/controller/quad.rs`
7. `crates/veloxity_core/src/mixer/matrix.rs`
8. `comms/veloxity_mavlink/src/link.rs`

## Maintained Tutorials

- [Veloxity ROScopter sim end-to-end](tutorials/veloxity-roscopter-sim-end-to-end.md)
- [Pico 2 W sensor bring-up](tutorials/pico2w-sensor-bringup.md)

ROSplane tutorials were removed from this branch's active documentation because ROSplane has not
been retested against the current Veloxity simulator path.
