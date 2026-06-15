# Veloxity Documentation

This directory is the source of truth for the current branch. Older experiment notes and untested
flows should not be reintroduced unless they are retested and updated against the current code.

## Reader Paths

### I Want To Understand The Repository

1. [Repository map](repository-map.md)
2. [Build and tool commands](build-and-tools.md)
3. [Core architecture](architecture-guide.md)

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
fixed 1.5 kHz control loop and bounded service phases. The latest 120-second loaded Saleae and
MAVLink confirmation kept every measured control-deadline-to-complete latency inside the 1.5 kHz
budget while maintaining IMU, RC, attitude, output, GNSS, status, and heartbeat telemetry rates.
Measured hardware status, Saleae scope-pin meanings, and exact build/test commands live in the
RP2350 guide and hardware bring-up notes.

### I Want To Modify Firmware Logic

1. [Core architecture](architecture-guide.md)
2. `crates/veloxity_core/src/world.rs`
3. `crates/veloxity_core/src/control.rs`
4. `crates/veloxity_core/src/estimator/quad.rs`
5. `crates/veloxity_core/src/controller/quad.rs`
6. `crates/veloxity_core/src/mixer/matrix.rs`
7. `comms/veloxity_mavlink/src/link.rs`

## Maintained Tutorials

- [Veloxity ROScopter sim end-to-end](tutorials/veloxity-roscopter-sim-end-to-end.md)
- [Pico 2 W sensor bring-up](tutorials/pico2w-sensor-bringup.md)

ROSplane tutorials were removed from this branch's active documentation because ROSplane has not
been retested against the current Veloxity simulator path.
