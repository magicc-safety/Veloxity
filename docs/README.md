# Voloxide Documentation

This directory is the source of truth for the current branch. Older experiment notes and untested
flows should not be reintroduced unless they are retested and updated against the current code.

## Reader Paths

### I Want To Understand The Repository

1. [Repository map](repository-map.md)
2. [Build and tool commands](build-and-tools.md)
3. [Core architecture](architecture-guide.md)

### I Want To Run The Simulator

1. [Build and tool commands](build-and-tools.md)
2. [Voloxide ROScopter sim end-to-end](tutorials/voloxide-roscopter-sim-end-to-end.md)

### I Want To Work On Hardware

1. [Board bring-up guide](boards/README.md)
2. [RP2350 / Pico 2 W guide](boards/rp2350-pico2w.md)
3. [Pico 2 W wiring](pico2w-esc-imu-pinout.md)
4. [ESP32C5 ESP-NOW UART bridge](../tools/espnow_uart_bridge/README.md)
5. [STM32 board guide](boards/stm32.md)

### I Want To Modify Firmware Logic

1. [Core architecture](architecture-guide.md)
2. `crates/voloxide_core/src/world.rs`
3. `crates/voloxide_core/src/control.rs`
4. `crates/voloxide_core/src/estimator/quad.rs`
5. `crates/voloxide_core/src/controller/quad.rs`
6. `crates/voloxide_core/src/mixer/matrix.rs`
7. `comms/voloxide_mavlink/src/link.rs`

## Maintained Tutorials

- [Voloxide ROScopter sim end-to-end](tutorials/voloxide-roscopter-sim-end-to-end.md)
- [Pico 2 W sensor bring-up](tutorials/pico2w-sensor-bringup.md)

ROSplane tutorials were removed from this branch's active documentation because ROSplane has not
been retested against the current Voloxide simulator path.
