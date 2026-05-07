# RustFlight

**RustFlight** is a Rust-based port of the **ROSFlight** project, bringing real-time, high-performance flight control to the Rust ecosystem. This project leverages Rust's safety and concurrency features to create reliable, maintainable flight control firmware.

## Workspace Structure

| Crate | Target | Purpose |
|---|---|---|
| `rustflight_core` | host | `no_std` algorithm library: board I/O, world scheduler, estimator, controller, mixer, MAVLink |
| `stm_32` | `thumbv7em-none-eabihf` | STM32/Embassy HAL and peripheral drivers |
| `nucleo` | `thumbv7em-none-eabihf` | Binary with `BoardIo` and PWM drivers for the Nucleo-H753ZI dev board |
| `pixracerpro` | `thumbv7em-none-eabihf` | Binary with `BoardIo` and PWM drivers for the Pixracer Pro flight controller |
| `sim` | host | Binary with `BoardIo` and PWM drivers for host-side simulation via Zenoh |

MAVLink message types are code-generated at build time by `rustflight_core/build.rs` from `mavlink_definitions/` using the `mavspec` crate. They are accessible as `rustflight_core::mavlink::*`.

## Prerequisites

- Rust toolchain with the `thumbv7em-none-eabihf` target:
  ```bash
  rustup target add thumbv7em-none-eabihf
  ```
- `probe-rs` for flashing and running on hardware:
  ```bash
  cargo install probe-rs-tools
  ```
- A debug probe connected to the target board (e.g., ST-Link on the Nucleo).

## Building

```bash

# Build for the Pixracer Pro (embedded target)
cargo build -p pixracerpro --target thumbv7em-none-eabihf

# Build for the Nucleo-H753ZI (embedded target)
cargo build -p nucleo --target thumbv7em-none-eabihf

# Format code
cargo fmt
```

## Flashing and Running

The embedded runner is configured in `.cargo/config.toml`:

```toml
[target.thumbv7em-none-eabihf]
runner = "probe-rs run --chip STM32H743IIKx"
```

### Nucleo-H753ZI

```bash
cargo run -p nucleo --target thumbv7em-none-eabihf --bin rustflight
```

### Pixracer Pro

```bash
cargo run -p pixracerpro --target thumbv7em-none-eabihf --bin rustflight
```

Both boards run the same `rustflight` binary entry point. Each binary wires its board, MAVLink interface, quadrotor body components, state manager, and PWM driver into the shared `World` scheduler.

### Sim

```bash
cargo run -p sim --bin rustflight
```

The sim board uses [Zenoh](https://zenoh.io/) for inter-process communication. It subscribes to a `rust/tick` topic to drive the main loop and publishes actuator commands over Zenoh topics. (**WIP**)

## Debugging

To enable defmt logging, uncomment the `defmt` dependency in the relevant crate's `Cargo.toml` and the `link-arg=-Tdefmt.x` line in `.cargo/config.toml`. Then uncomment `defmt::` calls in the source. defmt output is read via `probe-rs`.

## MAVLink

The MAVLink parser (`rustflight_core::comm_manager::mavlink_parser`) is board-agnostic. It operates on raw `&[u8]` bytes: `MavlinkParser::feed_byte` accumulates bytes and returns a frame once the start byte, length, and CRC all match. `process_mavlink_frame` decodes the frame into a typed `Rosflight` dialect message.

`MavlinkInterface` (in `rustflight_core::comm_manager::comm_link_trait::mavlink`) implements the `CommInterface<B: BoardIo>` trait and wires the parser into the main loop via `comm_manager`.

## Branching Strategy

Branch names follow the convention `[username]/[feature]`:

```
johndoe/param_server
```

## License

This project is licensed under the [NO IDEA](LICENSE).

## Acknowledgments

This project is a port of [ROSFlight](https://github.com/rosflight/rosflight), originally written in C++. See the [ROSFlight docs](https://docs.rosflight.org/latest/) for protocol and system context.
