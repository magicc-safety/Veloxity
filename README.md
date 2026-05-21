<img src="assets/voloxide-logo.svg" alt="Voloxide logo" width="250" height="250" align="center">

## Voloxide

Voloxide is a Rust-based port of the **ROSFlight** project. The repository is organized as a
single Rust workspace because the core firmware contracts, protocol adapters, board firmware, and
simulator endpoint evolve together.

## Repository Structure

| Path | Role |
|---|---|
| `crates/voloxide_core` | `no_std` flight-control library: contracts, scheduler, state, estimator, controller, mixer, sensors, PWM, RC, and params |
| `comms/voloxide_mavlink` | MAVLink communication implementation for the core `CommInterface` contract |
| `platforms/stm_32` | STM32/Embassy platform and peripheral drivers shared by STM32 boards |
| `boards/nucleo` | Nucleo-H753ZI firmware application package |
| `boards/pixracerpro` | Pixracer Pro firmware application package |
| `sim/firmware` | Host-side simulator firmware FFI/staticlib used by the ROS 2 shim |
| `sim/ros2/voloxide_sil_board_shim` | ROS 2 shim that exposes the Voloxide simulator firmware as a ROSflight SIL board |
| `docs/tutorials` | Supported ROScopter and ROSplane operator tutorials |
| `scripts` | Supported ROScopter and ROSplane demo helpers |
| `xtask` | Repo-specific build/test command wrapper |

MAVLink message types are code-generated at build time by `comms/voloxide_mavlink/build.rs` from
`comms/voloxide_mavlink/mavlink_definitions/` using the `mavspec` crate. They are internal to the
`voloxide_mavlink` communication implementation.

## Build And Test

Run Rust commands from the Voloxide repo root:

```bash
cd ~/Voloxide
```

Root `cargo build` uses the workspace `default-members`, which are intentionally host-compatible.
It builds the core library, MAVLink implementation, simulator FFI library, and `xtask`; it does not
try to build embedded firmware for the host target.

```bash
cargo build
```

Use the repo command wrapper for the common checks:

```bash
cargo xtask check-host
cargo xtask test-host
```

Build the simulator static library used by the ROS 2 shim:

```bash
cargo xtask build-sim-lib
```

Equivalent direct command:

```bash
cargo build -p sim --lib
```

## Prerequisites

- Rust toolchain.
- For embedded board firmware, add the Cortex-M target:
  ```bash
  rustup target add thumbv7em-none-eabihf
  ```
- For flashing and running on hardware, install `probe-rs`:
  ```bash
  cargo install probe-rs-tools
  ```
- For ROSflight SIL demos, source ROS 2 and the ROSflight workspace in your shell before building or
  running Voloxide. The Voloxide scripts use the environment you already sourced; they do not source
  ROSflight helper scripts from outside this repository.
  ```bash
  # Example only; use your local ROSflight setup.
  source ~/rosflight/workspace/install/setup.zsh
  ```
- For the recommended ROS 2 middleware:
  ```bash
  sudo apt-get install -y ros-jazzy-rmw-zenoh-cpp
  ```

## ROSflight SIL

Build the Rust simulator firmware static library and ROS 2 shim:

```bash
cd ~/Voloxide
source scripts/build_and_source_ros2_shim.zsh
```

Set `VOLOXIDE_SIM_PARAM_DIR` to an explicit runtime directory before running SIL demos manually.
The simulator firmware writes its saved parameter file there. Since `target/` is ignored by Git,
runtime parameter stores and other disposable SIL state stay out of the repository.

The current ROSflight 2.0 software-in-the-loop workflow uses the ROS 2 shim in
`sim/ros2/voloxide_sil_board_shim`. The shim calls the Rust simulator firmware through a C FFI
static library and exposes the same simulator firmware endpoint contract as the upstream C
`sil_board`.

Detailed operator guides:

- [Run ROScopter Waypoint Following With Voloxide](docs/tutorials/voloxide-roscopter-waypoints.md)
- [Run ROSplane Fixed-Wing Waypoint Following With Voloxide](docs/tutorials/voloxide-rosplane-waypoints.md)

## Embedded Firmware

Embedded board firmware is built explicitly because those packages target
`thumbv7em-none-eabihf`:

```bash
cargo xtask check-board pixracerpro
cargo xtask check-board nucleo
```

Equivalent direct commands:

```bash
cargo build -p pixracerpro --target thumbv7em-none-eabihf
cargo build -p nucleo --target thumbv7em-none-eabihf
```

### Nucleo-H753ZI

```bash
cargo run -p nucleo --target thumbv7em-none-eabihf --bin voloxide
```

### Pixracer Pro

```bash
cargo run -p pixracerpro --target thumbv7em-none-eabihf --bin voloxide
```

Both boards run a `voloxide` binary entry point. Each binary wires its board, `MavlinkInterface`,
quadrotor body components, state manager, and PWM driver into the shared `World` scheduler.

## MAVLink

The MAVLink parser (`voloxide_mavlink::parser`) is board-agnostic. It operates on raw `&[u8]`
bytes: `MavlinkParser::feed_byte` accumulates bytes and returns a frame once the start byte,
length, and CRC all match. `process_mavlink_frame` decodes the frame into a typed `Rosflight`
dialect message.

`voloxide_mavlink::MavlinkInterface` implements the
`voloxide_core::comm::interface::CommInterface<B: BoardIo>` trait and wires the parser into the
main loop via `comm`.

## Branching Strategy

Branch names follow the convention `[username]/[feature]`:

```text
johndoe/param_server
```

## License

This project is licensed under the [NO IDEA](LICENSE).

## Acknowledgments

This project is a port of [ROSFlight](https://github.com/rosflight/rosflight), originally written
in C++. See the [ROSFlight docs](https://docs.rosflight.org/latest/) for protocol and system
context.
