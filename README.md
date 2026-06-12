<img src="assets/voloxide-logo.svg" alt="Voloxide logo" width="180" height="180">

# Voloxide

Voloxide is a Rust firmware implementation for the ROSflight ecosystem. The current branch focuses
on making the Rust firmware interchangeable with the upstream ROSflight C firmware in the ROSflight
multirotor simulator, while keeping embedded firmware paths for RP2350/Pico 2 W, Nucleo-H753ZI, and
Pixracer Pro/STM32H7.

The repository is intentionally one Rust workspace. The core flight code, MAVLink adapter,
simulator FFI library, board crates, and platform crates share contracts and must stay in sync.

## Start Here

Read these in order if you are new to the repository:

1. [Documentation index](docs/README.md)
2. [Repository map](docs/repository-map.md)
3. [Build and tool commands](docs/build-and-tools.md)
4. [Core architecture](docs/architecture-guide.md)
5. [ROSflight simulator setup](docs/tutorials/voloxide-roscopter-sim-end-to-end.md)
6. [Board bring-up guide](docs/boards/README.md)

## Current Support Status

| Area | Status |
| --- | --- |
| ROSflight multirotor simulator with Rust firmware | Actively tested in this branch |
| ROScopter waypoint/autonomy stack on top of Voloxide SIL | Actively tested in this branch |
| RP2350/Pico 2 W hardware path | Active hardware bring-up path; high-rate ISM330DHCX intake with a stable 1.5 kHz fixed control loop validated under loaded telemetry |
| ESP32C5 ESP-NOW UART bridge | Tested as an isolated UART-over-air link |
| Nucleo-H753ZI | Retained and compile-current; needs renewed sensor bring-up |
| Pixracer Pro / STM32H7 | Retained and compile-current; needs renewed sensor bring-up |
| ROSplane simulator/autonomy | Not documented as supported on this branch because it has not been retested |

## Quick Build

Run from the repository root:

```bash
cargo xtask check-host
cargo xtask test-host
cargo xtask check-board pico2w
cargo xtask check-board nucleo
cargo xtask check-board pixracerpro
```

The current RP2350/Pico 2 W hardware path uses the ISM330DHCX data-ready interrupt for high-rate
IMU intake and runs the full estimator/controller/mixer/PWM pipeline at a fixed 1.5 kHz. The hot
path is IMU-only and reuses the latest command/sensor state; RC, barometer, magnetometer,
telemetry, parameters, and transport work run in bounded service phases. The default `pico2w`
release build selects this baseline. See
[RP2350 / Pico 2 W](docs/boards/rp2350-pico2w.md) for the measured timing and flash commands.

For ROSflight simulation, source ROS 2 and the ROSflight workspace first, then build the shim:

```bash
source scripts/build_and_source_ros2_shim.zsh
ros2 launch voloxide_sil_board_shim multirotor_standalone_sil.launch.py use_rviz:=true
```

The script builds:

- `target/debug/libsim.a`
- `workspace/install/voloxide_sil_board_shim`

Generated local artifacts can be removed with:

```bash
cargo xtask clean-generated
```

## Repository Rules

- Do not modify `rosflight_io` to make Voloxide work. Voloxide adapts to the existing ROSflight
  ROS 2 stack.
- Voloxide scripts assume ROS 2 and the ROSflight workspace are already sourced by the caller.
- Keep generated artifacts out of Git: `target/`, `workspace/`, ESP-IDF build directories, and
  runtime parameter stores are disposable.
- Prefer adding documentation under `docs/` and linking it from [docs/README.md](docs/README.md)
  instead of growing this README without structure.

## License

This project is licensed under the [NO IDEA](LICENSE).

## Upstream Context

Voloxide is built to interoperate with [ROSflight](https://github.com/rosflight/rosflight). See the
[ROSflight docs](https://docs.rosflight.org/latest/) for system-level ROSflight concepts and
message/service behavior.
