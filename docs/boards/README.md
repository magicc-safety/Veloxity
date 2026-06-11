# Board Bring-Up Guide

Voloxide has one actively exercised hardware path on this branch and two retained STM32 paths that
are being brought back to current core APIs.

| Board | Crate | Target | Status |
| --- | --- | --- | --- |
| Raspberry Pi Pico 2 W / RP2350 | `boards/pico2w` | `thumbv8m.main-none-eabihf` | Active hardware bring-up path; native 3.333 kHz ISM330DHCX timing validated under loaded telemetry, with rare strict-300-us misses. |
| Nucleo-H753ZI | `boards/nucleo` | `thumbv7em-none-eabihf` | Retained and compile-current target; sensor validation still needed. |
| Pixracer Pro / STM32H7 | `boards/pixracerpro` | `thumbv7em-none-eabihf` | Retained and compile-current target; sensor validation still needed. |

## Shared Firmware Shape

Every board crate performs the same high-level steps:

1. Initialize board peripherals.
2. Load persisted parameters or write defaults.
3. Construct the board-specific `World` type.
4. Call `world.run_once()` forever.

The board crate chooses the concrete types for:

- `BoardIo`
- estimator
- controller
- mixer
- communication interface
- PWM driver
- floating-point type where the board uses the explicit generic form

The core loop lives in `crates/voloxide_core/src/world.rs`; board crates should not duplicate that
logic.

The Pico 2 W firmware uses the finer-grained realtime scheduler rather than a plain `run_once()`
loop. Its hot path is `World::run_imu_control_tick()`, which drains only the IMU queue before
running estimator/controller/mixer/PWM work. Slower work is sliced through service phases.

## Board Guides

- [RP2350 / Pico 2 W](rp2350-pico2w.md)
- [STM32 boards: Nucleo and Pixracer Pro](stm32.md)

## Common Commands

```bash
rustup target add thumbv8m.main-none-eabihf
rustup target add thumbv7em-none-eabihf

cargo xtask check-board pico2w
cargo xtask check-board nucleo
cargo xtask check-board pixracerpro
```

Build firmware:

```bash
cargo xtask build-board pico2w
cargo xtask build-board nucleo
cargo xtask build-board pixracerpro
```

Flash through the configured Cargo runner:

```bash
cargo xtask flash-board pico2w
cargo xtask flash-board nucleo
cargo xtask flash-board pixracerpro
```

For board-specific flashing notes, use the individual board guides.
