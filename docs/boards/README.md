# Board Bring-Up Guide

Veloxity has two actively exercised hardware paths on this branch and one retained STM32 path that
is kept compile-current.

| Board | Crate | Target | Status |
| --- | --- | --- | --- |
| Raspberry Pi Pico 2 W / RP2350 | `boards/pico2w` | `thumbv8m.main-none-eabihf` | Active hardware bring-up path; high-rate ISM330DHCX intake, fixed 1.5 kHz timing measurements, and ongoing IMU delay investigation. |
| Nucleo-H753ZI | `boards/nucleo` | `thumbv7em-none-eabihf` | Retained and compile-current target; sensor validation still needed. |
| Pixracer Pro / STM32H7 | `boards/pixracerpro` | `thumbv7em-none-eabihf` | Active STM32 validation path; fixed 400 Hz control timing and high-rate MAVLink telemetry validated on hardware. |

## Shared Firmware Shape

Every board crate performs the same high-level setup:

1. Initialize board peripherals.
2. Load persisted parameters or write defaults.
3. Construct the board-specific `World` type.
4. Enter the board's scheduler loop.

The board crate chooses the concrete types for:

- `BoardIo`
- estimator
- controller
- mixer
- communication interface
- PWM driver
- floating-point type where the board uses the explicit generic form

The generic core loop lives in `crates/veloxity_core/src/world.rs`; board crates should not
duplicate flight logic. Pico 2 W and Pixracer Pro both use the finer-grained realtime scheduler:
`realtime_scheduler_step()` chooses between `run_imu_control_tick()` for fresh IMU samples and
bounded service phases for slower work. Nucleo remains on the ordinary `world.run_once()` shape
while it stays compile-current.

When adding a new realtime board, start with the service-phase scheduler defaults. Add a
post-control telemetry burst only after hardware measurements show unused post-control slack and
insufficient telemetry selection opportunities. The helper is
`World::run_realtime_telemetry_stage_budgeted(max_streams)`; the board entrypoint should keep the
budget as a board-local constant and document the Saleae/MAVLink evidence that supports it.
If a board needs a specific control-rate stream to be tried first, use
`World::run_realtime_telemetry_stage_prioritized(priority_streams, max_streams)` with a board-owned
priority list rather than adding stream-specific control flow to the core. Each priority entry names
a `NamedTelemetryStream` and a `RealtimeTelemetryPriorityGate`; use `DueDeadline` for normal
rate-gated priority and `FreshSample` only for streams whose cadence is already controlled by the
board's realtime loop.

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
