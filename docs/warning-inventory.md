# Warning Inventory

This inventory records the current Rust warning surface before warning-free cleanup. Commands were run from the Voloxide workspace unless noted.

## Summary

Initial warning inventory:

| Package / target | Warning count | Notes |
| --- | ---: | --- |
| `voloxide_core --lib` | 75 | Shared firmware core warnings. |
| `voloxide_mavlink` | 5 | Small cleanup surface. |
| `sim --lib` | 12 | Mostly unused imports/dead helper types. |
| `stm_32 --target thumbv7em-none-eabihf` | 68 | Shared embedded HAL/sensor backend warnings. |
| `pixracerpro --target thumbv7em-none-eabihf` | 51 | Board-specific warnings plus inherited shared warnings. |
| `nucleo --target thumbv7em-none-eabihf` | 47 | Board-specific warnings plus inherited shared warnings. |

Some package checks repeat dependency warnings. The counts above are the package-specific warning blocks reported by Cargo.

Cleanup result after the first warning-free pass:

| Package / target | Current package warnings |
| --- | ---: |
| `voloxide_core --lib` | 0 |
| `voloxide_mavlink` | 0 |
| `sim --lib` | 0 |
| `stm_32 --target thumbv7em-none-eabihf` | 0 |
| `pixracerpro --target thumbv7em-none-eabihf` | 0 |
| `nucleo --target thumbv7em-none-eabihf` | 0 |

Cargo still reports a future-incompatibility notice for the external dependency `num-bigint-dig v0.8.5` when checking `sim`. That is dependency metadata, not a warning emitted by Voloxide code.

## Commands Used

```sh
cargo check -p voloxide_core --lib --message-format=short
cargo check -p sim --lib --message-format=short
cargo check -p stm_32 --target thumbv7em-none-eabihf --message-format=short
cargo check -p pixracerpro --target thumbv7em-none-eabihf --message-format=short
cargo check -p nucleo --target thumbv7em-none-eabihf --message-format=short
```

## Cleanup Policy

- Prefer deleting unused imports, removing unnecessary `mut`, and replacing unused catch-all bindings with `_`.
- Use `let _ = ...` only when intentionally discarding a return value.
- Do not silence warnings broadly at crate level.
- Do not delete fields/constants that represent ROSflight parity intent without checking usage and tests.
- Do not make board-driver behavioral changes just to silence a warning; ignored hardware I/O results need explicit review.
- Preserve ECS ownership: warning cleanup must not bypass resources, events, systems, or state-machine transitions.

## `voloxide_core`

Warnings: 75.

Current status: resolved.

Low-risk cleanup:

- `crates/voloxide_core/src/comm/messages.rs`
  - Unused imports: `ErrorFlag`, `PhantomData`, `enums::*`.
  - Macro-generated `Option::insert` return values are unused; use `let _ = ...`.
- `crates/voloxide_core/src/command.rs`
  - Unused imports at the top of the file.
  - Many unused catch-all bindings named `other` / `other_type`.
  - Initial `roll_pitch_type` assignment is overwritten before read.
  - `ATTITUDE_ANGLE_MODE` is currently unused; verify whether it is future parity intent before deleting.
- `crates/voloxide_core/src/controller/quad.rs`
  - Unused import `StateManager`.
  - Many unused catch-all bindings named `other`.
- `crates/voloxide_core/src/sensors/processors.rs`
  - Unused import `core::default`.
  - `GYRO_MAX_CALIBRATION_DELTA` is unused.
  - Fields `max_gyro` and `min_gyro` are never read.
- `crates/voloxide_core/src/state_machine/tests.rs`
  - Unused imports, macro, and helper functions.
- `crates/voloxide_core/src/vehicle/quadrotor.rs`
  - Unused import `packets::*`.
- `crates/voloxide_core/src/world.rs`
  - Unused imports.
  - Unnecessary mutable `params`.
- `crates/voloxide_core/src/board/dummy.rs`
  - Unused parameters `buf` and `high`; rename to `_buf`, `_high` if intentionally unused.
- `crates/voloxide_core/src/comm.rs`
  - Unnecessary `mut`.
- `crates/voloxide_core/src/rc/system.rs`
  - Unnecessary `mut`.
- `crates/voloxide_core/src/rc.rs`
  - Many unused catch-all bindings and `channel_name`.

Needs review before changing:

- Calibration constants/fields in sensor processors may represent incomplete C parity checks.
- Command-mode constants may reflect MAVLink/offboard parity even when not currently used.

## `voloxide_mavlink`

Warnings: 5.

Current status: resolved.

Low-risk cleanup:

- `comms/voloxide_mavlink/src/conversions.rs`
  - Unused doc comment.
- `comms/voloxide_mavlink/src/link.rs`
  - Unused import `core::result::Result`.
  - Unused imports `DialectVersion`, `FrameBuilder`.
  - Unnecessary parentheses around a match scrutinee.
- `comms/voloxide_mavlink/src/parser.rs`
  - Unused imports `Frame`, `Sender`.

## `sim`

Warnings: 12.

Current status: resolved.

Remaining cleanup:

- `sim/firmware/src/ffi.rs`
  - `TriggerRequest` and `TriggerResponse` are never constructed; verify whether service work still needs these types.

## `stm_32`

Warnings: 68.

Current status: resolved.

Low-risk cleanup:

- Unused imports:
  - `platforms/stm_32/src/peripherals/bmi08x.rs`: `core::module_path`.
  - `platforms/stm_32/src/peripherals/dps310.rs`: `core::module_path`.
  - `platforms/stm_32/src/peripherals/iis2mdc.rs`: `core::module_path`.
  - `platforms/stm_32/src/peripherals/llv3hp.rs`: `I2cDeviceError`.
  - `platforms/stm_32/src/peripherals/pps.rs`: `Timer`.
  - `platforms/stm_32/src/telem.rs`: `Channel`, `voloxide_core::packets`.
- Style:
  - `platforms/stm_32/src/peripherals/ublox.rs`: unnecessary parentheses around a pattern and an `if` condition.
- Unnecessary `mut`:
  - `dps310.rs`, `iis2mdc.rs`, `sbus.rs`, `sd_card.rs`, `vcp.rs`.
- Unused local variables where the value is intentionally ignored:
  - Several driver status/result variables can become `_status`, `_result`, or `let _ = ...` after reviewing intent.

Needs review before changing:

- Ignored hardware I/O results in `adis16500.rs`, `bmi08x.rs`, and `dlhrl20g.rs`.
- Driver status reads that may have been intended as diagnostics.
- `llv3hp.rs` timestamp variables; confirm whether range timing is supposed to be reported.

## `pixracerpro`

Warnings: 51.

Current status: resolved.

Low-risk cleanup:

- `boards/pixracerpro/src/board.rs`
  - Unused import `Timer`.
  - Unreachable catch-all pattern in a board read path.
  - Unnecessary `mut` bindings.
  - Unused local variables such as `spi1_bus`, `uart6_tx`.
  - Ignored spawner result needs explicit handling or `let _ =` after review.
- `boards/pixracerpro/src/pwm.rs`
  - Unused import `self`.
  - Inner `#![no_std]` attribute warning; this attribute belongs only at crate root.
- `boards/pixracerpro/src/stm32h7x3_common.rs`
  - Unused imports.
  - Unnecessary parentheses.
  - Dead shared statics for unused buses.

Needs review before changing:

- Board `probe` field and methods may be intentional diagnostic scaffolding. Either wire them into diagnostics or add a narrow `#[allow(dead_code)]` with a reason.

## `nucleo`

Warnings: 47.

Current status: resolved.

Low-risk cleanup:

- `boards/nucleo/src/board.rs`
  - Unreachable catch-all pattern.
  - Unnecessary `mut` bindings.
  - Unused local variables such as `spi2_bus`, `dlhr_sensor`, `uart1_tx`.
  - Ignored spawner results need explicit handling or `let _ =` after review.
- `boards/nucleo/src/stm32h7x3_common.rs`
  - Unused imports.
  - Unnecessary parentheses.
  - Dead shared statics for unused buses.

Needs review before changing:

- Board `probe` field and methods may be intentional diagnostic scaffolding. Either wire them into diagnostics or add a narrow `#[allow(dead_code)]` with a reason.

## Recommended Cleanup Order

1. `voloxide_mavlink`, `sim`, and low-risk `voloxide_core` warnings.
2. Re-run host tests: `cargo test -p voloxide_core --lib`, `cargo test -p sim --lib`.
3. Shared embedded backend syntax cleanup in `stm_32`.
4. Board-specific cleanup in `pixracerpro` and `nucleo`.
5. Review ignored hardware I/O results one by one.
6. Re-run embedded checks for `pixracerpro` and `nucleo`.
