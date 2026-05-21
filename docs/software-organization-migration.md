# Voloxide Software Organization Migration Record

## Working Agreement

This migration reorganizes Voloxide so the source tree communicates the same architecture that the
type system already uses.

User requirements for this work:

- Treat this repository as Voloxide; do not rename anything back to Rustflight.
- Keep `voloxide_core` focused on protocol-neutral flight-stack contracts, systems, resources, and
  scheduling.
- Move MAVLink out of `voloxide_core` so it is selected at compile time like board
  implementations.
- Prefer the modern Rust module layout: `module.rs` plus `module/child.rs`, not `module/mod.rs`.
- Keep a migration note under `docs/` that records intent, steps, verification, and remaining work.
- Preserve current behavior while making ownership boundaries easier to understand.
- Do not modify external ROSflight workspace sources or generated/install/package files.

## Current Context

Voloxide already composes major implementation choices at compile time:

- Board implementations live outside core in crates such as `sim`, `pixracerpro`, and `nucleo`.
- `World<B, E, C, M, CI, PD>` accepts board, estimator, controller, mixer, communication interface,
  and PWM driver types as generic parameters.
- Platform support lives separately in `stm_32`.

The communication stack is the exception. `voloxide_core` defines the `CommInterface` trait, but it
also owns the MAVLink implementation, generated MAVLink dialect, parser, and MAVLink conversion
logic. This makes the source tree imply that MAVLink is intrinsic to core, even though the runtime
composition model treats communication as an injected implementation.

## Target Dependency Direction

The intended dependency direction is:

```text
voloxide_core
voloxide_mavlink -> voloxide_core
sim              -> voloxide_core + voloxide_mavlink
pixracerpro      -> voloxide_core + voloxide_mavlink
nucleo           -> voloxide_core + voloxide_mavlink
stm_32           -> voloxide_core
```

`voloxide_core` should not depend on `mavio`, `mavspec`, generated MAVLink code, or MAVLink XML
definitions.

## Target Module Style

Use the modern Rust file layout:

```text
src/comm.rs
src/comm/interface.rs
src/comm/manager.rs
src/comm/messages.rs
```

Avoid adding new `mod.rs` files. Existing `mod.rs` files can be migrated when the owning module is
being touched for a coherent reason.

## Initial Migration Slice

The first implementation slice moves MAVLink into a new `voloxide_mavlink` crate.

Planned changes:

- Add `voloxide_mavlink` as a workspace member.
- Move MAVLink XML definitions and build-time code generation into `voloxide_mavlink`.
- Move the MAVLink parser and `MavlinkInterface` implementation into `voloxide_mavlink`.
- Keep `CommInterface` and protocol-neutral communication messages in `voloxide_core`.
- Update `sim`, `pixracerpro`, and `nucleo` to instantiate `voloxide_mavlink::MavlinkInterface`.
- Remove `mavio`/`mavspec` dependencies from `voloxide_core`.

## Conversion Boundary

Moving MAVLink out of core changes where conversion impls should live. The previous code used
`impl From<mavlink_type> for core_type` and `impl From<core_type> for mavlink_type` inside
`voloxide_core`, where the generated MAVLink types were local.

With MAVLink in `voloxide_mavlink`, the generated MAVLink types are local to that crate instead.
That crate can own ordinary `From` impls between MAVLink wire types and protocol-neutral core
messages. `voloxide_core` remains protocol-neutral and does not know whether an end user selects
MAVLink, another serial protocol, or a custom communication implementation.

## Heapless Assessment

`heapless` is used for bounded queues where it can simplify no-allocation storage.

Good candidate:

- Replacing the hand-written `EventQueue<T, N>` internals with a small wrapper around
  `heapless::Deque<T, N>`.

Likely not solved by `heapless`:

- Verbose `World<B, E, C, M, CI, PD>` type syntax.
- Associated-type bounds for estimator/controller/mixer composition.
- Board-local lifetimes around PWM and peripherals.

Syntax cleanup for the generic composition should primarily use board-local type aliases and
constructors. `heapless` should be adopted only where it simplifies bounded storage without changing
timing or allocation behavior.

## Progress Log

### 2026-05-18: Started MAVLink Crate Split

- Fetched and fast-forwarded local `main` to `origin/main` at `8246d8d`.
- Created this migration note before continuing code changes.
- Started creating `voloxide_mavlink` as the compile-time MAVLink implementation crate.

### 2026-05-18: Completed Initial MAVLink Crate Split

- Added `voloxide_mavlink` as a workspace member.
- Moved MAVLink XML definitions, build-time generation, parser, link implementation, and conversion
  tests into `voloxide_mavlink`.
- Removed `mavio`, `mavspec`, generated MAVLink exports, and MAVLink implementation files from
  `voloxide_core`.
- Updated `sim`, `pixracerpro`, and `nucleo` to instantiate
  `voloxide_mavlink::MavlinkInterface`.
- Added `#![no_main]` to the embedded board binaries so target checks recognize the
  `cortex_m_rt::entry` entry point.
- Updated the README to describe the new crate boundary.

### 2026-05-18: Introduced `voloxide_core::comm`

- Moved the core communication manager from `crates/voloxide_core/src/comm_manager.rs` to
  `crates/voloxide_core/src/comm.rs`.
- Moved the communication interface trait from
  `crates/voloxide_core/src/comm_manager/comm_link_trait.rs` to
  `crates/voloxide_core/src/comm/interface.rs`.
- Added temporary compatibility re-exports so existing `voloxide_core::comm_manager` and
  `comm_link_trait` paths could still resolve during the migration.
- Updated internal core code and `voloxide_mavlink` to use the new
  `voloxide_core::comm::interface::CommInterface` path.

### 2026-05-18: Removed Communication Compatibility Re-Exports

- Updated remaining live code references from `comm_manager` and `comm_link_trait` to `comm` and
  `comm::interface`.
- Removed the temporary `pub use comm as comm_manager` and `pub use interface as comm_link_trait`
  re-exports.
- Updated `stm_32` serial/telemetry processors to import
  `voloxide_core::comm::interface::EmbeddedComInterface`.

### 2026-05-18: Renamed `bodytype` Domain To `vehicle`

- Moved `crates/voloxide_core/src/bodytype.rs` to `crates/voloxide_core/src/vehicle.rs`.
- Moved `crates/voloxide_core/src/bodytype/quadrotor.rs` to
  `crates/voloxide_core/src/vehicle/quadrotor.rs`.
- Updated core, sim, Nucleo, and Pixracer Pro code to use `voloxide_core::vehicle`.
- Removed now-empty legacy `bodytype` and `comm_manager` source directories.
- The `BodyType` trait remained briefly after this slice to keep the behavioral surface small; it
  was removed in a later cleanup slice.

### 2026-05-18: Adopted Heapless Event Queues And Cleaned Composition Syntax

- Added a direct `heapless` dependency to `voloxide_core`.
- Replaced the hand-written `EventQueue<T, N>` ring-buffer internals with a wrapper around
  `heapless::Deque<T, N>`.
- Preserved the existing `EventQueue` API, queue capacities, FIFO behavior, overflow error, and
  `push_or_log` behavior.
- Scoped the event-queue iterator test so it no longer keeps an immutable iterator alive while
  mutating the queue afterward.
- Added a local `SimWorld` type alias in the sim binary to hide the full
  `World<Board, QuadEstimator, QuadController, MatrixMixer, MavlinkInterface, SimPwmDriver>` type.
- Simplified embedded board entrypoint associated-type construction from fully qualified
  `voloxide_core::vehicle::quadrotor::Quadrotor` paths to the imported `Quadrotor` type. This was
  later simplified further when the `BodyType` marker trait was removed.

This confirms `heapless` is useful for bounded queue storage. It does not remove the core
`World<B, E, C, M, CI, PD>` generic shape; local aliases and board-specific constructors are the better
tool for that syntax.

### 2026-05-18: Removed Marker Vehicle Wrapper And Stale Estimator Names

- Removed the `BodyType` trait and the `Quadrotor` marker struct.
- Changed `World` from `World<B, BT, CI, PD>` to `World<B, E, C, M, CI, PD>`, where `E`, `C`, and
  `M` are the concrete estimator, controller, and mixer types.
- Changed the control pipeline context to accept the concrete estimator/controller/mixer types
  directly rather than passing through a marker vehicle type.
- Changed `voloxide_core::vehicle::quadrotor` to expose concrete aliases:
  `Estimator`, `Controller`, and `Mixer`, plus a `mixer(&Params)` constructor helper.
- Renamed stale migration-era estimator names:
  - `NamedEstimator` -> `Estimator`
  - `AttitudeStateTrait` -> `AttitudeEstimate`
  - `estimate_named` -> `estimate`
  - `estimate_named_with_external_attitude` -> `estimate_with_external_attitude`

MAVLink/core message mapping stays local to the MAVLink implementation crate. `voloxide_core`
continues to expose protocol-neutral communication messages and the `CommInterface` contract only.

### 2026-05-18: Split MAVLink Conversion Module

- Split MAVLink/core message conversion impls and conversion tests out of
  `comms/voloxide_mavlink/src/link.rs`.
- Added `comms/voloxide_mavlink/src/conversions.rs`.
- Kept `link.rs` focused on `MavlinkInterface`, frame construction, serial I/O, and the
  `CommInterface` implementation.

### 2026-05-18: Removed MAV-Specific Conversion Traits

- Removed the local `FromMav` and `ToMav` traits from `voloxide_mavlink`.
- Replaced them with ordinary `From` impls owned by `voloxide_mavlink`, where the generated MAVLink
  types are local.
- Updated `MavlinkInterface` call sites to use `From`/`Into` conversions.
- Updated the estimator integration test to use the renamed `Estimator::estimate` API so full
  package tests cover the post-refactor names.
- Kept `voloxide_core` protocol-neutral for users that provide non-MAVLink communication
  implementations.

### 2026-05-18: Completed Software Organization Steps 1-6

- Step 1: kept adapter responsibilities outside `voloxide_core`; MAVLink remains in
  `voloxide_mavlink`, sim transport remains in `sim`, and board startup remains in board crates.
- Step 2: moved `crates/voloxide_core/src/comm_messages.rs` to `crates/voloxide_core/src/comm/messages.rs` and
  moved `crates/voloxide_core/src/sensorprocessors.rs` to
  `crates/voloxide_core/src/sensors/processors.rs`.
- Step 3: added board-local `SimWorld`, `PixracerWorld`, and `NucleoWorld` aliases plus local
  `init_world` constructors at the runtime entrypoints.
- Step 4: split `World::run_once` into explicit high-level scheduler phases for communication and
  parameter service, sensor ingestion and health, RC/state updates, control/mixing, and telemetry.
  Existing test-facing wrapper methods were preserved.
- Step 5: removed broad workspace lint allowances so unused/stale hand-written code is visible
  again; protocol-shaped exceptions remain local.
- Step 6: replaced `micro_algebra` with `nalgebra` in core math code and tests, removed the old
  estimator/controller CSV-writing integration tests, removed checked-in CSV/PNG artifacts, and
  removed the old estimator/controller plotting/data-generation scripts.

### 2026-05-18: Grouped Domain Systems Under Owning Modules

- Replaced the remaining flat top-level `*_system.rs`, `*_manager.rs`, and stale quad file names
  with domain-owned modules.
- Parameter ownership is now:
  - `crates/voloxide_core/src/params.rs`
  - `crates/voloxide_core/src/params/service.rs`
  - `crates/voloxide_core/src/params/reactions.rs`
- Logging ownership is now:
  - `crates/voloxide_core/src/log.rs`
  - `crates/voloxide_core/src/log/drain.rs`
- Command and companion ownership is now:
  - `crates/voloxide_core/src/command.rs`
  - `crates/voloxide_core/src/command/service.rs`
  - `crates/voloxide_core/src/companion.rs`
- Sensor ownership is now:
  - `crates/voloxide_core/src/sensors.rs`
  - `crates/voloxide_core/src/sensors/ingestion.rs`
  - `crates/voloxide_core/src/sensors/processors.rs`
  - `crates/voloxide_core/src/sensors/health.rs`
- RC and PWM ownership is now:
  - `crates/voloxide_core/src/rc.rs`
  - `crates/voloxide_core/src/rc/system.rs`
  - `crates/voloxide_core/src/pwm.rs`
  - `crates/voloxide_core/src/pwm/system.rs`
- Control pipeline ownership is now `crates/voloxide_core/src/control.rs`.
- Quad implementations now use role names:
  - `crates/voloxide_core/src/controller/quad.rs`
  - `crates/voloxide_core/src/estimator/quad.rs`
  - `crates/voloxide_core/src/mixer/matrix.rs`
- Migrated `state_machine/mod.rs` to `state_machine.rs` with tests remaining in
  `state_machine/tests.rs`, preserving the modern Rust `module.rs` plus `module/child.rs` layout.

## Verification Log

- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p voloxide_core --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p voloxide_mavlink --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p sim` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p voloxide_mavlink --lib` passes: 4 tests.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p voloxide_core --lib` passes: 144 tests.
- After the `voloxide_core::comm` namespace move, these checks were rerun and pass:
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p voloxide_core --lib`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p sim`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p voloxide_core --lib`: 144 tests.
- After the `voloxide_mavlink::conversions` split:
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p voloxide_mavlink --lib`: 4 tests.
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p sim`
- After removing communication compatibility re-exports:
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p voloxide_core --lib`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p stm_32 --target thumbv7em-none-eabihf`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p sim`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p voloxide_core --lib`: 144 tests.
- After moving `bodytype` to `vehicle`:
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p voloxide_core --lib`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p sim`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p voloxide_core --lib`: 144 tests.
- After adopting `heapless::Deque` for `EventQueue` and cleaning composition syntax:
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p voloxide_core --lib`: 144 tests.
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p sim`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p voloxide_mavlink --lib`: 4 tests.
- After removing `BodyType` and stale estimator wrapper names:
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p voloxide_core --lib`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p sim`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p voloxide_core --lib`: 144 tests.

- After removing MAV-specific conversion traits and fixing the stale estimator integration-test names:
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt --check`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p voloxide_core`: 144 lib tests, 1 controller integration test, 1 estimator integration test, 15 mixer integration tests.
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p sim --lib`: 9 tests.
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p voloxide_mavlink --lib`: 4 tests.
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p voloxide_core --lib`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p voloxide_mavlink --lib`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p sim`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf`
  - `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p stm_32 --target thumbv7em-none-eabihf`

- After software organization steps 1-6:
  - `rustfmt --edition 2024` was run directly on touched Rust files because `cargo fmt` is not
    installed in the available toolchain.
  - `CARGO_HOME=/tmp/cargo-home RUSTC=/run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc /run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo check -p voloxide_core --lib` passes.
  - `CARGO_HOME=/tmp/cargo-home RUSTC=/run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc /run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo test -p voloxide_core --lib` passes: 144 tests.
  - `CARGO_HOME=/tmp/cargo-home RUSTC=/run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc /run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo test -p voloxide_core --tests` passes: 144 lib tests and 15 mixer integration tests.
  - `CARGO_HOME=/tmp/cargo-home RUSTC=/run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc /run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo check -p voloxide_mavlink --lib --message-format short` passes.
  - `CARGO_HOME=/tmp/cargo-home RUSTC=/run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc /run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo check -p sim --message-format short` passes.
  - `cargo check -p pixracerpro --target thumbv7em-none-eabihf` is blocked in this container because
    the available Rust 1.95.0 toolchain does not have the `thumbv7em-none-eabihf` target installed
    (`can't find crate for core`).

- After grouping domain systems under owning modules:
  - `/run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt --edition 2024` was run on touched Rust files.
  - `CARGO_HOME=/tmp/cargo-home RUSTC=/run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc /run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo check -p voloxide_core --lib --message-format short` passes.
  - `CARGO_HOME=/tmp/cargo-home RUSTC=/run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc /run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo test -p voloxide_core --tests --message-format short` passes: 144 lib tests and 15 mixer integration tests.
  - `CARGO_HOME=/tmp/cargo-home RUSTC=/run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc /run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo check -p voloxide_mavlink --lib --message-format short` passes.
  - `CARGO_HOME=/tmp/cargo-home RUSTC=/run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc /run/host/home/skink/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo check -p sim --message-format short` passes.
  - `cargo check -p pixracerpro --target thumbv7em-none-eabihf` remains blocked in this container because the target is not installed (`can't find crate for core`).

## Remaining Work

- Install the embedded `thumbv7em-none-eabihf` Rust target before rerunning Pixracer Pro, Nucleo, and
  STM32 target checks in this container.
- Decide whether to clean up the warnings exposed by removing broad workspace `unused` allowances in
  this branch or leave them as a visible follow-up.
