# Software Organization Next Steps

## Purpose

This document records the next organization pass after the ECS migration and the software
reorganization work. The goal is to keep Voloxide clean, professional, easy to read, and easy for
new students to modify without weakening the compile-time guarantees that protect flight behavior.

The next implementation phase should complete steps 1-5 below, rerun the full verification gate,
and then reassess whether Voloxide should move farther toward a more vanilla entity-component-system
shape.

## Current Direction

Voloxide should keep these boundaries clear:

- `voloxide_core` owns protocol-neutral and board-neutral flight-stack logic.
- `voloxide_core` exposes resources, systems, events, parameters, packets, scheduler logic, and
  protocol-neutral communication contracts.
- `voloxide_mavlink` owns MAVLink dialect generation, parsing, frame construction, and conversions
  between MAVLink wire types and core communication messages.
- Board and runtime crates such as `sim`, `pixracerpro`, and `nucleo` choose concrete board,
  communication, PWM, estimator, controller, and mixer implementations.
- The external ROSflight workspace remains unmodified. Voloxide adapts to existing ROSflight
  behavior.

The code should make that hierarchy visible from file names, module paths, and construction APIs.

## Step 1: Tighten Crate Responsibilities

`voloxide_core` should remain free of MAVLink, ROS 2 transport, board startup, and runtime-specific
implementation details. Adapter crates should depend inward on core, not the other way around.

Concrete work:

- Audit `voloxide_core` for protocol-specific names or assumptions that belong in adapter crates.
- Keep communication contracts in core, but keep protocol encoders/decoders outside core.
- Keep board startup and peripheral ownership in board crates.
- Keep sim transport concerns in `sim`.
- Make sure new code follows the dependency direction:

```text
voloxide_core
voloxide_mavlink -> voloxide_core
sim              -> voloxide_core + voloxide_mavlink
pixracerpro      -> voloxide_core + voloxide_mavlink + stm_32
nucleo           -> voloxide_core + voloxide_mavlink + stm_32
stm_32           -> voloxide_core
```

Expected benefit:

- New students can learn core flight behavior without learning MAVLink or board startup first.
- Non-MAVLink communication implementations remain first-class.
- Compile-time composition stays explicit at crate boundaries.

## Step 2: Rename Around Roles, Not Migration History

Some module names still reflect historical migrations or implementation mechanics rather than the
role a reader is looking for. Names should answer: what part of the flight stack is this?

Candidate cleanup areas:

- Move `comm_messages` toward `comm/messages` if that can be done without churn.
- Move `sensorprocessors` toward `sensors/processors`.
- Consider grouping control pipeline code under `control/pipeline`.
- Keep `vehicle/quadrotor` as the place for quadrotor-specific concrete aliases and helpers.
- Avoid compatibility names such as old body/HList/marker terminology in live code.
- Keep MAVLink-specific names inside `voloxide_mavlink`.

Expected benefit:

- File paths become a map of the architecture.
- Readers do not need migration history to understand current code.
- Refactors become easier because ownership boundaries are visible.

## Step 3: Add Board-Local Construction Helpers

The static `World<B, E, C, M, CI, PD>` shape is valuable, but the raw generic type is too noisy at
entrypoints. Board crates should hide that syntax behind local aliases and constructors.

Concrete work:

- Add board-local type aliases such as `SimWorld`, `PixracerWorld`, and `NucleoWorld` where useful.
- Add board-local `init_world` or similarly named constructors that assemble the concrete board,
  estimator, controller, mixer, communication interface, and PWM driver.
- Keep core generic and reusable. Do not make core depend on board-local aliases.
- Prefer constructors over type erasure.

Expected benefit:

- Entrypoints become short and readable.
- The compile-time type relationships remain intact.
- Students can modify a board composition in one local place.

## Step 4: Make Scheduler Stages Read Like a Table of Contents

`World` should make the flight loop easy to scan. The current systems are increasingly separated;
the next pass should make the stage orchestration equally clear.

Concrete work:

- Group resources and stage runners around the major flight-loop phases:
  - communication and parameter service
  - companion inputs and commands
  - sensor ingestion and health
  - RC and state machine updates
  - control and mixing
  - PWM output
  - telemetry and logs
- Keep stage functions explicit and ordered.
- Use small context structs where they clarify borrow boundaries.
- Avoid hiding ordering in a framework before the ordering is obvious in plain Rust.

Expected benefit:

- The scheduler becomes teachable.
- System ordering is easy to audit before ROSflight integration tests.
- Future ECS movement has a clearer starting point.

## Step 5: Tighten Lint Scope

The workspace currently allows some naming and unused-code lints broadly. Some generated or wire
protocol code may need exceptions, but the whole workspace should not normalize those exceptions.

Concrete work:

- Move broad lint allowances down to the modules or crates that truly need them.
- Prefer idiomatic Rust naming in hand-written code.
- Keep generated MAVLink or protocol-shaped names isolated from core style rules.
- Re-enable useful compiler pressure where it helps catch stale migration code.

Expected benefit:

- Stale compatibility names become easier to catch.
- Hand-written code looks more professional and idiomatic.
- Students get clearer compiler feedback.

## Vanilla ECS Reassessment

After steps 1-5, reassess whether Voloxide should move farther toward a vanilla
entity-component-system architecture.

A fuller ECS could help if the remaining code still has these problems:

- Adding a system requires touching too many unrelated scheduler/resource fields.
- Resource ownership is hard to see from function signatures.
- Tests need large world fixtures for small behavior.
- Ordering dependencies are implicit or scattered.
- New students struggle to find where data lives and where it changes.

Potential benefits of pushing farther:

- More uniform system signatures.
- Better separation between resources, events, and systems.
- More testable individual systems.
- A scheduler that can declare ordering more mechanically.
- Easier insertion of new systems without editing a large central struct.

Potential costs:

- A generic ECS framework may hide flight-loop ordering behind framework machinery.
- Dynamic resource lookup or runtime borrow checks can make failures less direct.
- Embedded/no-std constraints may limit crate choices.
- The current static relationships between estimator, controller, and mixer are valuable and should
  not be erased casually.
- Full ECS conversion may add conceptual overhead for students if it becomes framework-first instead
  of flight-stack-first.

Preferred near-term direction:

- Keep an ECS style in plain Rust: resources, events, systems, explicit stages, typed queues, and
  small context structs.
- Do not adopt an off-the-shelf ECS crate unless the post-cleanup code still has clear pain that the
  crate solves better than explicit Rust.
- Preserve compile-time relationships for safety-critical flight-control composition.

## Heapless And Runtime Polymorphism Tradeoffs

`heapless` is a good fit for bounded storage. It does not reduce compile-time safety when used for
fixed-capacity queues, strings, maps, or buffers. It can improve clarity because capacity becomes an
explicit part of the type or constructor.

Good uses:

- Event queues.
- Bounded telemetry/log queues.
- Fixed-size protocol buffers where allocation is not appropriate.
- Embedded resource collections with known maximum size.

Runtime polymorphism is a separate question. Replacing generic parameters with trait objects can
reduce type verbosity, but it also weakens some useful static relationships.

Tradeoffs:

- Trait objects can reduce monomorphized code size in some cases.
- Trait objects add dynamic dispatch and may reduce optimization opportunities.
- Associated type relationships such as `Controller<State = E::State>` and
  `MixerInput = C::ControlOutput` are harder to preserve behind trait objects.
- Object-safety constraints can force less natural trait APIs.
- Some construction mistakes move from compile-time type errors to runtime wiring errors.

Preferred approach:

- Keep static generics inside firmware `World` composition.
- Hide verbosity with board-local aliases and constructors.
- Use runtime polymorphism mainly at desktop/sim/plugin boundaries where runtime selection is truly
  needed.

## Dependency Note: `num-bigint-dig`

`num-bigint-dig v0.8.5` is not a direct Voloxide dependency. It is pulled in by the sim dependency
stack:

```text
sim -> zenoh -> zenoh-transport -> rsa -> num-bigint-dig
```

It appears because Zenoh brings in RSA support through its transport/security stack. It is not part
of `voloxide_core` and is not used by the embedded board crates directly.

Follow-up work:

- Investigate whether `sim` can disable unnecessary Zenoh default features.
- Preserve the ROSflight sim integration behavior while reducing unused transport/security weight.
- Rerun `cargo tree -i num-bigint-dig` after any Zenoh feature changes.

## Verification Gate For This Phase

After implementing steps 1-5, rerun at least:

```text
RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt --check
RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p voloxide_core
RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p sim --lib
RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo test -p voloxide_mavlink --lib
RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p voloxide_core --lib
RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p voloxide_mavlink --lib
RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p sim
RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf
RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf
RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p stm_32 --target thumbv7em-none-eabihf
```

Then reassess:

- Is `World` still too hard to read?
- Are resource dependencies clear from function signatures?
- Can a new student add a small system without touching unrelated code?
- Are board entrypoints short and obvious?
- Are protocol-specific details isolated from core?
- Is a fuller ECS framework still worth its added abstraction?



As a final note, we should NO LONGER be using micro_algebra... that package is old and we should be doing all our math with a more modern, supported rust crate for linear algebra.
