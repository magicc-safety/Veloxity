# Moving From HLists Toward Static ECS-Style Systems

## Context

RustFlight currently uses HLists to encode board sensor inventory, sensor processing pipelines, body-type sensor requirements, telemetry packet access, and compile-time compatibility between boards and vehicle bodies.

That design solved an important early problem: it let the compiler prove that a selected board could provide the packet types required by a selected body type. It also made the raw-to-processed sensor pipeline generic over different board shapes.

The downside is that the type system is now carrying too much architectural bookkeeping. Adding a board/body/configuration requires positional type indices such as `There<There<There<Here>>>`, and board implementations have to write through nested tuple fields such as `sensors.1.1.1.0`. This makes the architecture hard to extend, hard to read, and brittle when sensors, telemetry streams, or body requirements change.

The goal of this proposal is to keep RustFlight modular and deterministic while replacing HList rigidity with named resources, fixed-order systems, and bounded events.

## Current HList Responsibilities

The current HList design provides five main capabilities:

1. Board-specific raw sensor inventory.
2. Raw-to-processed sensor processor ordering.
3. Body-type required sensor selection.
4. Telemetry access to processed packet types.
5. Compile-time board/body compatibility checks.

Those capabilities are valuable. The proposed architecture keeps the capabilities but moves them into simpler constructs.

## Proposed Direction

Use a static ECS-style architecture rather than a conventional dynamic ECS crate.

In this context, ECS means:

- Resources: singleton state owned by the flight stack, such as params, comms, sensors, estimator, controller, mixer, RC, state manager, and PWM.
- Systems: explicit functions or traits that read and write a small set of resources.
- Events: bounded queues or single-slot event buffers used to decouple producers from consumers.
- Schedule: a fixed, deterministic order for running systems each control loop.

This is closer to "resources plus systems plus events" than to game-style entity/component tables. RustFlight has a small number of singleton subsystems, not thousands of similar entities.

## Named Sensor Resources

Replace HList-based raw and processed sensor sets with named structs.

```rust
pub struct SensorBus {
    pub imu: Option<Result<ImuPacket, SensorError>>,
    pub mag: Option<Result<MagPacket, SensorError>>,
    pub baro: Option<Result<BaroPacket, SensorError>>,
    pub pitot: Option<Result<PitotPacket, SensorError>>,
    pub range: Option<Result<RangePacket, SensorError>>,
    pub gnss: Option<Result<GNSSPacket, SensorError>>,
    pub battery: Option<Result<BatteryPacket, SensorError>>,
    pub rc: Option<Result<RcPacket, SensorError>>,
    pub attitude: Option<Result<AttitudePacket, SensorError>>,
}

pub struct ProcessedSensors {
    pub imu: Option<ImuPacket>,
    pub mag: Option<MagPacket>,
    pub baro: Option<BaroPacket>,
    pub pitot: Option<PitotPacket>,
    pub range: Option<RangePacket>,
    pub gnss: Option<GNSSPacket>,
    pub battery: Option<BatteryPacket>,
    pub rc: Option<RcPacket>,
    pub attitude: Option<AttitudePacket>,
}
```

This removes positional access from board code and telemetry code. A board fills `raw.imu` or `raw.rc`; telemetry reads `processed.imu` or `processed.battery`.

## Sensor Processing As Systems

Instead of mapping an HList of processors over an HList of raw packets, sensor processing becomes a set of named systems.

```rust
fn update_board_sensors<B: BoardTrait>(board: &mut B, raw: &mut SensorBus);

fn process_imu(
    raw: &mut SensorBus,
    processed: &mut ProcessedSensors,
    params: &mut Params,
    flags: &mut CalibrationFlags,
);

fn process_mag(
    raw: &mut SensorBus,
    processed: &mut ProcessedSensors,
    params: &Params,
);
```

Each processor can still be modular and board-independent. The difference is that the dispatch is explicit and named rather than recursive over type-level list structure.

## Estimator And Body Inputs

The estimator trait can move away from HList inputs.

Current shape:

```rust
pub trait Estimator {
    type Inputs: HList;
    type State: AttitudeStateTrait;

    fn estimate(&mut self, inputs: &Self::Inputs, params: &Params, dt: f64) -> Self::State;
}
```

Proposed shape:

```rust
pub trait Estimator {
    type State: AttitudeStateTrait;

    fn estimate(
        &mut self,
        sensors: &ProcessedSensors,
        params: &Params,
        dt: f64,
    ) -> Self::State;
}
```

Body-type requirements can be represented explicitly instead of by sculpting a type-level sensor list.

```rust
pub trait SensorRequirements {
    const NEEDS_IMU: bool;
    const NEEDS_MAG: bool;
    const NEEDS_RC: bool;
}
```

or:

```rust
pub trait BodyType {
    fn validate_sensors(sensors: &ProcessedSensors) -> SensorAvailability;
}
```

This trades some compile-time proof for much clearer code and better extensibility. Missing required sensors become explicit runtime health/state errors, which is already a natural fit for a flight stack.

## Event Model

Events should be used when one subsystem needs to announce something without knowing every subsystem that may care.

Representative events:

```rust
pub enum Event {
    ParamSetRequested { id: ParamId, value: ParamValue },
    ParamChanged { id: ParamId, old: ParamValue, new: ParamValue },
    OffboardControlReceived(OffboardControlMsg),
    CommandReceived(RosflightCmd),
    ImuReceived(ImuPacket),
    RcReceived(RcPacket),
    CalibrationComplete(CalibrationKind),
}
```

The parameter path is the clearest example. Today, the comm layer applies a param set and sends the `PARAM_VALUE` response before dependent modules run their callbacks. With events, the order can become:

1. Decode `PARAM_SET`.
2. Emit `ParamSetRequested`.
3. Apply and validate the parameter change.
4. Emit `ParamChanged`.
5. Run all systems that subscribe to `ParamChanged`.
6. Send `PARAM_VALUE` after dependent systems have reacted.

This better matches ROSflight callback behavior while keeping ownership explicit in Rust.

## Fixed Schedule

The main loop should become a deterministic system schedule.

```rust
receive_comm();
apply_param_requests();
dispatch_param_changed();
update_board_sensors();
process_sensors();
run_rc();
run_command_manager();
run_state_machine();
run_estimator_controller_mixer_if_new_imu();
send_telemetry();
send_comm_responses();
```

The schedule is intentionally explicit. RustFlight should not need a dynamic scheduler for the embedded core. Fixed ordering keeps timing, side effects, and safety behavior understandable.

## World Shape

A static world can hold all singleton resources.

```rust
pub struct World<B, BT, CI, PD>
where
    B: BoardTrait,
    BT: BodyType,
    CI: CommInterface<B>,
    PD: PwmDriver,
{
    pub board: B,
    pub raw_sensors: SensorBus,
    pub processed_sensors: ProcessedSensors,
    pub params: Params,
    pub comm: CommManager<B, CI>,
    pub rc: Rc,
    pub command: CommandManager,
    pub state: StateManager,
    pub estimator: BT::Estimator,
    pub controller: BT::Controller,
    pub mixer: BT::Mixer,
    pub pwm: PD,
    pub events: EventQueues,
}
```

This preserves the current board/body generic structure without forcing every interaction through HList associated types.

## Why Not A General ECS Crate?

The core crate is `no_std`, deterministic, and embedded-oriented. General ECS crates often assume dynamic storage, allocation, runtime queries, or flexible scheduling. RustFlight does not need most of that.

The better fit is a small static ECS-inspired architecture:

- no allocation required
- bounded event buffers
- named resources
- fixed schedule
- static dispatch where useful
- readable subsystem boundaries

## Migration Plan

1. Add `SensorBus` and `ProcessedSensors` alongside the existing HList system.
2. Convert the `sim` board to fill `SensorBus`.
3. Convert telemetry to read named processed sensor fields instead of `HListGet`.
4. Convert `QuadEstimator` to accept `ProcessedSensors` or a named estimator sensor view.
5. Remove `Configuration::SculptIndices` and packet index associated types.
6. Replace `CommManager::act_on_messages` with event-producing message handling.
7. Add a staged parameter flow: request, apply, dispatch `ParamChanged`, then acknowledge.
8. Convert RC, command manager, estimator, controller, mixer, and sensor processors to respond to events where callbacks are currently needed.
9. Remove the HList module once all boards and body types no longer depend on it.

## Open Design Questions

- Should events be one global enum or split into domain-specific queues such as `CommEvents`, `ParamEvents`, `SensorEvents`, and `ControlEvents`?
- Should event queues be fixed-capacity ring buffers, single-slot latest-value buffers, or both depending on event type?
- Should estimator inputs be the full `ProcessedSensors` struct or a smaller typed view such as `EstimatorSensors`?
- Should missing required sensors be checked during init, each loop, or only when entering specific flight states?
- Should parameter subscribers be explicit systems in the schedule or registered through a static trait list?

## Core Principle

The replacement for HLists should not be less modular. It should make the modularity easier to see.

Subsystems should communicate through named resources and emitted events. The main loop should define when communication happens. Modules should not need to know every other module directly, and the type system should enforce important invariants without requiring positional type-level bookkeeping for normal development.

## Ports As The Central Abstraction

Ports plus a fixed schedule are the best fit for RustFlight.

A `World` is still useful internally as the owner of all state, but modules should not receive `&mut World`. Systems should receive ports, where each port is a narrow capability: read params, write state, emit log, send comm response, read sensors, write PWM, and so on.

## Why Ports Are Best

Ports give the ROSflight feel:

```text
a module can interact with the rest of the system
```

but with Rust constraints:

```text
a module can only interact through capabilities it was explicitly given
```

That means the function signature becomes the dependency graph.

```rust
fn rc_system(ctx: RcSystemCtx<'_>) { ... }
```

If `RcSystemCtx` has no `PwmPort`, RC cannot touch PWM. If it has no `CommTxPort`, RC cannot send MAVLink directly. That is compile-time enforced.

## Port Types

Ports should be split by capability, not by subsystem alone.

```rust
pub struct ParamsReadPort<'a> {
    params: &'a Params,
}

pub struct ParamsWritePort<'a> {
    params: &'a mut Params,
}

pub struct StateReadPort<'a> {
    state: &'a StateManager,
}

pub struct StateWritePort<'a> {
    state: &'a mut StateManager,
}

pub struct SensorReadPort<'a> {
    sensors: &'a ProcessedSensors,
}

pub struct EventEmitPort<'a, E> {
    queue: &'a mut EventQueue<E>,
}

pub struct EventReadPort<'a, E> {
    queue: &'a EventQueue<E>,
}

pub struct LogPort<'a> {
    logs: &'a mut LogQueue,
}
```

Then compose them into per-system contexts:

```rust
pub struct RcSystemCtx<'a> {
    pub params: ParamsReadPort<'a>,
    pub state: StateWritePort<'a>,
    pub rc_events: EventEmitPort<'a, RcEvent>,
    pub logs: LogPort<'a>,
}

pub struct ParamSystemCtx<'a> {
    pub params: ParamsWritePort<'a>,
    pub requests: EventReadPort<'a, ParamRequest>,
    pub changes: EventEmitPort<'a, ParamChanged>,
    pub responses: EventEmitPort<'a, CommResponse>,
}
```

This is cleaner than giving every system a pile of raw references, and safer than `&mut World`.

## Ports Should Have Methods, Not Public Fields

Ports should usually expose methods, not raw references.

Instead of:

```rust
pub struct StateWritePort<'a> {
    pub state: &'a mut StateManager,
}
```

prefer:

```rust
pub struct StateWritePort<'a> {
    state: &'a mut StateManager,
}

impl<'a> StateWritePort<'a> {
    pub fn request_arm(&mut self, params: &Params) {
        self.state.update(Event::REQUEST_ARM, params);
    }

    pub fn request_disarm(&mut self, params: &Params) {
        self.state.update(Event::REQUEST_DISARM, params);
    }

    pub fn set_error(&mut self, error: ErrorFlag, params: &Params) {
        self.state.update(Event::ERROR_OCCURRED(error), params);
    }

    pub fn clear_error(&mut self, error: ErrorFlag, params: &Params) {
        self.state.update(Event::ERROR_CLEARED(error), params);
    }
}
```

That gives a tighter API. A module can request arm/disarm, but it cannot arbitrarily rewrite the state manager internals.

## Example: RC

Instead of RC taking `&mut StateManager`, `&Params`, and maybe `&mut CommManager`, it gets an RC context:

```rust
pub struct RcRunCtx<'a> {
    pub params: ParamsReadPort<'a>,
    pub state: StateWritePort<'a>,
    pub logs: LogPort<'a>,
}

impl Rc {
    pub fn run(&mut self, now_ms: u32, mut ctx: RcRunCtx<'_>) {
        let arm_threshold = ctx.params.float(ParamId::PARAM_ARM_THRESHOLD, 0.15);

        if self.should_request_arm(arm_threshold) {
            ctx.state.request_arm(ctx.params.raw());
        }
    }
}
```

The RC module can still communicate with the system, but only through approved capabilities.

## Example: Param Change Flow

```rust
pub struct ParamApplyCtx<'a> {
    pub params: ParamsWritePort<'a>,
    pub requests: EventDrainPort<'a, ParamSetRequested>,
    pub changes: EventEmitPort<'a, ParamChanged>,
    pub responses: EventEmitPort<'a, CommResponse>,
}

fn apply_param_requests(mut ctx: ParamApplyCtx<'_>) {
    while let Some(req) = ctx.requests.next() {
        let old = ctx.params.get(req.id);

        if ctx.params.set(req.id, req.value).is_ok() {
            let new = ctx.params.get(req.id);
            ctx.changes.emit(ParamChanged { id: req.id, old, new });
            ctx.responses.emit(CommResponse::ParamAccepted { id: req.id, value: new });
        }
    }
}
```

Then subscribers react:

```rust
pub struct RcParamCtx<'a> {
    pub rc: &'a mut Rc,
    pub params: ParamsReadPort<'a>,
    pub changes: EventReadPort<'a, ParamChanged>,
    pub logs: LogPort<'a>,
}

fn rc_on_param_changed(mut ctx: RcParamCtx<'_>) {
    for change in ctx.changes.iter() {
        if change.affects_rc_mapping() {
            ctx.rc.reload_mapping(&ctx.params);
            ctx.logs.info("RC mapping updated");
        }
    }
}
```

The acknowledgement can be scheduled after all subscribers:

```rust
receive_comm();
apply_param_requests();
rc_on_param_changed();
command_on_param_changed();
estimator_on_param_changed();
send_comm_responses();
```

That gives ROSflight callback semantics with explicit causality.

## Who Creates Ports?

Only the scheduler creates ports.

```rust
fn run(&mut self) {
    receive_comm(CommRxCtx {
        comm: CommRxPort::new(&mut self.world.comm, &mut self.world.board),
        param_requests: EventEmitPort::new(&mut self.world.events.param_requests),
        command_events: EventEmitPort::new(&mut self.world.events.commands),
    });

    apply_param_requests(ParamApplyCtx {
        params: ParamsWritePort::new(&mut self.world.params),
        requests: EventDrainPort::new(&mut self.world.events.param_requests),
        changes: EventEmitPort::new(&mut self.world.events.param_changes),
        responses: EventEmitPort::new(&mut self.world.events.comm_responses),
    });

    rc_on_param_changed(RcParamCtx {
        rc: &mut self.world.rc,
        params: ParamsReadPort::new(&self.world.params),
        changes: EventReadPort::new(&self.world.events.param_changes),
        logs: LogPort::new(&mut self.world.events.logs),
    });
}
```

This keeps borrowing local and auditable.

## Port Rules

Use these rules:

1. `World` owns state but is only touched by the scheduler.
2. Systems receive context structs.
3. Context structs contain ports.
4. Ports expose methods, not raw fields, when invariants matter.
5. Read and write ports are separate types.
6. Event queues are domain-specific.
7. No dynamic subscriber registry in the embedded core.
8. Cross-module communication uses events or ports, not arbitrary peer references.

This gives RustFlight a strong architecture:

```text
World = ownership
Ports = capabilities
Systems = behavior
Events = decoupled communication
Schedule = causality
```

Ports should be treated as the central abstraction for the restructuring effort.

## Implementation Log

This section records the current restructuring work so another engineer can resume without needing the conversation history.

### Branch And Commit State

- Working branch: `tmp_restructuring`.
- Branch source: local `main`.
- First local commit completed: `52cb537 Document HList to ECS restructuring plan`.
- The first commit contains this architecture note.
- Local commit author used for the first commit: `Codex <codex@local>`, because the repository did not have a Git author identity configured.

### User Decisions So Far

- The architecture document should be committed first. This is done.
- The first implementation target is the parameter callback path.
- Functionality inside existing modules should remain intact as much as possible.
- A clean rewrite using the new ports/events methodology is preferred over preserving the old static callback shape.
- Introducing a new `World` or scheduler type is acceptable.
- The long-term direction is to remove HLists and the rigid wiring systems.
- Event queues should be fixed-size ring buffers.
- Existing core tests are considered poorly structured for this migration. The plan is to create a better dummy-board-based test suite for core.
- After the core parameter path and core test infrastructure are stable, the next target is `sim`, then `pixracerpro`.

### Question Clarification

The earlier question "Should `sim` remain the proving ground while we restructure core, or should core tests use only dummy board/test fixtures until core is stable?" means:

- Option A: during core refactors, test through a fake/dummy board and keep all tests inside `rustflight_core`.
- Option B: during core refactors, also use the `sim` crate as an integration target to prove the migrated core works in a realistic application.

The current user preference is to recreate the dummy board for tests first, then finish `sim` after this core work is done.

## Current Parameter-Path Rewrite

The first implementation slice replaces the old parameter callback path with an event-driven staged path.

### Old Behavior

Before this work:

1. `CommManager::act_on_messages` decoded `PARAM_SET`.
2. It mutated `Params` directly.
3. It sent the MAVLink `PARAM_VALUE` acknowledgement immediately.
4. It returned `Option<ParamId>`.
5. `ROSFlight::run` later called `Rc::param_change_callback` and manually updated command-manager failsafe config.

That meant RustFlight could acknowledge a parameter change before all interested modules had reacted to it.

### New Intended Behavior

The new path is:

1. `CommManager::act_on_messages` decodes `PARAM_SET`.
2. It emits `ParamSetRequested`.
3. `param_system::apply_param_requests` mutates `Params`.
4. It emits `ParamChanged`.
5. Existing module reactions run from `ParamChanged`.
6. It emits `CommResponse::ParamValue`.
7. `CommManager::send_comm_responses` sends the MAVLink `PARAM_VALUE` acknowledgement after reactions have run.

This is the first concrete use of the ports/events model.

### Files Added

`rustflight_core/src/events.rs`

- Adds `EventQueue<T, const N: usize>`.
- Uses a fixed-size ring buffer backed by `[Option<T>; N]`.
- Provides `push`, `pop`, `iter`, `clear`, `len`, and `is_empty`.
- Adds `EventQueueError::Full`.
- Adds parameter-path event types:
  - `ParamSetRequested`
  - `ParamChanged`
  - `CommResponse::ParamValue`
- Adds queue capacities:
  - `PARAM_SET_REQUEST_QUEUE_CAPACITY`
  - `PARAM_CHANGED_QUEUE_CAPACITY`
  - `COMM_RESPONSE_QUEUE_CAPACITY`
- Adds `ParamEventQueues`.
- Adds focused unit tests for FIFO order and non-draining iteration.

`rustflight_core/src/ports.rs`

- Adds initial port types:
  - `ParamsReadPort`
  - `ParamsWritePort`
  - `EventEmitPort`
  - `EventDrainPort`
  - `EventReadPort`
- These are intentionally narrow capability wrappers.
- The scheduler or high-level orchestration code is expected to construct ports from world fields.

`rustflight_core/src/param_system.rs`

- Adds `ParamApplyCtx`.
- Adds `apply_param_requests`.
- This function drains `ParamSetRequested`, mutates params, emits `ParamChanged`, and emits deferred `CommResponse::ParamValue`.
- Adds a focused unit test proving that a param request mutates params and queues the ack response instead of sending it immediately.

### Files Modified

`rustflight_core/src/lib.rs`

- Exposes new modules:
  - `events`
  - `ports`
  - `param_system`

`rustflight_core/src/comm_manager.rs`

- Imports `CommResponse`, `ParamEventQueues`, and `ParamSetRequested`.
- Changes `act_on_messages` so it no longer returns `Option<ParamId>`.
- Adds a `param_events: &mut ParamEventQueues` argument.
- Keeps existing behavior for param-list streaming, timesync, offboard control, and ROSflight command handling.
- Changes `PARAM_SET` handling:
  - decodes the parameter name
  - looks up the static param definition
  - pushes `ParamSetRequested`
  - does not mutate `Params`
  - does not send `PARAM_VALUE` immediately
- Adds `send_comm_responses`, which drains `CommResponse` events and sends `PARAM_VALUE`.
- `send_comm_responses` also updates `CommManager::sysid` when the accepted response is for `PARAM_SYSTEM_ID`, preserving existing sysid behavior while moving it to the response stage.

`rustflight_core/src/rosflight.rs`

- Adds a `param_events: ParamEventQueues` field to `ROSFlight`.
- Initializes it in `ROSFlight::init`.
- After `comm_manager.act_on_messages`, calls `param_system::apply_param_requests` using ports.
- Iterates `param_events.changes` and preserves existing module reactions:
  - `Rc::param_change_callback`
  - `CommandManager::update_failsafe_config` for `PARAM_FAILSAFE_THROTTLE` and `PARAM_FIXED_WING`
- Calls `comm_manager.send_comm_responses` after reactions.
- Clears `param_events.changes` after the stage.
- Removes the old later `if let Some(param_id) = changed_param_id` callback block.

`rustflight_core/src/param_reactions.rs`

- Adds named systems for parameter-change subscribers.
- Adds `RcParamChangedCtx`.
- Adds `rc_on_param_changed`.
- Adds `CommandParamChangedCtx`.
- Adds `command_on_param_changed`.
- `ROSFlight::run` now calls these named systems instead of embedding the reaction loop directly.
- `rc_on_param_changed` still calls the existing `Rc::param_change_callback` to preserve behavior.
- `command_on_param_changed` preserves the existing failsafe-config update behavior for `PARAM_FAILSAFE_THROTTLE` and `PARAM_FIXED_WING`.
- Adds focused coverage for command-manager reaction filtering.

`rustflight_core/src/rc.rs`

- `Rc::param_change_callback` no longer takes `Board` or `CommManager`.
- `Rc::log_switch_mappings` no longer takes `CommManager`.
- This removes unnecessary peer-module access from the RC parameter reaction path.
- RC mapping behavior is preserved.
- Logging still goes through the existing global `log_info!` path for now.

`rustflight_core/src/sensors.rs`

- Adds `SensorBus`.
- Adds `ProcessedSensors`.
- Both are named resource structs that mirror the current raw and processed HList sensor inventories.
- This is non-behavior-changing scaffolding for the HList removal.
- Both resources have `clear` helpers.
- Adds focused coverage that default sensor resources are empty.

### Validation Status

`cargo check -p rustflight_core --lib` passes after the initial parameter-path rewrite.

Focused unit checks for the new modules pass:

- `cargo test -p rustflight_core events::tests --lib`
- `cargo test -p rustflight_core param_system::tests --lib`
- `cargo test -p rustflight_core comm_manager::tests --lib`
- `cargo test -p rustflight_core param_reactions::tests --lib`
- `cargo test -p rustflight_core sensors::tests --lib`

Additional status:

- `cargo test -p rustflight_core --lib` currently runs 42 library tests.
- 30 pass and 12 fail.
- The failures are in existing `state_machine::tests`, mostly around arming and `UNCALIBRATED_IMU` expectations.
- These failures are not introduced by the new parameter event path, but they confirm that the current test suite needs cleanup as part of the core test rebuild.

Formatting status:

- `cargo fmt` could not run because `cargo-fmt`/`rustfmt` is not installed for the current `stable-aarch64-unknown-linux-gnu` toolchain.
- The command failed with: `error: 'cargo-fmt' is not installed for the toolchain 'stable-aarch64-unknown-linux-gnu'`.

`cargo test -p rustflight_core` currently does not pass, but the failures are from legacy tests that already do not match current APIs:

- mixer tests call `mixer.mix(&input)` but the current trait requires a state manager argument.
- controller tests call `controller.control(...)` without the current `dt` argument.
- estimator tests call `estimator.estimate(...)` without the current `dt` argument.

The user has confirmed that the current testing infrastructure is not well written and should be replaced with a better dummy-board-based setup.

## Test Support Progress

A new test-only support module has been started:

`rustflight_core/src/test_support.rs`

- Compiled only under `#[cfg(test)]`.
- Adds `TestBoard`.
- `TestBoard` implements `BoardTrait`.
- It uses `HNil` for raw sensors, processed sensors, and processor list so communication/parameter tests do not depend on the old nested HList sensor fixture.
- Adds `RecordingCommLink`.
- `RecordingCommLink` implements `CommInterface<TestBoard>`.
- It records outgoing `ParamValueMsg` responses for assertions.
- Other comm send methods are no-op placeholders for now.

New comm-manager tests:

- `param_set_emits_request_without_mutating_or_acknowledging`
  - injects a `ParamSetMsg` directly into `CommManager::msgs`
  - calls `act_on_messages`
  - verifies `Params` is not mutated at decode time
  - verifies no `PARAM_VALUE` ack is sent at decode time
  - verifies `ParamSetRequested` is emitted
- `send_comm_responses_sends_param_value_and_updates_sysid`
  - pushes a deferred `CommResponse::ParamValue`
  - calls `send_comm_responses`
  - verifies a param value response is sent
  - verifies `CommManager::sysid` updates for `PARAM_SYSTEM_ID`
- `param_set_pipeline_defers_ack_until_after_apply_stage`
  - composes the current core parameter pipeline in one test
  - injects `PARAM_SET`
  - verifies decode emits a request without mutating params or sending ack
  - applies parameter requests through `param_system::apply_param_requests`
  - verifies `Params` mutates and `ParamChanged` is emitted
  - verifies the ack is still not sent until `send_comm_responses`
  - verifies `send_comm_responses` sends `PARAM_VALUE` and updates `sysid`

This is the first step toward replacing the old test infrastructure with small, targeted dummy-board fixtures.

### Immediate Next Steps

1. Commit the dummy-board test-support slice and this updated implementation log.
2. Decide whether existing state-machine tests should be repaired, replaced, or temporarily isolated while the scheduler is being rewritten.
3. Add a proper `LogPort` or log event path and migrate global logging usage toward it.
4. Install `rustfmt` or run formatting in an environment where it is available.
5. Start moving board sensor ingestion from HLists into `SensorBus`.

### Important Design Caveats In The Current Slice

- `ROSFlight::run` still acts as the scheduler. A full `World`/scheduler type has not been introduced yet.
- The current slice still calls existing callback-style functions from the `ParamChanged` stage. This is intentional to preserve behavior while changing ordering and communication shape.
- `Rc::param_change_callback` still accepts `board` and `comm_manager` because it currently logs through the old logging pathway. This should later become a `LogPort`.
- `CommandManager::update_failsafe_config` is still called directly. This should later become a command-manager parameter reaction system with a typed context.
- Param request queue overflow is currently ignored with `let _ = ...`. A later pass should decide whether queue overflow sets an error, emits a statustext, drops newest, drops oldest, or increments diagnostics.
- Invalid parameter names still do not produce a NACK or statustext. This matches the previous incomplete behavior but should be revisited.
- `ParamSetRequested` currently carries both `ParamId` and raw `param_id_bytes` so the outgoing response can preserve the received MAVLink parameter id bytes.
- `CommResponse::ParamValue` currently stores the complete outgoing `ParamValueMsg`. A later design could instead store a semantic response and let the comm response system build the MAVLink message.

## Planned Core Migration After Parameter Path

After the parameter path is stable and committed, proceed through core in this order:

1. Rebuild dummy-board-based test infrastructure.
2. Add tests around the parameter flow:
   - `PARAM_SET` emits `ParamSetRequested`.
   - applying request mutates `Params`.
   - `ParamChanged` is visible to subscribers before `PARAM_VALUE` is sent.
   - `PARAM_SYSTEM_ID` updates `CommManager::sysid` before or during response send.
   - RC mapping params trigger RC remapping.
   - failsafe params trigger command-manager failsafe config update.
3. Introduce a clearer scheduler boundary around `ROSFlight::run`.
4. Move parameter reaction blocks out of `ROSFlight::run` into named systems:
   - `rc_on_param_changed`
   - `command_on_param_changed`
   - future estimator/controller/mixer reactions
5. Introduce `LogPort` and stop passing `CommManager` into RC callbacks for logging.
6. Introduce domain-specific event queues beyond params:
   - comm requests/responses
   - command events
   - sensor events
   - log events
   - state events if needed
7. Introduce named sensor resources:
   - `SensorBus`
   - `ProcessedSensors`
8. Convert sensor processors from HList mapping to named systems.
9. Convert telemetry to read from named sensor resources instead of `HListGet`.
10. Convert estimator inputs away from HLists.
11. Remove `Configuration::SculptIndices` and packet index associated types.
12. Remove HList dependencies from board and bodytype traits.
13. Delete `hlist.rs` only after no crates depend on it.

## Planned Crate Order

1. `rustflight_core`
2. `sim`
3. `pixracerpro`
4. `nucleo`
5. `stm_32`

The exact order after `pixracerpro` can change based on hardware priorities, but `core` must stabilize first.
