# Moving From HLists Toward Static ECS-Style Systems

## Working Agreement

This branch is experimental, but it should still be developed in small, inspectable steps.

User requirements for ongoing work:

- Work on the local branch `tmp_restructuring`.
- Keep a dense local git history.
- Commit frequently after each coherent, validated migration slice.
- Prefer small commits that are easy to bisect over large accumulated rewrites.
- Document the design intent, current progress, tests, and next steps as work proceeds.
- Keep this markdown useful as a handoff record so another engineer can continue without reconstructing context from memory.
- Mark each stage of the migration with what changed, why it changed, how it preserves ROSflight compatibility, and what remains.
- Add tests for each new component or scheduler handoff introduced in the next part of the stack.
- Run focused tests for each added component before moving on.
- Also run broader validation checks, especially `cargo check -p rustflight_core --lib` and `cargo check -p sim`, before committing a completed slice.
- Preserve ROSflight/rosflight_io wire behavior while improving internal causality.
- Responses that imply completed work should be sent after the relevant owning system completes that work.
- Avoid one-off patches that duplicate old shortcuts in the new architecture.
- Continue building the new World/ports/events path in parallel with the legacy HList path until the new path is verified enough to delete the old path.

Workflow notes for future agents:

- Before editing, inspect the current tree with `git status --short`.
- Use `rg` first for repository search.
- Use `apply_patch` for manual edits.
- Do not revert user changes.
- Treat each migration as a narrow slice:
  - add the event/resource/port shape,
  - move ownership of mutation to the domain system,
  - wire `World`,
  - keep legacy `ROSFlight` compatibility wired where needed,
  - add component tests,
  - add World handoff tests when the scheduler path changes,
  - update this document,
  - validate,
  - commit locally.
- `cargo fmt` is currently unavailable in this environment because `cargo-fmt` is not installed.
- If sandbox namespace errors occur on read-only shell commands, rerun the same command with the approved/escalated path rather than changing the workflow.

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

`rustflight_core/src/sensor_systems.rs`

- Adds `SensorProcessorSet`.
- Adds `process_sensor_bus`.
- This reuses the existing sensor processor objects but runs them over named `SensorBus` fields.
- This is a bridge away from HList mapping, not a rewrite of calibration logic.
- The default processor set uses the current real IMU and magnetometer processors plus passthrough processors for the remaining sensor types.
- Tests show a raw RC packet moving into `ProcessedSensors::rc` and being consumed from `SensorBus::rc`.

`rustflight_core/src/board.rs`

- Adds default `BoardTrait::update_sensor_bus`.
- The default implementation clears the named sensor bus.
- Existing boards keep compiling because this method has a default implementation.
- `sim` should be the first crate to override this method when moving board sensor ingestion off HLists.

### Validation Status

`cargo check -p rustflight_core --lib` passes after the initial parameter-path rewrite.

Focused unit checks for the new modules pass:

- `cargo test -p rustflight_core events::tests --lib`
- `cargo test -p rustflight_core param_system::tests --lib`
- `cargo test -p rustflight_core comm_manager::tests --lib`
- `cargo test -p rustflight_core param_reactions::tests --lib`
- `cargo test -p rustflight_core sensors::tests --lib`
- `cargo test -p rustflight_core sensor_systems::tests --lib`

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
5. Start moving `sim` board sensor ingestion from `update_sensors`/HLists into `update_sensor_bus`/`SensorBus`.

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

## Sim Migration Progress

`sim/src/board.rs`

- The old HList-oriented sim board has been deleted and replaced.
- The replacement board implements `BoardTrait` with `HNil` associated sensor types for the old HList path.
- `update_sensors` is now a no-op.
- `update_sensor_bus` is the active board-layer sensor path.
- The new method fills named `SensorBus` fields:
  - `imu`
  - `mag`
  - `baro`
  - `gnss`
  - `rc`
- Currently absent sim sources are left empty after `SensorBus::clear`:
  - `pitot`
  - `range`
  - `battery`
  - `attitude`
- MAVLink is exposed through UDP.
- Default MAVLink bind address: `127.0.0.1:14557`.
- Default MAVLink remote address: `127.0.0.1:14520`.
- These can be changed with:
  - `RUSTFLIGHT_MAVLINK_BIND`
  - `RUSTFLIGHT_MAVLINK_REMOTE`
- Zenoh is used as the board-layer bridge for ROS-shaped sensor messages.
- Default Zenoh endpoint: `tcp/127.0.0.1:7447`.
- This can be changed with `RUSTFLIGHT_ZENOH_ENDPOINT`.

`sim/src/bin/rustflight.rs`

- The old `ROSFlight`/HList sim binary has been deleted and replaced.
- The replacement binary instantiates the new `World`.
- It subscribes to `rust/tick` over Zenoh.
- On each tick it runs `World::run_comm_param_sensor_stages`.
- It does not instantiate `ROSFlight`.
- It does not carry a `Configuration` implementation or HList packet indices.
- The new sim currently exercises the safe scheduler subset:
  - MAVLink receive/decode
  - params/events/reactions/responses
  - Zenoh-backed sensor ingestion
  - named sensor processing
- Estimator/controller/mixer execution is still not wired in this new sim path.

ROSflight SIL reference findings:

- ROSflight documentation describes the firmware as a core library plus board implementations.
- In SIL, the same firmware core is used with a simulated board layer.
- Hardware talks to `rosflight_io` over serial USB/UART.
- ROSflight SIL talks to `rosflight_io` over UDP on localhost, simulating the serial link.
- `rosflight_io` is the ROS 2 gateway and converts between ROS 2 interfaces and MAVLink.
- The ROSflight docs explicitly describe running `rosflight_io` in simulation with `udp:=true`.
- Therefore, this branch should treat UDP MAVLink as the first SIL transport.
- A virtual USB/PTY transport can still be added later if a specific colcon/ROS integration requires a serial-looking device, but it is not the ROSflight default SIL path.
- In this RustFlight design, a Zenoh bridge is assumed to provide ROS-shaped sensor messages into the board layer.
- The board layer interprets those Zenoh payloads as ROS message structs and converts them into RustFlight packets.

Validation:

- `cargo check -p sim` passes.
- `cargo check -p rustflight_core --lib` passes.

Next sim step:

- Continue wiring the new `World` scheduler into the sim path:
  - telemetry
  - richer PWM publication diagnostics
  - explicit sim heartbeat/timing policy
  - board IO adapter tests
- Keep using UDP MAVLink for `rosflight_io` compatibility unless a serial/PTY requirement is confirmed.

## Scheduler Strategy Decision

Decision: build a parallel `World`/scheduler implementation beside the existing `ROSFlight` loop.

Rationale:

- The old HList-based loop works and provides a behavioral reference.
- Rewriting the old loop in place would create a mixed architecture and make regressions harder to isolate.
- A parallel path lets the new design grow with tests before it replaces flight behavior.
- Once the new path is stable, the duplicated old HList path can be deleted deliberately.

The new path must follow these rules:

- `World` owns resources.
- Scheduler methods borrow resources from `World`.
- Systems receive ports/context structs.
- Events connect modules.
- No HLists in the new scheduler path.
- No `&mut World` passed to systems.
- No hidden subscriber registry.

Migration shape:

1. Add `world` module in `rustflight_core`.
2. Add a parallel `World<B, BT, CI, PD>` type.
3. Reuse existing modules where possible:
   - `Params`
   - `CommManager`
   - `Rc`
   - `CommandManager`
   - `StateManager`
   - estimator/controller/mixer/PWM fields, even before all stages use them
4. Use named resources in the new path:
   - `SensorBus`
   - `ProcessedSensors`
   - `SensorProcessorSet`
   - `ParamEventQueues`
5. Keep old `ROSFlight::run` intact while the new scheduler matures.
6. Add tests against the new scheduler.
7. Migrate `sim` to instantiate/use the new scheduler first.
8. Migrate `pixracerpro` after `sim` proves the new path.
9. Delete HList-based path only after the replacement is proven.

Initial scheduler scope:

- comm receive
- message handling into events
- parameter apply
- parameter reactions
- deferred comm responses
- named sensor bus ingestion
- named sensor processing
- RC receive/run
- command manager run
- state manager run

Initial scheduler non-goals:

- Do not change telemetry source yet.
- Do not delete HList code yet.
- Do not change runtime behavior of existing `ROSFlight::run`.

Acceptance criteria for first `World` slice:

- `rustflight_core` library checks.
- focused `world` tests pass.
- existing focused parameter/sensor tests still pass.
- old `ROSFlight` path remains available.

Acceptance criteria for each additional scheduler/component slice:

- Each newly added component path needs focused tests before that slice is considered complete.
- Tests should exercise the component in isolation where possible.
- Tests should also exercise the component through the new `World` scheduler when the scheduler is responsible for connecting it to other modules.
- Documentation must list which tests were added or run for the slice.
- A slice should not advance to the next stack layer until its focused tests pass, unless a legacy test blocker is explicitly documented.

## World Scheduler Progress

`rustflight_core/src/world.rs`

- Adds parallel `World<B, BT, CI, PD>`.
- This is separate from the existing `ROSFlight` type.
- `World` owns:
  - board
  - params
  - param event queues
  - comm manager
  - named raw sensor bus
  - named processed sensors
  - named sensor processor set
  - RC
  - command manager
  - state manager
  - calibration flags
  - estimator/controller/mixer/PWM fields for future stages
- Adds `World::init`.
- Adds `World::run_comm_param_sensor_stages`.
- Adds `World::run_comm_param_sensor_stages_only`.
- Adds `World::run_rc_command_state_stages`.
- Adds `World::run_control_stages_if_new_imu`.
- The first scheduler method runs only:
  - comm receive
  - old message decode into events
  - param apply
  - parameter reactions
  - deferred comm responses
  - `BoardTrait::update_sensor_bus`
  - `process_sensor_bus`
  - named RC packet handoff into `Rc`
  - RC manager run
  - command manager run
  - state manager run
  - estimator update when a new named IMU packet is available
  - estimator health propagation into the state manager
  - controller update from the current command/state/params
  - mixer update
  - PWM command output
- It does not change the existing `ROSFlight::run` path.

`rustflight_core/src/estimator.rs`

- Adds `NamedEstimator`.
- This adapts estimators away from HList inputs in the new scheduler path.
- The trait takes `ProcessedSensors`, `Params`, and `dt`.
- It returns the estimator state type.

`rustflight_core/src/estimator/quad_estimator.rs`

- Implements `NamedEstimator` for `QuadEstimator`.
- The current implementation adapts `ProcessedSensors::{imu, mag}` back into the existing estimator input HList internally.
- This keeps estimator math behavior intact while moving the scheduler boundary to named resources.
- Adds `Default` for `AttitudeState` so `World` can own a last-known state before the first estimator update.

Test support:

- Adds `TestPwm` inside `world` tests.
- `TestPwm` records send count and last command payload length for scheduler verification.
- Adds `world_scheduler_runs_deferred_param_pipeline`.
- The test proves the new world scheduler can process `PARAM_SET` through the deferred event path and update `SYS_ID`.
- Adds `world_scheduler_processes_named_rc_packet`.
- The test proves a named `ProcessedSensors::rc` packet can flow through the RC/command/state scheduler stage without raising `RC_LOST`.
- Adds `world_control_stage_runs_once_per_imu_timestamp`.
- The test proves the estimator/controller/mixer/PWM stage runs on a new IMU timestamp, does not repeat for the same timestamp, stores actuator output, and calls PWM exactly once per new IMU sample.

New testing requirement:

- For each component added in the next part of the stack, add or run a focused test before moving to the next component.
- Component tests should cover the local system behavior.
- Scheduler tests should cover the handoff between components.
- This applies to telemetry, PWM publishing, board IO adapters, sim transport, and later pixracerpro migration.

## Sim PWM Output Progress

`sim/src/pwm.rs`

- The parallel sim already publishes PWM output to Zenoh on `sim/pwm_output`.
- The `World` control stage now reaches this driver through `PwmDriver::send_commands`.
- Added focused component tests for the sim PWM output mapping.
- These tests instantiate the driver with an in-memory channel rather than a live Zenoh session.
- This keeps the test deterministic and local to the component.

Tests added:

- `pwm::tests::send_commands_scales_clamps_and_publishes_pwm_output`
  - Proves normalized commands clamp to `[0.0, 1.0]`.
  - Proves commands map to the expected `1000..2000` microsecond output range.
  - Proves `send_commands` queues a ROS-shaped `PwmOutput` message with the board timestamp.
- `pwm::tests::disable_sets_channel_to_minimum_pwm`
  - Proves disabling a channel sets that channel back to `1000`.

Validation:

- `cargo test -p sim pwm::tests --lib` passes.
- `cargo check -p sim` passes.

## Named Telemetry Progress

`rustflight_core/src/comm_manager.rs`

- Adds `CommManager::send_named_telemetry_streams`.
- This is the telemetry equivalent of the old HList-based `send_telemetry_streams`.
- It accepts `ProcessedSensors` directly instead of a board-specific `ProcessedSensorSet` plus HList index configuration.
- It keeps the same MAVLink-facing message types:
  - heartbeat
  - status
  - small IMU
  - attitude quaternion
  - barometer
  - magnetometer
  - range
  - battery status
  - GNSS
  - RC channels
  - output raw
- The old HList telemetry method remains in place as the behavioral reference.
- The new method is now called from the `World` control stage after estimator/controller/mixer/PWM output.

`rustflight_core/src/test_support.rs`

- Extends `RecordingCommLink` to count telemetry messages.
- Records the last output-raw message so telemetry payload mapping can be asserted in tests.
- Adds a test-only `CommManager::comm_link` accessor to inspect the recording link without making the production field public.

Tests added or extended:

- `comm_manager::tests::named_telemetry_sends_sensor_state_and_output_messages`
  - Proves named telemetry sends heartbeat, status, IMU, attitude, and output-raw messages.
  - Proves output-raw telemetry preserves the scheduler timestamp and actuator command payload.
- `world::tests::world_control_stage_runs_once_per_imu_timestamp`
  - Extended to prove the `World` control stage hands actuator output into named telemetry exactly once per new IMU sample.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p sim` passes.

## Board Boundary Progress

`rustflight_core/src/board.rs`

- Adds `BoardIo`.
- `BoardIo` is the smaller board-facing trait for the new `World` path.
- It contains only named/runtime IO operations:
  - `update_sensor_bus`
  - serial receive/transmit
  - clocks
  - optional test pins
- `BoardIo` has no HList-associated types.
- Existing `BoardTrait` remains for the legacy HList path.
- A blanket `impl<T: BoardTrait> BoardIo for T` keeps old boards working while the new path migrates.

`rustflight_core/src/world.rs`

- `World` now requires `B: BoardIo` instead of `B: BoardTrait`.
- This removes HList-associated board types from the `World` type boundary.

`rustflight_core/src/comm_manager.rs`

- `CommManager` now requires `B: BoardIo`.
- The legacy HList telemetry method keeps a local `B: BoardTrait` bound because it still accepts `B::ProcessedSensorSet`.
- The named telemetry method uses `ProcessedSensors` and only needs `BoardIo`.

`rustflight_core/src/pwm.rs`

- `PwmDriver::flush` and `PwmDriver::send_commands` now accept `B: BoardIo`.
- This lets the new scheduler output path use a board interface without HList-associated types.

`rustflight_core/src/rc.rs`

- Removes the unnecessary `BoardTrait` bound from `Rc::init`.
- RC initialization currently reads params and does not need board access.

`rustflight_core/src/comm_manager/comm_link_trait.rs`

- `CommInterface` now accepts `B: BoardIo`.
- The MAVLink implementation also now only requires `BoardIo`.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p sim pwm::tests --lib` passes.
- `cargo check -p sim` passes.

## Sim Board Boundary Progress

`sim/src/board.rs`

- The sim board now implements `BoardIo` directly.
- It no longer implements `BoardTrait`.
- It no longer declares `RawSensorSet`, `ProcessedSensorSet`, or `ProcessorHList`.
- It no longer imports `HNil`.
- This makes the new sim path independent of the legacy HList board boundary.

`sim/src/pwm.rs`

- The PWM component tests now use a direct `BoardIo` test board.
- The test board no longer needs `BoardTrait` or `HNil`.

Validation:

- `cargo check -p sim` passes.
- `cargo test -p sim pwm::tests --lib` passes.
- `rg -n "HNil|BoardTrait|RawSensorSet|ProcessedSensorSet|ProcessorHList" sim/src` returns no matches.

## Named Estimator Progress

`rustflight_core/src/estimator/quad_estimator.rs`

- Extracts the quad estimator math into `QuadEstimator::estimate_packets`.
- The legacy HList `Estimator::estimate` entry point now delegates into `estimate_packets`.
- The new `NamedEstimator::estimate_named` entry point also delegates into `estimate_packets`.
- The named estimator path no longer constructs `HCons(..., HNil)` internally.
- This preserves estimator behavior while removing HList from the new scheduler-facing estimator path.

Tests added:

- `estimator::quad_estimator::tests::named_estimator_matches_legacy_hlist_entrypoint`
  - Proves the named estimator entry point returns the same state as the legacy HList entry point for the same IMU packet.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo test -p rustflight_core estimator::quad_estimator::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p sim` passes.

## World Sensor Health Progress

`rustflight_core/src/world.rs`

- Adds IMU health tracking to the new `World` path.
- Mirrors the legacy 100 ms `IMU_NOT_RESPONDING` timeout.
- Records the last scheduler time at which a processed IMU packet was present.
- Clears `IMU_NOT_RESPONDING` when processed IMU data is available.
- Raises `IMU_NOT_RESPONDING` when no processed IMU has been seen for more than 100 ms.
- Preserves the legacy calibration behavior:
  - if the state machine is calibrating and gyro calibration is not active, insert `CalibrationFlags::GYRO`
  - after sensor processing removes `CalibrationFlags::GYRO`, send `Event::CALIBRATION_COMPLETE`

Tests added:

- `world::tests::world_sensor_health_sets_and_clears_imu_timeout`
  - Proves the new World path raises `IMU_NOT_RESPONDING` after the timeout.
  - Proves the error clears when a processed IMU packet is present again.

Validation:

- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## Calibration ACK Causality Progress

Design correction:

- Calibration command receipt must not report success.
- Calibration command receipt starts calibration work.
- The sensor/calibration system owns completion.
- Communication ACK success is sent only after the relevant calibration flag has been cleared by processing.
- This matches the broader plan: requests emit work, systems perform work, and responses are emitted after work completes.

`rustflight_core/src/comm_manager.rs`

- Adds a pending calibration ACK slot in `CommManager`.
- Calibration commands now set the corresponding `CalibrationFlags` and store the pending command.
- Calibration commands no longer send immediate success ACKs.
- Adds `send_completed_calibration_ack`.
- `send_completed_calibration_ack` sends `RosflightCmdSuccess` only when the pending command's flag is no longer active.
- Non-calibration commands still use the immediate command ACK path.

`rustflight_core/src/world.rs`

- After sensor processing and sensor health/calibration updates, World calls `send_completed_calibration_ack`.
- This places the success response after the stage that can observe calibration completion.

`rustflight_core/src/test_support.rs`

- `RecordingCommLink` now records command ACK count and last command ACK.

Tests added:

- `comm_manager::tests::calibration_command_ack_is_deferred_until_flag_clears`
  - Proves a gyro calibration command sets the gyro calibration flag.
  - Proves no ACK is sent at command receipt time.
  - Proves success ACK is sent only after the gyro flag clears.
- `world::tests::world_sends_calibration_ack_after_calibration_flag_clears`
  - Proves the scheduler path preserves this deferred ACK behavior.

PWM note:

- A draft direct PWM enable/disable polling change was intentionally not kept.
- PWM should follow the same design principle as calibration:
  - state/command facts should cause scheduled output intent
  - PWM hardware/sim output should react in its own stage
  - tests should verify transition-driven behavior, not a loop shortcut

Validation:

- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## PWM Output State Progress

Design correction:

- PWM enable/disable should not be a hidden loop shortcut.
- PWM output state should follow explicit state-machine facts.
- The scheduler should call a PWM output system after state updates.
- The PWM system should only touch hardware/sim output when the desired output state changes.

`rustflight_core/src/pwm_system.rs`

- Adds `PwmOutputState`.
- Adds `sync_pwm_output_state`.
- `sync_pwm_output_state` reads `StateManager::is_armed`.
- It enables PWM only on a transition from disabled to armed.
- It disables PWM only on a transition from enabled to not armed.
- On disable, it flushes the PWM driver so hardware/sim output receives the disabled state.
- It returns `Ok(true)` only when it changed PWM output state.

`rustflight_core/src/world.rs`

- Adds `pwm_output: PwmOutputState` to `World`.
- Initializes it from `pwm.is_enabled`.
- Adds `World::run_pwm_output_stage`.
- Schedules `run_pwm_output_stage` after RC, command, and state-manager updates.
- This keeps the PWM side effect after the state facts are known.

Tests added:

- `pwm_system::tests::pwm_output_state_enables_and_disables_only_on_state_transitions`
  - Proves no PWM call happens when desired state is unchanged.
  - Proves arming enables once.
  - Proves repeated sync while armed does not enable again.
  - Proves disarming disables once and flushes once.
- `world::tests::world_pwm_output_stage_follows_armed_state_transitions`
  - Proves the World scheduler handoff follows real state-machine arm/disarm transitions.
  - The test uses calibrated gyro params and valid failsafe throttle so arming is not forced around preflight rules.

Validation:

- `cargo test -p rustflight_core pwm_system::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## PWM Command Write Progress

Design correction:

- The control stage should compute actuator commands.
- The PWM system should decide whether those commands reach PWM hardware/sim output.
- Telemetry can still report computed actuator commands.
- PWM command writes should be gated by explicit `PwmOutputState`.

`rustflight_core/src/pwm_system.rs`

- Adds `write_pwm_commands`.
- `write_pwm_commands` sends commands only when `PwmOutputState` is enabled.
- It returns `false` when output is disabled and no PWM driver write occurred.
- It returns `true` after writing commands to the PWM driver.

`rustflight_core/src/world.rs`

- `run_control_stages_if_new_imu` now delegates PWM writes to `write_pwm_commands`.
- The control stage still stores `latest_actuator_commands`.
- The control stage still sends named telemetry with the computed actuator commands.
- The control-stage test now arms through the real state-machine path before expecting PWM command writes.

Tests added or extended:

- `pwm_system::tests::write_pwm_commands_only_writes_when_output_enabled`
  - Proves disabled output prevents PWM command writes.
  - Proves enabled output writes exactly once.
- `world::tests::world_control_stage_runs_once_per_imu_timestamp`
  - Extended so PWM command writes are expected only after the World PWM output stage has enabled output through a real armed state.

Validation:

- `cargo test -p rustflight_core pwm_system::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## Current Handoff Notes

Current design rule:

- Communication code should decode requests and emit events.
- Domain systems should mutate domain resources.
- Completion systems should emit responses after work is actually complete.
- World owns resources and schedules stages.
- Systems should receive narrow ports/resources, not `&mut World`.

Current stage ownership:

- `CommManager::process_incoming_messages`
  - Owns transport decode into stored incoming messages.
  - Should not mutate flight resources.
- `CommManager::act_on_messages`
  - Owns conversion from incoming MAVLink/ROSflight messages into internal events or immediate responses for truly immediate commands.
  - Parameter set now emits `ParamSetRequested`.
  - Calibration commands are being moved to emit `CalibrationRequested`.
  - It should not directly set calibration flags in the new path.
- `param_system::apply_param_requests`
  - Owns parameter mutation.
  - Emits `ParamChanged`.
  - Emits deferred parameter response only after mutation.
- `param_reactions`
  - Owns module reactions to `ParamChanged`.
  - RC and command manager no longer receive direct parameter callbacks from comms.
- `command_system::apply_calibration_requests`
  - Owns conversion from calibration request events into `CalibrationFlags`.
  - This is the current in-progress command slice.
- `sensor_systems::process_sensor_bus`
  - Owns raw named sensor packets to processed named sensor packets.
  - Calibration processors clear calibration flags when calibration completes.
- `World::update_sensor_health_and_calibration`
  - Owns IMU timeout/error propagation.
  - Owns state-machine `CALIBRATION_COMPLETE` after calibration flags clear.
  - Calls `CommManager::send_completed_calibration_ack` after completion is observable.
- `pwm_system::sync_pwm_output_state`
  - Owns PWM enable/disable transitions from state-machine armed facts.
- `pwm_system::write_pwm_commands`
  - Owns gated PWM command writes.
  - Control computes actuator commands; PWM system decides if they reach output.

Most recent command slice:

- Added `CalibrationRequested`.
- Added `CommandEventQueues`.
- Added `command_system::apply_calibration_requests`.
- `CommManager::act_on_messages` now pushes `CalibrationRequested` instead of mutating `CalibrationFlags`.
- `World` and legacy `ROSFlight` now drain calibration requests into calibration flags.
- Tests prove:
  - comm command receipt emits calibration request and does not set flags directly
  - command system sets the right flags
  - World still sends calibration success ACK only after flags clear

What to check if resuming from here:

- Run `cargo test -p rustflight_core command_system::tests --lib`.
- Run `cargo test -p rustflight_core comm_manager::tests --lib`.
- Run `cargo test -p rustflight_core world::tests --lib`.
- Run `cargo check -p rustflight_core --lib`.
- Run `cargo check -p sim`.
- If these pass, the command-event slice should be commit-ready.
- If they fail, likely places to inspect are:
  - `rustflight_core/src/events.rs`
  - `rustflight_core/src/command_system.rs`
  - `rustflight_core/src/comm_manager.rs::act_on_messages`
  - `rustflight_core/src/world.rs::run_comm_param_sensor_stages_only`
  - legacy compatibility wiring in `rustflight_core/src/rosflight.rs`

## Command Event Progress

Design correction:

- Calibration commands should follow the same request/work/response shape as parameter changes.
- Comms should not directly mutate `CalibrationFlags`.
- Comms should emit a calibration request event.
- A command/calibration system should apply that request to calibration resources.
- Completion ACKs should still wait until processing clears the relevant calibration flag.

`rustflight_core/src/events.rs`

- Adds `CalibrationRequested`.
- Adds `CommandEventQueues`.
- Adds fixed-capacity calibration request queue storage.

`rustflight_core/src/command_system.rs`

- Adds `CalibrationRequestCtx`.
- Adds `apply_calibration_requests`.
- This system drains calibration request events and sets the requested `CalibrationFlags`.

`rustflight_core/src/comm_manager.rs`

- `act_on_messages` now receives `CommandEventQueues`.
- Calibration commands push `CalibrationRequested`.
- Calibration commands no longer mutate `CalibrationFlags` directly.
- Pending calibration ACK behavior remains deferred until completion.

`rustflight_core/src/world.rs`

- Adds `command_events: CommandEventQueues`.
- Schedules `command_system::apply_calibration_requests` after comm message handling and before sensor processing.
- Existing calibration completion ACK behavior continues after sensor processing observes cleared flags.

`rustflight_core/src/rosflight.rs`

- Adds legacy compatibility wiring so the old loop also drains `CommandEventQueues`.
- This keeps the legacy path compiling while the new World path matures.

Tests added or updated:

- `command_system::tests::apply_calibration_requests_sets_requested_flags`
  - Proves calibration request events set the expected calibration flags.
- `comm_manager::tests::calibration_command_ack_is_deferred_until_flag_clears`
  - Updated to prove comms emits a calibration request first and does not directly set flags.
  - Then applies the request through `command_system`.
  - Still proves ACK is sent only after the flag clears.
- `world::tests::world_sends_calibration_ack_after_calibration_flag_clears`
  - Proves the full World scheduler path still defers ACK until completion.

Validation:

- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core pwm_system::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## Offboard Command Event Progress

Design correction:

- Offboard control messages should not mutate `CommandManager` directly from comms.
- Comms should emit an offboard control request event with the receipt timestamp.
- The command system should apply the request to `CommandManager`.
- World should schedule that command system before RC/command/state/control stages use the command state.

`rustflight_core/src/events.rs`

- Adds `OffboardControlRequested`.
- Adds fixed-capacity offboard control request queue storage to `CommandEventQueues`.

`rustflight_core/src/command_system.rs`

- Adds `OffboardControlCtx`.
- Adds `apply_offboard_control_requests`.
- This system drains offboard request events and calls `CommandManager::set_new_offboard_command`.

`rustflight_core/src/comm_manager.rs`

- `act_on_messages` now pushes `OffboardControlRequested` when an offboard control message is received.
- It no longer calls `CommandManager::set_new_offboard_command` directly.
- `act_on_messages` no longer receives `&mut CommandManager`.
- This narrows comms access to the command subsystem and improves blame/diagnosis when command state changes.

`rustflight_core/src/world.rs`

- Schedules `apply_offboard_control_requests` in `run_comm_param_sensor_stages_only`.
- The offboard command request is applied before later command/state/control stages consume command state.

`rustflight_core/src/rosflight.rs`

- Adds legacy compatibility scheduling for `apply_offboard_control_requests`.

Tests added:

- `command_system::tests::apply_offboard_control_requests_updates_command_manager`
  - Proves the command system applies offboard request events to `CommandManager`.
- `comm_manager::tests::offboard_control_message_emits_command_event`
  - Proves comms emits an offboard event and does not directly activate offboard command state.
- `world::tests::world_applies_offboard_control_command_event`
  - Proves the World scheduler drains the event and updates command state.

Validation:

- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core pwm_system::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## Compile-Time Checkability Status

Current assessment:

- The remake is moving in the intended compile-time-checkable direction.
- The most important improvement is narrowing who can mutate each resource.
- `World` owns resources explicitly.
- Systems receive narrow resource/context structs instead of `&mut World`.
- Event queues carry requests between systems.
- Domain systems mutate domain resources.
- Completion/response systems send responses after work is actually complete.

Compile-time boundaries already improved:

- `World` uses `BoardIo`, not the HList-bearing `BoardTrait`.
- The new sim board implements `BoardIo` directly and has no HList scaffolding.
- `CommManager::act_on_messages` no longer receives `&mut CommandManager`.
- Therefore comms cannot directly mutate command state.
- Calibration commands now emit `CalibrationRequested`.
- Therefore comms no longer directly sets `CalibrationFlags`.
- Offboard control messages now emit `OffboardControlRequested`.
- Therefore comms no longer directly calls `CommandManager::set_new_offboard_command`.
- Parameter set requests emit `ParamSetRequested`.
- Therefore comms no longer mutates params for `PARAM_SET`.
- PWM enable/disable is handled by `pwm_system::sync_pwm_output_state`.
- PWM command writes are handled by `pwm_system::write_pwm_commands`.
- Therefore control computes actuator commands, while PWM output policy is owned by the PWM system.

Still not final:

- `CommandEventQueues` is still a shared queue bundle.
- Later we may split queues into narrower ports per system when the shape stabilizes.
- `CalibrationFlags` is still a compact active-calibration resource.
- It is acceptable for now because sensor processors need a persistent active state across many samples.
- Later this should likely become a richer `CalibrationState` with request/active/completed/failed fields.
- `CommManager::act_on_messages` still receives `&mut Params`.
- Some command paths still mutate params directly.
- The current in-progress target is `SetParamDefaults`, which should become an event and deferred ACK.
- The old HList `ROSFlight` path still exists for reference and compatibility.
- The new `World` path is increasingly independent, but deletion of HList should wait until parity is proven.

Current compile-time goal:

- If a module should not mutate a resource, its system function should not receive `&mut` access to that resource.
- If a module should only request work, it should receive an event emit port or event queue, not the target resource.
- If a response depends on completed work, the response should be emitted after the completing system runs.
- Tests should cover each component system and the scheduler handoff through `World`.

Testing detail:

- Running `world::tests` pulled in RC logging, which uses `critical-section`.
- Host tests need a critical-section implementation.
- Added `critical-section = { version = "1.2.0", features = ["std"] }` under `rustflight_core` dev-dependencies.
- This is test-only support for host tests and does not change the embedded library check.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core param_system::tests --lib` passes.
- `cargo test -p rustflight_core sensor_systems::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core param_reactions::tests --lib` passes.
- `cargo check -p sim` passes.

## Param Defaults Command Event Progress

Reason for this change:

- `SetParamDefaults` is a ROSflight command path, so it should preserve ROSflight wire compatibility.
- The old shape mutated params inside comms and immediately reported command success.
- That is the same category of shortcut as the earlier parameter-set and calibration problems.
- The new rule is that comms records intent, the owning system performs the mutation, and the success ACK is sent only after that mutation has completed.

Design now implemented:

- `CommManager` emits `ParamDefaultsRequested`.
- `CommManager` no longer calls `Params::set_defaults` from the command parser.
- `command_system::apply_param_defaults_requests` drains the request queue and calls `Params::set_defaults`.
- The command system returns the applied command so the response stage can prove that the command was actually handled.
- `CommManager::send_completed_param_defaults_ack` sends the ROSflight `RosflightCmdAckMsg` only after the apply stage reports completion.
- The ACK remains ROSflight-compatible: it still uses `RosflightCmd::SetParamDefaults` and `RosflightCmdSuccess`.

Compile-time boundary improvement:

- Comms still needs read/query access to params for MAVLink parameter messages.
- Comms no longer has authority over the default-reset mutation path.
- The reset mutation belongs to the command-system stage, which receives a narrow `ParamDefaultsCtx`.
- This matches the broader ports pattern: request port into a fixed-size queue, narrow mutable resource access in the owning system, response after completion.

Files changed in this slice:

- `rustflight_core/src/events.rs`
  - Adds `ParamDefaultsRequested`.
  - Adds a fixed-capacity param-defaults request queue to `CommandEventQueues`.
- `rustflight_core/src/command_system.rs`
  - Adds `ParamDefaultsCtx`.
  - Adds `apply_param_defaults_requests`.
  - Adds component coverage for default reset application.
- `rustflight_core/src/comm_manager.rs`
  - Emits `ParamDefaultsRequested` for `RosflightCmd::SetParamDefaults`.
  - Stores a pending defaults ACK.
  - Sends the ACK after the scheduler confirms that defaults were applied.
- `rustflight_core/src/world.rs`
  - Schedules default-reset request application in the World comm/param/sensor stage.
  - Sends the deferred defaults ACK after the apply stage.
- `rustflight_core/src/rosflight.rs`
  - Mirrors the same scheduling step in the legacy compatibility path.

Tests added:

- `command_system::tests::apply_param_defaults_requests_resets_params_and_reports_command`
  - Proves the command system owns the reset mutation.
- `comm_manager::tests::set_param_defaults_emits_request_and_defers_ack`
  - Proves comms emits a request, does not reset params immediately, and sends ACK only after completion.
- `world::tests::world_applies_param_defaults_and_sends_ack_after_apply`
  - Proves the scheduler drains the request, resets params, and sends the ROSflight-compatible success ACK.

Validation:

- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core pwm_system::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Next planned migration target:

- Continue removing direct mutation shortcuts from command and communication handling.
- Prioritize paths where ROSflight expects a command response or externally visible behavior after completed work.
- Keep each new step independently tested at the component level and through `World`.

## Param Request List Event Progress

Reason for this change:

- `PARAM_REQUEST_LIST` is a ROSflight/rosflight_io compatibility path.
- The old implementation made `CommManager::act_on_messages` own the active parameter iterator and stream parameter values directly.
- That forced the comm parser to receive `&mut Params` and a mutable `params_iter` even though comms should only recognize the inbound request.
- The new architecture should keep communication parsing separate from parameter-list streaming.

Design now implemented:

- `CommManager` emits `ParamListRequested` when a `PARAM_REQUEST_LIST` message arrives.
- `CommManager` no longer owns or advances a parameter iterator.
- `CommManager::act_on_messages` no longer receives `&mut Params`.
- `CommManager::act_on_messages` no longer receives `&mut Option<ParamIter>`.
- `param_system::ParamListState` owns the active parameter-list stream state.
- `param_system::service_param_list_requests` drains list requests, reads params through `ParamsReadPort`, and emits one `CommResponse::ParamValue` per scheduler call.
- `CommManager::send_comm_responses` remains the wire-output stage that sends `PARAM_VALUE` messages through the configured comm link.

ROSflight compatibility:

- The external behavior remains a stream of `PARAM_VALUE` messages in response to `PARAM_REQUEST_LIST`.
- The stream still sends one parameter value at a time across scheduler calls, matching the old incremental behavior.
- The wire message type and parameter payload shape are unchanged.

Compile-time boundary improvement:

- Comms can no longer mutate params from `act_on_messages`.
- Parameter-list streaming now belongs to the parameter system.
- The parameter-list system receives only a read port to params and mutable access to its own `ParamListState`.
- This moves another communication-driven behavior into the ports/events/scheduler pattern.

Files changed in this slice:

- `rustflight_core/src/events.rs`
  - Adds `ParamListRequested`.
  - Adds a fixed-capacity list request queue to `ParamEventQueues`.
- `rustflight_core/src/param_system.rs`
  - Adds `ParamListState`.
  - Adds `ParamListCtx`.
  - Adds `service_param_list_requests`.
- `rustflight_core/src/comm_manager.rs`
  - Emits list request events.
  - Removes param iterator ownership from `act_on_messages`.
  - Narrows the `act_on_messages` signature.
- `rustflight_core/src/world.rs`
  - Owns `ParamListState`.
  - Schedules `service_param_list_requests` before comm responses are sent.
- `rustflight_core/src/rosflight.rs`
  - Mirrors the same compatibility scheduling in the legacy path.

Tests added:

- `param_system::tests::service_param_list_requests_streams_one_param_per_call`
  - Proves the parameter system owns the streaming state and emits one response per call.
- `comm_manager::tests::param_request_list_emits_request_without_streaming_from_comms`
  - Proves comms only emits the request and does not send parameter values directly.
- `world::tests::world_scheduler_streams_param_request_list_through_param_system`
  - Proves the World scheduler accepts the request, streams through the parameter system, and sends ROSflight-compatible parameter values.

Validation:

- `cargo test -p rustflight_core param_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## Dedicated Comm Response Queue Progress

Reason for this change:

- `CommResponse` had grown beyond parameter acknowledgements.
- It now carries parameter values, command ACKs, and version messages.
- Keeping that response queue inside `ParamEventQueues` made the ownership boundary misleading.
- Before adding board/persistence command responses, comm responses need their own resource.

Design now implemented:

- Added `CommEventQueues`.
- Moved the fixed-capacity response queue to `CommEventQueues::responses`.
- `ParamEventQueues` now owns only parameter requests and parameter change events.
- `World` owns `comm_events` separately from `param_events`.
- Legacy `ROSFlight` also owns `comm_events` separately for compatibility.
- Parameter systems emit responses through `CommEventQueues`.
- `CommManager::act_on_messages` queues immediate command responses through `CommEventQueues`.
- `CommManager::send_comm_responses` drains `CommEventQueues`.

Compile-time boundary improvement:

- Parameter systems do not need mutable access to all parameter events to send responses.
- Communication responses now have an explicit resource boundary.
- This prepares the next board/persistence command work, where board systems can emit command ACKs without pretending they are parameter events.

Files changed in this slice:

- `rustflight_core/src/events.rs`
  - Adds `CommEventQueues`.
  - Removes response storage from `ParamEventQueues`.
- `rustflight_core/src/world.rs`
  - Adds a `comm_events` resource.
  - Wires parameter response emit ports to `comm_events.responses`.
  - Sends responses from `comm_events`.
- `rustflight_core/src/rosflight.rs`
  - Mirrors the same resource split in the legacy path.
- `rustflight_core/src/comm_manager.rs`
  - Accepts `CommEventQueues` in command parsing.
  - Drains `CommEventQueues` in response sending.
  - Updates tests to use the distinct queue.

Validation:

- `cargo test -p rustflight_core events::tests --lib` passes.
- `cargo test -p rustflight_core param_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Next planned migration target:

- Add explicit board/persistence command request events for `ReadParams`, `WriteParams`, `Reboot`, and `RebootToBootloader`.
- Initially these can return unsupported/failed if no board persistence or reboot hooks exist.
- The important architectural step is that comms emits request intent and a board/system stage owns completion and ACK timing.

## Board Command Event Progress

Reason for this change:

- `ReadParams`, `WriteParams`, `Reboot`, and `RebootToBootloader` were still inline placeholders inside command parsing.
- The old behavior sent a failed ACK immediately because the commands were not implemented.
- The new architecture should still report failure on unsupported boards, but the ACK should come from the system that owns board/persistence behavior.

Design now implemented:

- Added default board hooks to `BoardIo` and `BoardTrait`:
  - `read_params`
  - `write_params`
  - `reboot`
  - `reboot_to_bootloader`
- The default implementation returns `false`, meaning unsupported or not completed.
- Added `BoardCommandRequested`.
- Added a fixed-capacity board command request queue to `CommandEventQueues`.
- `CommManager` emits board command requests for:
  - `RosflightCmd::ReadParams`
  - `RosflightCmd::WriteParams`
  - `RosflightCmd::Reboot`
  - `RosflightCmd::RebootToBootloader`
- `CommManager` no longer immediately ACKs those commands from the parser when a request is queued.
- `command_system::apply_board_command_requests` owns calling the board hooks and emits `CommResponse::CmdAck`.
- `World` schedules board command application before comm responses are sent.
- Legacy `ROSFlight` mirrors the same scheduling step for compatibility.

ROSflight compatibility:

- Unsupported board commands still result in `RosflightCmdFailed`.
- The wire ACK type remains `RosflightCmdAckMsg`.
- The ACK timing is now after the board-command stage runs, which matches the architecture rule that success/failure should reflect completed work.
- Future board implementations can override the hooks and return success only after the board operation has completed.

Compile-time boundary improvement:

- Comms can no longer be the place where board/persistence command behavior is implemented.
- Board command completion belongs to a command-system stage with explicit access to:
  - the board,
  - params,
  - board command request queue,
  - comm response queue.
- This keeps board mutation and response causality localized.

Files changed in this slice:

- `rustflight_core/src/board.rs`
  - Adds default board command hooks to `BoardIo` and `BoardTrait`.
  - Forwards hooks through the legacy `BoardTrait` to `BoardIo` blanket implementation.
- `rustflight_core/src/events.rs`
  - Adds `BoardCommandRequested`.
  - Adds `board_command_requests` to `CommandEventQueues`.
- `rustflight_core/src/command_system.rs`
  - Adds `BoardCommandCtx`.
  - Adds `apply_board_command_requests`.
- `rustflight_core/src/comm_manager.rs`
  - Emits board command requests for the board/persistence command arms.
  - Defers ACK when the request is queued.
- `rustflight_core/src/world.rs`
  - Schedules board command requests through the new command-system stage.
- `rustflight_core/src/rosflight.rs`
  - Mirrors the same scheduling step in the legacy path.

Tests added:

- `command_system::tests::apply_board_command_requests_reports_unsupported_as_failed_ack`
  - Proves unsupported board hooks become failed command ACK responses.
- `comm_manager::tests::board_command_emits_request_and_defers_ack`
  - Proves comms emits request intent and does not ACK immediately.
- `world::tests::world_routes_board_command_and_acks_unsupported_after_apply_stage`
  - Proves World drains the request and sends the failed ACK after the board-command stage.

Validation:

- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Next planned migration target:

- Decide whether to implement simulated board persistence for `ReadParams`/`WriteParams` or leave board hooks unsupported until the pixracerpro migration.
- Move `RcCalibration` out of the inline command parser next, because it needs a persistent RC calibration state rather than a placeholder failed ACK.

## RC Trim Calibration Event Progress

Source-compatibility note:

- Before implementing this slice, we checked ROSflight documentation for current main/git-main behavior.
- ROSflight does not perform software calibration of RC transmitter endpoints.
- ROSflight does expose `calibrate_rc_trim` through `rosflight_io`.
- That service instructs firmware to calibrate RC trim values.
- The documented behavior is to use current transmitter trim offsets to compute equilibrium/feed-forward torques.
- Those torques are represented by `X_EQ_TORQUE`, `Y_EQ_TORQUE`, and `Z_EQ_TORQUE`.
- Therefore RustFlight should not implement generic RC endpoint calibration for `RosflightCmd::RcCalibration`.
- It should implement RC trim calibration.

Reason for this change:

- The starting RustFlight code had the `RcCalibration` command enum and MAVLink mapping, but the command arm was a placeholder that always failed.
- `X_EQ_TORQUE`, `Y_EQ_TORQUE`, and `Z_EQ_TORQUE` existed but were not written by any RC trim calibration path.
- The quad controller also was not consuming those equilibrium torque parameters.
- To match ROSflight behavior, the command must set equilibrium torque params and those params must affect controller output.

Design now implemented:

- Added `RcTrimCalibrationRequested`.
- Added a fixed-capacity RC trim calibration request queue to `CommandEventQueues`.
- `CommManager` emits `RcTrimCalibrationRequested` for `RosflightCmd::RcCalibration`.
- `CommManager` does not ACK the command immediately when the request is queued.
- `command_system::apply_rc_trim_calibration_requests` owns the work:
  - it reads current processed RC stick values,
  - writes `PARAM_X_EQ_TORQUE`,
  - writes `PARAM_Y_EQ_TORQUE`,
  - writes `PARAM_Z_EQ_TORQUE`,
  - emits `CommResponse::CmdAck`.
- If no RC channels have been received, the command fails.
- `World` schedules RC trim calibration after comm command parsing and before comm responses are sent.
- Legacy `ROSFlight` mirrors the same scheduling step.
- `QuadController` now adds the equilibrium torque params to the controller torque output while armed.

ROSflight compatibility:

- The ROSflight command slot remains `ROSFLIGHT_CMD_RC_CALIBRATION`.
- In this architecture, that command is interpreted as RC trim calibration, matching `rosflight_io`'s `calibrate_rc_trim` service.
- The command ACK is sent only after the RC trim calibration system has run.
- The command succeeds only if there is current RC input available.
- The calibrated values are stored in the same equilibrium torque params documented by ROSflight.

Compile-time boundary improvement:

- Comms only emits request intent.
- The RC trim calibration system receives exactly the resources it needs:
  - read access to `Rc`,
  - mutable access to `Params`,
  - request drain port,
  - response emit port.
- Controller consumption of equilibrium torques is localized to the controller.
- This keeps parsing, RC-derived parameter mutation, response emission, and control use separate and testable.

Files changed in this slice:

- `rustflight_core/src/events.rs`
  - Adds `RcTrimCalibrationRequested`.
  - Adds `rc_trim_calibration_requests` to `CommandEventQueues`.
- `rustflight_core/src/comm_manager.rs`
  - Emits RC trim calibration requests for `RosflightCmd::RcCalibration`.
- `rustflight_core/src/command_system.rs`
  - Adds `RcTrimCalibrationCtx`.
  - Adds `apply_rc_trim_calibration_requests`.
- `rustflight_core/src/world.rs`
  - Schedules RC trim calibration requests.
- `rustflight_core/src/rosflight.rs`
  - Mirrors scheduling in the legacy path.
- `rustflight_core/src/controller/quad_controller.rs`
  - Adds equilibrium torque params to armed controller output.

Tests added:

- `command_system::tests::apply_rc_trim_calibration_requests_sets_equilibrium_torques_and_acks`
  - Proves the system writes `X_EQ_TORQUE`, `Y_EQ_TORQUE`, and `Z_EQ_TORQUE` from current RC stick offsets and emits success ACK.
- `comm_manager::tests::rc_trim_calibration_emits_request_and_defers_ack`
  - Proves comms emits request intent and does not ACK immediately.
- `world::tests::world_routes_rc_trim_calibration_and_sets_equilibrium_torques`
  - Proves the World scheduler drains the request, writes params, and sends success ACK.
- `controller::quad_controller::tests::controller_adds_equilibrium_torque_params_to_control_output`
  - Proves calibrated equilibrium torque params affect controller output.

Validation:

- `cargo test -p rustflight_core controller::quad_controller::tests --lib` passes.
- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Remaining question:

- The exact upstream ROSflight source formula should still be checked when the source is locally available.
- The implemented behavior follows the documented contract: current RC trim offsets become equilibrium torque params that are added to future controller outputs.

## Command Response Queue Progress

Reason for this change:

- `CommManager::act_on_messages` still sent some wire responses directly while parsing inbound commands.
- Direct sends make parsing, decision-making, and wire output happen in one function.
- The new architecture is easier to diagnose when parsing emits response intent and a later response stage owns actual transmission.

Design now implemented:

- `CommResponse` now supports:
  - `ParamValue`,
  - `CmdAck`,
  - `Version`.
- `RosflightCmd::SendVersion` now enqueues a version response.
- Immediate command ACKs now enqueue `CommResponse::CmdAck`.
- `CommManager::send_comm_responses` sends command ACKs and version messages in addition to parameter values.

ROSflight compatibility:

- `RosflightCmd::SendVersion` still produces a `ROSFLIGHT_VERSION` message and a successful `ROSFLIGHT_CMD_ACK`.
- The response order is preserved by the fixed-capacity FIFO response queue.
- The wire message types and payloads remain unchanged.

Compile-time boundary improvement:

- Command parsing no longer directly performs these wire writes.
- The response queue is now the boundary between parsing decisions and comm-link output.
- This keeps response causality inspectable: command parser emits response events, response stage transmits them.

Files changed in this slice:

- `rustflight_core/src/events.rs`
  - Adds `CommResponse::CmdAck`.
  - Adds `CommResponse::Version`.
- `rustflight_core/src/comm_manager.rs`
  - Enqueues version and immediate command ACK responses.
  - Sends the new response variants from `send_comm_responses`.
- `rustflight_core/src/test_support.rs`
  - Records version messages for tests.
- `rustflight_core/src/param_system.rs`
  - Updates tests to handle the expanded response enum.

Tests added:

- `comm_manager::tests::send_comm_responses_sends_command_ack_and_version`
  - Proves the response stage sends queued ACK and version messages.
- `comm_manager::tests::send_version_command_enqueues_version_and_ack_responses`
  - Proves command parsing queues responses and does not send directly.

Validation:

- `cargo test -p rustflight_core param_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## Param Set Name Resolution Progress

Reason for this change:

- `PARAM_SET` already used the new request/apply/response pipeline.
- However, `CommManager` was still resolving the parameter name against `PARAM_DEFINITIONS`.
- That meant communication code still knew too much about parameter ownership.
- The better boundary is for comms to emit the raw MAVLink parameter bytes and requested value, while the parameter system resolves and mutates.

Design now implemented:

- `ParamSetRequested` now carries only:
  - the requested value,
  - the raw 16-byte MAVLink parameter identifier.
- `CommManager` pushes `ParamSetRequested` without consulting `PARAM_DEFINITIONS`.
- `param_system::apply_param_requests` resolves the parameter name bytes.
- Unknown or invalid parameter names are ignored by the parameter system.
- Valid requests mutate params, emit `ParamChanged`, and emit `CommResponse::ParamValue`.

Compile-time boundary improvement:

- `CommManager` no longer imports `PARAM_DEFINITIONS`.
- `CommManager` no longer owns parameter-name resolution.
- All parameter lookup and mutation for `PARAM_SET`, `PARAM_REQUEST_READ`, and `PARAM_REQUEST_LIST` is now in `param_system`.
- This makes parameter causality easier to diagnose: inbound comms produce events; parameter systems decide what happens to parameters.

Files changed in this slice:

- `rustflight_core/src/events.rs`
  - Narrows `ParamSetRequested`.
- `rustflight_core/src/param_system.rs`
  - Moves set-request name resolution into `apply_param_requests`.
  - Reuses the same parameter-name resolution logic for request-read by name.
- `rustflight_core/src/comm_manager.rs`
  - Removes direct parameter-definition lookup from `PARAM_SET`.
  - Removes the now-unused direct `send_param_value` helper.

Tests:

- Existing `param_system::tests::apply_param_requests_mutates_params_and_defers_ack` now covers parameter-system name resolution for set requests.
- Existing `comm_manager::tests::param_set_emits_request_without_mutating_or_acknowledging` now proves comms emits raw request intent only.
- Existing `world::tests::world_scheduler_runs_deferred_param_pipeline` continues to prove end-to-end World behavior.

Validation:

- `cargo test -p rustflight_core param_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Current status after this slice:

- `params_iter` is gone from `World`.
- `params_iter` is gone from legacy `ROSFlight`.
- `params_iter` is gone from `CommManager`.
- `Params::iter` and `ParamIter` still exist in `params2`; they are no longer part of the active comm scheduling path and can be removed later if no remaining use appears.

## Param Request Read Event Progress

Reason for this change:

- `PARAM_REQUEST_READ` is part of the MAVLink parameter protocol expected by ROSflight tooling.
- The message was stored by the parser but not serviced by `CommManager::act_on_messages`.
- Implementing it through the new event path improves compatibility while preserving the architectural rule that comms should not own parameter lookup behavior.

Design now implemented:

- `CommManager` emits `ParamReadRequested` when a `PARAM_REQUEST_READ` message arrives.
- `ParamReadRequested` carries the parsed `ParamIdentifier`.
- `param_system::service_param_read_requests` resolves the request by index or by parameter name.
- The parameter system emits `CommResponse::ParamValue`.
- `CommManager::send_comm_responses` sends the ROSflight/MAVLink `PARAM_VALUE` message through the configured comm link.

ROSflight compatibility:

- Requests with `param_index >= 0` are resolved by index.
- Requests with `param_index == -1` are resolved by the `param_id` bytes parsed into `ParamIdentifier::ID`.
- Responses use the existing `ParamValueMsg` payload shape.
- Invalid parameter identifiers currently produce no response, matching the conservative behavior of ignoring malformed or unknown parameter requests.

Compile-time boundary improvement:

- Parameter lookup now belongs to `param_system`.
- Comms receives only the parameter request event queue, not broad mutable parameter access.
- `service_param_read_requests` uses `ParamsReadPort`, so this path can read parameters but cannot mutate them.

Files changed in this slice:

- `rustflight_core/src/comm_messages.rs`
  - Derives `PartialEq` for `ParamIdentifier` so request events can be compared in tests.
- `rustflight_core/src/events.rs`
  - Adds `ParamReadRequested`.
  - Adds a fixed-capacity read request queue to `ParamEventQueues`.
- `rustflight_core/src/param_system.rs`
  - Adds `ParamReadCtx`.
  - Adds `service_param_read_requests`.
  - Adds parameter identifier resolution by index or name.
- `rustflight_core/src/comm_manager.rs`
  - Emits read request events and does not directly read/send parameter values for this path.
- `rustflight_core/src/world.rs`
  - Schedules request-read servicing before comm responses are sent.
- `rustflight_core/src/rosflight.rs`
  - Mirrors the same compatibility scheduling in the legacy path.

Tests added:

- `param_system::tests::service_param_read_requests_responds_by_index_and_id`
  - Proves the parameter system resolves both MAVLink index and name forms.
- `comm_manager::tests::param_request_read_emits_request_without_reading_from_comms`
  - Proves comms only emits a request and does not send a value directly.
- `world::tests::world_scheduler_answers_param_request_read_through_param_system`
  - Proves the scheduler emits the ROSflight-compatible `PARAM_VALUE` response through the new path.

Validation:

- `cargo test -p rustflight_core param_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
