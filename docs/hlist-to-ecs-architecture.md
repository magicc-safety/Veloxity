# RustFlight World/Ports Architecture Migration Record

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
- Treat current ROSflight 2.x behavior from `rosflight_firmware` and `rosflight_ros_pkgs/rosflight_io` as the compatibility target.
- Responses that imply completed work should be sent after the relevant owning system completes that work.
- Avoid one-off patches that duplicate old shortcuts in the new architecture.
- Keep the active `World`/ports/events/resource path compatible with the ROSflight 2.x wire architecture while removing stale migration scaffolding as each replacement is proven.

Workflow notes for future agents:

- Before editing, inspect the current tree with `git status --short`.
- Use `rg` first for repository search.
- Use `apply_patch` for manual edits.
- Do not revert user changes.
- Treat each migration as a narrow slice:
  - add the event/resource/port shape,
  - move ownership of mutation to the domain system,
  - wire `World`,
  - preserve ROSflight wire compatibility and command ordering,
  - add component tests,
  - add World handoff tests when the scheduler path changes,
  - update this document,
  - validate,
  - commit locally.
- `cargo fmt` is available through the workspace-local toolchain:
  `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt`.
- If sandbox namespace errors occur on read-only shell commands, rerun the same command with the approved/escalated path rather than changing the workflow.

## Current Context

RustFlight formerly used HLists to encode board sensor inventory, sensor processing pipelines, body-type sensor requirements, telemetry packet access, and compile-time compatibility between boards and vehicle bodies. The active source now runs through `World`, `BoardIo`, named sensor resources, bounded event queues, and explicit systems.

That design solved an important early problem: it let the compiler prove that a selected board could provide the packet types required by a selected body type. It also made the raw-to-processed sensor pipeline generic over different board shapes.

The downside is that the type system is now carrying too much architectural bookkeeping. Adding a board/body/configuration requires positional type indices such as `There<There<There<Here>>>`, and board implementations have to write through nested tuple fields such as `sensors.1.1.1.0`. This makes the architecture hard to extend, hard to read, and brittle when sensors, telemetry streams, or body requirements change.

The goal remains to keep RustFlight modular and deterministic while replacing HList rigidity with named resources, fixed-order systems, and bounded events.

## Original HList Responsibilities

The old HList design provided five main capabilities:

1. Board-specific raw sensor inventory.
2. Raw-to-processed sensor processor ordering.
3. Body-type required sensor selection.
4. Telemetry access to processed packet types.
5. Compile-time board/body compatibility checks.

Those capabilities remain valuable. The active architecture keeps the capabilities but moves them into simpler constructs.

## Active Direction

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
fn update_board_sensors<B: BoardIo>(board: &mut B, raw: &mut SensorBus);

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

## Status Telemetry Command-State Progress

Reason for this change:

- Current ROSflight 2.x status telemetry reports whether RC override is active and whether offboard control is active.
- RustFlight's telemetry path still had zero placeholders for both fields.
- That made `rosflight_io` status consumers see a less faithful view of the flight stack even after offboard commands were routed through events.

Design now implemented:

- Both legacy HList telemetry and named `World` telemetry now ask `CommandManager` for:
  - `rc_override_active()`,
  - `is_offboard_active()`.
- `CommManager` still owns wire transmission, but it no longer fabricates these fields.
- `CommandManager` remains the only owner of the override/offboard decision state.

ROSflight compatibility:

- This follows current ROSflight 2.x `CommManager::send_status`, which uses command-manager state for `rc_override` and `offboard`.
- `rosflight_io` receives the same status-field intent: offboard activity is observable in the status stream after an offboard control command is active.

Compile-time boundary improvement:

- Telemetry receives a shared reference to `CommandManager`; it can observe command state but cannot mutate command state.
- The World scheduler keeps mutation in the command system and read-only status reporting in telemetry.
- This is the ports pattern in a small form: telemetry gets a read capability for exactly the state it must publish.

Files changed in this slice:

- `rustflight_core/src/comm_manager.rs`
  - Replaces status telemetry placeholder fields with command-manager accessors.
  - Adds a focused test for offboard status reporting.
- `rustflight_core/src/test_support.rs`
  - Records the latest status message so telemetry tests can assert field contents.

Tests added:

- `comm_manager::tests::named_status_telemetry_reports_command_manager_override_state`
  - Proves named telemetry reports active offboard state through `RosflightStatusMsg`.

Validation status:

- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Future follow-up:

- `rc_override_active()` is currently boolean, while upstream reports a bitmask-like `rc_override_` value. Keep this noted for future command-manager parity work.
- The next parity slice should inspect whether local RC override reasoning has enough channel-level detail to publish the full ROSflight 2.x override value instead of a boolean.

## Status Telemetry Wire-Width Progress

Reason for this change:

- While checking the RC override follow-up, the local MAVLink dialect was found to define `ROSFLIGHT_STATUS.rc_override` as `uint8_t`.
- Current ROSflight 2.x defines that field as `uint16_t`.
- The upstream command manager uses that width because override reasons are a bitmask with values up through `0x200`.

Design now implemented:

- Local `rustflight_core/mavlink_definitions/rosflight.xml` now defines `ROSFLIGHT_STATUS.rc_override` as `uint16_t`.
- Local `RosflightStatusMsg` now carries `rc_override: u16`.
- The current boolean local override state is widened at the telemetry boundary instead of truncating the field type.

ROSflight compatibility:

- This aligns the local wire schema with current ROSflight 2.x for `ROSFLIGHT_STATUS`.
- It does not yet mean local RustFlight computes every upstream override reason.
- It removes a blocking schema mismatch so a future command-manager slice can publish the full upstream bitmask.

Compile-time boundary improvement:

- The message type now prevents accidental truncation when the command manager grows from boolean override reporting to upstream-style bitmask reporting.
- Telemetry remains read-only over command state.

Files changed in this slice:

- `rustflight_core/mavlink_definitions/rosflight.xml`
  - Changes `ROSFLIGHT_STATUS.rc_override` from `uint8_t` to `uint16_t`.
- `rustflight_core/src/comm_messages.rs`
  - Changes `RosflightStatusMsg::rc_override` from `u8` to `u16`.
- `rustflight_core/src/comm_manager.rs`
  - Widens the current boolean override value to `u16` when building status messages.

Validation status:

- `cargo test -p rustflight_core comm_manager::tests::named_status_telemetry_reports_command_manager_override_state --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Future follow-up:

- Implement upstream-style RC override reason bits in `CommandManager`:
  - `OVERRIDE_ATT_SWITCH = 0x1`,
  - `OVERRIDE_THR_SWITCH = 0x2`,
  - `OVERRIDE_X = 0x4`,
  - `OVERRIDE_Y = 0x8`,
  - `OVERRIDE_Z = 0x10`,
  - `OVERRIDE_T = 0x20`,
  - `OVERRIDE_OFFBOARD_X_INACTIVE = 0x40`,
  - `OVERRIDE_OFFBOARD_Y_INACTIVE = 0x80`,
  - `OVERRIDE_OFFBOARD_Z_INACTIVE = 0x100`,
  - `OVERRIDE_OFFBOARD_T_INACTIVE = 0x200`.

## RC Override Bitmask Progress

Reason for this change:

- Current ROSflight 2.x reports `rc_override` as a `uint16_t` reason bitmask.
- After aligning the local wire width, RustFlight still only widened a boolean override state into that field.
- That preserved message width but did not preserve the diagnostic meaning expected by `rosflight_io` consumers.

Design now implemented:

- `CommandManager` now stores a `u16` `rc_override` mask.
- The upstream ROSflight 2.x override reason constants are represented locally:
  - attitude override switch,
  - throttle override switch,
  - X/Y/Z stick override,
  - throttle minimum override,
  - X/Y/Z offboard inactive,
  - throttle offboard inactive.
- Attitude muxing now returns the specific attitude reason bits while still selecting RC or offboard channel outputs.
- Throttle muxing now returns the specific throttle reason bits while preserving the existing local behavior of applying the throttle override decision to the three local force channels.
- Status telemetry now reads `CommandManager::get_rc_override()` instead of widening `rc_override_active()`.

ROSflight compatibility:

- Status telemetry now carries the same style of RC override reason bitmask as current ROSflight 2.x.
- This improves diagnosability in `rosflight_io` because status consumers can distinguish "stick moved", "switch forced", and "offboard inactive" cases instead of seeing only `0` or `1`.

Compile-time boundary improvement:

- `CommandManager` owns all override decision state.
- `CommManager` receives only a shared `CommandManager` reference and can publish the mask but cannot mutate command decisions.
- This keeps the command/telemetry boundary compatible with the ports model: command has write ownership of command state; telemetry has read-only visibility.

Files changed in this slice:

- `rustflight_core/src/command_manager.rs`
  - Adds upstream-style override constants.
  - Stores and exposes `get_rc_override()`.
  - Produces a bitmask during muxing.
  - Adds focused unit tests for stick/throttle and inactive-offboard override bits.
- `rustflight_core/src/comm_manager.rs`
  - Publishes the command-manager override bitmask in status telemetry.

Validation status:

- `cargo test -p rustflight_core command_manager::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Future follow-up:

- Compare the local force-channel muxing shape against upstream's `MuxChannel`/`Mixer::NUM_MIXER_OUTPUTS` model when the mixer ownership rewrite happens.
- The current local throttle override still applies to `fx`, `fy`, and `fz` together, matching the pre-existing RustFlight behavior; upstream evaluates the selected throttle axis but then muxes by channel masks.

## Mixer Output Ownership Progress

Reason for this change:

- The previous aux/PWM composition was still based on "primary slice length": primary mixer outputs owned channels `0..primary_len`, and aux commands could only use channels after that slice.
- Current ROSflight 2.x uses mixer output types to decide channel ownership.
- In upstream, AUX commands may fill channels inside the primary mixer range only when the selected mixer marks that output as AUX.
- This makes output causality clearer than relying on array length.

Design now implemented:

- `MixerOutputType` was added with:
  - `Aux`,
  - `Motor`,
  - `Servo`.
- The `Mixer` trait now exposes a read-only `output_types()` port.
- `QuadMixer` reports its four outputs as motor-owned.
- `pwm_system::compose_pwm_outputs` now receives:
  - primary command values,
  - primary output ownership,
  - optional aux command state,
  - state and params for motor safety mapping.
- For each of the 14 output channels:
  - non-AUX mixer-owned channels use the primary mixer value and primary output type,
  - AUX-owned channels use the aux command type/value,
  - channels beyond the primary ownership list default to AUX-owned.
- Raw output mapping now follows the output type:
  - `Servo`: clamp `[-1, 1]`, then map to `[0, 1]`,
  - `Motor`: zero while disarmed, otherwise apply clamp/idle/spin-when-armed behavior,
  - `Aux`: output zero.

ROSflight compatibility:

- This matches the important ROSflight 2.x rule: aux commands do not overwrite mixer-owned motor/servo channels, but they can fill channels the mixer marks AUX.
- Current quad behavior remains four motor-owned channels.
- AUX commands on channels beyond those four still work as before, but now because those channels are explicitly treated as AUX-owned rather than because they are past `primary_len`.
- Primary motor outputs now pass through motor raw-output rules in the composition stage, including armed idle-throttle enforcement. The old test expectation that primary value `0.1` stayed `0.1` while armed with idle `0.2` was corrected to `0.2`, matching the typed motor output stage.

Compile-time boundary improvement:

- PWM composition no longer guesses ownership from command array length.
- The mixer exposes only a shared output-type view; PWM can read ownership but cannot mutate mixer internals.
- The World scheduler wires the mixer output-type port into the PWM system explicitly.

Files changed in this slice:

- `rustflight_core/src/mixer.rs`
  - Adds `MixerOutputType`.
  - Adds `Mixer::output_types()`.
- `rustflight_core/src/mixer/quad_mixer.rs`
  - Reports four motor-owned outputs.
- `rustflight_core/src/pwm_system.rs`
  - Reworks `compose_pwm_outputs` around typed ownership.
  - Adds focused tests for aux-owned slots inside the primary range and motor safety mapping.
- `rustflight_core/src/world.rs`
  - Passes `self.mixer.output_types()` into PWM composition.

Validation:

- `cargo test -p rustflight_core pwm_system::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Future follow-up:

- The local mixer still lacks upstream's full primary/secondary mixer selection model driven by RC override masks.
- Fixed-wing mixer output ownership still needs to be introduced when the fixed-wing path is migrated into the new named-resource architecture.
- `ROSFLIGHT_OUTPUT_RAW` telemetry now receives the final composed 14-channel output array. Continue comparing local mixer/PWM semantics against upstream as mixer parity grows.

## Output Raw Telemetry Ownership Progress

Upstream source findings:

- Current upstream `rosflight_firmware` was refreshed from `origin/main`.
- Firmware source revision checked: `099a9846406d9f20b2bae08a2ea3dda74a01cf59`.
- `CommManager::send_output_raw` sends `RF_.mixer_.get_outputs()`.
- `Mixer::get_outputs()` returns the mixer's `raw_outputs_` array.
- Therefore `ROSFLIGHT_OUTPUT_RAW` should report the final mixed/raw 14-channel output state, not the controller's primary actuator-command vector.

Design now implemented:

- `World::run_control_stages_if_new_imu` still computes primary actuator commands through controller and mixer.
- `pwm_system::compose_pwm_outputs` still owns the final 14-channel composition from primary mixer output ownership plus aux command state and motor safety rules.
- Named telemetry now receives the composed `pwm_outputs` array.
- `latest_actuator_commands` remains the primary mixer output record for internal diagnostics; it is no longer the value handed to `ROSFLIGHT_OUTPUT_RAW` in the World path.

ROSflight compatibility:

- This aligns named `World` telemetry with upstream's `mixer_.get_outputs()` source for `ROSFLIGHT_OUTPUT_RAW`.
- Aux-owned channels and motor safety mapping are now visible in `output_raw` telemetry after composition, matching the intent that telemetry reports final output state.
- Wire message type and payload width remain unchanged.

Compile-time boundary improvement:

- Telemetry still receives read-only values from the scheduler.
- The PWM system remains the owner of output composition.
- CommManager remains the owner of wire transmission.
- The scheduler explicitly connects composition output to telemetry, so output telemetry semantics are visible at the stage boundary.

Files changed in this slice:

- `rustflight_core/src/world.rs`
  - Passes composed `pwm_outputs` into named telemetry instead of primary `actuator_commands`.
  - Extends the control-stage test to assert aux-composed channels are present in `ROSFLIGHT_OUTPUT_RAW`.

Validation:

- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p sim pwm::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

RC trim compatibility note:

- Current upstream `Controller::calculate_equilbrium_torque_from_rc` does not write raw RC stick offsets into equilibrium torque params.
- It runs controller PID logic once with a fake level estimator state, `dt = 0`, RC control input, and integrators disabled.
- It then adds the resulting torque outputs to the existing `X_EQ_TORQUE`, `Y_EQ_TORQUE`, and `Z_EQ_TORQUE` params.
- RustFlight has now been corrected to follow that ownership and calculation shape in the new command-system path.

## RC Trim Controller Ownership Progress

Reason for this change:

- The previous RC trim command-system slice wrote raw RC stick offsets directly into equilibrium torque params.
- Current upstream ROSflight 2.x makes RC trim calibration controller-owned:
  - command handling requests RC calibration,
  - controller runs PID logic once against fake level attitude,
  - resulting PID torques are added to existing equilibrium torque params,
  - command ACK succeeds when disarmed.
- RustFlight needed to preserve the event/scheduler ownership model while matching that upstream calculation.

Upstream source findings:

- Current upstream `rosflight_firmware` was refreshed from `origin/main`.
- Firmware source revision checked: `099a9846406d9f20b2bae08a2ea3dda74a01cf59`.
- `CommManager::command_callback` initializes command result to success when disarmed and calls `RF_.controller_.calculate_equilbrium_torque_from_rc()` for `COMMAND_RC_CALIBRATION`.
- `Controller::calculate_equilbrium_torque_from_rc` uses a fake level estimator state and calls `run_pid_loops(0, fake_state, RF_.command_manager_.rc_control(), false)`.
- It adds `pid_output.u[3..5]` to `X_EQ_TORQUE`, `Y_EQ_TORQUE`, and `Z_EQ_TORQUE`.

Design now implemented:

- Adds `Controller::RcTrimCalibrator`.
- `QuadController` implements `RcTrimCalibrator`.
- The implementation runs the existing quad controller PID path against `AttitudeState::default()`, RC control input, `dt = 0`, and without existing equilibrium torque feed-forward.
- `CommandManager` exposes read-only `rc_control()`.
- `command_system::apply_rc_trim_calibration_requests` now receives:
  - read-only command-manager access,
  - mutable controller access,
  - mutable params access,
  - read-only state access.
- The command system rejects the request while armed, matching upstream command gating.
- When disarmed, it adds controller-calculated torques to existing equilibrium torque params and emits success ACK.
- `World` and legacy `ROSFlight` pass their controller and command-manager resources into this system.

ROSflight compatibility:

- `ROSFLIGHT_CMD_RC_CALIBRATION` still maps to RC trim calibration.
- The external ACK remains `ROSFLIGHT_CMD_ACK`.
- Disarmed requests now succeed even though the result depends on current command-manager RC-control state, matching upstream's command callback result behavior.
- Equilibrium torque params now accumulate controller PID torque output rather than raw stick offsets.

Compile-time boundary improvement:

- Comms still only emits `RcTrimCalibrationRequested`.
- Command system owns command authorization and response emission.
- Controller owns the RC-trim calculation.
- CommandManager exposes only a read capability for the current RC control input.

Files changed in this slice:

- `rustflight_core/src/controller.rs`
  - Adds `RcTrimCalibrator`.
- `rustflight_core/src/controller/quad_controller.rs`
  - Adds controller-owned RC trim torque calculation.
  - Refactors normal control through a shared PID helper while preserving armed gating.
- `rustflight_core/src/command_manager.rs`
  - Adds read-only `rc_control()`.
- `rustflight_core/src/command_system.rs`
  - Routes RC trim calibration through the controller and adds torque output to existing params.
- `rustflight_core/src/world.rs`
  - Wires controller and command-manager resources into the RC trim command system.
- `rustflight_core/src/rosflight.rs`
  - Mirrors the compatibility scheduling in the legacy path.

Tests added or updated:

- `controller::quad_controller::tests::rc_trim_calibration_uses_pid_output_without_existing_equilibrium_torques`
  - Proves the controller-owned calculation returns PID torque output and does not include existing equilibrium torque params.
- `command_system::tests::apply_rc_trim_calibration_requests_sets_equilibrium_torques_and_acks`
  - Updated to prove RC trim adds controller-calculated torque output to existing equilibrium params.
- `world::tests::world_routes_rc_trim_calibration_and_sets_equilibrium_torques`
  - Updated to prove the World scheduler handoff uses current command-manager RC control input and sends success ACK.

Validation:

- `cargo test -p rustflight_core controller::quad_controller::tests --lib` passes.
- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p sim pwm::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## Armed Command Compatibility Progress

Upstream source findings:

- We cloned current upstream sources into `/tmp` for this check:
  - `https://github.com/rosflight/rosflight_firmware`
  - `https://github.com/rosflight/rosflight_ros_pkgs`
- `rosflight_io` documents and exposes user-facing services for parameter read/write, calibration, reboot, reboot-to-bootloader, and version/startup behavior.
- Current upstream firmware maps these ROSflight command enum values into the internal `CommLinkInterface::Command` enum:
  - read params,
  - write params,
  - set param defaults,
  - accel/gyro/baro/airspeed calibration,
  - RC trim calibration,
  - reboot,
  - reboot-to-bootloader,
  - send version.
- Current upstream firmware does not map `ROSFLIGHT_CMD_RESET_ORIGIN` or `ROSFLIGHT_CMD_SEND_ALL_CONFIG_INFOS` into the internal command enum.
- Upstream MAVLink decode treats unmapped commands as unsupported and sends `ROSFLIGHT_CMD_FAILED`.
- Therefore our current unsupported failure for `ResetOrigin` and `SendAllConfigInfos` is ROSflight-compatible externally, even though internally we route them through explicit command-system request queues.

Important upstream behavior:

- In upstream `CommManager::command_callback`, command actions are rejected while the vehicle is armed.
- If armed, the command callback reports failure and does not perform the command action.
- This applies to the mapped command actions, including version, parameter persistence/defaults, calibration, RC trim calibration, and reboot commands.

Reason for this change:

- RustFlight command systems were correctly routed through events, but they did not yet enforce the upstream "no command actions while armed" rule.
- Enforcing this in `CommManager` would reintroduce parser authority over command semantics.
- The correct place is the command-system stage, which can read state and decide whether the requested work is allowed.

Design now implemented:

- Added `VersionRequested`.
- `CommManager` now emits `VersionRequested` for `RosflightCmd::SendVersion` instead of queueing version and ACK responses directly.
- `command_system::apply_version_requests` owns version command completion:
  - if disarmed, it queues the version response followed by a success ACK,
  - if armed, it queues a failed ACK and does not send a version message.
- `command_system::apply_calibration_requests` now reads `StateManager`.
  - if armed, it queues a failed ACK and does not set calibration flags,
  - if disarmed, it sets calibration flags and reports the started command so the scheduler can track the deferred completion ACK.
- `command_system::apply_param_defaults_requests` now reads `StateManager`.
  - if armed, it queues a failed ACK and does not reset params,
  - if disarmed, it resets params and queues success.
- `command_system::apply_board_command_requests` now reads `StateManager`.
  - if armed, it fails without calling board hooks,
  - if disarmed, it calls the board hook and ACKs based on completion.
- `command_system::apply_rc_trim_calibration_requests` now reads `StateManager`.
  - if armed, it fails without changing equilibrium torque params,
  - if disarmed and RC input exists, it writes the torque params and ACKs success.
- `World` and legacy `ROSFlight` now pass state read access into these command systems.

Compile-time boundary improvement:

- Command systems now have explicit read-only access to state where command permission depends on armed/disarmed status.
- Comms still cannot mutate state, params, calibration flags, board persistence, or RC trim params.
- The function signatures make the dependency clear: command systems that enforce the armed rule receive `&StateManager`.
- Version is now consistent with the rest of the command architecture: parser emits request, command system decides, response stage transmits.

Files changed in this slice:

- `rustflight_core/src/events.rs`
  - Adds `VersionRequested`.
  - Adds a fixed-capacity version request queue to `CommandEventQueues`.
- `rustflight_core/src/comm_manager.rs`
  - Emits version requests instead of direct version/ACK responses.
  - Removes the param-defaults pending ACK slot because the command system now owns default-reset success/failure ACKs directly.
  - Keeps calibration pending ACK only for accepted calibration work whose success depends on later sensor-processing completion.
- `rustflight_core/src/command_system.rs`
  - Adds state-read gating to command actions.
  - Adds version request handling.
  - Adds tests for armed rejection and version behavior.
- `rustflight_core/src/world.rs`
  - Schedules version requests.
  - Passes state read access into state-gated command systems.
  - Adds a World test proving armed command rejection does not mutate params.
- `rustflight_core/src/rosflight.rs`
  - Mirrors the state-gated command-system scheduling in the legacy loop.

Tests added or updated:

- `command_system::tests::command_requests_fail_without_mutation_when_armed`
  - Proves armed calibration/default-reset requests fail without mutating resources.
- `command_system::tests::apply_version_requests_sends_version_only_when_disarmed`
  - Proves disarmed version requests send version plus success ACK, while armed version requests send failed ACK only.
- `comm_manager::tests::send_version_command_enqueues_version_and_ack_responses`
  - Updated so comms emits version request intent and the command system queues responses.
- `world::tests::world_rejects_command_actions_while_armed`
  - Proves the World scheduler rejects a command action while armed and does not reset params.

Validation:

- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Next planned migration target:

- Continue comparing command behavior to upstream ROSflight main:
  - confirm whether param read/list/set behavior should be allowed while armed, because upstream's armed rejection is specific to ROSflight command messages, not MAVLink parameter messages,
  - inspect reboot/write-param board hooks before pixracerpro migration so hardware side effects follow the same state-gated command-system rule.

## Companion Input Event Progress

Reason for this change:

- `rosflight_io` publishes companion-computer inputs that are not parameter requests and not ROSflight command ACK flows:
  - companion heartbeat,
  - aux command,
  - external attitude.
- Current upstream firmware handles these through callbacks:
  - heartbeat marks the companion link connected,
  - aux command updates mixer aux-command state,
  - external attitude updates estimator external-attitude state.
- RustFlight already parsed and stored these inbound MAVLink messages in `Messages`, but `CommManager::act_on_messages` did not consume them.
- That meant the messages were effectively silent no-ops in the scheduler path.

Design choice:

- Keep this as one grouped `companion_system` boundary instead of creating separate systems for heartbeat, aux, and external attitude.
- This avoids unnecessary subsystem sprawl while still giving these related companion-computer inputs an explicit owner.
- The current slice stores latest input facts only.
- It intentionally does not yet wire aux commands into the mixer or external attitude into the estimator.
- Those integrations should be separate slices because they change control/estimator behavior and need targeted tests.

Design now implemented:

- Added companion input events:
  - `CompanionHeartbeatReceived`,
  - `AuxCommandReceived`,
  - `ExternalAttitudeReceived`.
- Added `CompanionEventQueues`.
- Added `companion_system` with compact state resources:
  - `CompanionLinkState`,
  - `AuxCommandState`,
  - `ExternalAttitudeState`.
- `CommManager::act_on_messages` now emits companion input events when those messages are present.
- `World` owns companion input queues and state resources.
- `World` schedules companion input application immediately after comm parsing.
- Legacy `ROSFlight` mirrors the same scheduling while it still exists.

ROSflight compatibility:

- `rosflight_io` sends `aux_command` and `external_attitude` MAVLink messages from ROS topics.
- Upstream firmware has callbacks for these messages.
- RustFlight now has an explicit event handoff for those same inbound messages instead of silently ignoring them.
- The behavior is still incomplete compared with upstream because aux commands are not yet applied to mixer output and external attitude is not yet applied to estimator state.
- This slice is still compatibility progress because it preserves the inputs as typed scheduler facts for later owner systems.

Compile-time boundary improvement:

- Comms remains a parser and event producer.
- Comms does not receive mutable mixer or estimator access.
- Companion input state is explicit and owned by the scheduler.
- Later mixer/estimator integrations can read these resources through narrow contexts instead of receiving broad comm access.

Files changed in this slice:

- `rustflight_core/src/events.rs`
  - Adds companion input event types.
  - Adds fixed-capacity companion input queues.
  - Adds `CompanionEventQueues`.
- `rustflight_core/src/companion_system.rs`
  - Adds grouped companion input state resources.
  - Adds systems to drain companion input queues and store latest facts.
- `rustflight_core/src/lib.rs`
  - Exposes the new grouped companion system module.
- `rustflight_core/src/comm_manager.rs`
  - Emits companion input events for heartbeat, aux command, and external attitude messages.
- `rustflight_core/src/world.rs`
  - Owns companion input queues and state resources.
  - Schedules companion input application after comm parsing.
- `rustflight_core/src/rosflight.rs`
  - Mirrors companion input scheduling in the legacy loop.

Tests added:

- `companion_system::tests::companion_heartbeat_marks_link_connected_and_records_latest`
  - Proves heartbeat events update companion link state.
- `companion_system::tests::aux_command_records_latest_command`
  - Proves aux command events update latest aux command state.
- `companion_system::tests::external_attitude_records_latest_attitude`
  - Proves external attitude events update latest external attitude state.
- `comm_manager::tests::companion_inputs_emit_companion_events`
  - Proves comm parsing emits typed companion input events.
- `world::tests::world_applies_companion_input_events`
  - Proves the World scheduler drains those events and updates companion input state.

Validation:

- `cargo test -p rustflight_core companion_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Next planned migration target:

- Then choose one behavior integration:
  - apply aux command state to mixer output with focused mixer tests, or
  - apply external attitude state to estimator with focused estimator tests.
- Do not wire both in one slice; they affect different control paths and should remain easy to review.

## External Attitude Estimator Handoff Progress

Reason for this change:

- The previous companion-input slice preserved external attitude messages as explicit `ExternalAttitudeState`.
- Upstream ROSflight handles external attitude in the estimator, not in comms.
- The new architecture should follow that ownership:
  - comms emits external attitude input,
  - companion input stage stores the latest pending fact,
  - the estimator stage consumes that fact through an estimator-specific input path.

Upstream compatibility note:

- Upstream firmware stores the external attitude quaternion and marks it for use on the next estimator run.
- During estimator run, upstream computes an external-attitude correction term and uses `FILTER_KP_EXT`; it does not simply mutate estimator state from the comm callback.
- This slice preserves the ownership and scheduling semantics, but it does not yet implement the full upstream external-attitude correction math.
- Current RustFlight `QuadEstimator` is simpler than upstream and now consumes the pending external attitude by applying the provided quaternion before its next named estimator update.
- A later estimator-parity slice should replace this with the upstream-style correction term when the local estimator math is ready.

Design now implemented:

- `NamedEstimator` now has `estimate_named_with_external_attitude`.
- The default trait implementation calls `estimate_named`, so estimators that do not support external attitude are not forced to change behavior.
- `QuadEstimator` overrides the new method.
- `World::run_control_stages_if_new_imu` takes the pending `ExternalAttitudeState::latest` value and passes it into the named estimator path.
- The pending external attitude is consumed once with `take()`, matching upstream's "update next run" shape.

Compile-time boundary improvement:

- The estimator owns external attitude consumption.
- Comms does not receive mutable estimator access.
- The scheduler is the only place that connects companion input state to estimator input.
- The function signature makes the dependency explicit.

Files changed in this slice:

- `rustflight_core/src/estimator.rs`
  - Adds `estimate_named_with_external_attitude` to `NamedEstimator`.
- `rustflight_core/src/estimator/quad_estimator.rs`
  - Adds external attitude consumption for the named estimator path.
  - Adds focused estimator coverage.
- `rustflight_core/src/world.rs`
  - Passes pending external attitude into the estimator stage.
  - Proves the pending value is consumed by the scheduler.

Tests added or updated:

- `estimator::quad_estimator::tests::named_estimator_consumes_external_attitude_on_next_run`
  - Proves the named estimator path consumes an external attitude input.
- `world::tests::world_control_stage_runs_once_per_imu_timestamp`
  - Extended to prove the World control stage passes pending external attitude into the estimator and consumes it once.

Validation:

- `cargo test -p rustflight_core estimator::quad_estimator::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core companion_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Next planned migration target:

- Either implement upstream-style external attitude correction in `QuadEstimator`, or move aux command state into the mixer path as a separate, focused slice.

## Aux Command Output Composition Progress

Compatibility target:

- Current ROSflight 2.x behavior from `rosflight_firmware` and `rosflight_ros_pkgs/rosflight_io`.
- `rosflight_io` publishes `aux_command` MAVLink messages from the ROS `aux_command` topic.
- Current ROSflight firmware stores aux commands in the mixer path and applies them during output mixing.

Reason for this change:

- The companion-input slice preserved aux commands in `AuxCommandState`.
- Those values still did not affect outputs.
- The new architecture should not let comms mutate the mixer directly.
- The output-stage owner should decide how primary actuator commands and aux commands combine before PWM write.

Design choice:

- Keep this as an output-composition slice, not a broad mixer-trait rewrite.
- The current local `QuadMixer` produces four primary motor channels.
- The current sim PWM driver supports 14 output channels.
- Upstream ROSflight has richer mixer output typing and channel ownership.
- Instead of changing all mixer traits now, this slice composes aux commands onto unused channels after primary mixing and before PWM writes.
- This is intentionally incremental; exact upstream parity for channel ownership should come later with a richer mixer output type.

Design now implemented:

- Added `pwm_system::compose_pwm_outputs`.
- The function:
  - preserves primary mixer outputs in the leading channels,
  - applies aux commands only after the primary command slice,
  - maps servo aux values from `[-1.0, 1.0]` to `[0.0, 1.0]`,
  - applies motor aux values only when armed,
  - applies idle-throttle behavior for armed aux motors when `SPIN_MOTORS_WHEN_ARMED` is enabled,
  - leaves disabled aux channels low.
- `World::run_control_stages_if_new_imu` now composes PWM outputs before calling `write_pwm_commands`.
- Telemetry still reports primary actuator commands for now.

ROSflight 2.x compatibility:

- This matches the current ROSflight principle that aux commands are applied in the output path, not in comm parsing.
- It also preserves motor safety behavior: aux motor output is forced low while disarmed.
- It is not full mixer-output parity yet because RustFlight does not yet model upstream's per-channel output type ownership.
- The deferred parity work is explicitly to replace this simple composition with typed mixer output ownership.

Compile-time boundary improvement:

- Comms only emits aux input events.
- `companion_system` stores latest aux command state.
- The output stage reads `AuxCommandState` and params/state to compose PWM outputs.
- Mixer and PWM interactions remain scheduler-owned rather than callback-owned.

Files changed in this slice:

- `rustflight_core/src/pwm_system.rs`
  - Adds `PWM_OUTPUT_CHANNELS`.
  - Adds `compose_pwm_outputs`.
  - Adds focused aux composition tests.
- `rustflight_core/src/world.rs`
  - Uses `compose_pwm_outputs` before PWM writes.
  - Extends World control-stage coverage to prove aux values reach the PWM command slice.

Tests added or updated:

- `pwm_system::tests::compose_pwm_outputs_preserves_primary_and_applies_aux_to_unused_channels`
  - Proves primary outputs are preserved and servo/motor aux commands are applied to unused channels.
- `pwm_system::tests::compose_pwm_outputs_forces_aux_motors_low_when_disarmed`
  - Proves aux motor commands do not drive outputs while disarmed.
- `world::tests::world_control_stage_runs_once_per_imu_timestamp`
  - Extended to prove the World output stage sends a 14-channel composed PWM command slice with aux values.

Validation:

- `cargo test -p rustflight_core pwm_system::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core companion_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Next planned migration target:

- Later parity work:
  - introduce typed mixer output channel ownership,
  - move from simple "primary slice then aux channels" composition to ROSflight 2.x style output-type composition,
  - decide whether telemetry should report primary actuator commands or final composed output commands.

## Remaining Placeholder Command Event Progress

Reason for this change:

- `ResetOrigin` and `SendAllConfigInfos` were the last obvious ROSflight command arms still handled as inline placeholders inside `CommManager::act_on_messages`.
- Even though RustFlight does not yet implement origin reset or config-info streaming, the parser should not be the place that decides and sends the failure.
- The parser should emit command intent.
- A scheduled owning system should decide whether the command can be completed and should emit the ACK.

ROSflight compatibility:

- The ROSflight command enum values remain unchanged:
  - `ROSFLIGHT_CMD_RESET_ORIGIN`
  - `ROSFLIGHT_CMD_SEND_ALL_CONFIG_INFOS`
- The wire response remains `ROSFLIGHT_CMD_ACK`.
- Because there is no current origin/navigation resource or config-info message support in RustFlight, both commands still return `RosflightCmdFailed`.
- The externally visible unsupported behavior is preserved, but the ACK is now produced after a command-system stage runs.

Design now implemented:

- Added `ResetOriginRequested`.
- Added `ConfigInfoRequested`.
- Added fixed-capacity request queues for both to `CommandEventQueues`.
- `CommManager` emits these requests and defers ACK when queueing succeeds.
- `command_system::apply_reset_origin_requests` drains reset-origin requests and currently emits failed ACKs.
- `command_system::apply_config_info_requests` drains config-info requests and currently emits failed ACKs.
- `World` schedules both systems before comm responses are sent.
- Legacy `ROSFlight` mirrors the same scheduling for compatibility while the old path still exists.

Compile-time boundary improvement:

- Comms no longer directly owns these command outcomes.
- Reset-origin behavior now has an explicit command-system placeholder that can later be replaced by an estimator/navigation-origin system.
- Config-info behavior now has an explicit command-system placeholder that can later be replaced by a config-info response system once the message shape is implemented.
- This keeps the causality visible: comm parser emits request, command system emits completion/failure, comm response stage transmits.

Files changed in this slice:

- `rustflight_core/src/events.rs`
  - Adds `ResetOriginRequested`.
  - Adds `ConfigInfoRequested`.
  - Adds fixed-capacity queues for both request types.
- `rustflight_core/src/comm_manager.rs`
  - Emits reset-origin and config-info requests instead of using inline placeholder ACK logic.
- `rustflight_core/src/command_system.rs`
  - Adds `ResetOriginCtx`.
  - Adds `ConfigInfoCtx`.
  - Adds request application systems that currently emit failed ACKs.
- `rustflight_core/src/world.rs`
  - Schedules the new request application systems.
- `rustflight_core/src/rosflight.rs`
  - Mirrors the same compatibility scheduling in the legacy loop.

Tests added:

- `command_system::tests::apply_reset_origin_requests_reports_unsupported_as_failed_ack`
  - Proves the command system owns reset-origin failure ACK emission.
- `command_system::tests::apply_config_info_requests_reports_unsupported_as_failed_ack`
  - Proves the command system owns config-info failure ACK emission.
- `comm_manager::tests::reset_origin_emits_request_and_defers_ack`
  - Proves comms emits reset-origin request intent and does not ACK immediately.
- `comm_manager::tests::send_all_config_infos_emits_request_and_defers_ack`
  - Proves comms emits config-info request intent and does not ACK immediately.
- `world::tests::world_routes_reset_origin_and_acks_unsupported_after_apply_stage`
  - Proves the World scheduler drains reset-origin requests and sends failed ACK after the apply stage.
- `world::tests::world_routes_config_info_and_acks_unsupported_after_apply_stage`
  - Proves the World scheduler drains config-info requests and sends failed ACK after the apply stage.

Validation:

- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Next planned migration target:

- Inspect `CommManager::act_on_messages` again and confirm that command parsing now only emits request/response events, except for truly immediate protocol operations such as timesync.
- Then move to the next compatibility gap: either implement a real origin-reset capability if ROSflight-main behavior and local estimator/navigation support make that possible, or start converting the next subsystem boundary where HList is still actively shaping the new path.

## Completed ACK Response Queue Progress

Reason for this change:

- Most command responses now flow through `CommEventQueues::responses`.
- Two completed-work ACK helpers still wrote directly to the comm link:
  - calibration completion ACKs,
  - parameter-default reset completion ACKs.
- Direct writes meant there were still multiple wire-output points for command ACKs.
- The response queue should be the single boundary between internal completion facts and external MAVLink transmission.

Design now implemented:

- Renamed `send_completed_calibration_ack` to `queue_completed_calibration_ack`.
- Renamed `send_completed_param_defaults_ack` to `queue_completed_param_defaults_ack`.
- Both helpers now push `CommResponse::CmdAck` instead of calling `comm_link.send_cmd_ack` directly.
- The pending ACK slot is cleared only if queueing succeeds.
- `World` now flushes `CommEventQueues` after sensor processing and calibration-completion observation.
- Legacy `ROSFlight` mirrors this response ordering while it still exists.

ROSflight compatibility:

- The wire payload is unchanged: completed work still produces `ROSFLIGHT_CMD_ACK`.
- Calibration success is still emitted only after the calibration flag clears.
- Parameter-default success is still emitted only after params are reset.
- The only internal difference is that ACK transmission is centralized through `CommManager::send_comm_responses`.

Compile-time boundary improvement:

- Completion helpers no longer need board access.
- Completion helpers now emit response intent into the response queue.
- Only `send_comm_responses` owns command ACK wire transmission for queued responses.
- This makes response causality easier to inspect and keeps parser/completion/transport stages separate.

Files changed in this slice:

- `rustflight_core/src/comm_manager.rs`
  - Changes completed-work ACK helpers to queue `CommResponse::CmdAck`.
  - Updates tests to assert ACKs are not transmitted until `send_comm_responses`.
- `rustflight_core/src/world.rs`
  - Queues parameter-default ACKs.
  - Queues calibration-completion ACKs.
  - Moves the scheduler response flush after sensor processing so calibration completion ACKs can transmit in the same scheduler call.
- `rustflight_core/src/rosflight.rs`
  - Mirrors completed ACK queueing and later response flushing in the legacy loop.

Tests updated:

- `comm_manager::tests::calibration_command_ack_is_deferred_until_flag_clears`
  - Now proves completion queues the ACK first and wire transmission happens only through `send_comm_responses`.
- `comm_manager::tests::set_param_defaults_emits_request_and_defers_ack`
  - Now proves default-reset completion queues the ACK first and wire transmission happens only through `send_comm_responses`.
- `world::tests::world_sends_calibration_ack_after_calibration_flag_clears`
  - Now reflects the response-queue boundary explicitly.

Validation:

- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

Next planned migration target:

- Continue collapsing direct wire-output paths where they are not telemetry streams or truly immediate protocol responses.

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

- Sim board persistence for `ReadParams`/`WriteParams` is now implemented.
- Move `RcCalibration` out of the inline command parser next, because it needs a persistent RC calibration state rather than a placeholder failed ACK.

## Sim Board Persistence Progress

Reason for this change:

- Board command events already route `ReadParams` and `WriteParams` through `BoardIo` hooks.
- The core default hooks correctly return unsupported, which is appropriate for boards that have not implemented persistence yet.
- The sim board does not require hardware-specific flash support, so it should exercise the board persistence path now instead of reporting unsupported forever.

Design now implemented:

- `sim::board::Board` now owns a parameter-store path.
- Default store path: `rustflight_sim.params`.
- Override environment variable: `RUSTFLIGHT_SIM_PARAM_STORE`.
- `BoardIo::write_params` writes all known params to a text file as `PARAM_NAME=value`.
- `BoardIo::read_params` reads that file back into the active `Params`.
- Value parsing uses each parameter's static default type from `PARAM_DEFINITIONS`.
- Unknown, malformed, or incorrectly typed lines are ignored so a partially edited sim file does not corrupt unrelated params.
- Writes go through a temporary file and rename step so complete store contents are replaced as one operation.

ROSflight compatibility:

- The ROSflight command slots remain unchanged:
  - `ROSFLIGHT_CMD_READ_PARAMS`,
  - `ROSFLIGHT_CMD_WRITE_PARAMS`.
- The command system still owns the read/write operation and emits the ACK after the board hook returns.
- Sim boards now ACK success when the filesystem operation succeeds.
- Hardware boards still inherit the unsupported default until their board-specific persistence hooks are implemented.

Compile-time boundary improvement:

- Comms still only emits board command intent.
- `command_system::apply_board_command_requests` remains the only system with board persistence authority.
- Sim-specific file format and filesystem behavior stay in the sim board layer, not in `rustflight_core`.

Files changed in this slice:

- `sim/src/board.rs`
  - Adds a parameter-store path to the sim board.
  - Implements `BoardIo::read_params`.
  - Implements `BoardIo::write_params`.
  - Adds typed text serialization helpers for `Params`.

Tests added:

- `sim::board::tests::sim_param_store_round_trips_known_param_values`
  - Proves known int, bool, and float parameters persist and restore.
- `sim::board::tests::sim_param_store_ignores_unknown_and_malformed_lines`
  - Proves unknown names, malformed lines, and invalid values are ignored without clobbering existing params.

Validation:

- `cargo test -p sim board::tests --lib` passes.
- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo test -p rustflight_core world::tests::world_routes_board_command_and_acks_unsupported_after_apply_stage --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p sim pwm::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## Log Response Queue Progress

Reason for this change:

- `CommManager::act_on_messages` now only decodes messages, emits request events, queues immediate command failures, and handles timesync as a truly immediate protocol response.
- The remaining non-telemetry direct wire-output shortcut was statustext logging.
- Legacy `ROSFlight::run` drained the global `Logger` and called `CommManager::send_statustext` directly.
- The new architecture should route logs the same way as other internal responses: internal fact first, wire output stage second.

Design now implemented:

- Adds `log_system`.
- Adds `LogDrainCtx`.
- Adds `log_system::drain_logs_to_comm_responses`.
- Adds `CommResponse::Statustext`.
- `CommManager::send_comm_responses` now sends statustext responses.
- Removes the direct `CommManager::send_statustext` helper.
- `World::run_comm_param_sensor_stages_only` drains logs into `CommEventQueues` before the response stage.
- Legacy `ROSFlight::run` mirrors this queueing path while it still exists.
- The existing global logger remains the producer for now; this slice centralizes wire output, but it does not yet replace global logging with explicit `LogPort` injection.

ROSflight compatibility:

- The wire message remains MAVLink `STATUSTEXT`.
- Log text is still bounded to the MAVLink 50-byte payload.
- At most five log entries are drained per scheduler pass, preserving the existing loop-budget guard.

Compile-time boundary improvement:

- Modules still emit logs through the logger, but they no longer cause direct comm-link writes.
- Comm response transmission is centralized in `CommManager::send_comm_responses`.
- `World` and legacy `ROSFlight` both use the same queued response boundary.

Files changed in this slice:

- `rustflight_core/src/events.rs`
  - Adds `CommResponse::Statustext`.
- `rustflight_core/src/log_system.rs`
  - Adds the logger-to-comm-response drain system.
- `rustflight_core/src/comm_manager.rs`
  - Sends queued statustext responses through `send_comm_responses`.
  - Removes the direct statustext helper.
- `rustflight_core/src/world.rs`
  - Schedules log draining before comm responses are sent.
- `rustflight_core/src/rosflight.rs`
  - Mirrors log draining into queued responses in the legacy loop.
- `rustflight_core/src/test_support.rs`
  - Records statustext messages for tests.

Tests added or updated:

- `log_system::tests::drain_logs_queues_statustext_responses`
  - Proves logger entries become queued statustext responses.
- `comm_manager::tests::send_comm_responses_sends_command_ack_and_version`
  - Extended to prove the response stage sends statustext responses.
- `world::tests::world_drains_logs_through_comm_response_stage`
  - Proves the World scheduler drains logs and transmits them only through the comm response stage.

Validation:

- `cargo test -p rustflight_core log_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests::send_comm_responses_sends_command_ack_and_version --lib` passes.
- `cargo test -p rustflight_core world::tests::world_drains_logs_through_comm_response_stage --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p sim board::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## Body Model Boundary Progress

Reason for this change:

- `World` no longer uses body-type sensor sculpting or HList sensor requirements.
- However, `World` still had a `BT: BodyType` bound.
- `BodyType` carries `RequiredSensors: HList` for the legacy `ROSFlight` loop, so the new scheduler still depended on a HList-bearing trait even though it did not use that capability.
- Removing that bound is a direct step toward making the new `World` path independently checkable before deleting the old HList path.

Design now implemented:

- Adds `BodyModel`.
- `BodyModel` contains only:
  - `Estimator`,
  - `Controller`,
  - `Mixer`.
- `BodyType` remains unchanged for the legacy HList path.
- `Quadrotor` implements both `BodyType` and `BodyModel`.
- `World` now requires `BT: BodyModel` instead of `BT: BodyType`.
- World tests now construct mixers through `<Quadrotor as BodyModel>::Mixer`.

Compile-time boundary improvement:

- `World` no longer depends on `BodyType::RequiredSensors`.
- The new scheduler body boundary is HList-free.
- The legacy path still carries HList requirements until `ROSFlight` can be deleted deliberately.

Files changed in this slice:

- `rustflight_core/src/bodytype.rs`
  - Adds HList-free `BodyModel`.
- `rustflight_core/src/bodytype/quadrotor.rs`
  - Implements `BodyModel` for `Quadrotor`.
- `rustflight_core/src/world.rs`
  - Switches the scheduler body bound from `BodyType` to `BodyModel`.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo test -p sim board::tests --lib` passes.
- `cargo check -p sim` passes.

## Test Board Boundary Progress

Reason for this change:

- The shared core `TestBoard` is used by the new comm, command, and World scheduler tests.
- It previously implemented `BoardTrait` only to satisfy board I/O calls, which forced the test fixture to declare dummy `HNil` raw sensors, processed sensors, and processor lists.
- Those associated types are legacy HList scaffolding and are not needed by the new scheduler-facing tests.

Design now implemented:

- `TestBoard` implements `BoardIo` directly.
- `TestBoard` no longer imports or declares `HNil`, `RawSensorSet`, `ProcessedSensorSet`, or `ProcessorHList`.
- Comm manager tests import `BoardIo` explicitly where they call board clock helpers through the trait.

Compile-time boundary improvement:

- The shared core test board no longer depends on the legacy HList-bearing `BoardTrait`.
- The comm, command-system, and World tests now exercise a HList-free board fixture through the same `BoardIo` boundary used by the new scheduler path.
- This keeps the legacy `BoardTrait` available for the old `ROSFlight` path while reducing the surface area that must survive after HList deletion.

Files changed in this slice:

- `rustflight_core/src/test_support.rs`
  - Moves `TestBoard` from `BoardTrait` to direct `BoardIo`.
- `rustflight_core/src/comm_manager.rs`
  - Updates test imports from `BoardTrait` to `BoardIo`.

Validation:

- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p sim board::tests --lib` passes.

## Stale BoardTrait Import Cleanup

Reason for this change:

- After `TestBoard` moved to direct `BoardIo`, a few modules still imported `BoardTrait` even though they no longer used the HList-bearing board trait.
- These stale imports made the new path look more coupled to the legacy board shape than it actually is.

Design now implemented:

- `CommandManager` no longer imports `BoardTrait`.
- `StateManager` no longer imports `BoardTrait`.
- No behavior changed; both modules were already operating without board sensor associated types.

Remaining `BoardTrait` references:

- `comm_manager.rs` keeps a local bound for the legacy HList telemetry method.
- `rosflight.rs` remains the legacy HList scheduler/reference path.
- `board/dummy.rs` remains the legacy HList dummy board.
- `params.rs` is not exported by `lib.rs`; the active parameter module is `params.rs`.

Validation:

- `cargo test -p rustflight_core command_manager::tests --lib` passes.
- `cargo test -p rustflight_core command_system::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core state_machine::tests --lib` still has pre-existing arming/`UNCALIBRATED_IMU` failures noted in the earlier next-steps list; this slice did not change that behavior.

## Dummy Board Named Sensor Progress

Reason for this change:

- `DummyBoard` still exists for legacy binary scaffolding and the old `ROSFlight` path.
- It populated the legacy raw sensor HList, but the `BoardIo` named sensor hook inherited the default implementation that clears `SensorBus`.
- That meant dummy-board users had default packets on the old path and no packets on the new named sensor path.

Design now implemented:

- `DummyBoard::update_sensor_bus` now fills every named `SensorBus` field with the same default packet types used by `update_sensors`.
- The legacy HList `BoardTrait` implementation remains intact for compatibility.
- The new behavior is reached through the `BoardIo` blanket implementation, so it exercises the same board boundary used by `World`.

Tests added or updated:

- `board::dummy::tests::dummy_board_populates_named_sensor_bus`
  - Calls `BoardIo::update_sensor_bus` on `DummyBoard`.
  - Verifies IMU, mag, baro, pitot, range, GNSS, battery, RC, and attitude named fields are populated with successful packets.

Validation:

- `cargo test -p rustflight_core board::dummy::tests --lib` passes.
- `cargo test -p rustflight_core sensor_systems::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p sim board::tests --lib` passes.

## Telemetry Delegation And RC Compatibility Progress

Upstream compatibility check:

- Fetched `rosflight_firmware` `main` at `099a9846406d9f20b2bae08a2ea3dda74a01cf59`.
- Current upstream `CommManager::send_rc_raw` reads RC channels and sends only the first 8 values.
- The upstream formula is `channels[i] = rc_struct->chan[i] * 1000.0 + 1000`.
- Upstream packs `RC_CHANNELS` with `chancount = 0`, channels 9-18 as `0`, and RSSI as `0`.
- Upstream uses `RF_.board_.clock_millis()` for the RC telemetry timestamp.

Reason for this change:

- `CommManager` still had two telemetry implementations:
  - legacy `send_telemetry_streams` over `B::ProcessedSensorSet` plus `HListGet`,
  - new `send_named_telemetry_streams` over `ProcessedSensors`.
- Keeping two implementations risks wire-behavior drift while the new `World` path replaces the old HList scheduler.
- RC telemetry had already drifted: the named path scaled more than the upstream first 8 channels and reported `chancount = packet.n_chan`.

Design now implemented:

- Legacy `send_telemetry_streams` now converts the HList processed sensor set into `ProcessedSensors`.
- It then delegates to `send_named_telemetry_streams`.
- The legacy method still has local HList bounds because `ROSFlight` still passes `B::ProcessedSensorSet`.
- The message-building logic now lives in the named telemetry path.
- Named RC telemetry now matches upstream `send_rc_raw` packing:
  - first 8 channels scaled from normalized values,
  - remaining channels zero,
  - `chancount = 0`,
  - `rssi = 0`,
  - timestamp from `board.clock_millis()`.

Tests added or updated:

- `comm_manager::tests::named_rc_telemetry_matches_upstream_raw_channel_packing`
  - Verifies first 8 channels are scaled as upstream.
  - Verifies channels 9-18 are zero.
  - Verifies `chancount`, RSSI, and board-clock timestamp.
- `RecordingCommLink`
  - Records the last RC channels message for telemetry assertions.

Validation:

- `cargo test -p rustflight_core comm_manager::tests::named_rc_telemetry_matches_upstream_raw_channel_packing --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p sim board::tests --lib` passes.

## BodyModel Named Estimator Bound Progress

Reason for this change:

- `World` already requires `BT::Estimator: NamedEstimator`.
- However, `BodyModel` itself still allowed any estimator type, including an estimator that only implemented the legacy HList-bearing `Estimator` trait.
- That left the new body boundary less explicit than the scheduler that consumes it.

Design now implemented:

- `BodyModel::Estimator` is now bounded by `NamedEstimator`.
- Legacy `BodyType::Estimator` remains bounded by the old HList `Estimator` trait.
- `Quadrotor` still implements both `BodyType` and `BodyModel`.

Compile-time boundary improvement:

- Any body type used with the new `World` model must expose a named-resource estimator entry point.
- The legacy HList estimator trait remains available only through `BodyType` and `ROSFlight`.

Validation:

- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core estimator::quad_estimator::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p sim board::tests --lib` passes.

## World Sensor Ingestion Coverage Progress

Reason for this change:

- `World` already owned `SensorBus` and `ProcessedSensors`.
- The scheduler already called `BoardIo::update_sensor_bus` and `process_sensor_bus`.
- However, most World tests still injected `processed_sensors` directly.
- That proved downstream behavior, but it did not prove the board-to-World named sensor ingestion path without a legacy HList fixture.

Design now implemented:

- Added a World test-only `SensorStageBoard`.
- `SensorStageBoard` implements `BoardIo` directly.
- It does not implement `BoardTrait`.
- It has no `RawSensorSet`, `ProcessedSensorSet`, `ProcessorHList`, or HList indices.
- Its `update_sensor_bus` hook supplies IMU and RC packets through named `SensorBus` fields.
- Added a minimal test comm link for that board so the scheduler can run without reusing the legacy HList-compatible test board.

Test added:

- `world::tests::world_sensor_stage_ingests_board_sensor_bus_without_hlist_fixture`
  - Runs `World::run_comm_param_sensor_stages_only`.
  - Verifies the board `update_sensor_bus` hook is called.
  - Verifies raw named sensor slots are consumed by processing.
  - Verifies processed IMU and RC packets are populated.
  - Verifies IMU health clears through the named path.
  - Runs RC/state stages and verifies RC loss is not raised from the named RC packet.

Compile-time boundary improvement:

- The test proves the new scheduler can ingest board sensors through `BoardIo` and named resources without constructing any HList sensor fixture.
- This narrows the remaining HList dependency to the legacy `ROSFlight` path and explicitly retained legacy traits.

Validation:

- `cargo test -p rustflight_core world::tests::world_sensor_stage_ingests_board_sensor_bus_without_hlist_fixture --lib` passes.
- `cargo test -p rustflight_core sensor_systems::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p sim board::tests --lib` passes.

## Sim SensorBus Coverage Progress

Reason for this change:

- The new `World` scheduler now has test coverage proving board-backed named sensor ingestion.
- The sim board already implements `BoardIo::update_sensor_bus`, but sim tests only covered parameter persistence.
- Since sim is the no-hardware board path, it should be verified alongside core whenever the named board/sensor boundary advances.

Design now implemented:

- Added a sim board test that constructs a `Board` with local Tokio channels and a local UDP socket.
- The test queues ROS-like IMU, mag, baro, GNSS, and RC messages into the board receivers.
- It calls the real `BoardIo::update_sensor_bus` hook.
- It verifies the resulting named `SensorBus` packets and coordinate/value conversions.
- This does not require Zenoh, a simulator process, or hardware.

Test added:

- `sim::board::tests::sim_board_update_sensor_bus_converts_queued_messages`
  - Verifies IMU timestamp and axis sign conversions.
  - Verifies magnetometer axis sign conversions.
  - Verifies barometer Kelvin-to-Celsius conversion.
  - Verifies GNSS timestamp and lat/lon radian conversion.
  - Verifies RC microsecond-to-normalized-channel conversion and clamping.

Validation:

- `cargo test -p sim board::tests::sim_board_update_sensor_bus_converts_queued_messages --lib` passes.
- `cargo test -p sim board::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## Sensor Processor Boundary Progress

Reason for this change:

- `World` and `sensor_systems` now operate on named `SensorBus` and `ProcessedSensors`.
- However, `sensor_systems::process_sensor_bus` still used `hlist::Func` as its processor interface.
- That meant the named sensor path still had a direct dependency on the legacy HList processor abstraction.

Design now implemented:

- Added `SensorPacketProcessor<P>` in `sensorprocessors`.
- `SensorPacketProcessor` is a named packet processor trait:
  - input: `&mut Option<Result<P, SensorError>>`
  - context: calibration flags and params
  - output: `Option<P>`
- Existing legacy `Func` processors automatically implement `SensorPacketProcessor` through a blanket adapter in `sensorprocessors`.
- `sensor_systems` now depends on `SensorPacketProcessor` instead of `hlist::Func`.
- `process_sensor_bus` now calls `.process(...)` for each named packet field.

Compile-time boundary improvement:

- The new named sensor system no longer imports or bounds itself on `hlist::Func`.
- The only remaining `Func` bridge is isolated in `sensorprocessors`, where the legacy processor implementations already live.
- This creates a clear path to later convert concrete processors from `Func` implementations to native `SensorPacketProcessor` implementations without touching `World`.

Validation:

- `cargo test -p rustflight_core sensor_systems::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p sim board::tests --lib` passes.

## Passthrough Sensor Processor Conversion Progress

Reason for this change:

- `sensor_systems` now depends on `SensorPacketProcessor`, but all concrete processors still reached that trait through a blanket adapter from legacy `hlist::Func`.
- The next low-risk step is to move simple passthrough processors to native `SensorPacketProcessor` implementations while preserving their `Func` implementations for the legacy HList path.

Design now implemented:

- Removed the blanket `Func -> SensorPacketProcessor` implementation.
- Added native `SensorPacketProcessor` implementations for passthrough processors:
  - battery,
  - IMU,
  - baro,
  - pitot,
  - mag,
  - RC,
  - range,
  - GNSS,
  - PPS,
  - attitude.
- Kept the existing `Func` implementations on those processors so legacy HList mapping still compiles.
- Added explicit `Func` adapters only for complex processors that still need conversion:
  - `ImuProcessor`,
  - `BaroProcessor`,
  - `PitotProcessor`,
  - `MagProcessor`.

Compile-time boundary improvement:

- The named sensor path now uses native passthrough packet processors without relying on a blanket HList adapter.
- Remaining processor/HList coupling is explicit and limited to complex processors plus the legacy `Func` implementations retained for `ROSFlight`.

Validation:

- `cargo test -p rustflight_core sensor_systems::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p sim board::tests --lib` passes.

## Baro Processor Native Conversion Progress

Reason for this change:

- After passthrough processors moved to native `SensorPacketProcessor`, the remaining processor/HList bridge was limited to complex processors.
- `BaroProcessor` is the smallest complex processor, so it is the first safe conversion target.

Design now implemented:

- `BaroProcessor` now implements `SensorPacketProcessor<BaroPacket>` directly.
- Its legacy `Func` implementation remains for the old HList processor path.
- Both entry points delegate to the same internal `process_packet` implementation.
- Remaining explicit `Func` adapters:
  - `ImuProcessor`,
  - `PitotProcessor`,
  - `MagProcessor`.

Validation:

- `cargo test -p rustflight_core sensor_systems::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p sim board::tests --lib` passes.

## Pitot Processor Native Conversion Progress

Reason for this change:

- After `BaroProcessor` moved to native `SensorPacketProcessor`, the next low-risk complex processor target was `PitotProcessor`.
- This continues shrinking the explicit HList adapter surface without changing legacy `ROSFlight` processor behavior.

Design now implemented:

- `PitotProcessor` now implements `SensorPacketProcessor<PitotPacket>` directly.
- Its legacy `Func` implementation remains for the old HList processor path.
- Both entry points delegate to the same internal `process_packet` implementation.
- Remaining explicit `Func` adapters:
  - `ImuProcessor`,
  - `MagProcessor`.

Validation:

- `cargo test -p rustflight_core sensor_systems::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p sim board::tests --lib` passes.

## Mag Processor Native Conversion Progress

Reason for this change:

- After `PitotProcessor` moved to native `SensorPacketProcessor`, `MagProcessor` was the smaller remaining complex processor.
- This keeps reducing the HList bridge surface one processor at a time while retaining the old `ROSFlight` processor entry point.

Design now implemented:

- `MagProcessor` now implements `SensorPacketProcessor<MagPacket>` directly.
- Its legacy `Func` implementation remains for the old HList processor path.
- Both entry points delegate to the same internal `process_packet` implementation.
- Remaining explicit `Func` adapter:
  - `ImuProcessor`.

Validation:

- `cargo test -p rustflight_core sensor_systems::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p sim board::tests --lib` passes.

## IMU Processor Native Conversion Progress

Reason for this change:

- After `MagProcessor` moved to native `SensorPacketProcessor`, `ImuProcessor` was the last complex processor using the explicit `Func` adapter.
- Converting it completes the sensor processor boundary step: the named sensor path no longer needs any `Func -> SensorPacketProcessor` bridge.

Design now implemented:

- `ImuProcessor` now implements `SensorPacketProcessor<ImuPacket>` directly.
- Its legacy `Func` implementation remains for the old HList processor path.
- Both entry points delegate to the same internal `process_packet` implementation.
- Removed the now-unused `impl_sensor_packet_processor_via_func!` macro.
- No explicit `Func` adapters remain for the named sensor path.

Validation:

- `cargo test -p rustflight_core sensor_systems::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p sim board::tests --lib` passes.

## Legacy Estimator Named Delegation Progress

Reason for this change:

- The new `World` path already requires `NamedEstimator`.
- The legacy `Estimator` entry point for `QuadEstimator` still read its HList inputs and called the packet-level implementation directly.
- Keeping two entry points that bypass each other creates drift risk while the HList path is retained for compatibility.

Design now implemented:

- `QuadEstimator`'s legacy `Estimator::estimate` implementation now converts its IMU/mag HList input into `ProcessedSensors`.
- The legacy entry point delegates to `estimate_named`.
- The existing `NamedEstimator` implementation remains the scheduler-facing source of estimator behavior.
- The legacy `Estimator` trait and HList input type remain in place for `ROSFlight` compatibility.

Validation:

- `cargo test -p rustflight_core estimator::quad_estimator::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## Legacy RustFlight Scheduler Named Sensor Use Progress

Reason for this change:

- RustFlight's legacy `ROSFlight` scheduler still sculpted `BT::RequiredSensors` from the processed sensor HList to split RC input from estimator input.
- It also called the legacy `Estimator::estimate` entry point and legacy HList telemetry wrapper even though named equivalents now exist.
- This kept body-required HList sculpting in the control path after the estimator and telemetry compatibility shims were already available.

Design now implemented:

- Added `sensors::processed_sensors_from_hlist` as the explicit compatibility bridge from legacy processed HLists to `ProcessedSensors`.
- After legacy HList sensor processing, RustFlight's legacy `ROSFlight` scheduler converts the processed sensor HList into `ProcessedSensors` once through that helper.
- `ROSFlight` reads RC input from `ProcessedSensors::rc`.
- `ROSFlight` calls `NamedEstimator::estimate_named` with the named processed sensor struct.
- `ROSFlight` calls `send_named_telemetry_streams` directly.
- Removed the legacy scheduler's `BT::RequiredSensors: Plucker` and `B::ProcessedSensorSet: Sculptor` bounds.
- The legacy telemetry wrapper now uses the same HList-to-named helper before delegating to named telemetry.
- The raw-to-processed legacy HList map remains in place for the old scheduler path.

Validation:

- `cargo test -p rustflight_core sensors::tests --lib` passes.
- `cargo test -p rustflight_core estimator::quad_estimator::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## Legacy HList Telemetry Wrapper Removal Progress

Reason for this change:

- The local legacy `ROSFlight` scheduler now calls `CommManager::send_named_telemetry_streams` directly.
- `World` and the sim already use the same named telemetry path.
- The old `CommManager::send_telemetry_streams` wrapper had no remaining call sites and only preserved HList bounds inside comms.

Design now implemented:

- Removed `CommManager::send_telemetry_streams`.
- Removed the HList, `BodyType`, legacy `Estimator`, `Configuration`, and HList-to-named conversion imports that were only needed by that wrapper.
- `CommManager` telemetry now exposes the named `ProcessedSensors` API only.

Current boundary status:

- HList telemetry sculpting is gone from the local legacy `ROSFlight` scheduler.
- HList telemetry conversion is gone from `CommManager`.
- The remaining HList sensor conversion is isolated in the local legacy scheduler while it still maps the legacy board raw sensor HList into a processed HList before creating `ProcessedSensors`.

Validation:

- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

## Legacy RustFlight Named Sensor Bus Progress

Reason for this change:

- The local legacy `ROSFlight` scheduler still stored `B::RawSensorSet` and `B::ProcessorHList`.
- It still mapped raw HLists into processed HLists, then converted that processed HList into `ProcessedSensors`.
- `World` and sim already use the named `SensorBus -> ProcessedSensors` path through `sensor_systems::process_sensor_bus`.

Design now implemented:

- `ROSFlight` now stores `SensorBus`, `ProcessedSensors`, and `SensorProcessorSet`.
- `ROSFlight::run` now calls `board.update_sensor_bus` followed by `process_sensor_bus`.
- Removed the legacy scheduler's `HMappable` and `HListGet` sensor bounds.
- Removed `processed_sensors_from_hlist` use from `ROSFlight`.
- `Configuration` is now only a compatibility marker for existing `ROSFlight::init` call sites.
- Removed HList packet index associated types from the PixRacerPro and Nucleo marker configs.

Current boundary status:

- `ROSFlight` no longer has local HList sensor ingestion, processing, conversion, sculpting, or telemetry dispatch.
- HList sensor associated types remain on `BoardTrait` for compatibility with legacy board definitions.
- `processed_sensors_from_hlist` remains as a narrow testable compatibility helper, but the local legacy scheduler no longer calls it.
- Hardware package checks were attempted, but the current host environment compiles `cortex-m` for the host target and fails before reaching these crates.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core sensor_systems::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.

## BoardTrait HList Sensor Inventory Removal Progress

Reason for this change:

- After `ROSFlight` moved to `SensorBus`, the core `BoardTrait` no longer needed HList raw sensor, processed sensor, or processor associated types.
- Keeping those associated types made board implementations continue to look HList-shaped even though the active scheduler path no longer consumed them.

Design now implemented:

- Removed `RawSensorSet`, `ProcessedSensorSet`, `ProcessorHList`, and `update_sensors` from `BoardTrait`.
- Removed the legacy HList sensor implementation from `rustflight_core::board::dummy::DummyBoard`.
- Updated `pixracerpro::board::Board` to populate `SensorBus` directly through `update_sensor_bus`.
- Removed PixRacerPro board HList sensor imports and processor-list declarations.

Current boundary status:

- Core board traits no longer expose HList sensor inventory.
- Core dummy board is named-sensor-only.
- PixRacerPro board is named-sensor-only at the source level.
- Nucleo board still needs the same source-level cleanup.
- HList remains in retained estimator/body/processor compatibility shims and in `hlist.rs`.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core board::dummy::tests --lib` passes.
- `cargo test -p rustflight_core sensor_systems::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p pixracerpro` was attempted, but the current host environment compiles `cortex-m` for the host target and fails before reaching local PixRacerPro crate code.

## Legacy Func Sensor Processor Shim Removal Progress

Reason for this change:

- `sensor_systems::process_sensor_bus` now depends on `SensorPacketProcessor`.
- `ROSFlight`, `World`, and sim no longer use HList processor mapping.
- After `BoardTrait` stopped exposing `ProcessorHList`, the concrete sensor processors no longer needed legacy `hlist::Func` implementations.

Design now implemented:

- Removed the `hlist` import from `sensorprocessors.rs`.
- Removed all legacy `Func` implementations from passthrough processors.
- Removed all legacy `Func` implementations from `ImuProcessor`, `BaroProcessor`, `PitotProcessor`, and `MagProcessor`.
- Kept the native `SensorPacketProcessor` implementations unchanged.

Current boundary status:

- Named sensor processing no longer depends on `hlist::Func`.
- Processor HList compatibility shims are gone from core sensor processors.
- Remaining HList dependencies are now concentrated in retained estimator/body compatibility and the explicit `processed_sensors_from_hlist` helper.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core sensor_systems::tests --lib` passes.
- `cargo test -p rustflight_core estimator::quad_estimator::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.

## Legacy Estimator, Body Sensor Requirement, and HList Bridge Removal Progress

Reason for this change:

- The active estimator path uses `NamedEstimator` and `ProcessedSensors`.
- `BodyType::RequiredSensors` no longer had an active scheduler use after `ROSFlight` stopped sculpting sensor HLists.
- `processed_sensors_from_hlist` no longer had call sites after `ROSFlight` moved to `SensorBus`.

Design now implemented:

- Removed the legacy HList-bearing `Estimator` trait.
- Changed `BodyType::Estimator` to require `NamedEstimator`.
- Removed `BodyType::RequiredSensors`.
- Removed the quadrotor required-sensor HList.
- Removed `QuadEstimator`'s legacy HList `Estimator` implementation and its compatibility test.
- Removed `processed_sensors_from_hlist` and its HList-specific test.
- Removed stale HList imports from controller and hardware entrypoints.
- Updated the Nucleo board source to populate `SensorBus` directly, matching the core and PixRacerPro board boundary.

Current boundary status:

- Core and sim no longer use HLists for board sensors, sensor processors, estimator inputs, body sensor requirements, telemetry, or scheduler sensor routing.
- The remaining HList surface is isolated to `hlist.rs` itself and stale comments there.
- Embedded package checks still require the correct Cortex-M target environment before they can validate package code.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core estimator::quad_estimator::tests --lib` passes.
- `cargo test -p rustflight_core sensors::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.

## HList Removal Completion

Reason for this change:

- After the board, processor, estimator, body, telemetry, and scheduler paths moved to named resources, `hlist.rs` had no remaining source consumers.
- Embedded checks could now be run against the correct Cortex-M target instead of the host target.

Design now implemented:

- Installed the `thumbv7em-none-eabihf` target into a workspace-local Rustup home for embedded package validation.
- Fixed PixRacerPro's `PwmDriver` implementation to use the HList-free `BoardIo` bound.
- Updated Nucleo's board/PWM split to provide a `BoardPwmDriver` to `ROSFlight::init`.
- Updated Nucleo's `ROSFlight::init` call to pass `Params` and the PWM driver.
- Added Nucleo's missing `panic-halt` dependency and panic handler import.
- Removed `pub mod hlist`.
- Deleted `rustflight_core/src/hlist.rs`.

Current boundary status:

- HLIST is removed from the source architecture.
- The only remaining source hit for "hlist" is a test name documenting that the World sensor fixture is HList-free.
- Core, sim, PixRacerPro, and Nucleo compile through the new named-resource architecture.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core sensor_systems::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

## BoardTrait Removal Progress

Reason for this change:

- After HLIST removal, `BoardTrait` duplicated `BoardIo` and existed only to provide a blanket adapter.
- Keeping both traits made hardware and `ROSFlight` look like they still had a legacy board boundary even though the active board interface was already `BoardIo`.

Design now implemented:

- Removed `BoardTrait`.
- Removed the blanket `impl<T: BoardTrait> BoardIo for T`.
- `ROSFlight` now requires `B: BoardIo`.
- The compatibility `Configuration` marker now accepts `B: BoardIo`.
- Core dummy, PixRacerPro, and Nucleo boards now implement `BoardIo` directly.
- Removed stale `BoardTrait` imports and a commented Nucleo `BoardTrait` block.
- Updated the old `params.rs` board generic bounds to `BoardIo` so no source references remain.

Current boundary status:

- `BoardIo` is the only board interface in source.
- Core, sim, PixRacerPro, and Nucleo all compile without `BoardTrait`.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core board::dummy::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

## ROSFlight Configuration Marker Removal Progress

Reason for this change:

- `Configuration` used to carry HList wiring indices.
- After HLIST removal, it was only a zero-sized marker argument to `ROSFlight::init`.
- Keeping it forced hardware entrypoints to define marker config structs with no behavior.

Design now implemented:

- Removed `rosflight::Configuration`.
- Removed the `C` generic from `ROSFlight`.
- Removed the `_configuration` phantom field.
- Removed the config argument from `ROSFlight::init`.
- Deleted PixRacerPro and Nucleo marker config structs and impls.
- Hardware entrypoints now specify `Quadrotor` explicitly with `ROSFlight::<_, Quadrotor, _, _>::init`.

Current boundary status:

- `ROSFlight` no longer has HList-era configuration wiring.
- Hardware entrypoints construct the scheduler from concrete resources only.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core board::dummy::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

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
- `Params::iter` and `ParamIter` have since been removed from `params`; parameter listing now flows through `PARAM_DEFINITIONS` and the explicit request/event path.

## Stale Params Iterator Removal Progress

Reason for this change:

- After the parameter protocol moved to explicit request events, `Params::iter` and `ParamIter` no longer had active call sites.
- Keeping that cloned iterator API made it look like parameter listing still depended on mutable scheduler-owned iterator state.

Design now implemented:

- Removed `Params::iter`.
- Removed `ParamIter`.
- Removed the iterator-only unit test.
- Parameter listing remains handled by `param_system` using `PARAM_DEFINITIONS` and `Params` read access.

Validation:

- `cargo test -p rustflight_core params::tests --lib` passes.
- `cargo test -p rustflight_core param_system::tests --lib` passes.
- `cargo test -p rustflight_core comm_manager::tests --lib` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.

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

## Body Type Boundary Cleanup

Reason for this change:

- After HLIST removal, `BodyModel` and `BodyType` described the same compile-time aircraft body mapping.
- `ROSFlight` already used `BodyType`, while `World` still used `BodyModel`.
- Keeping both traits made the architecture look split even though there is now only one body abstraction.

Design now implemented:

- Removed the redundant `BodyModel` trait.
- Removed the duplicate `BodyModel` implementation for `Quadrotor`.
- Updated `World` to use `BodyType`, matching the `ROSFlight` body boundary.
- Updated world tests to construct the quadrotor mixer through `BodyType`.

Current status after this slice:

- `BodyType` is the single body-level associated-type boundary for estimator, controller, and mixer selection.
- `World` and `ROSFlight` now agree on the body abstraction.
- HLIST-era body model duplication is gone from core, sim, PixRacerPro, and Nucleo source scans.

Validation:

- `rg -n "BodyModel" rustflight_core/src sim/src pixracerpro/src nucleo/src` returns no matches.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

## Embedded World Entrypoint Progress

Reason for this change:

- Sim already used the new `World` scheduler.
- PixRacerPro and Nucleo still instantiated the legacy `ROSFlight` scheduler even though the board, body, comms, and PWM boundaries now compile against `World`.
- Keeping embedded entrypoints on `ROSFlight` delayed the transition from a duplicated scheduler to one producer/consumer world path.

Design now implemented:

- Updated `pixracerpro/src/bin/rustflight.rs` to instantiate `World`.
- Updated `nucleo/src/bin/rustflight.rs` to instantiate `World`.
- Both embedded loops now call `world.run_comm_param_sensor_stages()`, matching the sim entrypoint.
- Removed a stale PixRacerPro board comment that referred to ROSFlight.

Current status after this slice:

- Sim, PixRacerPro, and Nucleo all instantiate the same `World` scheduler architecture.
- `ROSFlight` still exists in core for now, but it is no longer used by board or sim crates.
- The next cleanup can focus on retiring or reducing `rustflight_core::rosflight` itself once any remaining compatibility concerns are checked.

Validation:

- `rg -n "ROSFlight|rosflight::" pixracerpro/src nucleo/src sim/src` returns no matches.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

## Legacy ROSFlight Scheduler Removal

Reason for this change:

- Sim, PixRacerPro, and Nucleo now all instantiate `World`.
- `rustflight_core::rosflight` had no live crate call sites after the embedded entrypoint migration.
- Keeping the duplicate scheduler made the architecture look split and kept old compatibility code in the active core API surface.

Design now implemented:

- Removed `rustflight_core/src/rosflight.rs`.
- Removed the `pub mod rosflight` export from `rustflight_core`.
- Updated current README wording to describe `World`, `BoardIo`, and PWM drivers instead of the old `Configuration`/`ROSFlight`/`BoardTrait` wiring.

Current status after this slice:

- The active scheduler path is `World`.
- Board and sim crates do not reference the legacy `ROSFlight` scheduler.
- Remaining source uses of `rosflight` are protocol/dialect names, not the removed scheduler module.

Validation:

- `rg -n "HList|HLIST|BoardTrait|BodyModel|pub mod rosflight|struct ROSFlight|rosflight::" rustflight_core/src pixracerpro/src nucleo/src sim/src README.md` returns only protocol/dialect timestamp references.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

## Stale Params Module Removal

Reason for this change:

- `params` is the active parameter API used by core, sim, PixRacerPro, and Nucleo.
- The old `rustflight_core/src/params.rs` file was no longer exported and had no live call sites.
- Keeping the stale module made the parameter boundary look duplicated after the event/port parameter migration.

Design now implemented:

- Removed `rustflight_core/src/params.rs`.
- Removed the stale commented `pub mod params` line from `rustflight_core/src/lib.rs`.
- Removed a stale commented `crate::params::Params` import from `packets.rs`.

Current status after this slice:

- `params` is the only parameter module in active source.
- No active source imports `crate::params`.

Validation:

- `rg -n "params::|pub mod params|mod params|params\\.rs|crate::params" rustflight_core/src pixracerpro/src nucleo/src sim/src README.md` returns only `params` matches.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt --check` passes.

## Stale Units Module Removal

Reason for this change:

- `rustflight_core/src/units.rs` was no longer exported.
- Active source had no references to `crate::units` or the old `ROSFlightTimestamp` type.
- Keeping the file left another unused pre-`World` API surface in core.

Design now implemented:

- Removed `rustflight_core/src/units.rs`.
- Removed the stale commented module line from `rustflight_core/src/lib.rs`.

Current status after this slice:

- Core has no active or commented `units` module export.
- The remaining ROSflight timestamp handling is owned by the MAVLink/protocol path.

Validation:

- `rg -n "units::|mod units|pub mod units|pub\\(crate\\) mod units|ROSFlightTimestamp|crate::units" rustflight_core/src pixracerpro/src nucleo/src sim/src README.md` returns no matches.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt --check` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

## Stale Param Types Removal

Reason for this change:

- The old `rustflight_core/src/params/param_types.rs` scaffolding was not exported.
- Active parameter IDs, values, defaults, and definitions now live in `params`.
- Keeping the stale directory left a second, inactive parameter model beside the active one.

Design now implemented:

- Removed `rustflight_core/src/params/param_types.rs`.

Current status after this slice:

- The old `params/` source directory is gone.
- Active source uses `params` for parameter values and definitions.

Validation:

- `rg --files rustflight_core/src | sort` shows no `rustflight_core/src/params/...` files.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt --check` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

## Params Module Rename

Reason for this change:

- After removing the stale old `params` module and `params/` scaffolding, the active parameter API was still named `params2`.
- That name was a migration artifact and made the completed parameter path look temporary.

Design now implemented:

- Renamed `rustflight_core/src/params2.rs` to `rustflight_core/src/params.rs`.
- Updated core, sim, PixRacerPro, and Nucleo imports from `params2` to `params`.
- Updated the architecture log references for the current active parameter module.

Current status after this slice:

- `params` is the active parameter module.
- No active source reference to `params2` remains.

Validation:

- `rg -n "params2|crate::params2|pub mod params2" rustflight_core/src pixracerpro/src nucleo/src sim/src README.md` returns no matches.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt --check` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

## World Control Resource And Run Naming

Reason for this change:

- `World` stored retained control-pipeline facts directly as scheduler fields.
- `latest_state`, `latest_actuator_commands`, and the IMU timestamp gate are retained world-owned data, not temporary system contexts.
- The main loop method was still named `run_comm_param_sensor_stages`, even though it also ran RC, command, state, PWM, and control stages.

Design now implemented:

- Added `ControlPipelineResource` for retained control-pipeline data.
- Moved latest estimator state, latest actuator commands, and last IMU timestamp into that resource.
- Renamed the full loop method to `run_once`.
- Kept `run_comm_param_sensor_stages` as the partial comm/param/sensor stage method used by focused tests.

Current status after this slice:

- Retained control outputs are grouped as a resource.
- No stored field is named as a control context; context remains reserved for future temporary borrow bundles passed to system functions.
- Sim, PixRacerPro, and Nucleo call `World::run_once()`.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt --check` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

## Stale Comment Cleanup

Reason for this change:

- After the HList/ROSFlight/params cleanup, active source still contained commented-out imports, commented debug statements, and obsolete migration comments.
- Those comments made it harder to distinguish real remaining work from old scaffolding.

Design now implemented:

- Removed commented-out imports/modules and commented debug print statements from active source.
- Removed obsolete comments that referenced already-completed logging or old scheduler movement.
- Kept real unresolved TODOs, such as RC-loss handling, board LED updates, VCP priority, and PixRacerPro PWM telemetry.

Current status after this slice:

- Active source no longer carries commented-out import/module/debug scaffolding.
- Remaining TODO scan hits represent actual unresolved work or ordinary explanatory comments.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt --check` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

## Architecture Review Checkpoint

Reason for this review:

- The branch had completed major HLIST, `ROSFlight`, `BoardTrait`, `BodyModel`, and params cleanup.
- Before continuing implementation, the codebase needed a synchronous review of whether the new architecture was becoming more understandable and closer to the original migration goal.

Reviewer findings:

- Understandability: `World::run_comm_param_sensor_stages` mixed too many pipeline phases in one method, and `World` exposed nearly every retained resource publicly.
- Goal progress: the active architecture is on track. Core, sim, PixRacerPro, and Nucleo now run through `World`, named resources, bounded queues, staged systems, and `BoardIo`; HLIST is no longer part of the source architecture.
- Verbosity: repeated World test setup, broad public fields, and generated-looking comments created noise that did not help a programmer interact with the code.
- Confusing artifacts: the top of this document still described an early parallel HLIST migration, board files still carried commented positional sensor examples, and embedded entrypoint file headers still used the old `typed_test.rs` label.

Recommended next ordered steps:

1. Update this architecture document so the opening state matches the current source architecture and preserve the review report as a handoff checkpoint.
2. Split the large `World::run_comm_param_sensor_stages` body into smaller named stage helpers without changing stage order.
3. Make `World` retained resources private so extension goes through explicit scheduler methods instead of external field mutation.
4. Remove stale HLIST/debug/comment artifacts from PixRacerPro, Nucleo, sim PWM, and embedded entrypoint headers.
5. Add a shared World test fixture/builder and move repeated baseline setup to it.

Current status:

- The architecture document opening now describes the current `World`/ports/events/resource architecture rather than the original parallel HLIST migration state.
- `World::run_comm_param_sensor_stages` now delegates to named private stage helpers while preserving the existing execution order.
- `World` retained resources are private to the scheduler.
- Stale HLIST/debug/comment artifacts from PixRacerPro, Nucleo, sim PWM, and embedded entrypoint headers have been removed.
- World tests now share a baseline fixture for the common `TestBoard`/`Quadrotor`/`RecordingCommLink`/`TestPwm` setup.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt --check` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

## World Stage Context Boundary Progress

Reason for this change:

- After the review cleanup, the remaining direct scheduler coupling was concentrated in `World::run_rc_command_state_stages`, `World::run_pwm_output_stage`, and `World::run_control_stages_if_new_imu`.
- The goal is to keep `World` as the deterministic scheduler/wiring owner while making producer/consumer ownership visible through explicit context structs.
- Runtime event producers also ignored queue overflow in several places, which hid dropped events.

Design now implemented:

- Added `rc_system::RcCommandStateCtx` and `run_rc_command_state` for the RC packet, RC manager, command manager, state manager, and params handoff.
- Added `pwm_system::PwmSyncCtx` so PWM enable/disable synchronization is an explicit context call.
- Added `control_system::ControlPipelineResource`, `ControlPipelineCtx`, and `run_control_pipeline_if_new_imu` for estimator, controller, mixer, PWM output, telemetry, auxiliary command, and external-attitude flow.
- `World` now constructs those contexts and delegates the stage work instead of owning the detailed control flow inline.
- Added `EventQueue::push_or_log` and `EventEmitPort::emit_or_log`.
- Runtime comm, command, and param producers now log and drop the new event when the target queue is full. Log draining still stops when the comm response queue is full to avoid recursively logging while draining logs.

Current status after this slice:

- Core stage boundaries are closer to the desired producer/consumer/context model.
- `World` remains the central deterministic schedule, but RC/command/state, PWM sync, and control pipeline work now have named borrow contexts.
- Event overflow handling is explicit for normal runtime producers instead of silent `let _ = ...` drops.
- Existing hardware entrypoints required no source changes; they still compile against the updated core.

Validation:

- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core rc_system::tests --lib` passes.
- `cargo test -p rustflight_core pwm_system::tests --lib` passes.
- `cargo test -p rustflight_core events::tests --lib` passes.
- `cargo test -p rustflight_core ports::tests --lib` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt --check` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

Known unrelated test status:

- `cargo test -p rustflight_core --lib` still hits the pre-existing state-machine `UNCALIBRATED_IMU` failures documented earlier; the affected World, RC, PWM, event, and port tests pass.

## State Machine Calibration Boundary Fix

Reason for this change:

- The next blocker was the long-standing `state_machine::tests` failure set.
- Those failures came from local `REQUEST_ARM` logic inspecting gyro bias params and inserting `UNCALIBRATED_IMU` directly inside the state machine.
- That made the state manager responsible for sensor calibration knowledge and kept full core validation red.

Upstream behavior checked:

- Current upstream `rosflight_firmware` main was inspected at commit `099a9846406d9f20b2bae08a2ea3dda74a01cf59`.
- Upstream `src/sensors.cpp` sets `ERROR_UNCALIBRATED_IMU` when all accel and gyro bias params are zero:
  `ACC_X_BIAS`, `ACC_Y_BIAS`, `ACC_Z_BIAS`, `GYRO_X_BIAS`, `GYRO_Y_BIAS`, and `GYRO_Z_BIAS`.
- Upstream `src/state_manager.cpp` does not compute IMU calibration from params on arm. It refuses arming through the existing error state when `ERROR_UNCALIBRATED_IMU` is already present.

Design now implemented:

- Removed the local state-machine gyro-bias inspection from `Preflight::REQUEST_ARM`.
- Kept `CAL_GYRO_ARM` behavior: when requested, arming enters `Calibrating` and `CALIBRATION_COMPLETE` transitions to `Armed`.
- Added a focused test proving an existing `UNCALIBRATED_IMU` error blocks arming.
- Cleaned up overflow-helper tests so warning logs do not leak into `log_system` tests when the full lib suite runs in parallel.

Current status after this slice:

- State-machine tests now match the upstream ownership boundary: sensors own calibration error production; state machine owns state transitions based on existing errors.
- Full `rustflight_core` library tests are green.
- The remaining calibration parity work is to move the upstream six-bias `UNCALIBRATED_IMU` production/clear behavior into the sensor/World path, not back into `StateMachine`.

Validation:

- `cargo test -p rustflight_core state_machine::tests --lib` passes.
- `cargo test -p rustflight_core --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt --check` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.

## IMU Calibration Health System Progress

Reason for this change:

- After moving calibration knowledge out of the state machine, the missing parity piece was to produce and clear `UNCALIBRATED_IMU` from the sensor health path.
- Upstream ROSflight 2.x sets the uncalibrated IMU error from the sensor path when all six accel and gyro bias params are zero.
- IMU health is safety-critical enough that this should be a named system/context, not an incidental helper hidden in `World`.

Design now implemented:

- Added `sensor_health_system::ImuCalibrationHealthCtx`.
- Added `sensor_health_system::update_imu_calibration_error`.
- `World::update_sensor_health_and_calibration` now delegates IMU calibration error ownership to that system when a processed IMU sample is present.
- The system sets `ErrorFlag::UNCALIBRATED_IMU` when all six bias params are zero:
  - `ACC_X_BIAS`
  - `ACC_Y_BIAS`
  - `ACC_Z_BIAS`
  - `GYRO_X_BIAS`
  - `GYRO_Y_BIAS`
  - `GYRO_Z_BIAS`
- The system clears `ErrorFlag::UNCALIBRATED_IMU` when any of those bias params is nonzero.
- The no-IMU path remains owned by `IMU_NOT_RESPONDING`; calibration status is only updated after valid IMU processing.

Current status after this slice:

- The sensor health system now owns the upstream-style calibration error production/clear behavior.
- State machine remains responsible only for transitions based on existing errors.
- Full core library tests remain green.

Validation:

- `cargo test -p rustflight_core world::tests --lib` passes.
- `cargo test -p rustflight_core sensor_health_system::tests --lib` passes.
- `cargo test -p rustflight_core --lib` passes.
- `cargo check -p rustflight_core --lib` passes.
- `cargo check -p sim` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo fmt --check` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p pixracerpro --target thumbv7em-none-eabihf` passes.
- `RUSTUP_HOME=/workspace/home/.rustup CARGO_HOME=/workspace/.cargo-home cargo check -p nucleo --target thumbv7em-none-eabihf` passes.
