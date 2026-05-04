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
