# Voloxide Architecture Guide

This guide defines the vocabulary used in the current Voloxide codebase and then walks the
architecture in the same order a reader would encounter it from `sim/`, through the ROS 2 shim,
through the Rust FFI boundary, into `voloxide_core::World`, and back out through MAVLink and PWM.

The architecture is static ECS-style Rust:

```text
resources + systems + events + explicit scheduler
```

It is not a dynamic ECS framework. There are no runtime component tables or entity IDs in the
flight stack. The main idea is that `World` owns long-lived resources, systems receive small context
structs, and events move work between systems without letting every subsystem mutate every other
subsystem directly.

## Vocabulary

Use these terms consistently when reading or editing this codebase.

| Term | Meaning in Voloxide | Concrete examples |
| --- | --- | --- |
| Contract | A trait that defines replaceable behavior. Contracts let core depend on capability rather than a concrete runtime. | `BoardIo`, `CommInterface`, `Estimator`, `Controller`, `Mixer`, `PwmDriver` |
| Resource | Long-lived state owned by `World` or by a runtime boundary. Resources are the ECS-style singleton data for the flight stack. | `Params`, `SensorBus`, `ProcessedSensors`, `Rc`, `CommandManager`, `StateManager`, `PwmOutputState` |
| System | A function that performs one scheduled operation over a small set of resources. Systems do not receive `&mut World`; they receive a context. | `apply_param_requests`, `run_rc_command_state`, `update_sensor_health`, `run_control_pipeline_if_new_imu` |
| Context | A struct containing the exact borrows a system needs. A context is the system's dependency list. | `ParamApplyCtx`, `RcCommandStateCtx`, `SensorHealthCtx`, `ControlPipelineCtx` |
| Event | A small copyable message placed in a bounded queue so one stage can request work from a later stage. | `ParamSetRequested`, `ParamChanged`, `CommResponse`, `OffboardControlRequested` |
| Event queue | A fixed-capacity FIFO queue for one event type. | `EventQueue<T, N>` |
| Emit | Push an event into an event queue. | `EventEmitPort::emit_or_log`, `EventQueue::push_or_log` |
| Drain | Pop events from a queue until the receiver is done. Draining consumes the events. | `EventDrainPort::next` |
| Read | Iterate over queued events without consuming them. | `EventReadPort::iter` |
| Port | A narrow wrapper that grants a system one kind of access to a resource or queue. | `ParamsReadPort`, `ParamsWritePort`, `EventEmitPort`, `EventDrainPort`, `EventReadPort` |
| Producer | Code that emits an event or writes an input resource for later use. | `CommManager::act_on_messages`, ROS shim subscriptions, `FfiBoard::update_sensor_bus` |
| Receiver | Code that drains or reads events and applies the requested work. | `params::service`, `command::service`, `companion`, `params::reactions` |
| Scheduler | Code that owns ordering. In core, this is `World::run_once` and the stage methods it calls. | `run_communication_and_parameter_service_stage`, `run_sensor_ingestion_and_health_stage` |
| Stage | A named scheduler section grouping related systems in order. | communication/parameter service, sensor ingestion/health, RC/state, control/mixing, telemetry |
| Adapter | Code outside `voloxide_core` that connects core contracts to a concrete runtime or protocol. | `voloxide_mavlink`, `sim`, `pico2w`, `pixracerpro`, `nucleo`, ROS 2 shim |
| Boundary | A place where one layer hands data to another layer through an explicit API. | ROS 2 C++ shim to Rust FFI, `CommInterface`, `BoardIo`, `PwmDriver` |
| Wire message | Protocol-shaped data at a communication boundary. Core keeps protocol-neutral message structs; MAVLink encoding lives in `voloxide_mavlink`. | `ParamValueMsg`, `RosflightCmdMsg`, `StatustextMsg` |
| Packet | Sensor or actuator data in Voloxide's firmware-facing representation. | `ImuPacket`, `RcPacket`, `BaroPacket`, `BatteryPacket` |

## Repository Tree

```text
Voloxide/
├── crates/
│   └── voloxide_core/
│       └── src/
│           ├── lib.rs
│           ├── world.rs
│           ├── board.rs
│           ├── comm.rs
│           ├── comm/
│           │   ├── interface.rs
│           │   └── messages.rs
│           ├── params.rs
│           ├── params/
│           │   ├── service.rs
│           │   └── reactions.rs
│           ├── command.rs
│           ├── command/
│           │   └── service.rs
│           ├── companion.rs
│           ├── sensors.rs
│           ├── sensors/
│           │   ├── ingestion.rs
│           │   ├── processors.rs
│           │   └── health.rs
│           ├── rc.rs
│           ├── rc/
│           │   └── system.rs
│           ├── control.rs
│           ├── pwm.rs
│           ├── pwm/
│           │   └── system.rs
│           ├── estimator.rs
│           ├── estimator/
│           │   └── quad.rs
│           ├── controller.rs
│           ├── controller/
│           │   └── quad.rs
│           ├── mixer.rs
│           ├── mixer/
│           │   └── matrix.rs
│           ├── state_machine.rs
│           ├── log.rs
│           ├── events.rs
│           ├── ports.rs
│           ├── packets.rs
│           ├── errors.rs
│           └── vehicle.rs
├── comms/
│   └── voloxide_mavlink/
│       └── src/
│           ├── link.rs
│           ├── conversions.rs
│           └── parser.rs
├── platforms/
│   ├── rp2350/
│   └── stm_32/
│       └── src/
│           └── peripherals/
├── boards/
│   ├── pico2w/
│   ├── nucleo/
│   └── pixracerpro/
├── sim/
│   ├── firmware/
│   │   └── src/
│   │       ├── ffi.rs
│   │       ├── board.rs
│   │       ├── pwm.rs
│   │       └── bin/
│   │           └── voloxide.rs
│   └── ros2/
│       └── voloxide_sil_board_shim/
│           ├── src/
│           ├── include/
│           └── launch/
├── docs/
├── scripts/
└── xtask/
```

## Dependency Direction

The intended dependency direction is:

```text
voloxide_core
├── has contracts
├── has resources
├── has systems
├── has scheduler
└── does not know MAVLink, ROS 2, or board startup

voloxide_mavlink
├── depends on voloxide_core
├── implements CommInterface
├── parses MAVLink frames
├── builds MAVLink frames
└── converts between MAVLink wire types and core comm messages

sim
├── depends on voloxide_core
├── depends on voloxide_mavlink
├── provides FFI board/PWM adapters for ROS 2 shim
└── exposes the simulator firmware through the ROS 2 shim FFI path

sim/ros2/voloxide_sil_board_shim
├── is a ROS 2 rclcpp package in this repo
├── subscribes/publishes ROSflight simulator topics
├── exposes sil_board/run
└── calls the Rust sim crate through C ABI

pico2w / pixracerpro / nucleo
├── choose board/PWM/comm concrete types
├── instantiate World for embedded targets
└── use platform crates for chip-family support where useful
```

Core should never depend outward on `sim`, `voloxide_mavlink`, ROS 2 packages, or board crates.

## Contracts

Contracts are the traits that let `World` remain generic:

```text
World<B, E, C, M, CI, PD>
├── B:  BoardIo
├── E:  Estimator
├── C:  Controller<State = E::State> + RcTrimCalibrator
├── M:  Mixer<MixerInput = C::ControlOutput>
├── CI: CommInterface<B>
└── PD: PwmDriver
```

The contracts are the branch points for concrete runtime behavior.

```text
BoardIo
├── update_sensor_bus
├── serial_rx_read
├── serial_tx_write
├── clock_millis
├── clock_micros
├── read_params
└── write_params

CommInterface
├── handle_incoming_messages
├── send_named_value
├── send_cmd_ack
├── send_statustext
├── send_hard_error
└── send telemetry messages

PwmDriver
├── enable_all
├── disable_all
├── configure_output_rates
├── send_commands
└── flush
```

The core scheduler does not branch on "am I sim or hardware?" Instead, it calls the contract. The
concrete implementation decides what that means.

## Resources In `World`

`World` owns the flight-stack resources:

```text
World
├── board: B
├── params: Params
├── param_list_state: ParamListState
├── param_events: ParamEventQueues
├── comm_events: CommEventQueues
├── command_events: CommandEventQueues
├── companion_events: CompanionEventQueues
├── companion_link: CompanionLinkState
├── pending_hard_error: Option<RosflightHardErrorMsg>
├── aux_commands: AuxCommandState
├── external_attitude: ExternalAttitudeState
├── comm: CommManager<B, CI>
├── raw_sensors: SensorBus
├── processed_sensors: ProcessedSensors
├── sensor_processors: SensorProcessorSet
├── rc: Rc
├── command: CommandManager
├── state: StateManager
├── cal_flags: CalibrationFlags
├── estimator: E
├── controller: C
├── mixer: M
├── control_pipeline: ControlPipelineResource
├── pwm_output: PwmOutputState
├── pwm: PD
└── last_imu_seen: u64
```

Those fields are not handed wholesale to systems. `World` creates contexts from them.

## Events And Ports

Events are declared in `crates/voloxide_core/src/events.rs`.

```text
ParamEventQueues
├── set_requests:  EventQueue<ParamSetRequested>
├── read_requests: EventQueue<ParamReadRequested>
├── list_requests: EventQueue<ParamListRequested>
└── changes:       EventQueue<ParamChanged>

CommandEventQueues
├── calibration_requests
├── offboard_control_requests
├── param_defaults_requests
├── board_command_requests
├── rc_trim_calibration_requests
├── version_requests
├── reset_origin_requests
└── config_info_requests

CompanionEventQueues
├── heartbeats
├── aux_commands
└── external_attitudes

CommEventQueues
└── responses: EventQueue<CommResponse>
```

Ports restrict access:

```text
EventEmitPort<T>
└── emit / emit_or_log

EventDrainPort<T>
└── next

EventReadPort<T>
└── iter

ParamsReadPort
└── get / raw

ParamsWritePort
├── get
└── set
```

The difference between drain and read matters:

```text
drain
├── receiver consumes the events
└── used for request queues

read
├── receiver observes events without removing them
└── used when several reactions need to observe the same event batch
```

Parameter reactions use read ports because both RC and command logic need to see the same
`ParamChanged` batch before `World` clears it.

## Contexts

A context is the only thing a system receives. That makes the system's authority visible.

Example:

```rust
pub struct ParamApplyCtx<'a> {
    pub params: ParamsWritePort<'a>,
    pub requests: EventDrainPort<'a, ParamSetRequested, PARAM_SET_REQUEST_QUEUE_CAPACITY>,
    pub changes: EventEmitPort<'a, ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
}
```

This means `apply_param_requests` can:

```text
├── write Params
├── drain ParamSetRequested
├── emit ParamChanged
└── emit CommResponse
```

It cannot:

```text
├── touch RC directly
├── touch CommandManager directly
├── call board APIs
└── send MAVLink bytes directly
```

Another example:

```rust
pub struct RcCommandStateCtx<'a> {
    pub now_ms: u32,
    pub sensors: &'a ProcessedSensors,
    pub rc: &'a mut Rc,
    pub command: &'a mut CommandManager,
    pub state: &'a mut StateManager,
    pub params: &'a Params,
}
```

This tells the reader that the RC/state system is allowed to read processed sensors and params,
mutate RC, mutate command state, and mutate the flight state machine.

## Scheduler

`World::run_once` is the generic scheduler entrypoint. It is used by simulation, host tests, and
boards that do not provide a board-specific realtime loop.

```text
run_once
├── run_communication_and_parameter_service_stage
├── run_sensor_ingestion_and_health_stage
├── run_rc_command_state_stages
├── run_control_and_mixing_stage_if_new_imu
└── run_telemetry_stage
```

The first stage is:

```text
run_communication_and_parameter_service_stage
├── process_comm_stage
├── apply_companion_events
├── apply_command_events
├── service_param_events
├── apply_param_reactions
└── request_gyro_calibration_if_needed
```

The sensor stage is:

```text
run_sensor_ingestion_and_health_stage
├── board.clock_micros
├── run_sensor_ingestion_stage
├── update_sensor_health_and_calibration
└── drain_logs_and_send_responses
```

The RC/state stage is:

```text
run_rc_command_state_stages
├── run_rc_command_state
├── run_pwm_output_stage
└── update_board_leds
```

The control stage is:

```text
run_control_and_mixing_stage_if_new_imu
└── run_control_pipeline_if_new_imu
    ├── require new IMU timestamp
    ├── estimator.estimate_with_external_attitude
    ├── controller.control
    ├── mixer.mix
    ├── pwm.configure_output_rates
    ├── compose_pwm_outputs
    ├── write_pwm_commands
    └── update ControlPipelineResource
```

The telemetry stage is:

```text
run_telemetry_stage
└── comm.send_named_telemetry_streams
    ├── status
    ├── attitude
    ├── IMU
    ├── baro/mag/range/GNSS/battery/diff pressure
    ├── RC raw
    └── output raw
```

The scheduler is deliberately explicit. Ordering is flight behavior.

## Realtime Scheduler

Board crates may use the finer-grained realtime scheduler methods instead of calling `run_once`
directly. The Pico 2 W firmware does this so an IMU data-ready event can preempt slower service
work without removing the ECS-style stage boundaries from `World`.

The realtime scheduler decision is:

```text
realtime_scheduler_step
├── ImuControl if BoardIo::imu_pending()
├── Service if a deferred service phase is due and still early after the last control closure
└── Idle otherwise
```

The IMU control path is intentionally short:

```text
run_imu_control_tick
├── board.update_imu_sensor
├── sensors::ingestion::process_imu_sensor
├── update_sensor_health_and_calibration
└── run_control_and_mixing_stage_if_new_imu
    └── estimator/controller/mixer/PWM for a new IMU timestamp
```

This path drains only the IMU producer queue. It does not run communication, telemetry, parameter
service, non-IMU sensors, RC command/state, log drain, serial flush, or deferred board actions.
Those operations are still part of the same `World` architecture; they are sliced into service
phases:

```text
run_service_step_with_deferral
├── Input
│   └── run_communication_and_parameter_service_stage
├── SensorsRc
│   ├── board.update_service_sensor_bus
│   ├── process non-IMU sensor packets
│   ├── update_sensor_health_and_calibration
│   └── run_rc_command_state_stages
├── Responses
│   └── drain_logs_and_send_responses_limited
├── Telemetry
│   └── run_realtime_telemetry_stage
├── Flush
│   └── board.serial_flush_budgeted
└── DeferredBoard
    └── board.run_deferred_board_actions
```

Each service call advances one phase. On the Pico 2 W, a service phase is allowed only when no IMU
sample is pending and the loop is still within the configured post-control service window. This is
what keeps telemetry, MAVLink command handling, RC interpretation, and board maintenance from
starting late enough to steal time from the next IMU close-loop pass.

RC command/state is deliberately in `SensorsRc`, not in `run_imu_control_tick`. CRSF packet parsing
and queuing are board work; interpreting the newest RC packet, updating the command mux, running the
state machine, syncing PWM output enable state, and updating LEDs are core `World` work. Keeping that
work in a service phase preserves the expected ROSflight behavior while preventing variable RC
packet arrivals from adding jitter to every IMU control closure.

On RP2350/Pico 2 W, IMU sampling, control cadence, and telemetry cadence are separate choices. The
default firmware samples the ISM330DHCX at the high-rate ODR, runs the full control pipeline at
`2 kHz`, and publishes bounded high-rate MAVLink telemetry. The board entry point is
`boards/pico2w/src/bin/voloxide.rs`; `imu-odr-1666hz` is the lower-rate hardware IMU override and
`ism330dhcx-1k666` remains only as a compatibility alias. Core 1 owns transport and producer work,
while the ISM330DHCX producer runs on an Embassy interrupt executor driven by `SIO_IRQ_BELL`. The
`scope-timing-pins` family exposes GP19 for control timing plus GP22 for the selected substage.
Barometer and magnetometer work should remain in the service-side sensor path so adding those
sensors does not turn the IMU interrupt path into a multi-sensor polling loop.

The retained STM32 boards use the generic scheduler shape rather than the Pico 2 W realtime split.
Their board crates instantiate `World`, wire STM32 peripheral tasks through the `BoardIo` contract,
and run the ordinary firmware loop while renewed sensor bring-up is completed. That difference is
intentional: RP2350 currently has measured high-rate hardware timing requirements, while the STM32
paths are compile-current retained targets awaiting fresh hardware validation.

## End-To-End Flow: ROSflight Standalone Sim

The active ROSflight integration path is:

```text
rosflight_sim standalone multirotor
├── publishes sensor topics
├── calls sil_board/run through rosflight_sil_manager
└── consumes sim/pwm_output

sim/ros2/voloxide_sil_board_shim
├── subscribes simulator sensor topics
├── subscribes sim/RC
├── exposes sil_board/run
├── builds VoloxideFfiSensorSnapshot
├── calls voloxide_sim_set_sensors
├── calls voloxide_sim_run_once
├── calls voloxide_sim_get_pwm
└── publishes sim/pwm_output

sim/firmware/src/ffi.rs
├── owns FfiBoard
├── owns FfiPwmDriver
├── instantiates World<FfiBoard, QuadEstimator, QuadController, MatrixMixer, MavlinkInterface, FfiPwmDriver>
├── maps FFI snapshots into SensorBus packets
├── maps PwmDriver commands into shared PWM outputs
└── owns UDP MAVLink socket for rosflight_io

voloxide_core::World
├── runs scheduler
├── consumes BoardIo sensors
├── processes MAVLink through CommInterface
├── runs estimator/controller/mixer
├── writes PWM through PwmDriver
└── queues telemetry responses

voloxide_mavlink
├── parses incoming MAVLink from rosflight_io
├── fills core Messages
├── serializes outgoing core comm messages
└── writes UDP bytes through BoardIo serial_tx_write

unmodified rosflight_io
├── sends param/command/offboard messages over UDP MAVLink
├── receives status/params/telemetry over UDP MAVLink
└── exposes ROS services and topics used by tests
```

## Reading `sim/firmware/src/ffi.rs`

`sim/firmware/src/ffi.rs` is the Rust side used by the ROS 2 C++ shim.

```text
sim/firmware/src/ffi.rs
├── FFI data structs
│   ├── VoloxideFfiImu
│   ├── VoloxideFfiMag
│   ├── VoloxideFfiBaro
│   ├── VoloxideFfiGnss
│   ├── VoloxideFfiAirspeed
│   ├── VoloxideFfiRange
│   ├── VoloxideFfiBattery
│   ├── VoloxideFfiRc
│   └── VoloxideFfiSensorSnapshot
├── FfiPwmDriver
│   └── implements PwmDriver
├── FfiBoard
│   └── implements BoardIo
├── FfiWorld type alias
├── VoloxideFfiHandle
├── voloxide_sim_create
├── voloxide_sim_destroy
├── voloxide_sim_set_sensors
├── voloxide_sim_run_once
└── voloxide_sim_get_pwm
```

The FFI snapshot is the input boundary from C++ into Rust:

```text
VoloxideFfiSensorSnapshot
├── has_imu + imu
├── has_mag + mag
├── has_baro + baro
├── has_gnss + gnss
├── has_airspeed + airspeed
├── has_range + range
├── has_battery + battery
└── has_rc + rc
```

`FfiBoard::update_sensor_bus` maps the latest snapshot into firmware packets:

```text
VoloxideFfiSensorSnapshot
└── FfiBoard::update_sensor_bus
    ├── ImuPacket
    ├── MagPacket
    ├── BaroPacket
    ├── GNSSPacket
    ├── PitotPacket
    ├── RangePacket
    ├── BatteryPacket
    └── RcPacket
```

`FfiBoard` also owns the MAVLink UDP socket:

```text
FfiBoard
├── serial_rx_read  -> UDP recv from rosflight_io
└── serial_tx_write -> UDP send to rosflight_io
```

That is why core still sees "serial" methods: `BoardIo` is the firmware contract, while the sim
adapter implements that contract using UDP.

`FfiPwmDriver` maps core PWM commands to shared output storage:

```text
World/control/pwm system
└── PwmDriver::send_commands
    └── FfiPwmDriver
        └── outputs: Arc<Mutex<[u16; 14]>>
```

The C++ shim later calls `voloxide_sim_get_pwm` and publishes those values as ROS 2
`sim/pwm_output`.

## Reading `sim/ros2/voloxide_sil_board_shim/src/voloxide_sil_board.cpp`

The C++ shim is the ROS 2 node boundary. It does not implement flight logic.

```text
VoloxideSilBoard node
├── node name: voloxide_sil_board
├── service: sil_board/run
├── publisher: sim/pwm_output
├── subscriptions
│   ├── sim/sensors/imu/data
│   ├── sim/sensors/imu/temperature
│   ├── sim/sensors/mag
│   ├── sim/sensors/baro
│   ├── sim/sensors/gnss
│   ├── sim/sensors/diff_pressure
│   ├── sim/sensors/range
│   ├── sim/sensors/battery
│   └── sim/RC
└── firmware_: VoloxideFfiHandle
```

Each subscription stores the latest ROS message and marks it available:

```text
ROS topic callback
├── latest_* = message
└── *_available = true
```

When `rosflight_sil_manager` calls `sil_board/run`, the shim executes:

```text
run_once
├── build_sensor_snapshot
├── voloxide_sim_set_sensors
├── voloxide_sim_run_once
├── voloxide_sim_get_pwm
└── publish_pwm
```

The shim uses a monotonic FCU clock for sensor timestamps:

```text
fcu_clock_micros
└── steady_clock since boot_time_
```

That prevents wall-clock jumps from becoming firmware time-backwards errors.

The FFI simulator requires `VOLOXIDE_SIM_PARAM_DIR` to point at a writable runtime directory before
`voloxide_sim_create` is called. The multirotor standalone launch defaults this to
`/tmp/voloxide-sim-params/multirotor`, and the launch argument `voloxide_param_dir:=...` can move it
to a persistent path.

## Simulator Integration Boundary

The simulator firmware package intentionally exposes only the FFI/staticlib path. Older direct
simulator experiments that subscribed to CDR messages on Zenoh from Rust were removed so the repo
has one supported ROSflight SIL path:

```text
ROSflight simulator topics
└── sim/ros2/voloxide_sil_board_shim
    └── sim/firmware/src/ffi.rs
        └── voloxide_core::World
```

`rmw_zenoh_cpp` may still be used as the ROS 2 middleware for the surrounding ROS graph, but the
Rust firmware crate does not depend on the Rust `zenoh` crate or start its own Zenoh session.

## End-To-End Flow: Parameter Set

This is the clearest example of events and contexts.

```text
rosflight_io
└── sends PARAM_SET over MAVLink
    └── voloxide_mavlink parses frame
        └── CommManager has msgs.param_set
            └── CommManager::act_on_messages
                └── emits ParamSetRequested
                    └── World::service_param_events
                        └── params::service::apply_param_requests
                            ├── drains ParamSetRequested
                            ├── writes Params
                            ├── emits ParamChanged
                            └── emits CommResponse::ParamValue
                                └── World::apply_param_reactions
                                    ├── rc_on_param_changed reads ParamChanged
                                    └── command_on_param_changed reads ParamChanged
                                        └── World::drain_logs_and_send_responses
                                            └── CommManager::send_comm_responses
                                                └── CommInterface::send_named_value
                                                    └── voloxide_mavlink writes MAVLink bytes
```

Ownership is split:

```text
comm.rs
└── translates decoded messages into events

params/service.rs
└── owns parameter request behavior

params/reactions.rs
└── owns consequences of parameter changes

comm.rs
└── owns sending queued communication responses

world.rs
└── owns ordering
```

## End-To-End Flow: Calibration Command

```text
rosflight_io
└── sends ROSFLIGHT_CMD_ACCEL_CALIBRATION or GYRO_CALIBRATION
    └── CommManager::act_on_messages
        └── emits CalibrationRequested
            └── World::apply_command_events
                └── command::service::apply_calibration_requests
                    ├── checks StateManager
                    ├── sets CalibrationFlags
                    ├── zeros relevant bias params
                    └── queues immediate ROSFLIGHT_CMD_ACK when calibration starts
                        └── sensor processors run calibration while sensor packets arrive
                            └── completion or failure updates calibration state/logs
```

The command service starts calibration and ACKs acceptance immediately, matching ROSflight 2.0.
The sensor processors finish calibration later and report completion/failure through state and logs,
not through a second command ACK.

## End-To-End Flow: RC And Arming

```text
ROS /sim/RC
└── C++ shim latest_rc_
    └── build_sensor_snapshot
        └── VoloxideFfiRc
            └── FfiBoard::update_sensor_bus
                └── RcPacket in SensorBus
                    └── sensors::ingestion::process_sensor_bus
                        └── ProcessedSensors.rc
                            └── rc::system::run_rc_command_state
                                ├── Rc::receive
                                ├── Rc::run
                                ├── CommandManager::run
                                └── StateManager::run
                                    └── World::run_pwm_output_stage
                                        └── pwm::system::sync_pwm_output_state
```

RC arming is not handled in the ROS shim. The shim only passes RC input through. The firmware logic
inside `Rc`, `CommandManager`, and `StateManager` decides whether the vehicle arms.

## End-To-End Flow: Control And PWM

```text
new IMU packet
└── sensors::ingestion
    └── ProcessedSensors.imu
        └── control::run_control_pipeline_if_new_imu
            ├── reject non-advancing IMU time
            ├── estimator.estimate_with_external_attitude
            ├── update estimator health error
            ├── controller.control
            ├── mixer.mix
            ├── update mixer health error
            ├── pwm.configure_output_rates
            ├── pwm::system::compose_pwm_outputs
            ├── pwm::system::write_pwm_commands
            └── ControlPipelineResource::set_latest
                └── telemetry later reads latest estimator/control/PWM state
```

`ControlPipelineCtx` is intentionally large because control is the point where estimator,
controller, mixer, state, sensors, auxiliary commands, and PWM meet. Even there, the dependencies
are explicit.

## End-To-End Flow: Telemetry And Logs

```text
systems
├── emit CommResponse events
└── write logs through log macros

World::drain_logs_and_send_responses
├── log::drain::drain_logs_to_comm_responses
│   └── emits CommResponse::Statustext when companion is connected
├── CommManager::send_comm_responses
│   └── sends queued response messages through CommInterface
├── board.serial_flush
└── board.run_deferred_board_actions

World::run_telemetry_stage
└── CommManager::send_named_telemetry_streams
    └── sends periodic telemetry through CommInterface
```

`CommResponse` is the common event type for deferred communication output:

```text
CommResponse
├── ParamValue
├── CmdAck
├── Version
├── Statustext
└── HardError
```

## Branching Rules

When reading code, classify each branch by where it belongs:

```text
Runtime branch
├── belongs outside core
├── examples: sim vs pixracerpro, MAVLink vs another comm implementation
└── implemented by choosing concrete contract implementations

Flight-behavior branch
├── belongs in core systems/resources
├── examples: armed vs disarmed, failsafe vs normal, new IMU vs no new IMU
└── implemented in systems such as state_machine, rc/system, control, pwm/system

Protocol branch
├── belongs in comm adapter or comm manager
├── examples: PARAM_SET vs ROSFLIGHT_CMD vs OFFBOARD_CONTROL
└── decoded into protocol-neutral core events/messages

Scheduler branch
├── belongs in World
├── examples: which stage runs before another stage
└── should stay explicit
```

Do not add runtime-specific branches to `voloxide_core` when a contract can express the same thing.

## Adding A New System

Use this checklist:

```text
1. Identify the owning domain.
   └── params, command, sensors, rc, pwm, log, etc.

2. Decide whether the behavior is a resource method or a system.
   ├── resource method: intrinsic behavior of one resource
   └── system: scheduled behavior involving multiple resources/events

3. Define events if the work is requested by another stage.
   └── add event type and queue in events.rs

4. Define a context.
   └── include only the needed borrows/ports

5. Implement the system.
   └── no &mut World

6. Wire the context in World.
   └── choose the exact scheduler stage

7. Add focused tests.
   ├── system test with local resources/queues
   └── World handoff test if ordering changed
```

## File Ownership Map

```text
board.rs
└── core board contract

comm.rs
├── protocol-neutral communication manager
├── decoded message handling
├── event emission from incoming messages
└── outgoing communication response sending

comm/interface.rs
└── communication contract implemented by adapters

comm/messages.rs
└── protocol-neutral ROSflight-shaped message structs/enums

params.rs
└── parameter definitions, IDs, defaults, storage

params/service.rs
└── parameter read/list/set request systems

params/reactions.rs
└── systems reacting to ParamChanged events

command.rs
└── command resource and control-source muxing

command/service.rs
└── command request systems and ACK emission

companion.rs
└── companion heartbeat, aux command, external attitude resources/systems

sensors.rs
└── SensorBus and ProcessedSensors resources

sensors/ingestion.rs
└── raw SensorBus to ProcessedSensors system

sensors/processors.rs
└── per-packet sensor correction/calibration processors

sensors/health.rs
└── sensor health and IMU calibration health system

rc.rs
└── RC resource and channel interpretation

rc/system.rs
└── processed RC packet, RC resource, command manager, state machine handoff

control.rs
└── estimator/controller/mixer/PWM control pipeline system

pwm.rs
└── PWM driver contract and protocol/rate helpers

pwm/system.rs
└── PWM enable state, output composition, command writing

state_machine.rs
└── armed/failsafe/error/calibrating state resource

log.rs
└── fixed-capacity global log queue and logging macros

log/drain.rs
└── log-to-CommResponse drain system

world.rs
└── resource owner and scheduler
```

## Code Reading Path For `sim/`

Use this order when stepping through the simulator integration:

```text
1. sim/ros2/voloxide_sil_board_shim/src/voloxide_sil_board.cpp
   ├── node construction
   ├── ROS subscriptions
   ├── sil_board/run service
   ├── build_sensor_snapshot
   └── publish_pwm

2. sim/ros2/voloxide_sil_board_shim/include/voloxide_sil_board_shim/voloxide_ffi.h
   └── C ABI declarations

3. sim/firmware/src/ffi.rs
   ├── FFI structs
   ├── FfiBoard::update_sensor_bus
   ├── FfiBoard serial_rx_read / serial_tx_write
   ├── FfiPwmDriver
   ├── voloxide_sim_create
   ├── voloxide_sim_set_sensors
   ├── voloxide_sim_run_once
   └── voloxide_sim_get_pwm

4. crates/voloxide_core/src/world.rs
   ├── World resources
   ├── run_once
   └── stage methods

5. crates/voloxide_core/src/comm.rs
   ├── process_incoming_messages
   ├── act_on_messages
   └── send_comm_responses

6. crates/voloxide_core/src/sensors/
   ├── ingestion.rs
   ├── processors.rs
   └── health.rs

7. crates/voloxide_core/src/rc/
   └── system.rs

8. crates/voloxide_core/src/control.rs

9. crates/voloxide_core/src/pwm/
   └── system.rs

10. comms/voloxide_mavlink/src/
    ├── parser.rs
    ├── conversions.rs
    └── link.rs
```

That path follows one simulator tick from ROS sensor input to firmware update to PWM output.

## Code Reading Path For `boards/pico2w`

Use this order when stepping through the active RP2350/Pico 2 W firmware:

```text
1. boards/pico2w/src/bin/voloxide.rs
   ├── default feature-driven hardware setup
   ├── core 0 realtime scheduler loop
   ├── core 1 Embassy executor tasks
   ├── ISM330DHCX interrupt executor startup
   └── telemetry-rate selection

2. boards/pico2w/src/board.rs
   ├── BoardIo implementation
   ├── newest-packet sensor queue drains
   ├── serial mailbox bridge
   ├── serial flush budget
   └── deferred board actions

3. boards/pico2w/src/ism330dhcx.rs
   └── IMU packet queue and data-ready bookkeeping

4. boards/pico2w/src/rc_receiver.rs
   └── CRSF packet parsing and RC queueing

5. boards/pico2w/src/gps.rs
   └── GPS PIO UART and magnetometer-facing path

6. crates/voloxide_core/src/world.rs
   ├── realtime_scheduler_step
   ├── run_imu_control_tick
   └── run_service_step_with_deferral
```

Use [RP2350 / Pico 2 W](boards/rp2350-pico2w.md) for exact feature choices, scope-pin meanings,
timing results, and flash commands.

## Code Reading Path For STM32 Boards

Use this order when renewing Nucleo-H753ZI or Pixracer Pro validation:

```text
1. boards/nucleo/src/bin/voloxide.rs
   └── Nucleo World construction and firmware loop

2. boards/nucleo/src/board.rs
   └── Nucleo BoardIo and board setup

3. boards/pixracerpro/src/bin/voloxide.rs
   └── Pixracer Pro World construction and firmware loop

4. boards/pixracerpro/src/board.rs
   └── Pixracer Pro BoardIo and board setup

5. platforms/stm_32/src/peripherals/
   ├── IMU drivers
   ├── barometer and magnetometer drivers
   ├── serial/RC drivers
   └── Embassy signal tasks

6. crates/voloxide_core/src/world.rs
   └── generic run_once scheduler and stage methods
```

Use [STM32 boards](boards/stm32.md) for the retained-target status and the validation order.
