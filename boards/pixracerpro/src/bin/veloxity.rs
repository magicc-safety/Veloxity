#![no_std]
#![no_main]
use cortex_m_rt::entry;
use panic_halt as _;
use pixracerpro::pwm::BoardPwmDriver;
use pixracerpro::*;
use stm_32::*;
use veloxity_core::world::ControlLoopRates;
#[cfg(not(feature = "legacy-run-once"))]
use veloxity_core::world::RealtimeSchedulerStep;
use veloxity_core::{
    board::BoardIo,
    comm::{
        NamedTelemetryStream, RealtimeTelemetryPriority, RealtimeTelemetryPriorityGate,
        TelemetryRates,
    },
    params::Params,
    state_machine::StateManager,
    vehicle::quadrotor,
    world::{RealtimeServicePolicy, World},
};
use veloxity_mavlink::MavlinkInterface;

type PixracerReal = f64;
const PIXRACER_CONTROL_LOOP_HZ: u16 = 400;
#[cfg(not(feature = "legacy-run-once"))]
const PIXRACER_TELEMETRY_STREAMS_PER_SERVICE_STEP: usize = 4;
#[cfg(not(feature = "legacy-run-once"))]
const PIXRACER_TELEMETRY_STREAMS_PER_TELEMETRY_PHASE: usize = 2;
#[cfg(not(feature = "legacy-run-once"))]
const PIXRACER_POST_CONTROL_TELEMETRY_STREAMS: usize = 4;
#[cfg(not(feature = "legacy-run-once"))]
const PIXRACER_POST_CONTROL_PRIORITY_STREAMS: &[RealtimeTelemetryPriority] =
    &[RealtimeTelemetryPriority {
        stream: NamedTelemetryStream::Imu,
        gate: RealtimeTelemetryPriorityGate::FreshSample,
    }];

type PixracerWorld<'a> = World<
    board::Board,
    quadrotor::Estimator<PixracerReal>,
    quadrotor::Controller<PixracerReal>,
    quadrotor::Mixer<PixracerReal>,
    MavlinkInterface,
    BoardPwmDriver<'a>,
    PixracerReal,
>;

fn init_world<'a>(
    board: board::Board,
    params: Params,
    pwm_driver: BoardPwmDriver<'a>,
) -> PixracerWorld<'a> {
    let mixer = quadrotor::mixer(&params);
    PixracerWorld::init(
        board,
        params,
        MavlinkInterface::new(),
        StateManager::new(),
        quadrotor::Estimator::default(),
        quadrotor::Controller::default(),
        mixer,
        pwm_driver,
    )
}

#[entry]
fn main() -> ! {
    // board implementation & servos object
    let (mut board, mut servos) = board::Board::new();
    let mut params = Params::default();
    if !board.read_params(&mut params) {
        params.set_defaults();
        let _ = board.write_params(&params);
    }

    // PWM Driver from servos object
    let pwm_driver = BoardPwmDriver::new(&mut servos);

    let mut world = init_world(board, params, pwm_driver);
    world.set_telemetry_rates(TelemetryRates::bounded_high_rate_transport());
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(PIXRACER_CONTROL_LOOP_HZ));

    loop {
        #[cfg(feature = "legacy-run-once")]
        {
            world.run_once();
        }
        #[cfg(not(feature = "legacy-run-once"))]
        match world.realtime_scheduler_step() {
            RealtimeSchedulerStep::ImuControl => {
                #[cfg(feature = "timing-diagnostics")]
                {
                    let class = world.run_imu_control_tick_classified();
                    if class.ran_control {
                        let _ = world.run_realtime_telemetry_stage_prioritized(
                            PIXRACER_POST_CONTROL_PRIORITY_STREAMS,
                            PIXRACER_POST_CONTROL_TELEMETRY_STREAMS,
                        );
                    }
                }
                #[cfg(not(feature = "timing-diagnostics"))]
                {
                    if world.run_imu_control_tick() {
                        let _ = world.run_realtime_telemetry_stage_prioritized(
                            PIXRACER_POST_CONTROL_PRIORITY_STREAMS,
                            PIXRACER_POST_CONTROL_TELEMETRY_STREAMS,
                        );
                    }
                }
            }
            RealtimeSchedulerStep::ControlUpdate => {
                #[cfg(feature = "timing-diagnostics")]
                {
                    let class = world.run_control_update_tick_classified();
                    if class.ran_control {
                        let _ = world.run_realtime_telemetry_stage_prioritized(
                            PIXRACER_POST_CONTROL_PRIORITY_STREAMS,
                            PIXRACER_POST_CONTROL_TELEMETRY_STREAMS,
                        );
                    }
                }
                #[cfg(not(feature = "timing-diagnostics"))]
                {
                    if world.run_control_update_tick() {
                        let _ = world.run_realtime_telemetry_stage_prioritized(
                            PIXRACER_POST_CONTROL_PRIORITY_STREAMS,
                            PIXRACER_POST_CONTROL_TELEMETRY_STREAMS,
                        );
                    }
                }
            }
            RealtimeSchedulerStep::Service => {
                let _ = world.run_prioritized_service_steps_with_policy(
                    RealtimeServicePolicy::all_available(
                        PIXRACER_TELEMETRY_STREAMS_PER_SERVICE_STEP,
                        PIXRACER_TELEMETRY_STREAMS_PER_TELEMETRY_PHASE,
                    ),
                );
            }
            RealtimeSchedulerStep::Idle => {}
        }
    }
}
