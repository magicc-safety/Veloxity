#![no_std]
#![no_main]
use cortex_m_rt::entry;
use panic_halt as _;
use pixracerpro::pwm::BoardPwmDriver;
use pixracerpro::*;
use stm_32::*;
use veloxity_core::world::ControlLoopRates;
use veloxity_core::world::RealtimeSchedulerStep;
use veloxity_core::{
    board::BoardIo,
    comm::TelemetryRates,
    params::Params,
    state_machine::StateManager,
    vehicle::quadrotor,
    world::{RealtimeServicePolicy, World},
};
use veloxity_mavlink::MavlinkInterface;

type PixracerReal = f64;
const PIXRACER_CONTROL_LOOP_HZ: u16 = 400;
const PIXRACER_TELEMETRY_STREAMS_PER_SERVICE_PHASE: usize = 2;

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
        match world.realtime_scheduler_step() {
            RealtimeSchedulerStep::ImuControl => {
                let _ = world.run_imu_control_tick();
            }
            RealtimeSchedulerStep::ControlUpdate => {
                let _ = world.run_control_update_tick();
            }
            RealtimeSchedulerStep::Service => {
                let _ = world.run_prioritized_service_steps_with_policy(
                    RealtimeServicePolicy::continuous_polling(
                        PIXRACER_TELEMETRY_STREAMS_PER_SERVICE_PHASE,
                    ),
                );
            }
            RealtimeSchedulerStep::Idle => {}
        }
    }
}
