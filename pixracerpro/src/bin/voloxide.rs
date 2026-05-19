#![no_std]
#![no_main]
use cortex_m_rt::entry;
use panic_halt as _;
use pixracerpro::pwm::BoardPwmDriver;
use pixracerpro::*;
use stm_32::*;
use voloxide_core::{
    board::BoardIo, params::Params, state_machine::StateManager, vehicle::quadrotor, world::World,
};
use voloxide_mavlink::MavlinkInterface;

type PixracerWorld<'a> = World<
    board::Board,
    quadrotor::Estimator,
    quadrotor::Controller,
    quadrotor::Mixer,
    MavlinkInterface,
    BoardPwmDriver<'a>,
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

    loop {
        world.run_once();
    }
}
