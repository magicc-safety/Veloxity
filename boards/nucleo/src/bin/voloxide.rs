#![no_std]
#![no_main]
use cortex_m_rt::entry;
use nucleo::*;
use panic_halt as _;
use stm_32::*;
use voloxide_core::{
    board::BoardIo, params::Params, state_machine::StateManager, vehicle::quadrotor, world::World,
};
use voloxide_mavlink::MavlinkInterface;

type NucleoReal = f64;

type NucleoWorld = World<
    board::Board,
    quadrotor::Estimator<NucleoReal>,
    quadrotor::Controller<NucleoReal>,
    quadrotor::Mixer<NucleoReal>,
    MavlinkInterface,
    board::BoardPwmDriver,
    NucleoReal,
>;

fn init_world(
    board: board::Board,
    params: Params,
    pwm_driver: board::BoardPwmDriver,
) -> NucleoWorld {
    let mixer = quadrotor::mixer(&params);
    NucleoWorld::init(
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
    // board implementation
    let (mut board, pwm_driver) = board::Board::new();
    let mut params = Params::default();
    if !board.read_params(&mut params) {
        params.set_defaults();
        let _ = board.write_params(&params);
    }

    let mut world = init_world(board, params, pwm_driver);

    loop {
        world.run_once();
    }
}
