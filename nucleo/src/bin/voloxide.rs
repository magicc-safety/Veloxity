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

#[entry]
fn main() -> ! {
    // board implementation
    let (mut board, pwm_driver) = board::Board::new();
    let mut params = Params::default();
    if !board.read_params(&mut params) {
        params.set_defaults();
        let _ = board.write_params(&params);
    }

    // body type instantiations
    let estimator = quadrotor::Estimator::default();
    let controller = quadrotor::Controller::default();
    let mixer = quadrotor::mixer(&params);

    // comm_link implementation
    let mavlink = MavlinkInterface::new();

    let state_manager = StateManager::new();

    let mut world = World::init(
        board,
        params,
        mavlink,
        state_manager,
        estimator,
        controller,
        mixer,
        pwm_driver,
    );

    loop {
        world.run_once();
    }
}
