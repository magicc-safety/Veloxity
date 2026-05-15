#![no_std]
use cortex_m_rt::entry;
use nucleo::*;
use panic_halt as _;
use voloxide_core::{
    board::BoardIo, bodytype::BodyType, bodytype::quadrotor::Quadrotor,
    comm_manager::comm_link_trait::mavlink::MavlinkInterface, controller::Controller, mixer::Mixer,
    params::Params, state_machine::StateManager, world::World,
};
use stm_32::*;

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
    let estimator =
        <voloxide_core::bodytype::quadrotor::Quadrotor as BodyType>::Estimator::default();
    let controller =
        <voloxide_core::bodytype::quadrotor::Quadrotor as BodyType>::Controller::default();
    let mixer = <voloxide_core::bodytype::quadrotor::Quadrotor as BodyType>::Mixer::new(&params);

    // comm_link implementation
    let mavlink = MavlinkInterface::new();

    let state_manager = StateManager::new();

    let mut world = World::<_, Quadrotor, _, _>::init(
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
