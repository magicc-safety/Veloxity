#![no_std]
use cortex_m_rt::entry;
use panic_halt as _;
use pixracerpro::pwm::BoardPwmDriver;
use pixracerpro::*;
use voloxide_core::{
    board::BoardIo, bodytype::BodyType, bodytype::quadrotor::Quadrotor,
    comm_manager::comm_link_trait::mavlink::MavlinkInterface, controller::Controller, mixer::Mixer,
    params::Params, state_machine::StateManager, world::World,
};
use stm_32::*;

#[entry]
fn main() -> ! {
    // board implementation & servos object
    let (mut board, mut servos) = board::Board::new();
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

    // state_manager
    let state_manager = StateManager::new();

    // PWM Driver from servos object
    let pwm_driver = BoardPwmDriver::new(&mut servos);

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
