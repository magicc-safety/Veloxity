#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use pico2w::{board, config::Pico2WConfig, pwm::PioPwmDriver};
use rp2350_platform::hal as rp;
use voloxide_core::{
    board::BoardIo, params::Params, state_machine::StateManager, vehicle::quadrotor, world::World,
};
use voloxide_mavlink::MavlinkInterface;

type Pico2WWorld = World<
    board::Board,
    quadrotor::Estimator,
    quadrotor::Controller,
    quadrotor::Mixer,
    MavlinkInterface,
    PioPwmDriver,
>;

fn init_world(board: board::Board, params: Params, pwm_driver: PioPwmDriver) -> Pico2WWorld {
    let mixer = quadrotor::mixer(&params);
    Pico2WWorld::init(
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
    let _peripherals = rp::init(Default::default());
    let config = Pico2WConfig::default();
    let (mut board, pwm_driver) = board::Board::new(config);

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
