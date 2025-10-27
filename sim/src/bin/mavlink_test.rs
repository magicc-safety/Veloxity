use cdr::{CdrLe, Infinite};
use serde::{Deserialize, Serialize};
use sim::board;
use zenoh::bytes::ZBytes;

use rustflight_core::{
    board::BoardTrait,
    bodytype::{quadrotor::Quadrotor, BodyType},
    comm_manager::comm_link_trait::{mavlink::MavlinkInterface, CommInterface},
    controller::{Controller, quad_controller::QuadController},
    estimator::{Estimator, quad_estimator::QuadEstimator},
    params2::Params,
    state_machine::StateManager,
    hlist::{Here, There},
    hlist_type,
    mixer::{Mixer, quad_mixer::{QuadMixer}},
    rustflight::{rustflight_typed::ROSFlight, Configuration},
};
use sim::pwm::SimPwmDriver;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
struct SimpleBoolResponse {
    result: bool,
}

// define the wiring diagram
#[derive(Default)]
pub struct SimQuadConfig;
impl Configuration<board::Board, Quadrotor> for SimQuadConfig {
    type SculptIndices = hlist_type![
        Here,
        Here, 
        There<There<There<There<There<Here>>>>>
    ];
    type RcPacketIndex = There<There<Here>>;
}


#[tokio::main]
async fn main() {
    // board implementation
    let board = board::Board::new().await;
    let pwm_driver = SimPwmDriver::new(&board.zenoh_session).await;
    let mut params = Params::new();
    
    // initialize the timing of the highest level loop through a tick callback 
    let tick_handler = board
        .zenoh_session
        .declare_subscriber("rust/tick")
        .await
        .unwrap();

    // body type instantiations
    let estimator = QuadEstimator::default();
    let controller = QuadController::default();
    let mixer = QuadMixer::new(&params);

    // zero-sized configuration marker (necessary)
    let config = SimQuadConfig::default();

    // comm_link implementation
    let mavlink = MavlinkInterface::new();

    let state_manager = StateManager::new();

    let mut rosflight = ROSFlight::init(1000, board, params, mavlink, state_manager, estimator, controller, mixer, config, pwm_driver);
    //let mut rosflight = ROSFlight::init(1000, board, params, mavlink, state_manager, estimator, controller, mixer, config);

    //let mut x: u64 = 0;

    while let Ok(_tick) = tick_handler.recv_async().await {
        //println!("tick: {}", x);
        //x += 1;

        rosflight.run();

        let response = SimpleBoolResponse { result: true };
        let zb = ZBytes::from(cdr::serialize::<_, _, CdrLe>(&response, Infinite).unwrap());

    }
}
