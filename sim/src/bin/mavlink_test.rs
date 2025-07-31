use cdr::{CdrLe, Infinite};
use serde::{Deserialize, Serialize};
use sim::board;
use zenoh::bytes::ZBytes;

use rustflight_core::{
    board::BoardTrait,
    bodytype::BodyType,
    bodytype::quadrotor::{QuadController, QuadEstimator, QuadMixer, Quadrotor},
    comm_manager::comm_link_trait::mavlink::MavlinkInterface,
    controller::Controller,
    estimator::Estimator,
    hlist::{Here, There},
    hlist_type,
    mixer::Mixer,
    rustflight::Configuration,
    rustflight::rustflight_typed::ROSFlight,
};

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
struct SimpleBoolResponse {
    result: bool,
}

// #[tokio::main]
//async fn main() {
//    let sim = Board::new().await;
//    let tick_handler = sim
//        .zenoh_listen_session
//        .declare_queryable("rt/tick")
//        .await
//        .unwrap();

//    let mavlink = MavlinkInterface::new();
//    let mut rosflight =
//        rustflight_core::rustflight::rustflight_sensors_comms::ROSFlight::init(1000, sim, mavlink);

//    while let Ok(query) = tick_handler.recv_async().await {
//        println!("Received query!");

//        rosflight.run();

//        let response = SimpleBoolResponse { result: true };
//        let zb = ZBytes::from(cdr::serialize::<_, _, CdrLe>(&response, Infinite).unwrap());
//        query.reply(query.key_expr().to_string(), zb).await.unwrap();
//    }
//}

// define the wiring diagram
#[derive(Default)]
pub struct SimQuadConfig;
impl Configuration<board::Board, Quadrotor> for SimQuadConfig {
    type SculptIndices = hlist_type![Here, Here, Here, There<There<Here>>];
}

#[tokio::main]
async fn main() {
    // board implementation
    let mut board = board::Board::new().await;
    let tick_handler = board
        .zenoh_connect_session
        .declare_subscriber("tick")
        .await
        .unwrap();

    // body type instantiations
    let estimator = QuadEstimator::default();
    let controller = QuadController::default();
    let mixer = QuadMixer::default();

    // zero-sized configuration marker (necessary)
    let config = SimQuadConfig::default();

    // comm_link implementation
    let mavlink = MavlinkInterface::new();

    let mut rosflight = ROSFlight::init(1000, board, mavlink, estimator, controller, mixer, config);

    while let Ok(_tick) = tick_handler.recv() {
        println!("Received query!");

        rosflight.run();

        let response = SimpleBoolResponse { result: true };
        let zb = ZBytes::from(cdr::serialize::<_, _, CdrLe>(&response, Infinite).unwrap());
    }
}
