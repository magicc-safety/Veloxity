use crate::board::sim_board::Sim;
use crate::comm_manager::comm_link_trait::mavlink::MavlinkInterface;
use cdr::{CdrLe, Infinite};
use rustflight_alpha::*;
use serde::{Deserialize, Serialize};
use zenoh::bytes::ZBytes;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
struct SimpleBoolResponse {
    result: bool,
}

#[tokio::main]
async fn main() {
    let sim = Sim::new().await;
    let tick_handler = sim
        .zenoh_listen_session
        .declare_queryable("rt/tick")
        .await
        .unwrap();

    let mavlink = MavlinkInterface::new();
    let mut rosflight = rustflight::rustflight_sensors_comms::ROSFlight::init(1000, sim, mavlink);

    while let Ok(query) = tick_handler.recv_async().await {
        println!("Received query!");

        rosflight.run();

        let response = SimpleBoolResponse { result: true };
        let zb = ZBytes::from(cdr::serialize::<_, _, CdrLe>(&response, Infinite).unwrap());
        query.reply(query.key_expr().to_string(), zb).await.unwrap();
    }
}
