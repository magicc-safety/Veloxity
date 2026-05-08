use rustflight_core::{
    bodytype::quadrotor::Quadrotor, comm_manager::comm_link_trait::mavlink::MavlinkInterface,
    controller::quad_controller::QuadController, estimator::quad_estimator::QuadEstimator,
    mixer::quad_mixer::QuadMixer, params2::Params, pwm::PwmDriver, state_machine::StateManager,
    world::World,
};
use sim::{board::Board, pwm::SimPwmDriver};

#[tokio::main]
async fn main() {
    let (board, zenoh_session) = Board::new().await;
    let tick_subscriber = zenoh_session
        .declare_subscriber("rust/tick")
        .await
        .expect("failed to subscribe to rust/tick");

    let params = Params::new();
    let mut pwm = SimPwmDriver::new(&zenoh_session).await;
    for channel in 0..pwm.len() {
        let _ = pwm.disable(channel);
    }

    let estimator = QuadEstimator::default();
    let controller = QuadController::default();
    let mixer = QuadMixer::new(&params);
    let mavlink = MavlinkInterface::new();
    let state = StateManager::new();

    let mut world = World::<Board, Quadrotor, MavlinkInterface, SimPwmDriver>::init(
        board, params, mavlink, state, estimator, controller, mixer, pwm,
    );

    while tick_subscriber.recv_async().await.is_ok() {
        world.run_comm_param_sensor_stages();
    }
}
