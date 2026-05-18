use sim::{board::Board, pwm::SimPwmDriver};
use voloxide_core::{
    controller::quad_controller::QuadController, estimator::quad_estimator::QuadEstimator,
    mixer::quad_mixer::QuadMixer, params::Params, pwm::PwmDriver, state_machine::StateManager,
    world::World,
};
use voloxide_mavlink::MavlinkInterface;

type SimWorld =
    World<Board, QuadEstimator, QuadController, QuadMixer, MavlinkInterface, SimPwmDriver>;

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

    let mut world = SimWorld::init(
        board, params, mavlink, state, estimator, controller, mixer, pwm,
    );

    while tick_subscriber.recv_async().await.is_ok() {
        world.run_once();
    }
}
