use sim::{board::Board, pwm::SimPwmDriver};
use voloxide_core::{
    board::BoardIo, controller::quad::QuadController, estimator::quad::QuadEstimator,
    mixer::matrix::MatrixMixer, params::Params, pwm::PwmDriver, state_machine::StateManager,
    world::World,
};
use voloxide_mavlink::MavlinkInterface;

type SimWorld =
    World<Board, QuadEstimator, QuadController, MatrixMixer, MavlinkInterface, SimPwmDriver>;

fn init_world(board: Board, params: Params, pwm: SimPwmDriver) -> SimWorld {
    let mixer = MatrixMixer::new(&params);
    SimWorld::init(
        board,
        params,
        MavlinkInterface::new(),
        StateManager::new(),
        QuadEstimator::default(),
        QuadController::default(),
        mixer,
        pwm,
    )
}

#[tokio::main]
async fn main() {
    let (mut board, zenoh_session) = Board::new().await;
    let tick_subscriber = zenoh_session
        .declare_subscriber("rust/tick")
        .await
        .expect("failed to subscribe to rust/tick");

    let mut params = Params::new();
    let _ = board.read_params(&mut params);
    let mut pwm = SimPwmDriver::new(&zenoh_session).await;
    for channel in 0..pwm.len() {
        let _ = pwm.disable(channel);
    }

    let mut world = init_world(board, params, pwm);

    while tick_subscriber.recv_async().await.is_ok() {
        world.run_once();
    }
}
