use crate::{controller, estimator, math::FlightFloat, params::Params};

pub type Estimator<R> = estimator::quad::QuadEstimator<R>;
pub type Controller<R> = controller::quad::QuadController<R>;
pub type Mixer<R> = crate::mixer::matrix::MatrixMixer<R>;

pub fn mixer<R: FlightFloat>(params: &Params) -> Mixer<R> {
    Mixer::new(params)
}
