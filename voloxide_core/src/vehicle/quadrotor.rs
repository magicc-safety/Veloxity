use crate::{controller, estimator, params::Params};

pub type Estimator = estimator::quad::QuadEstimator;
pub type Controller = controller::quad::QuadController;
pub type Mixer = crate::mixer::matrix::MatrixMixer;

pub fn mixer(params: &Params) -> Mixer {
    Mixer::new(params)
}
