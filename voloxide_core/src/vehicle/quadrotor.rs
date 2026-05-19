use crate::packets::*;
use crate::{controller, estimator, params::Params};

pub type Estimator = estimator::quad::QuadEstimator;
pub type Controller = controller::quad::QuadController;
pub type Mixer = crate::mixer::quad::QuadMixer;

pub fn mixer(params: &Params) -> Mixer {
    Mixer::new(params)
}
