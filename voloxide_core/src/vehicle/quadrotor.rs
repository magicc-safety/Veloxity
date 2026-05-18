use crate::packets::*;
use crate::{controller, estimator, params::Params};

pub type Estimator = estimator::quad_estimator::QuadEstimator;
pub type Controller = controller::quad_controller::QuadController;
pub type Mixer = crate::mixer::quad_mixer::QuadMixer;

pub fn mixer(params: &Params) -> Mixer {
    Mixer::new(params)
}
