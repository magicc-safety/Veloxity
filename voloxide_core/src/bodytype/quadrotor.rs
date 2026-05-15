use crate::bodytype::BodyType;
use crate::controller;
use crate::estimator;
use crate::mixer;
use crate::mixer::Mixer;
use crate::packets::*;

pub struct Quadrotor;

impl BodyType for Quadrotor {
    type Estimator = estimator::quad_estimator::QuadEstimator;
    type Controller = controller::quad_controller::QuadController;
    type Mixer = mixer::quad_mixer::QuadMixer;
}
