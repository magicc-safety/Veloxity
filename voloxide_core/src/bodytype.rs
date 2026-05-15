use crate::{controller::Controller, estimator::NamedEstimator, mixer::Mixer};

pub mod quadrotor;

pub trait BodyType {
    type Estimator: NamedEstimator;
    type Controller: Controller;
    type Mixer: Mixer;
}
