use crate::{
    comm_messages::messages::ExternalAttitudeMsg, params::Params, sensors::ProcessedSensors,
};
pub mod quad_estimator;

pub trait NamedEstimator {
    type State: AttitudeStateTrait;
    fn estimate_named(
        &mut self,
        sensors: &ProcessedSensors,
        params: &Params,
        dt: f64,
    ) -> Self::State;

    fn estimate_named_with_external_attitude(
        &mut self,
        sensors: &ProcessedSensors,
        params: &Params,
        dt: f64,
        _external_attitude: Option<ExternalAttitudeMsg>,
    ) -> Self::State {
        self.estimate_named(sensors, params, dt)
    }
}

pub trait AttitudeStateTrait {
    fn q(&self) -> [f32; 4];
    fn q_dot(&self) -> [f32; 4];
    fn is_healthy(&self) -> bool;
}
