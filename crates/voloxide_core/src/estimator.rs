use crate::{
    comm::messages::messages::ExternalAttitudeMsg, math::FlightFloat, params::Params,
    sensors::ProcessedSensors,
};
pub mod quad;

pub trait Estimator<R: FlightFloat> {
    type State: AttitudeEstimate;
    fn estimate(&mut self, sensors: &ProcessedSensors<R>, params: &Params, dt: R) -> Self::State;

    fn reset(&mut self) {}

    fn reset_adaptive_bias(&mut self) {}

    fn estimate_with_external_attitude(
        &mut self,
        sensors: &ProcessedSensors<R>,
        params: &Params,
        dt: R,
        _external_attitude: Option<ExternalAttitudeMsg>,
    ) -> Self::State {
        self.estimate(sensors, params, dt)
    }
}

pub trait AttitudeEstimate {
    fn q(&self) -> [f32; 4];
    fn q_dot(&self) -> [f32; 4];
    fn is_healthy(&self) -> bool;
}
