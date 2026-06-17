use crate::{
    comm::messages::messages::ExternalAttitudeMsg, math::FlightFloat, params::Params,
    sensors::ProcessedSensors,
};
pub mod quad;

pub struct EstimatorCtx<'a, R: FlightFloat> {
    pub sensors: &'a ProcessedSensors<R>,
    pub params: &'a Params,
    pub dt: R,
    pub external_attitude: Option<ExternalAttitudeMsg>,
}

pub trait Estimator<R: FlightFloat> {
    type State: AttitudeEstimate;
    fn estimate(&mut self, ctx: EstimatorCtx<'_, R>) -> Self::State;

    fn update_params(&mut self, _params: &Params) {}

    fn reset(&mut self) {}

    fn reset_adaptive_bias(&mut self) {}
}

pub trait AttitudeEstimate {
    fn q(&self) -> [f32; 4];
    fn q_dot(&self) -> [f32; 4];
    fn is_healthy(&self) -> bool;
}
