pub mod quad;
use crate::command::CombinedControl;
use crate::math::FlightFloat;
use crate::params::Params;
use crate::state_machine::StateManager;

pub struct ControllerCtx<'a, R: FlightFloat> {
    pub state_manager: &'a mut StateManager,
    pub command: &'a CombinedControl,
    pub params: &'a Params,
    pub air_density: R,
    pub dt: R,
}

pub trait Controller<R: FlightFloat> {
    type State;
    type ControlOutput;
    fn control(&mut self, state: &Self::State, ctx: ControllerCtx<'_, R>) -> Self::ControlOutput;
    fn update_gains(&mut self, params: &Params);
}

pub trait RcTrimCalibrator {
    fn calculate_equilibrium_torques_from_rc(
        &mut self,
        rc_control: &CombinedControl,
        params: &Params,
    ) -> [f32; 3];
}
