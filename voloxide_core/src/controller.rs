pub mod quad_controller;
use crate::command_manager::CombinedControl;
use crate::params::Params;
use crate::state_machine::StateManager;

pub struct ControllerCtx<'a> {
    pub state_manager: &'a mut StateManager,
    pub command: &'a CombinedControl,
    pub params: &'a Params,
    pub air_density: f64,
    pub dt: f64,
}

pub trait Controller {
    type State;
    type ControlOutput;
    fn control(&mut self, state: &Self::State, ctx: ControllerCtx<'_>) -> Self::ControlOutput;
    fn update_gains(&mut self, params: &Params);
}

pub trait RcTrimCalibrator {
    fn calculate_equilibrium_torques_from_rc(
        &mut self,
        rc_control: &CombinedControl,
        params: &Params,
    ) -> [f32; 3];
}
