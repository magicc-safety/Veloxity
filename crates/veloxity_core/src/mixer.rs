use crate::{
    math::FlightFloat,
    params::{ParamId, Params},
    state_machine::StateManager,
};

pub mod matrix;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MixerOutputType {
    Aux,
    Motor,
    Servo,
    Gpio,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MixerStatus {
    Healthy,
    InvalidMixer,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MixerRun<A> {
    pub commands: A,
    pub status: MixerStatus,
}

pub struct MixerCtx<'a, R: FlightFloat> {
    pub state: &'a StateManager,
    pub params: &'a Params,
    pub rc_override: u16,
    pub air_density: R,
    pub battery_voltage: Option<R>,
}

pub trait Mixer<R: FlightFloat> {
    type MixerInput;
    type ActuatorCommands: AsRef<[R]>;
    fn mix(
        &mut self,
        controls: &Self::MixerInput,
        ctx: MixerCtx<'_, R>,
    ) -> MixerRun<Self::ActuatorCommands>;

    fn output_types(&self) -> &[MixerOutputType] {
        &[]
    }

    fn default_pwm_rates(&self) -> &[R] {
        &[]
    }

    fn on_param_changed(&mut self, _params: &Params, _id: ParamId) -> Option<MixerStatus> {
        None
    }
}
