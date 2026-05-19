use crate::{params::Params, state_machine::StateManager};

pub mod quad;
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

pub struct MixerCtx<'a> {
    pub state: &'a StateManager,
    pub params: &'a Params,
    pub rc_override: u16,
    pub air_density: f64,
    pub battery_voltage: Option<f64>,
}

pub trait Mixer {
    type MixerInput;
    type ActuatorCommands: AsRef<[f64]>;
    fn mix(
        &mut self,
        controls: &Self::MixerInput,
        ctx: MixerCtx<'_>,
    ) -> MixerRun<Self::ActuatorCommands>;

    fn output_types(&self) -> &[MixerOutputType] {
        &[]
    }

    fn default_pwm_rates(&self) -> &[f64] {
        &[]
    }
}
