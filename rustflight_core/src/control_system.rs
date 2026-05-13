use crate::{
    board::BoardIo,
    bodytype::BodyType,
    command_manager::CommandManager,
    companion_system::{AuxCommandState, ExternalAttitudeState},
    controller::{Controller, ControllerCtx, RcTrimCalibrator},
    estimator::{AttitudeStateTrait, NamedEstimator},
    mixer::{Mixer, MixerCtx, MixerStatus},
    params::Params,
    pwm::PwmDriver,
    pwm_system::{PwmOutputState, compose_pwm_outputs, write_pwm_commands},
    sensors::ProcessedSensors,
    state_machine::{ErrorFlag, Event, StateManager},
};

pub struct ControlPipelineResource<S, A> {
    pub latest_estimator_state: S,
    pub latest_actuator_commands: Option<A>,
    pub latest_pwm_outputs: [f64; crate::pwm_system::PWM_OUTPUT_CHANNELS],
    pub latest_loop_time_us: u16,
    last_imu_time: u64,
}

impl<S, A> Default for ControlPipelineResource<S, A>
where
    S: Default,
{
    fn default() -> Self {
        Self {
            latest_estimator_state: Default::default(),
            latest_actuator_commands: None,
            latest_pwm_outputs: [0.0; crate::pwm_system::PWM_OUTPUT_CHANNELS],
            latest_loop_time_us: 0,
            last_imu_time: 0,
        }
    }
}

impl<S, A> ControlPipelineResource<S, A> {
    pub(crate) fn last_imu_time(&self) -> u64 {
        self.last_imu_time
    }

    pub(crate) fn set_last_imu_time(&mut self, timestamp: u64) {
        self.last_imu_time = timestamp;
    }

    pub(crate) fn set_latest(
        &mut self,
        state: S,
        actuator_commands: A,
        pwm_outputs: [f64; crate::pwm_system::PWM_OUTPUT_CHANNELS],
        loop_time_us: u16,
    ) {
        self.latest_estimator_state = state;
        self.latest_actuator_commands = Some(actuator_commands);
        self.latest_pwm_outputs = pwm_outputs;
        self.latest_loop_time_us = loop_time_us;
    }
}

pub struct ControlPipelineCtx<'a, B, BT, PD>
where
    B: BoardIo,
    BT: BodyType,
    BT::Estimator: NamedEstimator,
    BT::Controller: Controller<State = <BT::Estimator as NamedEstimator>::State> + RcTrimCalibrator,
    BT::Mixer: Mixer<MixerInput = <BT::Controller as Controller>::ControlOutput>,
    <BT::Mixer as Mixer>::ActuatorCommands: AsRef<[f64]> + Copy,
    <BT::Estimator as NamedEstimator>::State: Copy + Default,
    PD: PwmDriver,
{
    pub board: &'a mut B,
    pub params: &'a Params,
    pub sensors: &'a ProcessedSensors,
    pub external_attitude: &'a mut ExternalAttitudeState,
    pub aux_commands: &'a AuxCommandState,
    pub command: &'a CommandManager,
    pub state: &'a mut StateManager,
    pub estimator: &'a mut BT::Estimator,
    pub controller: &'a mut BT::Controller,
    pub mixer: &'a mut BT::Mixer,
    pub control_pipeline: &'a mut ControlPipelineResource<
        <BT::Estimator as NamedEstimator>::State,
        <BT::Mixer as Mixer>::ActuatorCommands,
    >,
    pub pwm_output: &'a PwmOutputState,
    pub pwm: &'a mut PD,
    pub dt: f64,
}

pub fn run_control_pipeline_if_new_imu<B, BT, PD>(ctx: ControlPipelineCtx<'_, B, BT, PD>) -> bool
where
    B: BoardIo,
    BT: BodyType,
    BT::Estimator: NamedEstimator,
    BT::Controller: Controller<State = <BT::Estimator as NamedEstimator>::State> + RcTrimCalibrator,
    BT::Mixer: Mixer<MixerInput = <BT::Controller as Controller>::ControlOutput>,
    <BT::Mixer as Mixer>::ActuatorCommands: AsRef<[f64]> + Copy,
    <BT::Estimator as NamedEstimator>::State: Copy + Default,
    PD: PwmDriver,
{
    let Some(imu_packet) = ctx.sensors.imu else {
        return false;
    };

    let current_time = imu_packet.header.timestamp;
    let last_imu_time = ctx.control_pipeline.last_imu_time();
    if last_imu_time != 0 && current_time <= last_imu_time {
        ctx.state.update(
            Event::ERROR_OCCURRED(ErrorFlag::TIME_GOING_BACKWARDS),
            ctx.params,
        );
        return false;
    }
    ctx.control_pipeline.set_last_imu_time(current_time);
    ctx.state.update(
        Event::ERROR_CLEARED(ErrorFlag::TIME_GOING_BACKWARDS),
        ctx.params,
    );

    let loop_start_us = ctx.board.clock_micros();
    let external_attitude = ctx.external_attitude.latest.take();
    let state = ctx.estimator.estimate_named_with_external_attitude(
        ctx.sensors,
        ctx.params,
        ctx.dt,
        external_attitude,
    );

    if state.is_healthy() {
        ctx.state.update(
            Event::ERROR_CLEARED(ErrorFlag::UNHEALTHY_ESTIMATOR),
            ctx.params,
        );
    } else {
        ctx.state.update(
            Event::ERROR_OCCURRED(ErrorFlag::UNHEALTHY_ESTIMATOR),
            ctx.params,
        );
    }

    let controls = ctx.controller.control(
        &state,
        ControllerCtx {
            state_manager: ctx.state,
            command: ctx.command.combined_control(),
            params: ctx.params,
            air_density: ctx.sensors.air_density(),
            dt: ctx.dt,
        },
    );
    let mixer_run = ctx.mixer.mix(
        &controls,
        MixerCtx {
            state: ctx.state,
            params: ctx.params,
            rc_override: ctx.command.get_rc_override(),
            air_density: ctx.sensors.air_density(),
            battery_voltage: ctx.sensors.battery.map(|battery| battery.voltage as f64),
        },
    );
    match mixer_run.status {
        MixerStatus::Healthy => ctx
            .state
            .update(Event::ERROR_CLEARED(ErrorFlag::INVALID_MIXER), ctx.params),
        MixerStatus::InvalidMixer => ctx
            .state
            .update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_MIXER), ctx.params),
    }
    let actuator_commands = mixer_run.commands;
    if ctx
        .pwm
        .configure_output_rates(ctx.mixer.default_pwm_rates())
        .is_err()
    {
        crate::log_warn!("PWM driver rejected mixer default output rates");
    }
    let pwm_outputs = compose_pwm_outputs(
        actuator_commands.as_ref(),
        ctx.mixer.output_types(),
        ctx.aux_commands.latest.as_ref(),
        ctx.state,
        ctx.params,
    );
    if let Err(error) = write_pwm_commands(ctx.board, ctx.pwm, ctx.pwm_output, &pwm_outputs) {
        crate::log_warn!("PWM driver rejected output command: {:?}", error);
    }
    let loop_time_us = ctx
        .board
        .clock_micros()
        .saturating_sub(loop_start_us)
        .min(u16::MAX as u64) as u16;
    ctx.control_pipeline
        .set_latest(state, actuator_commands, pwm_outputs, loop_time_us);
    true
}
