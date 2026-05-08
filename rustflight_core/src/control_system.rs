use crate::{
    board::BoardIo,
    bodytype::BodyType,
    comm_manager::{CommManager, comm_link_trait::CommInterface},
    command_manager::CommandManager,
    companion_system::{AuxCommandState, ExternalAttitudeState},
    controller::{Controller, RcTrimCalibrator},
    estimator::{AttitudeStateTrait, NamedEstimator},
    mixer::Mixer,
    params::Params,
    pwm::PwmDriver,
    pwm_system::{PwmOutputState, compose_pwm_outputs, write_pwm_commands},
    sensors::ProcessedSensors,
    state_machine::{ErrorFlag, Event, StateManager},
};

pub struct ControlPipelineResource<S, A> {
    pub latest_estimator_state: S,
    pub latest_actuator_commands: Option<A>,
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

    pub(crate) fn set_latest(&mut self, state: S, actuator_commands: A) {
        self.latest_estimator_state = state;
        self.latest_actuator_commands = Some(actuator_commands);
    }
}

pub struct ControlPipelineCtx<'a, B, BT, CI, PD>
where
    B: BoardIo,
    BT: BodyType,
    BT::Estimator: NamedEstimator,
    BT::Controller: Controller<State = <BT::Estimator as NamedEstimator>::State> + RcTrimCalibrator,
    BT::Mixer: Mixer<MixerInput = <BT::Controller as Controller>::ControlOutput>,
    <BT::Mixer as Mixer>::ActuatorCommands: AsRef<[f64]> + Copy,
    <BT::Estimator as NamedEstimator>::State: Copy + Default,
    CI: CommInterface<B>,
    PD: PwmDriver,
{
    pub board: &'a mut B,
    pub comm: &'a mut CommManager<B, CI>,
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

pub fn run_control_pipeline_if_new_imu<B, BT, CI, PD>(
    ctx: ControlPipelineCtx<'_, B, BT, CI, PD>,
) -> bool
where
    B: BoardIo,
    BT: BodyType,
    BT::Estimator: NamedEstimator,
    BT::Controller: Controller<State = <BT::Estimator as NamedEstimator>::State> + RcTrimCalibrator,
    BT::Mixer: Mixer<MixerInput = <BT::Controller as Controller>::ControlOutput>,
    <BT::Mixer as Mixer>::ActuatorCommands: AsRef<[f64]> + Copy,
    <BT::Estimator as NamedEstimator>::State: Copy + Default,
    CI: CommInterface<B>,
    PD: PwmDriver,
{
    let Some(imu_packet) = ctx.sensors.imu else {
        return false;
    };

    let current_time = imu_packet.header.timestamp;
    if current_time == ctx.control_pipeline.last_imu_time() {
        return false;
    }
    ctx.control_pipeline.set_last_imu_time(current_time);

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
        ctx.state,
        ctx.command.combined_control(),
        ctx.params,
        ctx.dt,
    );
    let actuator_commands = ctx.mixer.mix(&controls, ctx.state);
    let pwm_outputs = compose_pwm_outputs(
        actuator_commands.as_ref(),
        ctx.mixer.output_types(),
        ctx.aux_commands.latest.as_ref(),
        ctx.state,
        ctx.params,
    );
    write_pwm_commands(ctx.board, ctx.pwm, ctx.pwm_output, &pwm_outputs);
    let now_us = ctx.board.clock_micros();
    ctx.comm.send_named_telemetry_streams(
        ctx.board,
        now_us,
        ctx.state,
        ctx.command,
        &state,
        ctx.sensors,
        &pwm_outputs,
    );

    ctx.control_pipeline.set_latest(state, actuator_commands);
    true
}
