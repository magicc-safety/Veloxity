use crate::{
    board::BoardIo,
    command::CommandManager,
    companion::{AuxCommandState, ExternalAttitudeState},
    controller::{Controller, ControllerCtx, RcTrimCalibrator},
    estimator::{AttitudeEstimate, Estimator},
    math::FlightFloat,
    mixer::{Mixer, MixerCtx, MixerStatus},
    params::Params,
    pwm::PwmDriver,
    pwm::system::{PwmOutputState, compose_pwm_outputs, write_pwm_commands},
    sensors::ProcessedSensors,
    state_machine::{ErrorFlag, StateManager},
};

#[cfg(any(
    all(
        feature = "control-scope-estimator",
        feature = "control-scope-controller"
    ),
    all(feature = "control-scope-estimator", feature = "control-scope-mixer"),
    all(feature = "control-scope-estimator", feature = "control-scope-pwm"),
    all(feature = "control-scope-controller", feature = "control-scope-mixer"),
    all(feature = "control-scope-controller", feature = "control-scope-pwm"),
    all(feature = "control-scope-mixer", feature = "control-scope-pwm"),
))]
compile_error!("Enable only one control-scope-* feature at a time");

pub struct ControlPipelineResource<S, A, R: FlightFloat> {
    pub latest_estimator_state: S,
    pub latest_actuator_commands: Option<A>,
    pub latest_pwm_outputs: [R; crate::pwm::system::PWM_OUTPUT_CHANNELS],
    pub latest_loop_time_us: u16,
    last_imu_time: u64,
    pwm_rates_configured: bool,
}

impl<S, A, R> Default for ControlPipelineResource<S, A, R>
where
    S: Default,
    R: FlightFloat,
{
    fn default() -> Self {
        Self {
            latest_estimator_state: Default::default(),
            latest_actuator_commands: None,
            latest_pwm_outputs: [<R as FlightFloat>::from_f32(0.0);
                crate::pwm::system::PWM_OUTPUT_CHANNELS],
            latest_loop_time_us: 0,
            last_imu_time: 0,
            pwm_rates_configured: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlPipelineTiming {
    pub estimator_us: u16,
    pub controller_us: u16,
    pub mixer_us: u16,
    pub pwm_us: u16,
}

impl<S, A, R: FlightFloat> ControlPipelineResource<S, A, R> {
    pub(crate) fn last_imu_time(&self) -> u64 {
        self.last_imu_time
    }

    pub(crate) fn set_last_imu_time(&mut self, timestamp: u64) {
        self.last_imu_time = timestamp;
    }

    pub(crate) fn pwm_rates_configured(&self) -> bool {
        self.pwm_rates_configured
    }

    pub(crate) fn set_pwm_rates_configured(&mut self) {
        self.pwm_rates_configured = true;
    }

    pub(crate) fn invalidate_pwm_rates(&mut self) {
        self.pwm_rates_configured = false;
    }

    pub(crate) fn set_latest(
        &mut self,
        state: S,
        actuator_commands: A,
        pwm_outputs: [R; crate::pwm::system::PWM_OUTPUT_CHANNELS],
        loop_time_us: u16,
    ) {
        self.latest_estimator_state = state;
        self.latest_actuator_commands = Some(actuator_commands);
        self.latest_pwm_outputs = pwm_outputs;
        self.latest_loop_time_us = loop_time_us;
    }
}

pub struct ControlPipelineCtx<'a, B, E, C, M, PD, R: FlightFloat>
where
    B: BoardIo,
    E: Estimator<R>,
    C: Controller<R, State = E::State> + RcTrimCalibrator,
    M: Mixer<R, MixerInput = C::ControlOutput>,
    M::ActuatorCommands: AsRef<[R]> + Copy,
    E::State: Copy + Default,
    PD: PwmDriver<R>,
{
    pub board: &'a mut B,
    pub params: &'a Params,
    pub sensors: &'a ProcessedSensors<R>,
    pub external_attitude: &'a mut ExternalAttitudeState,
    pub aux_commands: &'a AuxCommandState,
    pub command: &'a CommandManager,
    pub state: &'a mut StateManager,
    pub estimator: &'a mut E,
    pub controller: &'a mut C,
    pub mixer: &'a mut M,
    pub control_pipeline: &'a mut ControlPipelineResource<E::State, M::ActuatorCommands, R>,
    pub pwm_output: &'a PwmOutputState,
    pub pwm: &'a mut PD,
    pub timing: Option<&'a mut ControlPipelineTiming>,
}

#[inline(always)]
fn control_scope_estimator<B: BoardIo>(board: &mut B, high: bool) {
    #[cfg(feature = "control-scope-estimator")]
    board.set_test_pin_3(high);
    #[cfg(not(feature = "control-scope-estimator"))]
    let _ = (board, high);
}

#[inline(always)]
fn control_scope_controller<B: BoardIo>(board: &mut B, high: bool) {
    #[cfg(feature = "control-scope-controller")]
    board.set_test_pin_3(high);
    #[cfg(not(feature = "control-scope-controller"))]
    let _ = (board, high);
}

#[inline(always)]
fn control_scope_mixer<B: BoardIo>(board: &mut B, high: bool) {
    #[cfg(feature = "control-scope-mixer")]
    board.set_test_pin_3(high);
    #[cfg(not(feature = "control-scope-mixer"))]
    let _ = (board, high);
}

#[inline(always)]
fn control_scope_pwm<B: BoardIo>(board: &mut B, high: bool) {
    #[cfg(feature = "control-scope-pwm")]
    board.set_test_pin_3(high);
    #[cfg(not(feature = "control-scope-pwm"))]
    let _ = (board, high);
}

pub fn run_control_pipeline_if_new_imu<B, E, C, M, PD, R>(
    mut ctx: ControlPipelineCtx<'_, B, E, C, M, PD, R>,
) -> bool
where
    B: BoardIo,
    E: Estimator<R>,
    C: Controller<R, State = E::State> + RcTrimCalibrator,
    M: Mixer<R, MixerInput = C::ControlOutput>,
    M::ActuatorCommands: AsRef<[R]> + Copy,
    E::State: Copy + Default,
    PD: PwmDriver<R>,
    R: FlightFloat,
{
    let Some(imu_packet) = ctx.sensors.imu else {
        return false;
    };

    let current_time = imu_packet.header.timestamp;
    let last_imu_time = ctx.control_pipeline.last_imu_time();
    if last_imu_time == 0 {
        ctx.control_pipeline.set_last_imu_time(current_time);
        return false;
    }

    if current_time < last_imu_time {
        ctx.state
            .set_error_flag(ErrorFlag::TIME_GOING_BACKWARDS, true, ctx.params);
        return false;
    }
    ctx.control_pipeline.set_last_imu_time(current_time);
    ctx.state
        .set_error_flag(ErrorFlag::TIME_GOING_BACKWARDS, false, ctx.params);
    ctx.board.set_test_pin_2(true);

    let dt = <R as FlightFloat>::from_u64(current_time.saturating_sub(last_imu_time))
        * <R as FlightFloat>::from_f32(1e-6);

    let loop_start_us = ctx.board.clock_micros();
    let external_attitude = ctx.external_attitude.latest.take();
    let estimator_start_us = ctx.timing.is_some().then(|| ctx.board.clock_micros());
    control_scope_estimator(ctx.board, true);
    let state = ctx.estimator.estimate_with_external_attitude_cached_params(
        ctx.sensors,
        ctx.params,
        dt,
        external_attitude,
    );
    control_scope_estimator(ctx.board, false);
    if let Some(estimator_start_us) = estimator_start_us {
        let elapsed_us = elapsed_u16(estimator_start_us, ctx.board.clock_micros());
        if let Some(timing) = &mut ctx.timing {
            timing.estimator_us = elapsed_us;
        }
    }

    if state.is_healthy() {
        ctx.state
            .set_error_flag(ErrorFlag::UNHEALTHY_ESTIMATOR, false, ctx.params);
    } else {
        ctx.state
            .set_error_flag(ErrorFlag::UNHEALTHY_ESTIMATOR, true, ctx.params);
    }

    let air_density = ctx.sensors.air_density();
    let controller_start_us = ctx.timing.is_some().then(|| ctx.board.clock_micros());
    control_scope_controller(ctx.board, true);
    let controls = ctx.controller.control(
        &state,
        ControllerCtx {
            state_manager: ctx.state,
            command: ctx.command.combined_control(),
            params: ctx.params,
            air_density,
            dt,
        },
    );
    control_scope_controller(ctx.board, false);
    if let Some(controller_start_us) = controller_start_us {
        let elapsed_us = elapsed_u16(controller_start_us, ctx.board.clock_micros());
        if let Some(timing) = &mut ctx.timing {
            timing.controller_us = elapsed_us;
        }
    }

    let mixer_start_us = ctx.timing.is_some().then(|| ctx.board.clock_micros());
    control_scope_mixer(ctx.board, true);
    let mixer_run = ctx.mixer.mix(
        &controls,
        MixerCtx {
            state: ctx.state,
            params: ctx.params,
            rc_override: ctx.command.get_rc_override(),
            air_density,
            battery_voltage: ctx
                .sensors
                .battery
                .map(|battery| <R as FlightFloat>::from_f32(battery.voltage)),
        },
    );
    control_scope_mixer(ctx.board, false);
    match mixer_run.status {
        MixerStatus::Healthy => {
            ctx.state
                .set_error_flag(ErrorFlag::INVALID_MIXER, false, ctx.params)
        }
        MixerStatus::InvalidMixer => {
            ctx.state
                .set_error_flag(ErrorFlag::INVALID_MIXER, true, ctx.params)
        }
    }
    if let Some(mixer_start_us) = mixer_start_us {
        let elapsed_us = elapsed_u16(mixer_start_us, ctx.board.clock_micros());
        if let Some(timing) = &mut ctx.timing {
            timing.mixer_us = elapsed_us;
        }
    }

    let pwm_start_us = ctx.timing.is_some().then(|| ctx.board.clock_micros());
    let actuator_commands = mixer_run.commands;
    control_scope_pwm(ctx.board, true);
    if !ctx.control_pipeline.pwm_rates_configured() {
        if ctx
            .pwm
            .configure_output_rates(ctx.mixer.default_pwm_rates())
            .is_ok()
        {
            ctx.control_pipeline.set_pwm_rates_configured();
        } else {
            crate::log_warn!("PWM driver rejected mixer default output rates");
        }
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
    control_scope_pwm(ctx.board, false);
    if let Some(pwm_start_us) = pwm_start_us {
        let elapsed_us = elapsed_u16(pwm_start_us, ctx.board.clock_micros());
        if let Some(timing) = &mut ctx.timing {
            timing.pwm_us = elapsed_us;
        }
    }
    let loop_time_us = ctx
        .board
        .clock_micros()
        .saturating_sub(loop_start_us)
        .min(u16::MAX as u64) as u16;
    ctx.control_pipeline
        .set_latest(state, actuator_commands, pwm_outputs, loop_time_us);
    ctx.board.set_test_pin_2(false);
    true
}

fn elapsed_u16(start_us: u64, end_us: u64) -> u16 {
    end_us.saturating_sub(start_us).min(u16::MAX as u64) as u16
}
