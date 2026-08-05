use crate::{
    board::BoardIo,
    comm::messages::messages::RosflightHardErrorMsg,
    comm::{
        CommManager, RealtimeTelemetryPriority, TelemetryCtx, TelemetryRates,
        interface::CommInterface,
    },
    command::CommandManager,
    command::service::{self as command_service, CommandRequestCtx},
    companion::{
        self, AuxCommandState, CompanionInputCtx, CompanionLinkState, ExternalAttitudeState,
    },
    control::{
        ControlPipelineCtx, ControlPipelineResource, ControlPipelineTiming,
        run_control_pipeline_if_new_imu,
    },
    controller::{Controller, RcTrimCalibrator},
    estimator::Estimator,
    events::{CommEventQueues, CommandEventQueues, CompanionEventQueues, ParamEventQueues},
    log::drain::{self as log_drain, LogDrainCtx},
    math::FlightFloat,
    params::reactions::{self, ParamReactionCtx},
    params::service::{self as param_service, ParamListState, ParamServiceCtx},
    params::{ParamId, ParamValue, Params},
    ports::EventEmitPort,
    pwm::PwmDriver,
    pwm::output_sync::{PwmOutputState, PwmSyncCtx, sync_pwm_output_state},
    rc::Rc,
    rc::command_state::{RcCommandStateCtx, run_rc_command_state},
    sensors::health::{SensorHealthCtx, update_imu_calibration_error, update_sensor_health},
    sensors::ingestion::{
        SensorIngestionCtx, SensorProcessorSet, process_imu_sensor, process_sensor_bus,
    },
    sensors::processors::CalibrationFlags,
    sensors::{ProcessedSensors, SensorBus},
    state_machine::{Event, StateManager},
};

const IMU_TIMEOUT_US: u64 = 100_000;
const REALTIME_SERVICE_RESPONSE_BUDGET: usize = 1;
const REALTIME_SERVICE_MIN_CONTROL_SLACK_US: u64 = 200;

#[derive(Clone, Copy, Debug)]
struct ImuSampleAccumulator<R: FlightFloat> {
    accel_sum: [R; 3],
    gyro_sum: [R; 3],
    temperature_sum: f32,
    count: u16,
    latest_header: crate::packets::RosflightPacketHeader,
    latest_seq: u32,
}

impl<R: FlightFloat> Default for ImuSampleAccumulator<R> {
    fn default() -> Self {
        Self {
            accel_sum: [<R as FlightFloat>::from_f32(0.0); 3],
            gyro_sum: [<R as FlightFloat>::from_f32(0.0); 3],
            temperature_sum: 0.0,
            count: 0,
            latest_header: crate::packets::RosflightPacketHeader::default(),
            latest_seq: 0,
        }
    }
}

impl<R: FlightFloat> ImuSampleAccumulator<R> {
    fn has_samples(&self) -> bool {
        self.count != 0
    }

    fn push(&mut self, sample: crate::packets::ImuPacket<R>) {
        self.accel_sum[0] += sample.accel[0];
        self.accel_sum[1] += sample.accel[1];
        self.accel_sum[2] += sample.accel[2];
        self.gyro_sum[0] += sample.gyro[0];
        self.gyro_sum[1] += sample.gyro[1];
        self.gyro_sum[2] += sample.gyro[2];
        self.temperature_sum += sample.temperature;
        self.count = self.count.saturating_add(1);
        self.latest_header = sample.header;
        self.latest_seq = sample.seq;
    }

    fn take_average(&mut self) -> Option<crate::packets::ImuPacket<R>> {
        if self.count == 0 {
            return None;
        }
        let count = <R as FlightFloat>::from_u64(self.count as u64);
        let temperature_count = self.count as f32;
        let sample = crate::packets::ImuPacket {
            header: self.latest_header,
            accel: [
                self.accel_sum[0] / count,
                self.accel_sum[1] / count,
                self.accel_sum[2] / count,
            ],
            gyro: [
                self.gyro_sum[0] / count,
                self.gyro_sum[1] / count,
                self.gyro_sum[2] / count,
            ],
            temperature: self.temperature_sum / temperature_count,
            seq: self.latest_seq,
        };
        *self = Self::default();
        Some(sample)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WorldReport {
    pub had_rx: bool,
    pub had_raw_sensor: bool,
    pub had_raw_imu: bool,
    pub had_raw_baro: bool,
    pub had_raw_rc: bool,
    pub had_processed_imu: bool,
    pub had_processed_baro: bool,
    pub had_processed_rc: bool,
    pub telemetry_due: bool,
    pub telemetry_deferred: bool,
    pub ran_control: bool,
    pub elapsed_after_control_us: u32,
    pub estimator_us: u16,
    pub controller_us: u16,
    pub mixer_us: u16,
    pub pwm_us: u16,
}

impl WorldReport {
    fn merge_from(&mut self, other: Self) {
        self.had_rx |= other.had_rx;
        self.had_raw_sensor |= other.had_raw_sensor;
        self.had_raw_imu |= other.had_raw_imu;
        self.had_raw_baro |= other.had_raw_baro;
        self.had_raw_rc |= other.had_raw_rc;
        self.had_processed_imu |= other.had_processed_imu;
        self.had_processed_baro |= other.had_processed_baro;
        self.had_processed_rc |= other.had_processed_rc;
        self.telemetry_due |= other.telemetry_due;
        self.telemetry_deferred |= other.telemetry_deferred;
        self.ran_control |= other.ran_control;
        self.elapsed_after_control_us = self
            .elapsed_after_control_us
            .saturating_add(other.elapsed_after_control_us);
        self.estimator_us = self.estimator_us.saturating_add(other.estimator_us);
        self.controller_us = self.controller_us.saturating_add(other.controller_us);
        self.mixer_us = self.mixer_us.saturating_add(other.mixer_us);
        self.pwm_us = self.pwm_us.saturating_add(other.pwm_us);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RealtimeSchedulerStep {
    ImuControl,
    ControlUpdate,
    Service,
    Idle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RealtimeServicePolicy {
    pub min_spacing_us: u64,
    pub telemetry_streams_per_phase: usize,
    pub continue_when_idle: bool,
}

impl RealtimeServicePolicy {
    pub const fn with_spacing(min_spacing_us: u64, telemetry_streams_per_phase: usize) -> Self {
        Self {
            min_spacing_us,
            telemetry_streams_per_phase,
            continue_when_idle: false,
        }
    }

    pub const fn continuous(telemetry_streams_per_phase: usize) -> Self {
        Self {
            min_spacing_us: 0,
            telemetry_streams_per_phase,
            continue_when_idle: false,
        }
    }

    pub const fn continuous_polling(telemetry_streams_per_phase: usize) -> Self {
        Self {
            min_spacing_us: 0,
            telemetry_streams_per_phase,
            continue_when_idle: true,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlLoopRates {
    /// Full estimator/controller/mixer/PWM update rate. A value of 0 runs control on every new
    /// IMU sample.
    pub control_hz: u16,
}

impl ControlLoopRates {
    pub const fn every_imu_sample() -> Self {
        Self { control_hz: 0 }
    }

    pub const fn fixed_rate_hz(control_hz: u16) -> Self {
        Self { control_hz }
    }
}

impl Default for ControlLoopRates {
    fn default() -> Self {
        Self::every_imu_sample()
    }
}

pub struct World<B, E, C, M, CI, PD, R: FlightFloat>
where
    B: BoardIo,
    E: Estimator<R>,
    C: Controller<R, State = E::State> + RcTrimCalibrator,
    M: crate::mixer::Mixer<R, MixerInput = C::ControlOutput>,
    M::ActuatorCommands: AsRef<[R]> + Copy,
    E::State: Copy + Default,
    CI: CommInterface<B>,
    PD: PwmDriver<R>,
{
    board: B,
    params: Params,
    param_list_state: ParamListState,
    param_events: ParamEventQueues,
    comm_events: CommEventQueues,
    command_events: CommandEventQueues,
    companion_events: CompanionEventQueues,
    companion_link: CompanionLinkState,
    pending_hard_error: Option<RosflightHardErrorMsg>,
    aux_commands: AuxCommandState,
    external_attitude: ExternalAttitudeState,
    comm: CommManager<B, CI>,
    raw_sensors: SensorBus<R>,
    processed_sensors: ProcessedSensors<R>,
    control_imu_accumulator: ImuSampleAccumulator<R>,
    sensor_processors: SensorProcessorSet<R>,
    rc: Rc,
    command: CommandManager,
    state: StateManager,
    cal_flags: CalibrationFlags,
    estimator: E,
    controller: C,
    mixer: M,
    control_pipeline: ControlPipelineResource<E::State, M::ActuatorCommands, R>,
    pwm_output: PwmOutputState,
    pwm: PD,
    last_imu_seen: u64,
    control_loop_rates: ControlLoopRates,
    last_control_update_us: u64,
    last_realtime_control_us: u64,
    next_realtime_service_us: u64,
}

impl<B, E, C, M, CI, PD, R> World<B, E, C, M, CI, PD, R>
where
    B: BoardIo,
    E: Estimator<R>,
    C: Controller<R, State = E::State> + RcTrimCalibrator,
    M: crate::mixer::Mixer<R, MixerInput = C::ControlOutput>,
    M::ActuatorCommands: AsRef<[R]> + Copy,
    E::State: Copy + Default,
    CI: CommInterface<B>,
    PD: PwmDriver<R>,
    R: FlightFloat,
{
    pub fn init(
        mut board: B,
        mut params: Params,
        comm_link: CI,
        mut state: StateManager,
        estimator: E,
        controller: C,
        mixer: M,
        pwm: PD,
    ) -> Self {
        crate::mixer::matrix::sync_reflected_mixer_params(
            &mut params,
            ParamId::PARAM_PRIMARY_MIXER,
        );
        crate::mixer::matrix::sync_reflected_mixer_params(
            &mut params,
            ParamId::PARAM_SECONDARY_MIXER,
        );

        state.update(Event::INITIALIZED, &params);
        // Match ROSflight C's Sensors::init_imu(): establish the calibration
        // interlock from persisted parameters before the first IMU sample or
        // arming request can be processed.
        update_imu_calibration_error(&mut state, &params);

        let mut rc = Rc::new();
        rc.init(&params);

        let mut command = CommandManager::new();
        command.init(&params, &mut state);

        let now_us = board.clock_micros();
        let mut comm = CommManager::new(comm_link, now_us);
        comm.configure_telemetry_from_params(&params);

        let pwm_output = PwmOutputState::new(pwm.is_enabled());

        let pending_hard_error = board.backup_memory_read().map(|data| {
            let _ = board.backup_memory_clear();
            RosflightHardErrorMsg {
                error_code: data.error_code,
                pc: data.pc,
                reset_count: data.reset_count,
                do_rearm: data.do_rearm,
            }
        });
        let do_rearm_after_hardfault = pending_hard_error
            .as_ref()
            .map(|msg| msg.do_rearm != 0)
            .unwrap_or(false);

        let mut world = Self {
            board,
            params,
            param_list_state: ParamListState::default(),
            param_events: ParamEventQueues::default(),
            comm_events: CommEventQueues::default(),
            command_events: CommandEventQueues::default(),
            companion_events: CompanionEventQueues::default(),
            companion_link: CompanionLinkState::default(),
            pending_hard_error,
            aux_commands: AuxCommandState::default(),
            external_attitude: ExternalAttitudeState::default(),
            comm,
            raw_sensors: SensorBus::default(),
            processed_sensors: ProcessedSensors::default(),
            control_imu_accumulator: ImuSampleAccumulator::default(),
            sensor_processors: SensorProcessorSet::default(),
            rc,
            command,
            state,
            cal_flags: CalibrationFlags::empty(),
            estimator,
            controller,
            mixer,
            control_pipeline: ControlPipelineResource::default(),
            pwm_output,
            pwm,
            last_imu_seen: now_us,
            control_loop_rates: ControlLoopRates::default(),
            last_control_update_us: now_us,
            last_realtime_control_us: now_us,
            next_realtime_service_us: now_us,
        };
        if do_rearm_after_hardfault {
            world
                .state
                .update(Event::HARDFAULT_REARM_REQUESTED, &world.params);
        }
        world.estimator.update_params(&world.params);
        world.controller.update_gains(&world.params);
        world
    }
}

mod control;
mod service;
mod telemetry;
#[cfg(test)]
mod tests;
