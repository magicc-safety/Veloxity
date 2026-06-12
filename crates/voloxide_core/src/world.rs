use crate::{
    board::BoardIo,
    comm::messages::messages::RosflightHardErrorMsg,
    comm::{CommManager, TelemetryRates, interface::CommInterface},
    command::CommandManager,
    command::service::{
        self as command_service, BoardCommandCtx, CalibrationRequestCtx, ConfigInfoCtx,
        OffboardControlCtx, ParamDefaultsCtx, ResetOriginCtx, VersionRequestCtx,
    },
    companion::{
        self, AuxCommandCtx, AuxCommandState, CompanionHeartbeatCtx, CompanionLinkState,
        ExternalAttitudeCtx, ExternalAttitudeState,
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
    params::reactions::{self, CommandParamChangedCtx, RcParamChangedCtx},
    params::service::{
        self as param_service, ParamApplyCtx, ParamListCtx, ParamListState, ParamReadCtx,
    },
    params::{ParamId, ParamValue, Params},
    ports::{EventDrainPort, EventEmitPort, EventReadPort, ParamsReadPort, ParamsWritePort},
    pwm::PwmDriver,
    pwm::system::{PwmOutputState, PwmSyncCtx, sync_pwm_output_state},
    rc::Rc,
    rc::system::{RcCommandStateCtx, run_rc_command_state},
    sensors::health::{SensorHealthCtx, update_sensor_health},
    sensors::ingestion::{SensorProcessorSet, process_imu_sensor, process_sensor_bus},
    sensors::processors::CalibrationFlags,
    sensors::{ProcessedSensors, SensorBus},
    state_machine::{ErrorFlag, Event, StateManager},
};
#[cfg(feature = "timing-diagnostics")]
use crate::{
    comm::messages::{enums::Severity, messages::StatustextMsg},
    events::CommResponse,
};
#[cfg(feature = "timing-diagnostics")]
use core::fmt::Write;
#[cfg(feature = "timing-diagnostics")]
use heapless::String;

const IMU_TIMEOUT_US: u64 = 100_000;
const REALTIME_SERVICE_RESPONSE_BUDGET: usize = 1;
const REALTIME_SERVICE_MIN_CONTROL_SLACK_US: u64 = 200;
const REALTIME_SERVICE_WINDOW_AFTER_CONTROL_US: u64 = 120;
#[cfg(feature = "timing-diagnostics")]
const TIMING_DIAGNOSTIC_INTERVAL_US: u64 = 1_000_000;
#[cfg(feature = "timing-diagnostics")]
const TIMING_CLASS_COUNT: usize = 5;

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

#[cfg(feature = "timing-diagnostics")]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WorldRunStats {
    pub total_us: u16,
    pub comm_us: u16,
    pub sensor_us: u16,
    pub control_us: u16,
    pub telemetry_us: u16,
    pub sensor_update_us: u16,
    pub sensor_process_us: u16,
    pub sensor_health_us: u16,
    pub log_response_us: u16,
    pub rc_us: u16,
    pub estimator_us: u16,
    pub controller_us: u16,
    pub mixer_us: u16,
    pub pwm_us: u16,
    pub telemetry_enqueue_us: u16,
    pub tx_flush_us: u16,
    pub board_service_us: u16,
    pub had_rx: bool,
    pub had_sensor: bool,
    pub had_imu: bool,
    pub ran_control: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WorldRunClass {
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RealtimeSchedulerStep {
    ImuControl,
    ControlUpdate,
    Service,
    Idle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlLoopRates {
    /// Full estimator/controller/mixer/PWM update rate. A value of 0 preserves the legacy
    /// behavior of running control on every new IMU sample.
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

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum RealtimeServicePhase {
    #[default]
    Input,
    Sensors,
    RcCommand,
    Responses,
    Telemetry0,
    Telemetry1,
    Telemetry2,
    Flush,
    DeferredBoard,
}

const REALTIME_TELEMETRY_STREAMS_PER_SERVICE_STEP: usize = 2;
const REALTIME_TELEMETRY_STREAMS_PER_TELEMETRY_PHASE: usize = 1;

impl RealtimeServicePhase {
    fn next(self) -> Self {
        match self {
            Self::Input => Self::Sensors,
            Self::Sensors => Self::RcCommand,
            Self::RcCommand => Self::Responses,
            Self::Responses => Self::Telemetry0,
            Self::Telemetry0 => Self::Telemetry1,
            Self::Telemetry1 => Self::Telemetry2,
            Self::Telemetry2 => Self::Flush,
            Self::Flush => Self::DeferredBoard,
            Self::DeferredBoard => Self::Input,
        }
    }
}

#[cfg(feature = "timing-diagnostics")]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct SensorStagePresence {
    had_sensor: bool,
    had_imu: bool,
}

#[cfg(feature = "timing-diagnostics")]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct SensorStageTiming {
    presence: SensorStagePresence,
    update_us: u16,
    process_us: u16,
    health_us: u16,
    log_response_us: u16,
}

#[cfg(feature = "timing-diagnostics")]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct TimingBucket {
    count: u32,
    total_us_sum: u32,
    comm_us_sum: u32,
    sensor_us_sum: u32,
    control_us_sum: u32,
    telemetry_us_sum: u32,
    sensor_update_us_sum: u32,
    sensor_process_us_sum: u32,
    sensor_health_us_sum: u32,
    log_response_us_sum: u32,
    rc_us_sum: u32,
    estimator_us_sum: u32,
    controller_us_sum: u32,
    mixer_us_sum: u32,
    pwm_us_sum: u32,
    telemetry_enqueue_us_sum: u32,
    tx_flush_us_sum: u32,
    board_service_us_sum: u32,
    total_us_max: u16,
}

#[cfg(feature = "timing-diagnostics")]
impl TimingBucket {
    fn record(&mut self, stats: WorldRunStats) {
        self.count = self.count.saturating_add(1);
        self.total_us_sum = self.total_us_sum.saturating_add(stats.total_us as u32);
        self.comm_us_sum = self.comm_us_sum.saturating_add(stats.comm_us as u32);
        self.sensor_us_sum = self.sensor_us_sum.saturating_add(stats.sensor_us as u32);
        self.control_us_sum = self.control_us_sum.saturating_add(stats.control_us as u32);
        self.telemetry_us_sum = self
            .telemetry_us_sum
            .saturating_add(stats.telemetry_us as u32);
        self.sensor_update_us_sum = self
            .sensor_update_us_sum
            .saturating_add(stats.sensor_update_us as u32);
        self.sensor_process_us_sum = self
            .sensor_process_us_sum
            .saturating_add(stats.sensor_process_us as u32);
        self.sensor_health_us_sum = self
            .sensor_health_us_sum
            .saturating_add(stats.sensor_health_us as u32);
        self.log_response_us_sum = self
            .log_response_us_sum
            .saturating_add(stats.log_response_us as u32);
        self.rc_us_sum = self.rc_us_sum.saturating_add(stats.rc_us as u32);
        self.estimator_us_sum = self
            .estimator_us_sum
            .saturating_add(stats.estimator_us as u32);
        self.controller_us_sum = self
            .controller_us_sum
            .saturating_add(stats.controller_us as u32);
        self.mixer_us_sum = self.mixer_us_sum.saturating_add(stats.mixer_us as u32);
        self.pwm_us_sum = self.pwm_us_sum.saturating_add(stats.pwm_us as u32);
        self.telemetry_enqueue_us_sum = self
            .telemetry_enqueue_us_sum
            .saturating_add(stats.telemetry_enqueue_us as u32);
        self.tx_flush_us_sum = self
            .tx_flush_us_sum
            .saturating_add(stats.tx_flush_us as u32);
        self.board_service_us_sum = self
            .board_service_us_sum
            .saturating_add(stats.board_service_us as u32);
        self.total_us_max = self.total_us_max.max(stats.total_us);
    }

    fn avg(sum: u32, count: u32) -> u32 {
        if count == 0 { 0 } else { sum / count }
    }
}

#[cfg(feature = "timing-diagnostics")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct TimingDiagnostics {
    last_emit_us: u64,
    buckets: [TimingBucket; TIMING_CLASS_COUNT],
}

#[cfg(feature = "timing-diagnostics")]
impl TimingDiagnostics {
    fn new(now_us: u64) -> Self {
        Self {
            last_emit_us: now_us,
            buckets: [TimingBucket::default(); TIMING_CLASS_COUNT],
        }
    }

    fn record(&mut self, stats: WorldRunStats) {
        self.buckets[timing_class_index(stats)].record(stats);
    }

    fn due(&self, now_us: u64) -> bool {
        now_us.saturating_sub(self.last_emit_us) >= TIMING_DIAGNOSTIC_INTERVAL_US
    }

    fn reset(&mut self, now_us: u64) {
        self.last_emit_us = now_us;
        self.buckets = [TimingBucket::default(); TIMING_CLASS_COUNT];
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
    last_rc_command_state_ms: Option<u32>,
    control_loop_rates: ControlLoopRates,
    last_control_update_us: u64,
    last_realtime_control_us: u64,
    last_realtime_service_control_us: u64,
    next_realtime_service_us: u64,
    realtime_service_phase: RealtimeServicePhase,
    #[cfg(feature = "timing-diagnostics")]
    timing_diagnostics: TimingDiagnostics,
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

        let mut rc = Rc::new();
        rc.init(&mut board, &params);

        let mut command = CommandManager::new();
        command.init(&params, &mut state);

        let now_us = board.clock_micros();
        let comm = CommManager::new(comm_link, now_us);

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
            last_rc_command_state_ms: None,
            control_loop_rates: ControlLoopRates::default(),
            last_control_update_us: now_us,
            last_realtime_control_us: now_us,
            last_realtime_service_control_us: 0,
            next_realtime_service_us: now_us,
            realtime_service_phase: RealtimeServicePhase::default(),
            #[cfg(feature = "timing-diagnostics")]
            timing_diagnostics: TimingDiagnostics::new(now_us),
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

    pub fn run_once(&mut self) -> bool {
        #[cfg(feature = "timing-diagnostics")]
        {
            let _ = self.run_once_measured();
            return true;
        }

        #[cfg(not(feature = "timing-diagnostics"))]
        {
            self.run_communication_and_parameter_service_stage();
            self.run_sensor_ingestion_and_health_stage();
            self.run_rc_command_state_stages();
            self.run_control_and_mixing_stage_if_new_imu();
            self.run_telemetry_stage();
            self.board.serial_flush();
            self.board.run_deferred_board_actions();
            true
        }
    }

    #[cfg(not(feature = "timing-diagnostics"))]
    pub fn run_once_classified(&mut self) -> WorldRunClass {
        self.run_once_budgeted_classified()
    }

    #[cfg(not(feature = "timing-diagnostics"))]
    pub fn run_once_spike_counted(&mut self) -> WorldRunClass {
        let pass_start_us = self.board.clock_micros();
        let had_rx = self.board.serial_rx_pending();
        self.run_communication_and_parameter_service_stage();

        self.board.update_sensor_bus(&mut self.raw_sensors);
        let had_raw_imu = self.raw_sensors.imu.is_some();
        let had_raw_baro = self.raw_sensors.baro.is_some();
        let had_raw_rc = self.raw_sensors.rc.is_some();
        let had_raw_sensor = raw_sensor_present(&self.raw_sensors);
        self.process_sensor_bus_after_update();
        self.update_sensor_health_and_calibration(self.board.clock_micros());

        #[cfg(feature = "rc-command-scope")]
        self.board.set_test_pin_3(true);
        self.run_rc_command_state_stages();
        #[cfg(feature = "rc-command-scope")]
        self.board.set_test_pin_3(false);
        let had_processed_imu = self.processed_sensors.imu.is_some();
        let had_processed_baro = self.processed_sensors.baro.is_some();
        let had_processed_rc = self.processed_sensors.rc.is_some();
        let ran_control = self.run_control_and_mixing_stage_if_new_imu();
        let telemetry_due = self
            .comm
            .named_telemetry_due(self.board.clock_micros(), &self.processed_sensors)
            || !self.comm_events.is_empty();
        let elapsed_after_control_us = self.board.clock_micros().saturating_sub(pass_start_us);

        self.drain_logs_and_send_responses();
        self.run_telemetry_stage();
        self.board.serial_flush();
        self.board.run_deferred_board_actions();

        WorldRunClass {
            had_rx,
            had_raw_sensor,
            had_raw_imu,
            had_raw_baro,
            had_raw_rc,
            had_processed_imu,
            had_processed_baro,
            had_processed_rc,
            telemetry_due,
            telemetry_deferred: false,
            ran_control,
            elapsed_after_control_us: elapsed_after_control_us.min(u32::MAX as u64) as u32,
            estimator_us: 0,
            controller_us: 0,
            mixer_us: 0,
            pwm_us: 0,
        }
    }

    #[cfg(not(feature = "timing-diagnostics"))]
    pub fn run_once_budgeted_classified(&mut self) -> WorldRunClass {
        let pass_start_us = self.board.clock_micros();
        let had_rx = self.board.serial_rx_pending();
        self.run_communication_and_parameter_service_stage();

        self.board.update_sensor_bus(&mut self.raw_sensors);
        let had_raw_imu = self.raw_sensors.imu.is_some();
        let had_raw_baro = self.raw_sensors.baro.is_some();
        let had_raw_rc = self.raw_sensors.rc.is_some();
        let had_raw_sensor = raw_sensor_present(&self.raw_sensors);
        self.process_sensor_bus_after_update();
        self.update_sensor_health_and_calibration(self.board.clock_micros());

        #[cfg(feature = "rc-command-scope")]
        self.board.set_test_pin_3(true);
        self.run_rc_command_state_stages();
        #[cfg(feature = "rc-command-scope")]
        self.board.set_test_pin_3(false);
        let had_processed_imu = self.processed_sensors.imu.is_some();
        let had_processed_baro = self.processed_sensors.baro.is_some();
        let had_processed_rc = self.processed_sensors.rc.is_some();
        let mut control_timing = ControlPipelineTiming::default();
        let ran_control =
            self.run_control_and_mixing_stage_if_new_imu_measured(&mut control_timing);
        let telemetry_due = self
            .comm
            .named_telemetry_due(self.board.clock_micros(), &self.processed_sensors)
            || !self.comm_events.is_empty();
        let elapsed_after_control_us = self.board.clock_micros().saturating_sub(pass_start_us);
        self.drain_logs_and_send_responses();
        self.run_telemetry_stage();
        self.board.serial_flush();
        self.board.run_deferred_board_actions();

        WorldRunClass {
            had_rx,
            had_raw_sensor,
            had_raw_imu,
            had_raw_baro,
            had_raw_rc,
            had_processed_imu,
            had_processed_baro,
            had_processed_rc,
            telemetry_due,
            telemetry_deferred: false,
            ran_control,
            elapsed_after_control_us: elapsed_after_control_us.min(u32::MAX as u64) as u32,
            estimator_us: control_timing.estimator_us,
            controller_us: control_timing.controller_us,
            mixer_us: control_timing.mixer_us,
            pwm_us: control_timing.pwm_us,
        }
    }

    #[cfg(feature = "timing-diagnostics")]
    pub fn run_once_classified(&mut self) -> WorldRunClass {
        let stats = self.run_once_measured();
        WorldRunClass {
            had_rx: stats.had_rx,
            had_raw_sensor: stats.had_sensor,
            had_raw_imu: stats.had_imu,
            had_raw_baro: false,
            had_raw_rc: false,
            had_processed_imu: stats.had_imu,
            had_processed_baro: false,
            had_processed_rc: false,
            telemetry_due: stats.telemetry_us != 0,
            telemetry_deferred: false,
            ran_control: stats.ran_control,
            elapsed_after_control_us: stats.total_us as u32,
            estimator_us: stats.estimator_us,
            controller_us: stats.controller_us,
            mixer_us: stats.mixer_us,
            pwm_us: stats.pwm_us,
        }
    }

    #[cfg(feature = "timing-diagnostics")]
    pub fn run_once_measured(&mut self) -> WorldRunStats {
        let pass_start_us = self.board.clock_micros();
        let comm_start_us = self.board.clock_micros();
        self.run_communication_and_parameter_service_stage();
        let had_rx = self.board.serial_rx_pending() || self.board.serial_rx_last_count() > 0;
        let comm_us = elapsed_u16(comm_start_us, self.board.clock_micros());

        let sensor_start_us = self.board.clock_micros();
        let sensor_timing = self.run_sensor_ingestion_and_health_stage_measured(true);
        let sensor_us = elapsed_u16(sensor_start_us, self.board.clock_micros());

        let rc_start_us = self.board.clock_micros();
        self.run_rc_command_state_stages();
        let rc_us = elapsed_u16(rc_start_us, self.board.clock_micros());

        let mut control_timing = ControlPipelineTiming::default();
        let control_start_us = self.board.clock_micros();
        let ran_control =
            self.run_control_and_mixing_stage_if_new_imu_measured(&mut control_timing);
        let control_us = elapsed_u16(control_start_us, self.board.clock_micros());

        let telemetry_start_us = self.board.clock_micros();
        let telemetry_enqueue_start_us = self.board.clock_micros();
        self.run_telemetry_stage();
        let telemetry_enqueue_us =
            elapsed_u16(telemetry_enqueue_start_us, self.board.clock_micros());
        let tx_flush_start_us = self.board.clock_micros();
        self.board.serial_flush();
        let tx_flush_us = elapsed_u16(tx_flush_start_us, self.board.clock_micros());
        let telemetry_us = elapsed_u16(telemetry_start_us, self.board.clock_micros());

        let board_service_start_us = self.board.clock_micros();
        self.board.run_deferred_board_actions();
        let board_service_us = elapsed_u16(board_service_start_us, self.board.clock_micros());

        let stats = WorldRunStats {
            total_us: elapsed_u16(pass_start_us, self.board.clock_micros()),
            comm_us,
            sensor_us,
            control_us,
            telemetry_us,
            sensor_update_us: sensor_timing.update_us,
            sensor_process_us: sensor_timing.process_us,
            sensor_health_us: sensor_timing.health_us,
            log_response_us: sensor_timing.log_response_us,
            rc_us,
            estimator_us: control_timing.estimator_us,
            controller_us: control_timing.controller_us,
            mixer_us: control_timing.mixer_us,
            pwm_us: control_timing.pwm_us,
            telemetry_enqueue_us,
            tx_flush_us,
            board_service_us,
            had_rx,
            had_sensor: sensor_timing.presence.had_sensor,
            had_imu: sensor_timing.presence.had_imu,
            ran_control,
        };
        self.record_timing_diagnostics(stats);
        stats
    }

    pub fn set_telemetry_rates(&mut self, telemetry_rates: TelemetryRates) {
        self.comm.set_telemetry_rates(telemetry_rates);
    }

    pub fn set_control_loop_rates(&mut self, control_loop_rates: ControlLoopRates) {
        self.control_loop_rates = control_loop_rates;
        self.last_control_update_us = self.board.clock_micros();
    }

    pub fn set_test_pin_3(&mut self, high: bool) {
        self.board.set_test_pin_3(high);
    }

    pub fn set_test_pin_2(&mut self, high: bool) {
        self.board.set_test_pin_2(high);
    }

    pub fn run_comm_param_sensor_stages(&mut self) {
        self.run_communication_and_parameter_service_stage();
        self.run_sensor_ingestion_and_health_stage();
    }

    pub fn imu_pending(&self) -> bool {
        self.board.imu_pending()
    }

    pub fn realtime_scheduler_step(&self) -> RealtimeSchedulerStep {
        if self.imu_pending() {
            return RealtimeSchedulerStep::ImuControl;
        }
        let now_us = self.board.clock_micros();
        if self.control_update_can_run_at(now_us) {
            return RealtimeSchedulerStep::ControlUpdate;
        }
        if now_us >= self.next_realtime_service_us
            && now_us.saturating_sub(self.last_realtime_control_us)
                <= REALTIME_SERVICE_WINDOW_AFTER_CONTROL_US
            && self.last_realtime_service_control_us != self.last_realtime_control_us
            && self.realtime_service_has_control_slack(now_us)
        {
            RealtimeSchedulerStep::Service
        } else {
            RealtimeSchedulerStep::Idle
        }
    }

    pub fn run_imu_control_tick(&mut self) -> bool {
        #[cfg(feature = "pre-control-scope")]
        self.board.set_test_pin_3(true);
        let now_us = self.board.clock_micros();
        self.board.update_imu_sensor(&mut self.raw_sensors);
        self.process_imu_sensor_after_update();
        self.record_control_imu_candidate();
        self.update_sensor_health_and_calibration(now_us);
        #[cfg(feature = "pre-control-scope")]
        self.board.set_test_pin_3(false);
        let ran_control = self.run_control_and_mixing_stage_if_control_due(now_us);
        if ran_control {
            self.last_realtime_control_us = self.board.clock_micros();
        }
        ran_control
    }

    pub fn run_control_update_tick(&mut self) -> bool {
        let now_us = self.board.clock_micros();
        let ran_control = self.run_control_and_mixing_stage_if_control_due(now_us);
        if ran_control {
            self.last_realtime_control_us = self.board.clock_micros();
        }
        ran_control
    }

    pub fn run_imu_control_tick_classified(&mut self) -> WorldRunClass {
        let pass_start_us = self.board.clock_micros();
        #[cfg(feature = "pre-control-scope")]
        self.board.set_test_pin_3(true);
        let now_us = self.board.clock_micros();
        self.board.update_imu_sensor(&mut self.raw_sensors);
        let had_raw_imu = self.raw_sensors.imu.is_some();
        let had_raw_sensor = had_raw_imu;
        self.process_imu_sensor_after_update();
        self.record_control_imu_candidate();
        self.update_sensor_health_and_calibration(now_us);
        let had_processed_imu = self.processed_sensors.imu.is_some();
        let had_processed_rc = self.processed_sensors.rc.is_some();
        #[cfg(feature = "pre-control-scope")]
        self.board.set_test_pin_3(false);
        let mut control_timing = ControlPipelineTiming::default();
        let ran_control =
            self.run_control_and_mixing_stage_if_control_due_measured(now_us, &mut control_timing);
        if ran_control {
            self.last_realtime_control_us = self.board.clock_micros();
        }
        let telemetry_due = self
            .comm
            .named_telemetry_due(self.board.clock_micros(), &self.processed_sensors)
            || !self.comm_events.is_empty();

        WorldRunClass {
            had_raw_sensor,
            had_raw_imu,
            had_processed_imu,
            had_processed_rc,
            telemetry_due,
            telemetry_deferred: false,
            ran_control,
            elapsed_after_control_us: self
                .board
                .clock_micros()
                .saturating_sub(pass_start_us)
                .min(u32::MAX as u64) as u32,
            estimator_us: control_timing.estimator_us,
            controller_us: control_timing.controller_us,
            mixer_us: control_timing.mixer_us,
            pwm_us: control_timing.pwm_us,
            ..WorldRunClass::default()
        }
    }

    pub fn run_service_step(&mut self) -> WorldRunClass {
        let pass_start_us = self.board.clock_micros();
        let had_rx = self.board.serial_rx_pending();
        self.run_service_input_stage();
        self.run_service_sensor_and_rc_stage();
        self.drain_logs_and_send_responses();
        self.run_telemetry_stage();
        self.board.serial_flush();
        self.board.run_deferred_board_actions();

        WorldRunClass {
            had_rx,
            telemetry_due: self
                .comm
                .named_telemetry_due(self.board.clock_micros(), &self.processed_sensors)
                || !self.comm_events.is_empty(),
            elapsed_after_control_us: self
                .board
                .clock_micros()
                .saturating_sub(pass_start_us)
                .min(u32::MAX as u64) as u32,
            ..WorldRunClass::default()
        }
    }

    pub fn run_service_step_with_deferral(
        &mut self,
        max_service_deferral_us: u64,
    ) -> WorldRunClass {
        let pass_start_us = self.board.clock_micros();
        let had_rx = self.board.serial_rx_pending();
        let phase = self.realtime_service_phase;
        self.realtime_service_phase = self.realtime_service_phase.next();

        self.run_realtime_telemetry_stage_budgeted(REALTIME_TELEMETRY_STREAMS_PER_SERVICE_STEP);

        match phase {
            RealtimeServicePhase::Input => self.run_service_input_stage(),
            RealtimeServicePhase::Sensors => self.run_service_sensor_stage(),
            RealtimeServicePhase::RcCommand => self.run_rc_command_state_stages(),
            RealtimeServicePhase::Responses => {
                self.drain_logs_and_send_responses_limited(REALTIME_SERVICE_RESPONSE_BUDGET);
            }
            RealtimeServicePhase::Telemetry0
            | RealtimeServicePhase::Telemetry1
            | RealtimeServicePhase::Telemetry2 => {
                self.run_realtime_telemetry_stage_budgeted(
                    REALTIME_TELEMETRY_STREAMS_PER_TELEMETRY_PHASE,
                );
            }
            RealtimeServicePhase::Flush => self.board.serial_flush_budgeted(1),
            RealtimeServicePhase::DeferredBoard => self.board.run_deferred_board_actions(),
        }

        self.next_realtime_service_us = self
            .board
            .clock_micros()
            .saturating_add(max_service_deferral_us);
        self.last_realtime_service_control_us = self.last_realtime_control_us;

        WorldRunClass {
            had_rx,
            elapsed_after_control_us: self
                .board
                .clock_micros()
                .saturating_sub(pass_start_us)
                .min(u32::MAX as u64) as u32,
            ..WorldRunClass::default()
        }
    }

    fn run_service_input_stage(&mut self) {
        self.run_communication_and_parameter_service_stage();
    }

    fn run_service_sensor_and_rc_stage(&mut self) {
        self.run_service_sensor_stage();
        self.run_rc_command_state_stages();
    }

    fn run_service_sensor_stage(&mut self) {
        let now_us = self.board.clock_micros();
        let latest_imu = self.processed_sensors.imu;
        let latest_mag = self.processed_sensors.mag;
        let latest_baro = self.processed_sensors.baro;
        let latest_pitot = self.processed_sensors.pitot;
        let latest_range = self.processed_sensors.range;
        let latest_gnss = self.processed_sensors.gnss;
        let latest_battery = self.processed_sensors.battery;
        let latest_rc = self.processed_sensors.rc;
        let latest_attitude = self.processed_sensors.attitude;

        self.board.update_service_sensor_bus(&mut self.raw_sensors);
        let had_raw_imu = self.raw_sensors.imu.is_some();
        let had_raw_mag = self.raw_sensors.mag.is_some();
        let had_raw_baro = self.raw_sensors.baro.is_some();
        let had_raw_pitot = self.raw_sensors.pitot.is_some();
        let had_raw_range = self.raw_sensors.range.is_some();
        let had_raw_gnss = self.raw_sensors.gnss.is_some();
        let had_raw_battery = self.raw_sensors.battery.is_some();
        let had_raw_rc = self.raw_sensors.rc.is_some();
        let had_raw_attitude = self.raw_sensors.attitude.is_some();
        self.process_sensor_bus_after_update();

        if !had_raw_imu {
            self.processed_sensors.imu = latest_imu;
        }
        if !had_raw_mag {
            self.processed_sensors.mag = latest_mag;
        }
        if !had_raw_baro {
            self.processed_sensors.baro = latest_baro;
        }
        if !had_raw_pitot {
            self.processed_sensors.pitot = latest_pitot;
        }
        if !had_raw_range {
            self.processed_sensors.range = latest_range;
        }
        if !had_raw_gnss {
            self.processed_sensors.gnss = latest_gnss;
        }
        if !had_raw_battery {
            self.processed_sensors.battery = latest_battery;
        }
        if !had_raw_rc {
            self.processed_sensors.rc = latest_rc;
        }
        if !had_raw_attitude {
            self.processed_sensors.attitude = latest_attitude;
        }

        self.update_sensor_health_and_calibration(now_us);
    }

    pub fn run_communication_and_parameter_service_stage(&mut self) {
        self.process_comm_stage();
        if self.has_pending_companion_work() {
            self.apply_companion_events();
        }
        if !self.command_events.is_empty() {
            self.apply_command_events();
        }
        if self.has_pending_param_work() {
            self.service_param_events();
        }
        if !self.param_events.changes.is_empty() {
            self.apply_param_reactions();
        }
        self.request_gyro_calibration_if_needed();
    }

    pub fn run_sensor_ingestion_and_health_stage(&mut self) {
        self.run_sensor_ingestion_and_health_stage_without_log_drain();
        self.drain_logs_and_send_responses();
    }

    fn run_sensor_ingestion_and_health_stage_without_log_drain(&mut self) {
        let now_us = self.board.clock_micros();

        self.run_sensor_ingestion_stage();
        self.update_sensor_health_and_calibration(now_us);
    }

    #[cfg(feature = "timing-diagnostics")]
    fn run_sensor_ingestion_and_health_stage_measured(
        &mut self,
        drain_logs: bool,
    ) -> SensorStageTiming {
        let now_us = self.board.clock_micros();

        let update_start_us = self.board.clock_micros();
        self.board.update_sensor_bus(&mut self.raw_sensors);
        let update_us = elapsed_u16(update_start_us, self.board.clock_micros());
        let sensor_presence = SensorStagePresence {
            had_sensor: raw_sensor_present(&self.raw_sensors),
            had_imu: self.raw_sensors.imu.is_some(),
        };

        let process_start_us = self.board.clock_micros();
        self.process_sensor_bus_after_update();
        let process_us = elapsed_u16(process_start_us, self.board.clock_micros());

        let health_start_us = self.board.clock_micros();
        self.update_sensor_health_and_calibration(now_us);
        let health_us = elapsed_u16(health_start_us, self.board.clock_micros());

        let log_response_us = if drain_logs {
            let log_start_us = self.board.clock_micros();
            self.drain_logs_and_send_responses();
            elapsed_u16(log_start_us, self.board.clock_micros())
        } else {
            0
        };

        SensorStageTiming {
            presence: sensor_presence,
            update_us,
            process_us,
            health_us,
            log_response_us,
        }
    }

    fn process_comm_stage(&mut self) {
        self.comm.process_incoming_messages(&mut self.board);
        if !self.comm.has_pending_messages() {
            return;
        }
        self.comm.act_on_messages(
            &mut self.param_events,
            &mut self.comm_events,
            &mut self.command_events,
            &mut self.companion_events,
            &mut self.board,
        );
    }

    fn has_pending_companion_work(&self) -> bool {
        !self.companion_events.is_empty()
            || (self.companion_link.connected && self.pending_hard_error.is_some())
    }

    fn has_pending_param_work(&self) -> bool {
        !self.param_events.set_requests.is_empty()
            || !self.param_events.read_requests.is_empty()
            || !self.param_events.list_requests.is_empty()
            || self.param_list_state.is_active()
    }

    fn apply_companion_events(&mut self) {
        companion::apply_companion_heartbeats(CompanionHeartbeatCtx {
            requests: EventDrainPort::new(&mut self.companion_events.heartbeats),
            state: &mut self.companion_link,
        });
        if self.companion_link.connected {
            if let Some(msg) = self.pending_hard_error.take() {
                self.comm_events
                    .responses
                    .push_or_log(crate::events::CommResponse::HardError(msg), "hard error");
            }
        }
        companion::apply_aux_commands(AuxCommandCtx {
            requests: EventDrainPort::new(&mut self.companion_events.aux_commands),
            state: &mut self.aux_commands,
        });
        companion::apply_external_attitudes(ExternalAttitudeCtx {
            requests: EventDrainPort::new(&mut self.companion_events.external_attitudes),
            state: &mut self.external_attitude,
        });
    }

    fn apply_command_events(&mut self) {
        command_service::apply_calibration_requests(CalibrationRequestCtx {
            requests: EventDrainPort::new(&mut self.command_events.calibration_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            state: &self.state,
            flags: &mut self.cal_flags,
            params: &mut self.params,
        });
        command_service::apply_offboard_control_requests(OffboardControlCtx {
            requests: EventDrainPort::new(&mut self.command_events.offboard_control_requests),
            command: &mut self.command,
            params: &self.params,
        });
        command_service::apply_param_defaults_requests(ParamDefaultsCtx {
            requests: EventDrainPort::new(&mut self.command_events.param_defaults_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            state: &self.state,
            params: &mut self.params,
        });

        command_service::apply_rc_trim_calibration_requests(
            command_service::RcTrimCalibrationCtx {
                requests: EventDrainPort::new(
                    &mut self.command_events.rc_trim_calibration_requests,
                ),
                responses: EventEmitPort::new(&mut self.comm_events.responses),
                state: &self.state,
                command: &self.command,
                controller: &mut self.controller,
                params: &mut self.params,
            },
        );

        command_service::apply_board_command_requests(BoardCommandCtx {
            requests: EventDrainPort::new(&mut self.command_events.board_command_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            state: &self.state,
            board: &mut self.board,
            params: &mut self.params,
        });

        command_service::apply_version_requests(VersionRequestCtx {
            requests: EventDrainPort::new(&mut self.command_events.version_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            state: &self.state,
        });

        command_service::apply_reset_origin_requests(ResetOriginCtx {
            requests: EventDrainPort::new(&mut self.command_events.reset_origin_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });

        command_service::apply_config_info_requests(ConfigInfoCtx {
            requests: EventDrainPort::new(&mut self.command_events.config_info_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });
    }

    fn service_param_events(&mut self) {
        param_service::service_param_read_requests(ParamReadCtx {
            params: ParamsReadPort::new(&self.params),
            requests: EventDrainPort::new(&mut self.param_events.read_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });

        param_service::service_param_list_requests(ParamListCtx {
            params: ParamsReadPort::new(&self.params),
            state: &mut self.param_list_state,
            requests: EventDrainPort::new(&mut self.param_events.list_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });

        param_service::apply_param_requests(ParamApplyCtx {
            params: ParamsWritePort::new(&mut self.params),
            requests: EventDrainPort::new(&mut self.param_events.set_requests),
            changes: EventEmitPort::new(&mut self.param_events.changes),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });
    }

    fn apply_param_reactions(&mut self) {
        let has_param_changes = self.param_events.changes.iter().next().is_some();
        for change in self.param_events.changes.iter() {
            let Some(status) = self.mixer.on_param_changed(&self.params, change.id) else {
                continue;
            };
            self.control_pipeline.invalidate_pwm_rates();
            match status {
                crate::mixer::MixerStatus::Healthy => self
                    .state
                    .update(Event::ERROR_CLEARED(ErrorFlag::INVALID_MIXER), &self.params),
                crate::mixer::MixerStatus::InvalidMixer => self.state.update(
                    Event::ERROR_OCCURRED(ErrorFlag::INVALID_MIXER),
                    &self.params,
                ),
            }
        }

        reactions::rc_on_param_changed(RcParamChangedCtx {
            rc: &mut self.rc,
            params: ParamsReadPort::new(&self.params),
            changes: EventReadPort::new(&self.param_events.changes),
        });

        reactions::command_on_param_changed(CommandParamChangedCtx {
            command: &mut self.command,
            state: &mut self.state,
            params: ParamsReadPort::new(&self.params),
            changes: EventReadPort::new(&self.param_events.changes),
        });

        if has_param_changes {
            self.estimator.update_params(&self.params);
            self.controller.update_gains(&self.params);
        }

        self.param_events.changes.clear();
    }

    fn request_gyro_calibration_if_needed(&mut self) {
        if self.state.is_calibrating() && !self.cal_flags.contains(CalibrationFlags::GYRO) {
            self.cal_flags.remove(CalibrationFlags::GYRO_FAILED);
            self.params
                .set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.0));
            self.params
                .set_by_id(ParamId::PARAM_GYRO_Y_BIAS, ParamValue::Float(0.0));
            self.params
                .set_by_id(ParamId::PARAM_GYRO_Z_BIAS, ParamValue::Float(0.0));
            self.cal_flags.insert(CalibrationFlags::GYRO);
        }
    }

    #[cfg(feature = "timing-diagnostics")]
    fn run_sensor_ingestion_stage(&mut self) -> SensorStagePresence {
        self.board.update_sensor_bus(&mut self.raw_sensors);
        let sensor_presence = SensorStagePresence {
            had_sensor: raw_sensor_present(&self.raw_sensors),
            had_imu: self.raw_sensors.imu.is_some(),
        };
        self.process_sensor_bus_after_update();
        sensor_presence
    }

    fn process_sensor_bus_after_update(&mut self) {
        let calibration_flags_before = self.cal_flags;
        process_sensor_bus(
            &mut self.raw_sensors,
            &mut self.processed_sensors,
            &mut self.sensor_processors,
            &mut self.cal_flags,
            &mut self.params,
        );
        if calibration_flags_before.contains(CalibrationFlags::GYRO)
            && !self.cal_flags.contains(CalibrationFlags::GYRO)
            && !self.cal_flags.contains(CalibrationFlags::GYRO_FAILED)
        {
            self.estimator.reset_adaptive_bias();
        }
        if calibration_flags_before.contains(CalibrationFlags::ACCEL)
            && !self.cal_flags.contains(CalibrationFlags::ACCEL)
            && !self.cal_flags.contains(CalibrationFlags::ACCEL_FAILED)
        {
            self.estimator.reset();
            self.control_pipeline = ControlPipelineResource::default();
        }
    }

    fn process_imu_sensor_after_update(&mut self) {
        let calibration_flags_before = self.cal_flags;
        process_imu_sensor(
            &mut self.raw_sensors,
            &mut self.processed_sensors,
            &mut self.sensor_processors.imu,
            &mut self.cal_flags,
            &mut self.params,
        );
        if calibration_flags_before.contains(CalibrationFlags::GYRO)
            && !self.cal_flags.contains(CalibrationFlags::GYRO)
            && !self.cal_flags.contains(CalibrationFlags::GYRO_FAILED)
        {
            self.estimator.reset_adaptive_bias();
        }
        if calibration_flags_before.contains(CalibrationFlags::ACCEL)
            && !self.cal_flags.contains(CalibrationFlags::ACCEL)
            && !self.cal_flags.contains(CalibrationFlags::ACCEL_FAILED)
        {
            self.estimator.reset();
            self.control_pipeline = ControlPipelineResource::default();
        }
    }

    fn record_control_imu_candidate(&mut self) {
        if let Some(imu) = self.processed_sensors.imu {
            self.control_imu_accumulator.push(imu);
        }
    }

    #[cfg(not(feature = "timing-diagnostics"))]
    fn run_sensor_ingestion_stage(&mut self) {
        self.board.update_sensor_bus(&mut self.raw_sensors);
        self.process_sensor_bus_after_update();
    }

    fn drain_logs_and_send_responses(&mut self) {
        log_drain::drain_logs_to_comm_responses(LogDrainCtx {
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            connected: self.companion_link.connected,
        });
        if self.comm_events.is_empty() {
            return;
        }
        self.comm
            .send_comm_responses(&mut self.board, &mut self.comm_events);
    }

    fn drain_logs_and_send_responses_limited(&mut self, max_responses: usize) {
        log_drain::drain_logs_to_comm_responses(LogDrainCtx {
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            connected: self.companion_link.connected,
        });
        if self.comm_events.is_empty() {
            return;
        }
        self.comm.send_comm_responses_limited(
            &mut self.board,
            &mut self.comm_events,
            max_responses,
        );
    }

    fn update_sensor_health_and_calibration(&mut self, now_us: u64) {
        update_sensor_health(SensorHealthCtx {
            now_us,
            sensors: &self.processed_sensors,
            params: &self.params,
            state: &mut self.state,
            last_imu_seen: &mut self.last_imu_seen,
            imu_timeout_us: IMU_TIMEOUT_US,
        });

        let failed_arm_calibration =
            self.state.is_calibrating() && self.cal_flags.contains(CalibrationFlags::GYRO_FAILED);
        let completed_arm_calibration =
            self.state.is_calibrating() && !self.cal_flags.contains(CalibrationFlags::GYRO);

        if failed_arm_calibration {
            self.state.update(Event::CALIBRATION_FAILED, &self.params);
            self.cal_flags.remove(CalibrationFlags::GYRO_FAILED);
        } else if completed_arm_calibration {
            self.state.update(Event::CALIBRATION_COMPLETE, &self.params);
        }
    }

    pub fn run_rc_command_state_stages(&mut self) {
        let now_ms = self.board.clock_millis();
        let has_new_rc = self.processed_sensors.rc.is_some();
        if !has_new_rc && self.last_rc_command_state_ms == Some(now_ms) {
            return;
        }
        self.last_rc_command_state_ms = Some(now_ms);

        run_rc_command_state(RcCommandStateCtx {
            now_ms,
            sensors: &self.processed_sensors,
            rc: &mut self.rc,
            command: &mut self.command,
            state: &mut self.state,
            params: &self.params,
        });
        self.run_pwm_output_stage();
        self.update_board_leds(now_ms);
    }

    pub fn run_pwm_output_stage(&mut self) -> bool {
        sync_pwm_output_state(PwmSyncCtx {
            board: &mut self.board,
            pwm: &mut self.pwm,
            output: &mut self.pwm_output,
            state: &self.state,
        })
        .unwrap_or(false)
    }

    pub fn run_control_stages_if_new_imu(&mut self) -> bool {
        self.run_control_and_mixing_stage_if_new_imu()
    }

    fn control_update_due_at(&self, now_us: u64) -> bool {
        let rate_hz = self.control_loop_rates.control_hz;
        if rate_hz == 0 {
            return false;
        }

        let interval_us = 1_000_000_u64 / rate_hz as u64;
        now_us.saturating_sub(self.last_control_update_us) >= interval_us
    }

    fn consume_control_update_deadline(&mut self, now_us: u64) {
        let rate_hz = self.control_loop_rates.control_hz;
        if rate_hz == 0 {
            return;
        }

        let interval_us = 1_000_000_u64 / rate_hz as u64;
        #[cfg(feature = "scope-timing-pins")]
        {
            self.board.set_test_pin_1(true);
            for _ in 0..8 {
                core::hint::spin_loop();
            }
            self.board.set_test_pin_1(false);
        }

        let elapsed_intervals = now_us.saturating_sub(self.last_control_update_us) / interval_us;
        self.last_control_update_us = self
            .last_control_update_us
            .saturating_add(elapsed_intervals.saturating_mul(interval_us));
    }

    fn control_update_can_run_at(&self, now_us: u64) -> bool {
        self.control_update_due_at(now_us) && self.control_imu_accumulator.has_samples()
    }

    fn realtime_service_has_control_slack(&self, now_us: u64) -> bool {
        let rate_hz = self.control_loop_rates.control_hz;
        if rate_hz == 0 {
            return true;
        }

        let interval_us = 1_000_000_u64 / rate_hz as u64;
        let elapsed_us = now_us.saturating_sub(self.last_control_update_us);
        elapsed_us
            .checked_add(REALTIME_SERVICE_MIN_CONTROL_SLACK_US)
            .is_some_and(|required_us| required_us < interval_us)
    }

    fn run_control_and_mixing_stage_if_control_due(&mut self, now_us: u64) -> bool {
        if self.control_loop_rates.control_hz != 0 {
            if !self.control_update_can_run_at(now_us) {
                return false;
            }
            self.consume_control_update_deadline(now_us);
            return self.run_control_and_mixing_stage_with_accumulated_imu();
        }
        if self.processed_sensors.imu.is_none() {
            return false;
        }
        self.run_control_and_mixing_stage_if_new_imu()
    }

    fn run_control_and_mixing_stage_if_control_due_measured(
        &mut self,
        now_us: u64,
        timing: &mut ControlPipelineTiming,
    ) -> bool {
        if self.control_loop_rates.control_hz != 0 {
            if !self.control_update_can_run_at(now_us) {
                return false;
            }
            self.consume_control_update_deadline(now_us);
            return self.run_control_and_mixing_stage_with_accumulated_imu_measured(timing);
        }
        if self.processed_sensors.imu.is_none() {
            return false;
        }
        self.run_control_and_mixing_stage_if_new_imu_measured(timing)
    }

    fn run_control_and_mixing_stage_with_accumulated_imu(&mut self) -> bool {
        let Some(averaged_imu) = self.control_imu_accumulator.take_average() else {
            return false;
        };
        let latest_imu = self.processed_sensors.imu;
        self.processed_sensors.imu = Some(averaged_imu);
        let ran_control = self.run_control_and_mixing_stage_if_new_imu();
        self.processed_sensors.imu = latest_imu;
        ran_control
    }

    fn run_control_and_mixing_stage_with_accumulated_imu_measured(
        &mut self,
        timing: &mut ControlPipelineTiming,
    ) -> bool {
        let Some(averaged_imu) = self.control_imu_accumulator.take_average() else {
            return false;
        };
        let latest_imu = self.processed_sensors.imu;
        self.processed_sensors.imu = Some(averaged_imu);
        let ran_control = self.run_control_and_mixing_stage_if_new_imu_measured(timing);
        self.processed_sensors.imu = latest_imu;
        ran_control
    }

    pub fn run_control_and_mixing_stage_if_new_imu(&mut self) -> bool {
        run_control_pipeline_if_new_imu(ControlPipelineCtx {
            board: &mut self.board,
            params: &self.params,
            sensors: &self.processed_sensors,
            external_attitude: &mut self.external_attitude,
            aux_commands: &self.aux_commands,
            command: &self.command,
            state: &mut self.state,
            estimator: &mut self.estimator,
            controller: &mut self.controller,
            mixer: &mut self.mixer,
            control_pipeline: &mut self.control_pipeline,
            pwm_output: &self.pwm_output,
            pwm: &mut self.pwm,
            timing: None,
        })
    }

    fn run_control_and_mixing_stage_if_new_imu_measured(
        &mut self,
        timing: &mut ControlPipelineTiming,
    ) -> bool {
        run_control_pipeline_if_new_imu(ControlPipelineCtx {
            board: &mut self.board,
            params: &self.params,
            sensors: &self.processed_sensors,
            external_attitude: &mut self.external_attitude,
            aux_commands: &self.aux_commands,
            command: &self.command,
            state: &mut self.state,
            estimator: &mut self.estimator,
            controller: &mut self.controller,
            mixer: &mut self.mixer,
            control_pipeline: &mut self.control_pipeline,
            pwm_output: &self.pwm_output,
            pwm: &mut self.pwm,
            timing: Some(timing),
        })
    }

    pub fn run_telemetry_stage(&mut self) {
        let now_us = self.board.clock_micros();
        if !self
            .comm
            .named_telemetry_due(now_us, &self.processed_sensors)
        {
            return;
        }

        let sensor_error_count = self.board.sensors_errors_count();
        self.comm.send_named_telemetry_streams(
            &mut self.board,
            now_us,
            &self.state,
            &self.command,
            &self.params,
            &self.control_pipeline.latest_estimator_state,
            &self.processed_sensors,
            &self.control_pipeline.latest_pwm_outputs,
            sensor_error_count,
            self.control_pipeline.latest_loop_time_us,
        );
    }

    fn run_realtime_telemetry_stage_budgeted(&mut self, max_streams: usize) {
        let mut sent = 0;
        while sent < max_streams && self.send_realtime_telemetry_stream() {
            sent += 1;
        }
    }

    fn send_realtime_telemetry_stream(&mut self) -> bool {
        let now_us = self.board.clock_micros();
        if !self
            .comm
            .named_telemetry_due(now_us, &self.processed_sensors)
        {
            return false;
        }

        let sensor_error_count = self.board.sensors_errors_count();
        self.comm.send_one_named_telemetry_stream(
            &mut self.board,
            now_us,
            &self.state,
            &self.command,
            &self.params,
            &self.control_pipeline.latest_estimator_state,
            &self.processed_sensors,
            &self.control_pipeline.latest_pwm_outputs,
            sensor_error_count,
            self.control_pipeline.latest_loop_time_us,
        )
    }

    #[cfg(feature = "timing-diagnostics")]
    fn record_timing_diagnostics(&mut self, stats: WorldRunStats) {
        let now_us = self.board.clock_micros();
        self.timing_diagnostics.record(stats);
        if !self.timing_diagnostics.due(now_us) {
            return;
        }

        for (index, label) in [b'I', b'R', b'S', b'U', b'C'].iter().copied().enumerate() {
            let bucket = self.timing_diagnostics.buckets[index];
            if bucket.count == 0 {
                continue;
            }
            let mut text = String::<50>::new();
            let _ = write!(
                text,
                "PERF {} n{} p{} m{} s{} k{} t{} x{}",
                label as char,
                bucket.count,
                TimingBucket::avg(bucket.total_us_sum, bucket.count),
                TimingBucket::avg(bucket.comm_us_sum, bucket.count),
                TimingBucket::avg(bucket.sensor_us_sum, bucket.count),
                TimingBucket::avg(bucket.control_us_sum, bucket.count),
                TimingBucket::avg(bucket.telemetry_us_sum, bucket.count),
                bucket.total_us_max,
            );

            let mut bytes = [0_u8; 50];
            let payload = text.as_bytes();
            bytes[..payload.len()].copy_from_slice(payload);
            self.comm_events.responses.push_or_log(
                CommResponse::Statustext(StatustextMsg {
                    severity: Severity::Debug,
                    text: bytes,
                }),
                "timing diagnostics",
            );

            self.push_timing_diagnostic_text(format_timing_control_detail(label, bucket));
            self.push_timing_diagnostic_text(format_timing_sensor_detail(label, bucket));
            self.push_timing_diagnostic_text(format_timing_board_detail(label, bucket));
        }

        while let Some(text) = self.board.board_diagnostic_text() {
            self.comm_events.responses.push_or_log(
                CommResponse::Statustext(StatustextMsg {
                    severity: Severity::Debug,
                    text,
                }),
                "board diagnostics",
            );
        }

        self.timing_diagnostics.reset(now_us);
    }

    #[cfg(feature = "timing-diagnostics")]
    fn push_timing_diagnostic_text(&mut self, text: String<50>) {
        let mut bytes = [0_u8; 50];
        let payload = text.as_bytes();
        bytes[..payload.len()].copy_from_slice(payload);
        self.comm_events.responses.push_or_log(
            CommResponse::Statustext(StatustextMsg {
                severity: Severity::Debug,
                text: bytes,
            }),
            "timing diagnostics",
        );
    }

    fn update_board_leds(&mut self, now_ms: u32) {
        if self.command.get_rc_override() != 0 {
            self.board.led0_on();
        } else {
            self.board.led0_off();
        }

        if self.state.is_in_failsafe() {
            if (now_ms / 100) % 2 == 0 {
                self.board.led1_on();
            } else {
                self.board.led1_off();
            }
        } else if self.state.is_in_error_state() {
            if (now_ms / 500) % 2 == 0 {
                self.board.led1_on();
            } else {
                self.board.led1_off();
            }
        } else if self.state.is_armed() {
            self.board.led1_on();
        } else {
            self.board.led1_off();
        }
    }
}

#[cfg(feature = "timing-diagnostics")]
fn elapsed_u16(start_us: u64, end_us: u64) -> u16 {
    end_us.saturating_sub(start_us).min(u16::MAX as u64) as u16
}

#[cfg(feature = "timing-diagnostics")]
fn format_timing_control_detail(label: u8, bucket: TimingBucket) -> String<50> {
    let mut text = String::<50>::new();
    let _ = write!(
        text,
        "PERC {} n{} e{} c{} m{} w{}",
        label as char,
        bucket.count,
        TimingBucket::avg(bucket.estimator_us_sum, bucket.count),
        TimingBucket::avg(bucket.controller_us_sum, bucket.count),
        TimingBucket::avg(bucket.mixer_us_sum, bucket.count),
        TimingBucket::avg(bucket.pwm_us_sum, bucket.count),
    );
    text
}

#[cfg(feature = "timing-diagnostics")]
fn format_timing_sensor_detail(label: u8, bucket: TimingBucket) -> String<50> {
    let mut text = String::<50>::new();
    let _ = write!(
        text,
        "PERS {} n{} u{} r{} h{} l{}",
        label as char,
        bucket.count,
        TimingBucket::avg(bucket.sensor_update_us_sum, bucket.count),
        TimingBucket::avg(bucket.sensor_process_us_sum, bucket.count),
        TimingBucket::avg(bucket.sensor_health_us_sum, bucket.count),
        TimingBucket::avg(bucket.log_response_us_sum, bucket.count),
    );
    text
}

#[cfg(feature = "timing-diagnostics")]
fn format_timing_board_detail(label: u8, bucket: TimingBucket) -> String<50> {
    let mut text = String::<50>::new();
    let _ = write!(
        text,
        "PERT {} n{} a{} q{} f{} b{}",
        label as char,
        bucket.count,
        TimingBucket::avg(bucket.rc_us_sum, bucket.count),
        TimingBucket::avg(bucket.telemetry_enqueue_us_sum, bucket.count),
        TimingBucket::avg(bucket.tx_flush_us_sum, bucket.count),
        TimingBucket::avg(bucket.board_service_us_sum, bucket.count),
    );
    text
}

fn raw_sensor_present<R: FlightFloat>(sensors: &SensorBus<R>) -> bool {
    sensors.imu.is_some()
        || sensors.mag.is_some()
        || sensors.baro.is_some()
        || sensors.pitot.is_some()
        || sensors.range.is_some()
        || sensors.gnss.is_some()
        || sensors.battery.is_some()
        || sensors.rc.is_some()
        || sensors.attitude.is_some()
}

#[cfg(feature = "timing-diagnostics")]
fn timing_class_index(stats: WorldRunStats) -> usize {
    if stats.ran_control {
        4
    } else if stats.had_imu {
        3
    } else if stats.had_sensor {
        2
    } else if stats.had_rx {
        1
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        comm::messages::{
            enums::{
                OffboardControlIgnore, OffboardControlMode, ParamIdentifier, RosflightAuxCmdType,
                RosflightCmd, RosflightCmdResponse,
            },
            messages::{
                ExternalAttitudeMsg, HeartbeatMsg, OffboardControlMsg, ParamRequestListMsg,
                ParamRequestReadMsg, ParamSetMsg, RosflightAuxCmdMsg, RosflightCmdMsg,
            },
        },
        estimator::AttitudeEstimate,
        packets::{ImuPacket, RC_PACKET_CHANNELS, RcPacket, RosflightPacketHeader},
        params::{ParamId, ParamValue},
        pwm::{PwmDriver, PwmError},
        state_machine::ErrorFlag,
        test_support::{RecordingCommLink, TestBoard},
        vehicle::quadrotor,
    };

    #[derive(Default)]
    struct SensorStageBoard {
        current_time_us: u64,
        imu: Option<ImuPacket<f64>>,
        rc: Option<RcPacket>,
        update_count: usize,
        serial_flush_count: usize,
        deferred_board_action_count: usize,
        rx_pending: bool,
    }

    impl BoardIo for SensorStageBoard {
        fn update_sensor_bus<R: crate::math::FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
            sensors.clear();
            self.update_count += 1;
            if let Some(imu) = self.imu.take() {
                sensors.imu = Some(Ok(imu.cast()));
            }
            if let Some(rc) = self.rc.take() {
                sensors.rc = Some(Ok(rc));
            }
        }

        fn imu_pending(&self) -> bool {
            self.imu.is_some()
        }

        fn update_imu_sensor<R: crate::math::FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
            sensors.clear();
            self.update_count += 1;
            if let Some(imu) = self.imu.take() {
                sensors.imu = Some(Ok(imu.cast()));
            }
        }

        fn update_service_sensor_bus<R: crate::math::FlightFloat>(
            &mut self,
            sensors: &mut SensorBus<R>,
        ) {
            sensors.clear();
            self.update_count += 1;
            if let Some(rc) = self.rc.take() {
                sensors.rc = Some(Ok(rc));
            }
        }

        fn serial_rx_read(
            &mut self,
            _buf: &mut [u8],
        ) -> Option<Result<usize, crate::errors::TelemError>> {
            None
        }

        fn serial_tx_write(
            &mut self,
            bytes: &[u8],
        ) -> Option<Result<usize, crate::errors::TelemError>> {
            Some(Ok(bytes.len()))
        }

        fn serial_rx_pending(&self) -> bool {
            self.rx_pending
        }

        fn clock_millis(&self) -> u32 {
            (self.current_time_us / 1000) as u32
        }

        fn clock_micros(&self) -> u64 {
            self.current_time_us
        }

        fn serial_flush(&mut self) {
            self.serial_flush_count += 1;
        }

        fn run_deferred_board_actions(&mut self) {
            self.deferred_board_action_count += 1;
        }
    }

    #[derive(Default)]
    struct SensorStageCommLink {
        baro_count: usize,
    }

    impl CommInterface<SensorStageBoard> for SensorStageCommLink {
        fn send_heartbeat(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: HeartbeatMsg,
        ) -> bool {
            true
        }

        fn send_named_value(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::ParamValueMsg,
        ) {
        }

        fn send_status(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::RosflightStatusMsg,
        ) {
        }

        fn send_timesync(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::TimesyncMsg,
        ) -> bool {
            true
        }

        fn send_version(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::RosflightVersionMsg,
        ) {
        }

        fn send_output_raw(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::RosflightOutputRawMsg,
        ) {
        }

        fn send_attitude(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::AttitudeQuaternionMsg,
        ) {
        }

        fn send_baro(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::SmallBaroMsg,
        ) {
            self.baro_count += 1;
        }

        fn send_diff_pressure(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::DiffPressureMsg,
        ) {
        }

        fn send_imu(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::SmallImuMsg,
        ) {
        }

        fn send_mag(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::SmallMagMsg,
        ) {
        }

        fn send_rc_raw(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::RcChannelsMsg,
        ) {
        }

        fn send_range(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::SmallRangeMsg,
        ) {
        }

        fn send_gnss(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::RosflightGnssMsg,
        ) {
        }

        fn send_cmd_ack(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::RosflightCmdAckMsg,
        ) {
        }

        fn send_rc_channels(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::RcChannelsMsg,
        ) {
        }

        fn send_battery_status(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::BatteryStatusMsg,
        ) {
        }

        fn send_statustext(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::StatustextMsg,
        ) {
        }

        fn send_hard_error(
            &mut self,
            _board: &mut SensorStageBoard,
            _system_id: u8,
            _msg: crate::comm::messages::messages::RosflightHardErrorMsg,
        ) {
        }

        fn handle_incoming_messages(
            &mut self,
            _board: &mut SensorStageBoard,
            _msgs: &mut crate::comm::messages::Messages,
        ) {
        }
    }

    pub struct TestPwm {
        enabled: bool,
        enable_all_count: usize,
        disable_all_count: usize,
        flush_count: usize,
        send_count: usize,
        configure_count: usize,
        last_commands: [f64; 14],
        last_command_len: usize,
        last_rates: [f64; 10],
        last_rate_len: usize,
    }

    impl TestPwm {
        fn new() -> Self {
            Self {
                enabled: false,
                enable_all_count: 0,
                disable_all_count: 0,
                flush_count: 0,
                send_count: 0,
                configure_count: 0,
                last_commands: [0.0; 14],
                last_command_len: 0,
                last_rates: [0.0; 10],
                last_rate_len: 0,
            }
        }
    }

    impl PwmDriver<f64> for TestPwm {
        fn len(&self) -> usize {
            0
        }

        fn is_enabled(&self) -> bool {
            self.enabled
        }

        fn enable(&mut self, _channel: usize) -> Result<(), PwmError> {
            self.enabled = true;
            Ok(())
        }

        fn disable(&mut self, _channel: usize) -> Result<(), PwmError> {
            self.enabled = false;
            Ok(())
        }

        fn enable_all(&mut self) -> Result<(), PwmError> {
            self.enabled = true;
            self.enable_all_count += 1;
            Ok(())
        }

        fn disable_all(&mut self) {
            self.enabled = false;
            self.disable_all_count += 1;
        }

        fn set_duty_cycle(&mut self, _channel: usize, _duty: u16) -> Result<(), PwmError> {
            Ok(())
        }

        fn flush<Board: BoardIo>(&mut self, _board: &mut Board) {
            self.flush_count += 1;
        }

        fn configure_output_rates(&mut self, rates_hz: &[f64]) -> Result<(), PwmError> {
            self.configure_count += 1;
            self.last_rate_len = rates_hz.len().min(self.last_rates.len());
            self.last_rates[..self.last_rate_len].copy_from_slice(&rates_hz[..self.last_rate_len]);
            Ok(())
        }

        fn send_commands<Board: BoardIo>(
            &mut self,
            _board: &mut Board,
            commands: &[f64],
        ) -> Result<(), PwmError> {
            self.send_count += 1;
            self.last_command_len = commands.len().min(self.last_commands.len());
            self.last_commands[..self.last_command_len]
                .copy_from_slice(&commands[..self.last_command_len]);
            Ok(())
        }
    }

    type TestWorld = World<
        TestBoard,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        RecordingCommLink,
        TestPwm,
        f64,
    >;

    fn test_world_with_params(params: Params) -> TestWorld {
        let mixer = quadrotor::mixer::<f64>(&params);

        TestWorld::init(
            TestBoard::default(),
            params,
            RecordingCommLink::new(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        )
    }

    fn test_world() -> TestWorld {
        test_world_with_params(Params::new())
    }

    #[test]
    fn world_init_reconciles_reflected_mixer_params_from_persisted_mixer_choice() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(10));
        params.set_by_id(ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0, ParamValue::Int(2));
        params.set_by_id(
            ParamId::PARAM_PRIMARY_MIXER_3_0,
            ParamValue::Float(-25303.715),
        );

        let world = test_world_with_params(params);

        assert_eq!(
            world
                .params
                .get_by_id(ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0),
            ParamValue::Int(1)
        );
        assert_eq!(
            world.params.get_by_id(ParamId::PARAM_PRIMARY_MIXER_3_0),
            ParamValue::Float(1.0)
        );
    }

    fn armed_test_world_with_params(params: Params) -> TestWorld {
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state.update_arming_safety(true, true);
        state.update(Event::REQUEST_ARM, &params);
        let mixer = quadrotor::mixer(&params);

        TestWorld::init(
            TestBoard::default(),
            params,
            RecordingCommLink::new(),
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        )
    }

    #[test]
    fn world_scheduler_runs_deferred_param_pipeline() {
        let mut world = test_world();

        crate::comm::messages::Store::store(
            &mut world.comm.msgs,
            ParamSetMsg {
                target_system: 1,
                target_component: 1,
                param_id: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
                param_value: ParamValue::Int(42),
            },
        );

        assert!(world.run_once());

        assert_eq!(
            world.params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );
        assert_eq!(world.comm.sysid, 42);
    }

    #[test]
    fn world_scheduler_streams_param_request_list_through_param_system() {
        let mut world = test_world();

        world.comm.msgs.param_request_list = Some(ParamRequestListMsg {
            target_system: 1,
            target_component: 1,
        });

        world.run_comm_param_sensor_stages();

        assert!(world.param_events.list_requests.is_empty());
        assert!(world.param_list_state.is_active());
        assert_eq!(world.comm.comm_link().sent_param_value_count, 1);
        let first = world.comm.comm_link().sent_param_values[0].unwrap();
        assert_eq!(first.param_index, ParamId::PARAM_BAUD_RATE as u16);
        assert_eq!(first.param_value, ParamValue::Int(921600));

        world.run_comm_param_sensor_stages();

        assert_eq!(world.comm.comm_link().sent_param_value_count, 2);
        let second = world.comm.comm_link().sent_param_values[1].unwrap();
        assert_eq!(second.param_index, ParamId::PARAM_SERIAL_DEVICE as u16);
    }

    #[test]
    fn world_scheduler_answers_param_request_read_through_param_system() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let mut world = test_world_with_params(params);

        world.comm.msgs.param_request_read = Some(ParamRequestReadMsg {
            target_system: 1,
            target_component: 1,
            param_identifier: ParamIdentifier::ID(*b"SYS_ID\0\0\0\0\0\0\0\0\0\0"),
        });

        world.run_comm_param_sensor_stages();

        assert!(world.param_events.read_requests.is_empty());
        assert_eq!(world.comm.comm_link().sent_param_value_count, 1);
        let response = world.comm.comm_link().sent_param_values[0].unwrap();
        assert_eq!(response.param_index, ParamId::PARAM_SYSTEM_ID as u16);
        assert_eq!(response.param_value, ParamValue::Int(42));
    }

    #[test]
    fn world_sensor_stage_ingests_board_sensor_bus_without_hlist_fixture() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));
        let board = SensorStageBoard {
            current_time_us: 25_000,
            imu: Some(ImuPacket {
                header: RosflightPacketHeader {
                    timestamp: 25_000,
                    status: 0,
                },
                accel: [0.0, 0.0, -9.80665],
                gyro: [0.1, 0.2, 0.3],
                temperature: 25.0,
                seq: 7,
            }),
            rc: Some(RcPacket {
                header: RosflightPacketHeader {
                    timestamp: 24_000,
                    status: 0,
                },
                n_chan: 1,
                chan: [0.5; RC_PACKET_CHANNELS],
                lol: false,
            }),
            update_count: 0,
            serial_flush_count: 0,
            deferred_board_action_count: 0,
            rx_pending: false,
        };
        let state = StateManager::new();
        let mixer = quadrotor::mixer::<f64>(&params);

        let mut world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            board,
            params,
            SensorStageCommLink::default(),
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        world.run_comm_param_sensor_stages();

        assert_eq!(world.board.update_count, 1);
        assert!(world.raw_sensors.imu.is_none());
        assert!(world.raw_sensors.rc.is_none());
        assert_eq!(
            world.processed_sensors.imu.unwrap().header.timestamp,
            25_000
        );
        assert_eq!(world.processed_sensors.rc.unwrap().chan[0], 0.5);
        assert!(
            !world
                .state
                .get_errors()
                .contains(crate::state_machine::ErrorFlag::IMU_NOT_RESPONDING)
        );

        world.run_rc_command_state_stages();

        assert!(
            !world
                .state
                .get_errors()
                .contains(crate::state_machine::ErrorFlag::RC_LOST)
        );
    }

    #[test]
    fn world_fast_tick_runs_sensor_rc_control_without_service_output() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));
        let mixer = quadrotor::mixer::<f64>(&params);
        let mut world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            SensorStageBoard {
                current_time_us: 10_000,
                imu: Some(ImuPacket {
                    header: RosflightPacketHeader {
                        timestamp: 10_000,
                        status: 0,
                    },
                    accel: [0.0, 0.0, -9.80665],
                    gyro: [0.0, 0.0, 0.0],
                    temperature: 25.0,
                    seq: 1,
                }),
                rc: Some(RcPacket {
                    header: RosflightPacketHeader {
                        timestamp: 10_000,
                        status: 0,
                    },
                    n_chan: 1,
                    chan: [0.5; RC_PACKET_CHANNELS],
                    lol: false,
                }),
                ..Default::default()
            },
            params,
            SensorStageCommLink::default(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        assert!(!world.run_imu_control_tick());
        assert_eq!(world.board.update_count, 1);
        assert_eq!(world.board.serial_flush_count, 0);
        assert_eq!(world.board.deferred_board_action_count, 0);

        world.board.current_time_us = 12_500;
        world.board.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 12_500,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 2,
        });

        assert!(world.run_imu_control_tick());
        assert_eq!(world.board.update_count, 2);
        assert_eq!(world.board.serial_flush_count, 0);
        assert_eq!(world.board.deferred_board_action_count, 0);
    }

    #[test]
    fn world_service_step_runs_service_sensors_comm_telemetry_and_board_service() {
        let params = Params::new();
        let mixer = quadrotor::mixer::<f64>(&params);
        let mut world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            SensorStageBoard {
                current_time_us: 1_100_000,
                ..Default::default()
            },
            params,
            SensorStageCommLink::default(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );
        world.processed_sensors.baro = Some(crate::packets::BaroPacket {
            altitude: 42.0,
            pressure: 90_000.0,
            temperature: 21.0,
            ..Default::default()
        });

        world.run_service_step();

        assert_eq!(world.board.update_count, 1);
        assert_eq!(world.board.serial_flush_count, 1);
        assert_eq!(world.board.deferred_board_action_count, 1);
        assert_eq!(world.comm.comm_link().baro_count, 1);
    }

    #[test]
    fn service_sensor_stage_preserves_previous_imu_for_health_when_service_poll_omits_imu() {
        let params = Params::new();
        let mixer = quadrotor::mixer::<f64>(&params);
        let mut world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            SensorStageBoard {
                current_time_us: IMU_TIMEOUT_US + 1,
                ..Default::default()
            },
            params,
            SensorStageCommLink::default(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );
        world.processed_sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });

        world.run_service_step();

        assert!(world.processed_sensors.imu.is_some());
        assert!(
            !world
                .state
                .get_errors()
                .contains(crate::state_machine::ErrorFlag::IMU_NOT_RESPONDING)
        );
    }

    #[test]
    fn realtime_scheduler_prefers_imu_and_idles_until_service_deadline() {
        let params = Params::new();
        let mixer = quadrotor::mixer::<f64>(&params);
        let mut world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            SensorStageBoard {
                current_time_us: 10_000,
                ..Default::default()
            },
            params,
            SensorStageCommLink::default(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        assert_eq!(
            world.realtime_scheduler_step(),
            RealtimeSchedulerStep::Service
        );
        world.run_service_step_with_deferral(1_000);
        assert_eq!(world.realtime_scheduler_step(), RealtimeSchedulerStep::Idle);

        world.control_pipeline.set_last_imu_time(10_000);
        world.board.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 10_500,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });
        assert_eq!(
            world.realtime_scheduler_step(),
            RealtimeSchedulerStep::ImuControl
        );

        world.board.current_time_us = 11_001;
        assert_eq!(
            world.realtime_scheduler_step(),
            RealtimeSchedulerStep::ImuControl
        );
        let _ = world.run_imu_control_tick();
        assert_eq!(
            world.realtime_scheduler_step(),
            RealtimeSchedulerStep::Service
        );
    }

    #[test]
    fn fixed_control_rate_ingests_imu_without_running_every_sample() {
        let params = Params::new();
        let mixer = quadrotor::mixer::<f64>(&params);
        let mut world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            SensorStageBoard::default(),
            params,
            SensorStageCommLink::default(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );
        world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(2_000));

        world.board.current_time_us = 500;
        world.board.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 500,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });
        assert!(!world.run_imu_control_tick());

        world.board.current_time_us = 780;
        world.board.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 780,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 2,
        });
        assert!(!world.run_imu_control_tick());

        world.board.current_time_us = 1_000;
        world.board.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 3,
        });
        assert!(world.run_imu_control_tick());
        assert!(world.control_pipeline.latest_actuator_commands.is_some());
    }

    #[test]
    fn fixed_control_rate_blocks_service_inside_control_deadline_guard() {
        let params = Params::new();
        let mixer = quadrotor::mixer::<f64>(&params);
        let mut world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            SensorStageBoard {
                current_time_us: 10_299,
                ..Default::default()
            },
            params,
            SensorStageCommLink::default(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );
        world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(2_000));
        world.last_control_update_us = 10_000;
        world.last_realtime_control_us = 10_200;
        world.last_realtime_service_control_us = 0;
        world.next_realtime_service_us = 0;
        world.processed_sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 10_000,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });

        assert_eq!(
            world.realtime_scheduler_step(),
            RealtimeSchedulerStep::Service
        );

        world.board.current_time_us = 10_300;
        assert_eq!(world.realtime_scheduler_step(), RealtimeSchedulerStep::Idle);
    }

    #[test]
    fn realtime_service_splits_sensor_rc_and_telemetry_micro_phases() {
        let params = Params::new();
        let mixer = quadrotor::mixer::<f64>(&params);
        let mut world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            SensorStageBoard {
                current_time_us: 1_100_000,
                rc: Some(RcPacket {
                    header: RosflightPacketHeader {
                        timestamp: 1_100_000,
                        status: 0,
                    },
                    n_chan: 1,
                    chan: [0.5; RC_PACKET_CHANNELS],
                    lol: false,
                }),
                ..Default::default()
            },
            params,
            SensorStageCommLink::default(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );
        world.processed_sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_100_000,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });

        world.run_service_step_with_deferral(0);
        assert_eq!(world.board.update_count, 0);

        world.run_service_step_with_deferral(0);
        assert_eq!(world.board.update_count, 1);

        world.run_service_step_with_deferral(0);
        assert_eq!(world.processed_sensors.rc.map(|rc| rc.n_chan), Some(1));

        world.run_service_step_with_deferral(0);

        world.run_service_step_with_deferral(0);
        world.run_service_step_with_deferral(0);
        world.run_service_step_with_deferral(0);

        world.run_service_step_with_deferral(0);
        assert_eq!(world.board.serial_flush_count, 1);

        world.run_service_step_with_deferral(0);
        assert_eq!(world.board.deferred_board_action_count, 1);
    }

    #[test]
    fn fixed_control_rate_can_run_between_imu_edges() {
        let params = Params::new();
        let mixer = quadrotor::mixer::<f64>(&params);
        let mut world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            SensorStageBoard::default(),
            params,
            SensorStageCommLink::default(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );
        world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(2_000));

        world.board.current_time_us = 280;
        world.board.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 280,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });
        assert_eq!(
            world.realtime_scheduler_step(),
            RealtimeSchedulerStep::ImuControl
        );
        assert!(!world.run_imu_control_tick());

        world.board.current_time_us = 500;
        assert_eq!(
            world.realtime_scheduler_step(),
            RealtimeSchedulerStep::ControlUpdate
        );
        assert!(!world.run_control_update_tick());

        world.board.current_time_us = 840;
        world.board.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 840,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 2,
        });
        assert!(!world.run_imu_control_tick());

        world.board.current_time_us = 1_000;
        assert_eq!(
            world.realtime_scheduler_step(),
            RealtimeSchedulerStep::ControlUpdate
        );
        assert!(world.run_control_update_tick());
        assert!(world.control_pipeline.latest_actuator_commands.is_some());
    }

    #[test]
    fn fixed_control_rate_does_not_rerun_stale_imu_sample() {
        let params = Params::new();
        let mixer = quadrotor::mixer::<f64>(&params);
        let mut world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            SensorStageBoard::default(),
            params,
            SensorStageCommLink::default(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );
        world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(400));

        world.board.current_time_us = 1_000;
        world.board.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });
        assert!(!world.run_imu_control_tick());

        world.board.current_time_us = 2_500;
        assert!(!world.run_control_update_tick());

        world.board.current_time_us = 5_000;
        assert!(!world.run_control_update_tick());
    }

    #[test]
    fn fixed_control_rate_does_not_consume_deadline_without_accumulated_imu() {
        let params = Params::new();
        let mixer = quadrotor::mixer::<f64>(&params);
        let mut world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            SensorStageBoard::default(),
            params,
            SensorStageCommLink::default(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );
        world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(2_000));

        world.board.current_time_us = 1_000;
        assert!(world.control_update_due_at(1_000));
        assert!(!world.run_control_update_tick());
        assert_eq!(world.last_control_update_us, 0);
    }

    #[test]
    fn fixed_control_rate_late_wake_skips_missed_intervals_without_bursting() {
        let params = Params::new();
        let mixer = quadrotor::mixer::<f64>(&params);
        let mut world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            SensorStageBoard::default(),
            params,
            SensorStageCommLink::default(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );
        world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(2_000));
        world.control_imu_accumulator.push(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_600,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });

        world.board.current_time_us = 1_600;
        assert!(world.control_update_can_run_at(1_600));
        world.consume_control_update_deadline(1_600);
        assert_eq!(world.last_control_update_us, 1_500);

        world.control_imu_accumulator.push(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_601,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 2,
        });
        world.board.current_time_us = 1_601;
        assert!(!world.control_update_can_run_at(1_601));
        assert_eq!(world.last_control_update_us, 1_500);
    }

    #[test]
    fn imu_accumulator_averages_samples_for_control_deadline() {
        let mut accumulator = ImuSampleAccumulator::<f64>::default();
        accumulator.push(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 100,
                status: 1,
            },
            accel: [1.0, 2.0, 3.0],
            gyro: [4.0, 5.0, 6.0],
            temperature: 20.0,
            seq: 7,
        });
        accumulator.push(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 200,
                status: 2,
            },
            accel: [3.0, 4.0, 5.0],
            gyro: [6.0, 7.0, 8.0],
            temperature: 22.0,
            seq: 8,
        });

        let averaged = accumulator.take_average().expect("averaged sample");

        assert_eq!(averaged.header.timestamp, 200);
        assert_eq!(averaged.header.status, 2);
        assert_eq!(averaged.seq, 8);
        assert_eq!(averaged.accel, [2.0, 3.0, 4.0]);
        assert_eq!(averaged.gyro, [5.0, 6.0, 7.0]);
        assert_eq!(averaged.temperature, 21.0);
        assert!(accumulator.take_average().is_none());
    }

    #[cfg(feature = "timing-diagnostics")]
    #[test]
    fn world_run_stats_classifies_idle_rx_sensor_and_control_passes() {
        let mut idle_world = test_world();
        let idle = idle_world.run_once_measured();
        assert!(!idle.had_rx);
        assert!(!idle.had_sensor);
        assert!(!idle.had_imu);
        assert!(!idle.ran_control);

        let params = Params::new();
        let mixer = quadrotor::mixer::<f64>(&params);
        let mut rx_world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            SensorStageBoard {
                rx_pending: true,
                ..Default::default()
            },
            params,
            SensorStageCommLink::default(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );
        let rx_only = rx_world.run_once_measured();
        assert!(rx_only.had_rx);
        assert!(!rx_only.had_sensor);
        assert!(!rx_only.ran_control);

        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));
        let mixer = quadrotor::mixer::<f64>(&params);
        let mut sensor_world = World::<
            SensorStageBoard,
            quadrotor::Estimator<f64>,
            quadrotor::Controller<f64>,
            quadrotor::Mixer<f64>,
            SensorStageCommLink,
            TestPwm,
            f64,
        >::init(
            SensorStageBoard {
                current_time_us: 20_000,
                rc: Some(RcPacket {
                    header: RosflightPacketHeader {
                        timestamp: 20_000,
                        status: 0,
                    },
                    n_chan: 1,
                    chan: [0.5; RC_PACKET_CHANNELS],
                    lol: false,
                }),
                ..Default::default()
            },
            params,
            SensorStageCommLink::default(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );
        let sensor_only = sensor_world.run_once_measured();
        assert!(sensor_only.had_sensor);
        assert!(!sensor_only.had_imu);
        assert!(!sensor_only.ran_control);

        sensor_world.board.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 22_000,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.1, 0.2, 0.3],
            temperature: 25.0,
            seq: 1,
        });
        sensor_world.board.current_time_us = 22_000;
        let first_imu = sensor_world.run_once_measured();
        assert!(first_imu.had_imu);
        assert!(!first_imu.ran_control);

        sensor_world.board.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 24_000,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.1, 0.2, 0.3],
            temperature: 25.0,
            seq: 2,
        });
        sensor_world.board.current_time_us = 24_000;
        let control = sensor_world.run_once_measured();
        assert!(control.had_imu);
        assert!(control.ran_control);
    }

    #[test]
    fn world_scheduler_processes_named_rc_packet() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));
        let mut world = test_world_with_params(params);

        world.processed_sensors.rc = Some(crate::packets::RcPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: 0,
                status: 0,
            },
            n_chan: 1,
            chan: [0.5; crate::packets::RC_PACKET_CHANNELS],
            lol: false,
        });

        world.run_rc_command_state_stages();

        assert!(
            !world
                .state
                .get_errors()
                .contains(crate::state_machine::ErrorFlag::RC_LOST)
        );
    }

    #[test]
    fn world_control_stage_runs_once_per_imu_timestamp() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_FAILSAFE_THROTTLE, ParamValue::Float(0.0));
        params.set_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE, ParamValue::Float(0.2));
        params.set_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED, ParamValue::Int(1));
        let mut world = test_world_with_params(params);

        world.state.update_arming_safety(true, true);
        world
            .state
            .update(crate::state_machine::Event::REQUEST_ARM, &world.params);
        assert!(world.run_pwm_output_stage());

        world.board.current_time_us = 1_100_000;
        world.external_attitude.latest = Some(ExternalAttitudeMsg {
            qw: 0.0,
            qx: 1.0,
            qy: 0.0,
            qz: 0.0,
        });
        let mut aux = RosflightAuxCmdMsg {
            type_array: [RosflightAuxCmdType::Disabled; 14],
            aux_cmd_array: [0.0; 14],
        };
        aux.type_array[4] = RosflightAuxCmdType::Servo;
        aux.aux_cmd_array[4] = -0.5;
        aux.type_array[5] = RosflightAuxCmdType::Motor;
        aux.aux_cmd_array[5] = 0.1;
        world.aux_commands.latest = Some(aux);
        world.processed_sensors.imu = Some(crate::packets::ImuPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: 1,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });

        assert!(!world.run_control_stages_if_new_imu());
        world
            .processed_sensors
            .imu
            .as_mut()
            .unwrap()
            .header
            .timestamp = 2;

        assert!(world.run_control_stages_if_new_imu());
        world.run_telemetry_stage();
        assert_eq!(world.pwm.send_count, 1);
        assert_eq!(world.comm.comm_link().heartbeat_count, 1);
        assert_eq!(world.comm.comm_link().status_count, 1);
        assert_eq!(world.comm.comm_link().imu_count, 1);
        assert_eq!(world.comm.comm_link().attitude_count, 1);
        assert_eq!(world.comm.comm_link().output_raw_count, 1);
        assert!(world.control_pipeline.latest_actuator_commands.is_some());
        assert!(world.external_attitude.latest.is_none());
        assert_eq!(
            world.control_pipeline.latest_estimator_state.q(),
            [1.0, 0.0, 0.0, 0.0]
        );
        assert_eq!(world.pwm.last_command_len, 14);
        assert_eq!(world.pwm.last_commands[4], 0.25);
        assert!((world.pwm.last_commands[5] - 0.2).abs() < 1e-6);
        let output_raw = world.comm.comm_link().last_output_raw.unwrap();
        assert_eq!(output_raw.values[4], 0.25);
        assert!((output_raw.values[5] - 0.2).abs() < 1e-6);

        assert!(world.run_control_stages_if_new_imu());
        assert_eq!(world.pwm.send_count, 2);
        assert_eq!(world.comm.comm_link().output_raw_count, 1);

        world
            .processed_sensors
            .imu
            .as_mut()
            .unwrap()
            .header
            .timestamp = 3;

        assert!(world.run_control_stages_if_new_imu());
        assert_eq!(world.pwm.send_count, 3);
        assert_eq!(world.comm.comm_link().output_raw_count, 1);
    }

    #[test]
    fn world_control_stage_flags_non_advancing_imu_time() {
        let mut world = test_world();
        world.processed_sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 10,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            ..Default::default()
        });

        assert!(!world.run_control_stages_if_new_imu());

        world.processed_sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 9,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            ..Default::default()
        });

        assert!(!world.run_control_stages_if_new_imu());
        assert!(
            world
                .state
                .get_errors()
                .contains(ErrorFlag::TIME_GOING_BACKWARDS)
        );
    }

    #[test]
    fn world_telemetry_stage_streams_non_imu_sensor_without_control_update() {
        let mut world = test_world();
        world.board.current_time_us = 1_100_000;
        world.processed_sensors.baro = Some(crate::packets::BaroPacket {
            altitude: 42.0,
            pressure: 90_000.0,
            temperature: 21.0,
            ..Default::default()
        });

        world.run_telemetry_stage();

        assert_eq!(world.comm.comm_link().baro_count, 1);
        assert_eq!(world.comm.comm_link().imu_count, 0);
        assert_eq!(world.comm.comm_link().last_baro.unwrap().altitude, 42.0);
    }

    #[test]
    fn world_telemetry_rates_match_rosflight_c_default_stream_cadence() {
        let mut world = test_world();

        for sample in 0..40 {
            world.board.current_time_us = 1_000_000 + sample * 2_500;
            world.processed_sensors.imu = Some(ImuPacket {
                header: RosflightPacketHeader {
                    timestamp: world.board.current_time_us,
                    status: 0,
                },
                accel: [0.0, 0.0, -9.80665],
                gyro: [0.0, 0.0, 0.0],
                temperature: 25.0,
                seq: sample as u32,
            });

            world.run_telemetry_stage();
        }

        assert_eq!(world.comm.comm_link().heartbeat_count, 1);
        assert_eq!(world.comm.comm_link().status_count, 1);
        assert_eq!(world.comm.comm_link().imu_count, 40);
        assert_eq!(world.comm.comm_link().attitude_count, 40);
        assert_eq!(world.comm.comm_link().output_raw_count, 5);
    }

    #[test]
    fn world_status_uses_board_error_count_and_control_loop_time() {
        let mut world = test_world();
        world.board.current_time_us = 1_100_000;
        world.board.sensor_errors_count = 7;
        world.control_pipeline.latest_loop_time_us = 123;

        world.run_telemetry_stage();

        let status = world.comm.comm_link().last_status.unwrap();
        assert_eq!(status.num_errors, 7);
        assert_eq!(status.loop_time_us, 123);
    }

    #[test]
    fn world_led_outputs_follow_rc_override_armed_error_and_failsafe_states() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));
        let mut world = test_world_with_params(params);

        world.update_board_leds(0);
        assert!(!world.board.led0_high);
        assert!(world.board.led1_high);
        world.update_board_leds(500);
        assert!(!world.board.led1_high);

        let mut channels = [0.5; RC_PACKET_CHANNELS];
        channels[0] = 0.8;
        world.processed_sensors.rc = Some(RcPacket {
            header: RosflightPacketHeader {
                timestamp: 1,
                status: 0,
            },
            n_chan: 1,
            chan: channels,
            lol: false,
        });
        world.run_rc_command_state_stages();
        world.update_board_leds(0);
        assert!(world.board.led0_high);

        world.state.update_arming_safety(true, true);
        world.state.update(Event::REQUEST_ARM, &world.params);
        world.update_board_leds(0);
        assert!(world.board.led1_high);

        world.state.update(
            Event::ERROR_OCCURRED(ErrorFlag::UNCALIBRATED_IMU),
            &world.params,
        );
        world.update_board_leds(500);
        assert!(!world.board.led1_high);

        let mut failsafe_world = armed_test_world_with_params(Params::new());
        failsafe_world.state.update(
            Event::ERROR_OCCURRED(ErrorFlag::RC_LOST),
            &failsafe_world.params,
        );
        failsafe_world.update_board_leds(100);
        assert!(!failsafe_world.board.led1_high);
        failsafe_world.update_board_leds(200);
        assert!(failsafe_world.board.led1_high);
    }

    #[test]
    fn world_replays_backup_hard_error_after_companion_heartbeat() {
        let params = Params::new();
        let mixer = quadrotor::mixer(&params);
        let board = TestBoard {
            backup_data: Some(crate::board::BackupData {
                error_code: 4,
                pc: 0x1234,
                reset_count: 2,
                do_rearm: 1,
            }),
            ..Default::default()
        };
        let mut world = TestWorld::init(
            board,
            params,
            RecordingCommLink::new(),
            StateManager::new(),
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );
        assert_eq!(world.board.backup_clear_count, 1);
        assert_eq!(world.pending_hard_error.unwrap().do_rearm, 1);
        assert!(world.state.is_armed());

        world.comm.msgs.heartbeat = Some(HeartbeatMsg {
            type_: 1,
            autopilot: 2,
            base_mode: 3,
            custom_mode: 4,
            system_status: 5,
            mavlink_version: 6,
        });
        world.run_comm_param_sensor_stages();

        assert_eq!(world.comm.comm_link().hard_error_count, 1);
        assert_eq!(world.comm.comm_link().last_hard_error.unwrap().pc, 0x1234);
    }

    #[test]
    fn world_control_stage_propagates_custom_zero_pwm_rates() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(11));
        let mut world = test_world_with_params(params);
        world.processed_sensors.imu = Some(crate::packets::ImuPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: 1,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });

        assert!(!world.run_control_stages_if_new_imu());
        world
            .processed_sensors
            .imu
            .as_mut()
            .unwrap()
            .header
            .timestamp = 2;

        assert!(world.run_control_stages_if_new_imu());

        assert_eq!(world.pwm.configure_count, 1);
        assert_eq!(world.pwm.last_rate_len, 10);
        assert_eq!(world.pwm.last_rates, [0.0; 10]);

        world
            .processed_sensors
            .imu
            .as_mut()
            .unwrap()
            .header
            .timestamp = 3;
        assert!(world.run_control_stages_if_new_imu());
        assert_eq!(world.pwm.configure_count, 1);
    }

    #[test]
    fn world_control_stage_reconfigures_pwm_rates_after_mixer_param_change() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(11));
        let mut world = test_world_with_params(params);
        world.processed_sensors.imu = Some(crate::packets::ImuPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: 1,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });

        assert!(!world.run_control_stages_if_new_imu());
        world
            .processed_sensors
            .imu
            .as_mut()
            .unwrap()
            .header
            .timestamp = 2;
        assert!(world.run_control_stages_if_new_imu());
        assert_eq!(world.pwm.configure_count, 1);

        world
            .params
            .set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(0));
        world
            .param_events
            .changes
            .push(crate::events::ParamChanged {
                id: ParamId::PARAM_PRIMARY_MIXER,
                old: ParamValue::Int(11),
                new: ParamValue::Int(0),
                param_id_bytes: [0; 16],
            })
            .unwrap();
        world.apply_param_reactions();

        world
            .processed_sensors
            .imu
            .as_mut()
            .unwrap()
            .header
            .timestamp = 3;
        assert!(world.run_control_stages_if_new_imu());
        assert_eq!(world.pwm.configure_count, 2);
        assert_eq!(world.pwm.last_rate_len, 10);
        assert_eq!(world.pwm.last_rates, [50.0; 10]);
    }

    #[test]
    fn world_sensor_health_sets_and_clears_imu_timeout() {
        let mut world = test_world();

        world.board.current_time_us = IMU_TIMEOUT_US + 1;
        world.update_sensor_health_and_calibration(world.board.clock_micros());

        assert!(
            world
                .state
                .get_errors()
                .contains(crate::state_machine::ErrorFlag::IMU_NOT_RESPONDING)
        );

        world.processed_sensors.imu = Some(crate::packets::ImuPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: IMU_TIMEOUT_US + 2,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });
        world.update_sensor_health_and_calibration(world.board.clock_micros());

        assert!(
            !world
                .state
                .get_errors()
                .contains(crate::state_machine::ErrorFlag::IMU_NOT_RESPONDING)
        );
    }

    #[test]
    fn world_sensor_health_sets_uncalibrated_imu_when_all_bias_params_are_zero() {
        let mut world = test_world();
        world.processed_sensors.imu = Some(crate::packets::ImuPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: 1,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });

        world.update_sensor_health_and_calibration(world.board.clock_micros());

        assert!(
            world
                .state
                .get_errors()
                .contains(crate::state_machine::ErrorFlag::UNCALIBRATED_IMU)
        );
    }

    #[test]
    fn world_sensor_health_clears_uncalibrated_imu_when_any_bias_param_is_nonzero() {
        let mut world = test_world();
        world.state.update(
            crate::state_machine::Event::ERROR_OCCURRED(
                crate::state_machine::ErrorFlag::UNCALIBRATED_IMU,
            ),
            &world.params,
        );
        world
            .params
            .set_by_id(ParamId::PARAM_ACC_X_BIAS, ParamValue::Float(0.01));
        world.processed_sensors.imu = Some(crate::packets::ImuPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: 1,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });

        world.update_sensor_health_and_calibration(world.board.clock_micros());

        assert!(
            !world
                .state
                .get_errors()
                .contains(crate::state_machine::ErrorFlag::UNCALIBRATED_IMU)
        );
    }

    #[test]
    fn world_sends_calibration_ack_when_calibration_starts() {
        let mut world = test_world();

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::GyroCalibration,
        });

        world.run_comm_param_sensor_stages();

        assert!(world.cal_flags.contains(CalibrationFlags::GYRO));
        assert_eq!(world.comm.comm_link().cmd_ack_count, 1);
        let ack = world.comm.comm_link().last_cmd_ack.unwrap();
        assert!(matches!(ack.command, RosflightCmd::GyroCalibration));
        assert!(matches!(
            ack.success,
            RosflightCmdResponse::RosflightCmdSuccess
        ));
    }

    #[test]
    fn world_pwm_output_stage_follows_armed_state_transitions() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_FAILSAFE_THROTTLE, ParamValue::Float(0.0));
        let mut world = test_world_with_params(params);

        assert!(!world.run_pwm_output_stage());
        assert_eq!(world.pwm.enable_all_count, 0);
        assert_eq!(world.pwm.disable_all_count, 0);

        world.state.update_arming_safety(true, true);
        world
            .state
            .update(crate::state_machine::Event::REQUEST_ARM, &world.params);

        assert!(world.run_pwm_output_stage());
        assert!(world.pwm_output.is_enabled());
        assert_eq!(world.pwm.enable_all_count, 1);

        assert!(!world.run_pwm_output_stage());
        assert_eq!(world.pwm.enable_all_count, 1);

        world
            .state
            .update(crate::state_machine::Event::REQUEST_DISARM, &world.params);

        assert!(world.run_pwm_output_stage());
        assert!(!world.pwm_output.is_enabled());
        assert_eq!(world.pwm.disable_all_count, 1);
        assert_eq!(world.pwm.flush_count, 1);
    }

    #[test]
    fn world_applies_offboard_control_command_event() {
        let mut world = armed_test_world_with_params(Params::new());

        world.comm.msgs.offboard_control = Some(OffboardControlMsg {
            mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
            ignore: OffboardControlIgnore::IGNORE_QY,
            qx: 0.1,
            qy: 0.2,
            qz: 0.3,
            fx: 0.4,
            fy: 0.5,
            fz: 0.6,
            passthrough: [0.0; 4],
        });

        world.run_comm_param_sensor_stages();

        assert!(world.command.is_offboard_active());
        assert!(world.command_events.offboard_control_requests.is_empty());
    }

    #[test]
    fn world_applies_companion_input_events() {
        let mut world = test_world();

        world.comm.msgs.heartbeat = Some(HeartbeatMsg {
            type_: 1,
            autopilot: 2,
            base_mode: 3,
            custom_mode: 4,
            system_status: 5,
            mavlink_version: 6,
        });
        let mut aux = RosflightAuxCmdMsg {
            type_array: [RosflightAuxCmdType::Disabled; 14],
            aux_cmd_array: [0.0; 14],
        };
        aux.type_array[3] = RosflightAuxCmdType::Servo;
        aux.aux_cmd_array[3] = 0.8;
        world.comm.msgs.aux_cmd = Some(aux);
        world.comm.msgs.external_attitude = Some(ExternalAttitudeMsg {
            qw: 1.0,
            qx: 0.1,
            qy: 0.2,
            qz: 0.3,
        });

        world.run_comm_param_sensor_stages();

        assert!(world.companion_link.connected);
        assert_eq!(
            world.companion_link.last_heartbeat.unwrap().system_status,
            5
        );
        let aux = world.aux_commands.latest.unwrap();
        assert!(matches!(aux.type_array[3], RosflightAuxCmdType::Servo));
        assert_eq!(aux.aux_cmd_array[3], 0.8);
        assert_eq!(world.external_attitude.latest.unwrap().qz, 0.3);
        assert!(world.companion_events.heartbeats.is_empty());
        assert!(world.companion_events.aux_commands.is_empty());
        assert!(world.companion_events.external_attitudes.is_empty());
    }

    #[test]
    fn world_applies_param_defaults_and_sends_ack_after_apply() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let mut world = test_world_with_params(params);

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::SetParamDefaults,
        });

        world.run_comm_param_sensor_stages();

        assert_eq!(
            world.params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(1)
        );
        assert!(world.command_events.param_defaults_requests.is_empty());
        assert_eq!(world.comm.comm_link().cmd_ack_count, 1);

        let ack = world.comm.comm_link().last_cmd_ack.unwrap();
        assert!(matches!(ack.command, RosflightCmd::SetParamDefaults));
        assert!(matches!(
            ack.success,
            RosflightCmdResponse::RosflightCmdSuccess
        ));
    }

    #[test]
    fn world_routes_board_command_and_acks_unsupported_after_apply_stage() {
        let mut world = test_world();

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::WriteParams,
        });

        world.run_comm_param_sensor_stages();

        assert!(world.command_events.board_command_requests.is_empty());
        assert_eq!(world.comm.comm_link().cmd_ack_count, 1);
        let ack = world.comm.comm_link().last_cmd_ack.unwrap();
        assert!(matches!(ack.command, RosflightCmd::WriteParams));
        assert!(matches!(
            ack.success,
            RosflightCmdResponse::RosflightCmdFailed
        ));
    }

    #[test]
    fn world_drains_logs_through_comm_response_stage() {
        while crate::log::Logger::pop().is_some() {}

        let mut world = test_world();
        world.companion_link.connected = true;
        while crate::log::Logger::pop().is_some() {}

        crate::log_info!("world log");
        world.run_comm_param_sensor_stages();

        assert_eq!(world.comm.comm_link().statustext_count, 1);
        let msg = world.comm.comm_link().last_statustext.unwrap();
        assert_eq!(&msg.text[..9], b"world log");
    }

    #[test]
    fn world_routes_rc_trim_calibration_and_sets_equilibrium_torques() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_RC_ATTITUDE_MODE, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(4));
        params.set_by_id(ParamId::PARAM_RC_MAX_ROLLRATE, ParamValue::Float(1.0));
        params.set_by_id(ParamId::PARAM_RC_MAX_PITCHRATE, ParamValue::Float(1.0));
        params.set_by_id(ParamId::PARAM_RC_MAX_YAWRATE, ParamValue::Float(1.0));
        params.set_by_id(ParamId::PARAM_PID_ROLL_RATE_P, ParamValue::Float(2.0));
        params.set_by_id(ParamId::PARAM_PID_PITCH_RATE_P, ParamValue::Float(3.0));
        params.set_by_id(ParamId::PARAM_PID_YAW_RATE_P, ParamValue::Float(4.0));
        params.set_by_id(ParamId::PARAM_X_EQ_TORQUE, ParamValue::Float(0.5));
        params.set_by_id(ParamId::PARAM_Y_EQ_TORQUE, ParamValue::Float(-0.5));
        params.set_by_id(ParamId::PARAM_Z_EQ_TORQUE, ParamValue::Float(0.25));
        let mut world = test_world_with_params(params);

        let mut channels = [0.5; crate::packets::RC_PACKET_CHANNELS];
        channels[0] = 0.55;
        channels[1] = 0.45;
        channels[3] = 0.60;
        world.rc.receive(&crate::packets::RcPacket {
            header: crate::packets::RosflightPacketHeader {
                timestamp: 1,
                status: 0,
            },
            n_chan: 4,
            chan: channels,
            lol: false,
        });
        world.run_rc_command_state_stages();

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::RcCalibration,
        });

        world.run_comm_param_sensor_stages();

        assert!(world.command_events.rc_trim_calibration_requests.is_empty());
        assert_eq!(
            world.params.get_by_id(ParamId::PARAM_X_EQ_TORQUE),
            ParamValue::Float(0.70000005)
        );
        assert_eq!(
            world.params.get_by_id(ParamId::PARAM_Y_EQ_TORQUE),
            ParamValue::Float(-0.8000001)
        );
        assert_eq!(
            world.params.get_by_id(ParamId::PARAM_Z_EQ_TORQUE),
            ParamValue::Float(1.0500002)
        );
        assert_eq!(world.comm.comm_link().cmd_ack_count, 1);
        let ack = world.comm.comm_link().last_cmd_ack.unwrap();
        assert!(matches!(ack.command, RosflightCmd::RcCalibration));
        assert!(matches!(
            ack.success,
            RosflightCmdResponse::RosflightCmdSuccess
        ));
    }

    #[test]
    fn world_routes_reset_origin_and_acks_unsupported_after_apply_stage() {
        let mut world = test_world();

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::ResetOrigin,
        });

        world.run_comm_param_sensor_stages();

        assert!(world.command_events.reset_origin_requests.is_empty());
        assert_eq!(world.comm.comm_link().cmd_ack_count, 1);
        let ack = world.comm.comm_link().last_cmd_ack.unwrap();
        assert!(matches!(ack.command, RosflightCmd::ResetOrigin));
        assert!(matches!(
            ack.success,
            RosflightCmdResponse::RosflightCmdFailed
        ));
    }

    #[test]
    fn world_routes_config_info_and_acks_unsupported_after_apply_stage() {
        let mut world = test_world();

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::SendAllConfigInfos,
        });

        world.run_comm_param_sensor_stages();

        assert!(world.command_events.config_info_requests.is_empty());
        assert_eq!(world.comm.comm_link().cmd_ack_count, 1);
        let ack = world.comm.comm_link().last_cmd_ack.unwrap();
        assert!(matches!(ack.command, RosflightCmd::SendAllConfigInfos));
        assert!(matches!(
            ack.success,
            RosflightCmdResponse::RosflightCmdFailed
        ));
    }

    #[test]
    fn world_rejects_command_actions_while_armed() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let mut world = armed_test_world_with_params(params);
        assert!(world.state.is_armed());

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::SetParamDefaults,
        });

        world.run_comm_param_sensor_stages();

        assert_eq!(
            world.params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );
        assert_eq!(world.comm.comm_link().cmd_ack_count, 1);
        let ack = world.comm.comm_link().last_cmd_ack.unwrap();
        assert!(matches!(ack.command, RosflightCmd::SetParamDefaults));
        assert!(matches!(
            ack.success,
            RosflightCmdResponse::RosflightCmdFailed
        ));
    }
}
