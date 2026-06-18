#[cfg(feature = "timing-diagnostics")]
use crate::comm::ImuTelemetryReadiness;
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
    pwm::system::{PwmOutputState, PwmSyncCtx, sync_pwm_output_state},
    rc::Rc,
    rc::system::{RcCommandStateCtx, run_rc_command_state},
    sensors::health::{SensorHealthCtx, update_sensor_health},
    sensors::ingestion::{
        SensorIngestionCtx, SensorProcessorSet, process_imu_sensor, process_sensor_bus,
    },
    sensors::processors::CalibrationFlags,
    sensors::{ProcessedSensors, SensorBus},
    state_machine::{Event, StateManager},
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
}

impl RealtimeServicePolicy {
    pub const fn with_spacing(min_spacing_us: u64, telemetry_streams_per_phase: usize) -> Self {
        Self {
            min_spacing_us,
            telemetry_streams_per_phase,
        }
    }

    pub const fn continuous(telemetry_streams_per_phase: usize) -> Self {
        Self {
            min_spacing_us: 0,
            telemetry_streams_per_phase,
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

#[cfg(feature = "timing-diagnostics")]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct RealtimeCadenceDiagnostics {
    control_due: u32,
    control_due_no_sample: u32,
    control_ran: u32,
    control_deadline_consumed: u32,
    imu_packet_taken: u32,
    imu_timestamp_changed: u32,
    priority_imu_attempt: u32,
    priority_imu_sent: u32,
    priority_imu_not_due: u32,
    priority_imu_stale: u32,
    priority_imu_no_imu: u32,
    last_control_run_us: Option<u64>,
    max_control_gap_us: u32,
    last_processed_imu_timestamp: Option<u64>,
    max_processed_imu_gap_us: u32,
    last_imu_telemetry_timestamp: Option<u64>,
    max_imu_telemetry_gap_us: u32,
}

#[cfg(feature = "timing-diagnostics")]
impl RealtimeCadenceDiagnostics {
    fn record_control_run(&mut self, now_us: u64) {
        self.control_ran = self.control_ran.saturating_add(1);
        if let Some(last_us) = self.last_control_run_us {
            self.max_control_gap_us = self
                .max_control_gap_us
                .max(now_us.saturating_sub(last_us).min(u32::MAX as u64) as u32);
        }
        self.last_control_run_us = Some(now_us);
    }

    fn record_processed_imu_timestamp(&mut self, timestamp_us: u64) {
        self.imu_timestamp_changed = self.imu_timestamp_changed.saturating_add(1);
        if let Some(last_us) = self.last_processed_imu_timestamp {
            self.max_processed_imu_gap_us = self
                .max_processed_imu_gap_us
                .max(timestamp_us.saturating_sub(last_us).min(u32::MAX as u64) as u32);
        }
        self.last_processed_imu_timestamp = Some(timestamp_us);
    }

    fn record_imu_telemetry_timestamp(&mut self, timestamp_us: u64) {
        if let Some(last_us) = self.last_imu_telemetry_timestamp {
            self.max_imu_telemetry_gap_us = self
                .max_imu_telemetry_gap_us
                .max(timestamp_us.saturating_sub(last_us).min(u32::MAX as u64) as u32);
        }
        self.last_imu_telemetry_timestamp = Some(timestamp_us);
    }

    fn reset_interval(&mut self) {
        self.control_due = 0;
        self.control_due_no_sample = 0;
        self.control_ran = 0;
        self.control_deadline_consumed = 0;
        self.imu_packet_taken = 0;
        self.imu_timestamp_changed = 0;
        self.priority_imu_attempt = 0;
        self.priority_imu_sent = 0;
        self.priority_imu_not_due = 0;
        self.priority_imu_stale = 0;
        self.priority_imu_no_imu = 0;
        self.max_control_gap_us = 0;
        self.max_processed_imu_gap_us = 0;
        self.max_imu_telemetry_gap_us = 0;
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
    next_realtime_service_us: u64,
    #[cfg(feature = "timing-diagnostics")]
    timing_diagnostics: TimingDiagnostics,
    #[cfg(feature = "timing-diagnostics")]
    realtime_cadence_diagnostics: RealtimeCadenceDiagnostics,
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
        rc.init(&params);

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
            next_realtime_service_us: now_us,
            #[cfg(feature = "timing-diagnostics")]
            timing_diagnostics: TimingDiagnostics::new(now_us),
            #[cfg(feature = "timing-diagnostics")]
            realtime_cadence_diagnostics: RealtimeCadenceDiagnostics::default(),
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
#[cfg(feature = "timing-diagnostics")]
mod diagnostics;
mod service;
mod telemetry;
#[cfg(test)]
mod tests;

#[cfg(feature = "timing-diagnostics")]
fn elapsed_u16(start_us: u64, end_us: u64) -> u16 {
    end_us.saturating_sub(start_us).min(u16::MAX as u64) as u16
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

#[cfg(feature = "timing-diagnostics")]
fn stats_from_realtime_result(result: WorldReport) -> WorldRunStats {
    WorldRunStats {
        total_us: result.elapsed_after_control_us.min(u16::MAX as u32) as u16,
        control_us: if result.ran_control {
            result.elapsed_after_control_us.min(u16::MAX as u32) as u16
        } else {
            0
        },
        estimator_us: result.estimator_us,
        controller_us: result.controller_us,
        mixer_us: result.mixer_us,
        pwm_us: result.pwm_us,
        had_rx: result.had_rx,
        had_sensor: result.had_raw_sensor,
        had_imu: result.had_raw_imu || result.had_processed_imu,
        ran_control: result.ran_control,
        ..WorldRunStats::default()
    }
}
