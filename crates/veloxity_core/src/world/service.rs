use super::*;

#[cfg(feature = "runtime-diagnostics")]
use crate::comm::NamedTelemetryStream;
#[cfg(feature = "runtime-diagnostics")]
use core::sync::atomic::{AtomicU32, Ordering};

#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_IMU_UNSENT_OVERWRITE: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_IMU_UNSENT_AGE_SUM_US: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_IMU_UNSENT_AGE_MAX_US: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_SERVICE_PHASE_COUNT: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_SERVICE_PHASE_SUM_US: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_SERVICE_PHASE_MAX_US: AtomicU32 = AtomicU32::new(0);

#[cfg(feature = "runtime-diagnostics")]
macro_rules! sensor_pipeline_counters {
    ($input:ident, $output:ident, $unsent:ident) => {
        #[unsafe(no_mangle)]
        pub static $input: AtomicU32 = AtomicU32::new(0);
        #[unsafe(no_mangle)]
        pub static $output: AtomicU32 = AtomicU32::new(0);
        #[unsafe(no_mangle)]
        pub static $unsent: AtomicU32 = AtomicU32::new(0);
    };
}

#[cfg(feature = "runtime-diagnostics")]
sensor_pipeline_counters!(
    VELOXITY_DIAG_MAG_PROCESS_INPUT,
    VELOXITY_DIAG_MAG_PROCESS_OUTPUT,
    VELOXITY_DIAG_MAG_UNSENT_OVERWRITE
);
#[cfg(feature = "runtime-diagnostics")]
sensor_pipeline_counters!(
    VELOXITY_DIAG_BARO_PROCESS_INPUT,
    VELOXITY_DIAG_BARO_PROCESS_OUTPUT,
    VELOXITY_DIAG_BARO_UNSENT_OVERWRITE
);
#[cfg(feature = "runtime-diagnostics")]
sensor_pipeline_counters!(
    VELOXITY_DIAG_PITOT_PROCESS_INPUT,
    VELOXITY_DIAG_PITOT_PROCESS_OUTPUT,
    VELOXITY_DIAG_PITOT_UNSENT_OVERWRITE
);
#[cfg(feature = "runtime-diagnostics")]
sensor_pipeline_counters!(
    VELOXITY_DIAG_RANGE_PROCESS_INPUT,
    VELOXITY_DIAG_RANGE_PROCESS_OUTPUT,
    VELOXITY_DIAG_RANGE_UNSENT_OVERWRITE
);
#[cfg(feature = "runtime-diagnostics")]
sensor_pipeline_counters!(
    VELOXITY_DIAG_GNSS_PROCESS_INPUT,
    VELOXITY_DIAG_GNSS_PROCESS_OUTPUT,
    VELOXITY_DIAG_GNSS_UNSENT_OVERWRITE
);
#[cfg(feature = "runtime-diagnostics")]
sensor_pipeline_counters!(
    VELOXITY_DIAG_BATTERY_PROCESS_INPUT,
    VELOXITY_DIAG_BATTERY_PROCESS_OUTPUT,
    VELOXITY_DIAG_BATTERY_UNSENT_OVERWRITE
);
#[cfg(feature = "runtime-diagnostics")]
sensor_pipeline_counters!(
    VELOXITY_DIAG_RC_PROCESS_INPUT,
    VELOXITY_DIAG_RC_PROCESS_OUTPUT,
    VELOXITY_DIAG_RC_UNSENT_OVERWRITE
);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_IMU_PROCESS_INPUT: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_IMU_PROCESS_OUTPUT: AtomicU32 = AtomicU32::new(0);

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
    pub fn run_prioritized_service_steps_with_policy(
        &mut self,
        policy: RealtimeServicePolicy,
    ) -> WorldReport {
        let pass_start_us = self.board.clock_micros();
        let mut result = WorldReport {
            had_rx: self.board.serial_rx_pending(),
            ..WorldReport::default()
        };

        while self.realtime_service_can_continue() {
            let step_result = self.run_prioritized_service_step(policy);
            let had_service_activity = step_result.had_rx
                || step_result.had_raw_sensor
                || step_result.telemetry_due
                || step_result.telemetry_deferred;
            result.merge_from(step_result);

            if policy.min_spacing_us != 0 {
                break;
            }
            if !had_service_activity && !policy.continue_when_idle {
                break;
            }
        }

        self.next_realtime_service_us = self
            .board
            .clock_micros()
            .saturating_add(policy.min_spacing_us);
        result.elapsed_after_control_us = self
            .board
            .clock_micros()
            .saturating_sub(pass_start_us)
            .min(u32::MAX as u64) as u32;
        #[cfg(feature = "runtime-diagnostics")]
        {
            VELOXITY_DIAG_SERVICE_PHASE_COUNT.fetch_add(1, Ordering::Relaxed);
            VELOXITY_DIAG_SERVICE_PHASE_SUM_US
                .fetch_add(result.elapsed_after_control_us, Ordering::Relaxed);
            VELOXITY_DIAG_SERVICE_PHASE_MAX_US
                .fetch_max(result.elapsed_after_control_us, Ordering::Relaxed);
        }
        result
    }

    pub(super) fn run_prioritized_service_step(
        &mut self,
        policy: RealtimeServicePolicy,
    ) -> WorldReport {
        let mut result = WorldReport::default();

        // Telemetry is the only service work that can lose a retained sample
        // when the next producer update replaces it. Give it the slack that
        // was established when this service opportunity began,
        // before variable-duration sensor, input, RC, and response work can
        // close the control-deadline guard.
        if self.realtime_service_can_continue() {
            result.telemetry_due |= if policy.drain_telemetry_with_available_slack {
                self.run_realtime_telemetry_stage_with_available_slack() != 0
            } else {
                self.run_realtime_telemetry_stage_budgeted(policy.telemetry_streams_per_phase) != 0
            };
        }

        let sensor_result = if self.realtime_service_can_continue() {
            self.run_service_sensor_stage()
        } else {
            WorldReport::default()
        };
        result.merge_from(sensor_result);

        if self.realtime_service_can_continue() {
            result.had_rx |= self.board.serial_rx_pending();
            self.run_service_input_stage();
        }

        if self.realtime_service_can_continue() {
            let fresh_rc = if sensor_result.had_raw_rc {
                self.processed_sensors.rc
            } else {
                None
            };
            self.run_rc_command_state_stages(fresh_rc);
            result.had_processed_rc =
                sensor_result.had_raw_rc && self.processed_sensors.rc.is_some();
        }

        if self.realtime_service_can_continue() {
            self.drain_logs_and_send_responses_limited(REALTIME_SERVICE_RESPONSE_BUDGET);
        }

        if self.realtime_service_can_continue() {
            self.board.serial_flush_budgeted(1);
        }

        if self.realtime_service_can_continue() {
            self.board.run_deferred_board_actions();
        }

        result
    }

    pub(super) fn run_service_input_stage(&mut self) {
        self.run_communication_and_parameter_service_stage();
    }

    pub(super) fn run_service_sensor_stage(&mut self) -> WorldReport {
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

        #[cfg(feature = "runtime-diagnostics")]
        {
            macro_rules! record_pipeline {
                ($had_raw:ident, $field:ident, $input:ident, $output:ident) => {
                    if $had_raw {
                        $input.fetch_add(1, Ordering::Relaxed);
                        if self.processed_sensors.$field.is_some() {
                            $output.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                };
            }
            record_pipeline!(
                had_raw_mag,
                mag,
                VELOXITY_DIAG_MAG_PROCESS_INPUT,
                VELOXITY_DIAG_MAG_PROCESS_OUTPUT
            );
            record_pipeline!(
                had_raw_baro,
                baro,
                VELOXITY_DIAG_BARO_PROCESS_INPUT,
                VELOXITY_DIAG_BARO_PROCESS_OUTPUT
            );
            record_pipeline!(
                had_raw_pitot,
                pitot,
                VELOXITY_DIAG_PITOT_PROCESS_INPUT,
                VELOXITY_DIAG_PITOT_PROCESS_OUTPUT
            );
            record_pipeline!(
                had_raw_range,
                range,
                VELOXITY_DIAG_RANGE_PROCESS_INPUT,
                VELOXITY_DIAG_RANGE_PROCESS_OUTPUT
            );
            record_pipeline!(
                had_raw_gnss,
                gnss,
                VELOXITY_DIAG_GNSS_PROCESS_INPUT,
                VELOXITY_DIAG_GNSS_PROCESS_OUTPUT
            );
            record_pipeline!(
                had_raw_battery,
                battery,
                VELOXITY_DIAG_BATTERY_PROCESS_INPUT,
                VELOXITY_DIAG_BATTERY_PROCESS_OUTPUT
            );
            record_pipeline!(
                had_raw_rc,
                rc,
                VELOXITY_DIAG_RC_PROCESS_INPUT,
                VELOXITY_DIAG_RC_PROCESS_OUTPUT
            );

            macro_rules! record_unsent {
                ($had_raw:ident, $before:ident, $field:ident, $stream:ident, $counter:ident) => {
                    if $had_raw
                        && let (Some(previous), Some(current)) =
                            ($before, self.processed_sensors.$field)
                        && previous.header.timestamp != current.header.timestamp
                        && !self.comm.telemetry_sample_was_sent(
                            NamedTelemetryStream::$stream,
                            previous.header.timestamp,
                        )
                    {
                        $counter.fetch_add(1, Ordering::Relaxed);
                    }
                };
            }
            record_unsent!(
                had_raw_mag,
                latest_mag,
                mag,
                Mag,
                VELOXITY_DIAG_MAG_UNSENT_OVERWRITE
            );
            record_unsent!(
                had_raw_baro,
                latest_baro,
                baro,
                Baro,
                VELOXITY_DIAG_BARO_UNSENT_OVERWRITE
            );
            record_unsent!(
                had_raw_pitot,
                latest_pitot,
                pitot,
                DiffPressure,
                VELOXITY_DIAG_PITOT_UNSENT_OVERWRITE
            );
            record_unsent!(
                had_raw_range,
                latest_range,
                range,
                Range,
                VELOXITY_DIAG_RANGE_UNSENT_OVERWRITE
            );
            record_unsent!(
                had_raw_gnss,
                latest_gnss,
                gnss,
                Gnss,
                VELOXITY_DIAG_GNSS_UNSENT_OVERWRITE
            );
            record_unsent!(
                had_raw_battery,
                latest_battery,
                battery,
                Battery,
                VELOXITY_DIAG_BATTERY_UNSENT_OVERWRITE
            );
            record_unsent!(
                had_raw_rc,
                latest_rc,
                rc,
                Rc,
                VELOXITY_DIAG_RC_UNSENT_OVERWRITE
            );
        }

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
        WorldReport {
            had_raw_sensor: had_raw_imu
                || had_raw_mag
                || had_raw_baro
                || had_raw_pitot
                || had_raw_range
                || had_raw_gnss
                || had_raw_battery
                || had_raw_rc
                || had_raw_attitude,
            had_raw_imu,
            had_raw_baro,
            had_raw_rc,
            had_processed_imu: self.processed_sensors.imu.is_some(),
            had_processed_baro: self.processed_sensors.baro.is_some(),
            had_processed_rc: self.processed_sensors.rc.is_some(),
            ..WorldReport::default()
        }
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
        self.request_gyro_calibration_if_needed();
        if self.param_events.full_refresh || !self.param_events.changes.is_empty() {
            self.apply_param_reactions();
        }
    }

    pub fn run_sensor_ingestion_and_health_stage(&mut self) {
        self.run_sensor_ingestion_and_health_stage_without_log_drain();
        self.drain_logs_and_send_responses();
    }

    pub(super) fn run_sensor_ingestion_and_health_stage_without_log_drain(&mut self) {
        let now_us = self.board.clock_micros();

        self.run_sensor_ingestion_stage();
        self.update_sensor_health_and_calibration(now_us);
    }

    pub(super) fn process_comm_stage(&mut self) {
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

    pub(super) fn has_pending_companion_work(&self) -> bool {
        !self.companion_events.is_empty()
            || (self.companion_link.connected && self.pending_hard_error.is_some())
    }

    pub(super) fn has_pending_param_work(&self) -> bool {
        !self.param_events.set_requests.is_empty()
            || !self.param_events.read_requests.is_empty()
            || !self.param_events.list_requests.is_empty()
            || self.param_list_state.is_active()
    }

    pub(super) fn apply_companion_events(&mut self) {
        companion::apply_companion_inputs(&mut CompanionInputCtx {
            events: &mut self.companion_events,
            comm_events: &mut self.comm_events,
            link: &mut self.companion_link,
            aux_commands: &mut self.aux_commands,
            external_attitude: &mut self.external_attitude,
            pending_hard_error: &mut self.pending_hard_error,
        });
    }

    pub(super) fn apply_command_events(&mut self) {
        command_service::apply_command_requests(&mut CommandRequestCtx {
            requests: &mut self.command_events,
            param_events: &mut self.param_events,
            comm_events: &mut self.comm_events,
            state: &self.state,
            command: &mut self.command,
            controller: &mut self.controller,
            board: &mut self.board,
            flags: &mut self.cal_flags,
            params: &mut self.params,
        });
    }

    pub(super) fn service_param_events(&mut self) {
        param_service::service_param_events(&mut ParamServiceCtx {
            params: &mut self.params,
            state: &mut self.param_list_state,
            events: &mut self.param_events,
            comm_events: &mut self.comm_events,
        });
    }

    pub(super) fn apply_param_reactions(&mut self) {
        let battery_monitor_changed = self.param_events.full_refresh
            || self.param_events.changes.iter().any(|change| {
                matches!(
                    change.id,
                    ParamId::PARAM_BATTERY_VOLTAGE_MULTIPLIER
                        | ParamId::PARAM_BATTERY_CURRENT_MULTIPLIER
                )
            });
        if battery_monitor_changed {
            let voltage_multiplier = match self
                .params
                .get_by_id(ParamId::PARAM_BATTERY_VOLTAGE_MULTIPLIER)
            {
                ParamValue::Float(value) => value,
                _ => 0.0,
            };
            let current_multiplier = match self
                .params
                .get_by_id(ParamId::PARAM_BATTERY_CURRENT_MULTIPLIER)
            {
                ParamValue::Float(value) => value,
                _ => 0.0,
            };
            self.board
                .configure_battery_monitor(voltage_multiplier, current_multiplier);
        }
        if self.param_events.full_refresh {
            self.comm.configure_telemetry_from_params(&self.params);
        } else {
            let now_us = self.board.clock_micros();
            for change in self.param_events.changes.iter() {
                self.comm
                    .update_telemetry_param(&self.params, change.id, now_us);
            }
        }
        reactions::apply_param_reactions(&mut ParamReactionCtx {
            events: &mut self.param_events,
            params: &self.params,
            rc: &mut self.rc,
            command: &mut self.command,
            state: &mut self.state,
            estimator: &mut self.estimator,
            controller: &mut self.controller,
            mixer: &mut self.mixer,
            control_pipeline: &mut self.control_pipeline,
        });
    }

    pub(super) fn request_gyro_calibration_if_needed(&mut self) {
        if self.state.is_calibrating() && !self.cal_flags.contains(CalibrationFlags::GYRO) {
            self.cal_flags.remove(CalibrationFlags::GYRO_FAILED);
            param_service::set_param_and_emit_change(
                &mut self.params,
                &mut self.param_events.changes,
                ParamId::PARAM_GYRO_X_BIAS,
                ParamValue::Float(0.0),
            );
            param_service::set_param_and_emit_change(
                &mut self.params,
                &mut self.param_events.changes,
                ParamId::PARAM_GYRO_Y_BIAS,
                ParamValue::Float(0.0),
            );
            param_service::set_param_and_emit_change(
                &mut self.params,
                &mut self.param_events.changes,
                ParamId::PARAM_GYRO_Z_BIAS,
                ParamValue::Float(0.0),
            );
            self.cal_flags.insert(CalibrationFlags::GYRO);
        }
    }

    pub(super) fn process_sensor_bus_after_update(&mut self) {
        let now_ms = self.board.clock_millis();
        let calibration_flags_before = self.cal_flags;
        let baro_bias_before = self.params.get_by_id(ParamId::PARAM_BARO_BIAS);
        let ground_level_before = self.params.get_by_id(ParamId::PARAM_GROUND_LEVEL);
        process_sensor_bus(SensorIngestionCtx {
            raw: &mut self.raw_sensors,
            processed: &mut self.processed_sensors,
            processors: &mut self.sensor_processors,
            flags: &mut self.cal_flags,
            params: &mut self.params,
            now_ms,
        });
        if calibration_flags_before.contains(CalibrationFlags::BARO)
            && !self.cal_flags.contains(CalibrationFlags::BARO)
            && !self.cal_flags.contains(CalibrationFlags::BARO_FAILED)
        {
            // The barometer processor owns the asynchronous sampling window
            // and writes these values directly.  Publish them now so
            // rosflight_io receives the completion acknowledgement as the
            // normal MAVLink parameter update.
            param_service::emit_param_change(
                &mut self.param_events.changes,
                ParamId::PARAM_BARO_BIAS,
                baro_bias_before,
                self.params.get_by_id(ParamId::PARAM_BARO_BIAS),
            );
            param_service::emit_param_change(
                &mut self.param_events.changes,
                ParamId::PARAM_GROUND_LEVEL,
                ground_level_before,
                self.params.get_by_id(ParamId::PARAM_GROUND_LEVEL),
            );
        }
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
            // Match ROSflight C: only a successful full accelerometer/IMU
            // calibration clears an uncalibrated error that was latched at
            // startup. Gyro-only calibration and live bias changes do not.
            self.state
                .set_error_flag(ErrorFlag::UNCALIBRATED_IMU, false, &self.params);
            self.estimator.reset();
            self.control_pipeline = ControlPipelineResource::default();
        }
    }

    pub(super) fn process_imu_sensor_after_update(&mut self) {
        #[cfg(feature = "runtime-diagnostics")]
        let previous_imu = self.processed_sensors.imu;
        #[cfg(feature = "runtime-diagnostics")]
        let had_raw_imu = self.raw_sensors.imu.is_some();
        let now_ms = self.board.clock_millis();
        let calibration_flags_before = self.cal_flags;
        process_imu_sensor(SensorIngestionCtx {
            raw: &mut self.raw_sensors,
            processed: &mut self.processed_sensors,
            processors: &mut self.sensor_processors,
            flags: &mut self.cal_flags,
            params: &mut self.params,
            now_ms,
        });
        #[cfg(feature = "runtime-diagnostics")]
        if had_raw_imu {
            VELOXITY_DIAG_IMU_PROCESS_INPUT.fetch_add(1, Ordering::Relaxed);
            if self.processed_sensors.imu.is_some() {
                VELOXITY_DIAG_IMU_PROCESS_OUTPUT.fetch_add(1, Ordering::Relaxed);
            }
        }
        #[cfg(feature = "runtime-diagnostics")]
        if let (Some(previous), Some(current)) = (previous_imu, self.processed_sensors.imu)
            && previous.header.timestamp != current.header.timestamp
            && !self
                .comm
                .telemetry_sample_was_sent(NamedTelemetryStream::Imu, previous.header.timestamp)
        {
            VELOXITY_DIAG_IMU_UNSENT_OVERWRITE.fetch_add(1, Ordering::Relaxed);
            let age_us = self
                .board
                .clock_micros()
                .saturating_sub(previous.header.timestamp)
                .min(u32::MAX as u64) as u32;
            VELOXITY_DIAG_IMU_UNSENT_AGE_SUM_US.fetch_add(age_us, Ordering::Relaxed);
            VELOXITY_DIAG_IMU_UNSENT_AGE_MAX_US.fetch_max(age_us, Ordering::Relaxed);
        }
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
            // See process_sensor_bus_after_update(): this is the high-rate
            // IMU path for the same successful full-calibration transition.
            self.state
                .set_error_flag(ErrorFlag::UNCALIBRATED_IMU, false, &self.params);
            self.estimator.reset();
            self.control_pipeline = ControlPipelineResource::default();
        }
    }

    pub(super) fn record_control_imu_candidate(&mut self) {
        if let Some(imu) = self.processed_sensors.imu {
            self.control_imu_accumulator.push(imu);
        }
    }

    pub(super) fn run_sensor_ingestion_stage(&mut self) {
        self.board.update_sensor_bus(&mut self.raw_sensors);
        self.process_sensor_bus_after_update();
    }

    pub(super) fn drain_logs_and_send_responses(&mut self) {
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

    pub(super) fn drain_logs_and_send_responses_limited(&mut self, max_responses: usize) {
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

    pub(super) fn update_sensor_health_and_calibration(&mut self, now_us: u64) {
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
}
