use super::*;

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
    pub fn run_once_classified(&mut self) -> WorldReport {
        self.run_once_budgeted_classified()
    }

    #[cfg(not(feature = "timing-diagnostics"))]
    pub fn run_once_spike_counted(&mut self) -> WorldReport {
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

        WorldReport {
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
    pub fn run_once_budgeted_classified(&mut self) -> WorldReport {
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

        WorldReport {
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
    pub fn run_once_classified(&mut self) -> WorldReport {
        let stats = self.run_once_measured();
        WorldReport {
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

    #[cfg(feature = "timing-diagnostics")]
    pub fn run_control_update_tick_classified(&mut self) -> WorldReport {
        let pass_start_us = self.board.clock_micros();
        let now_us = self.board.clock_micros();
        self.record_realtime_control_gate(now_us);
        let mut control_timing = ControlPipelineTiming::default();
        let ran_control =
            self.run_control_and_mixing_stage_if_control_due_measured(now_us, &mut control_timing);
        if ran_control {
            self.last_realtime_control_us = self.board.clock_micros();
            self.realtime_cadence_diagnostics
                .record_control_run(self.last_realtime_control_us);
        }

        let result = WorldReport {
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
            ..WorldReport::default()
        };
        if result.ran_control {
            self.record_timing_diagnostics(stats_from_realtime_result(result));
        }
        result
    }

    pub fn run_imu_control_tick_classified(&mut self) -> WorldReport {
        let pass_start_us = self.board.clock_micros();
        #[cfg(feature = "pre-control-scope")]
        self.board.set_test_pin_3(true);
        let now_us = self.board.clock_micros();
        #[cfg(feature = "timing-diagnostics")]
        self.record_realtime_control_gate(now_us);
        #[cfg(feature = "timing-diagnostics")]
        let previous_processed_imu_timestamp = self
            .processed_sensors
            .imu
            .map(|packet| packet.header.timestamp);
        self.board.update_imu_sensor(&mut self.raw_sensors);
        let had_raw_imu = self.raw_sensors.imu.is_some();
        let had_raw_sensor = had_raw_imu;
        #[cfg(feature = "timing-diagnostics")]
        if had_raw_imu {
            self.realtime_cadence_diagnostics.imu_packet_taken = self
                .realtime_cadence_diagnostics
                .imu_packet_taken
                .saturating_add(1);
        }
        self.process_imu_sensor_after_update();
        #[cfg(feature = "timing-diagnostics")]
        if let Some(packet) = self.processed_sensors.imu {
            if previous_processed_imu_timestamp != Some(packet.header.timestamp) {
                self.realtime_cadence_diagnostics
                    .record_processed_imu_timestamp(packet.header.timestamp);
            }
        }
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
            #[cfg(feature = "timing-diagnostics")]
            self.realtime_cadence_diagnostics
                .record_control_run(self.last_realtime_control_us);
        }
        let telemetry_due = self
            .comm
            .named_telemetry_due(self.board.clock_micros(), &self.processed_sensors)
            || !self.comm_events.is_empty();

        let result = WorldReport {
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
            ..WorldReport::default()
        };
        #[cfg(feature = "timing-diagnostics")]
        if result.ran_control {
            self.record_timing_diagnostics(stats_from_realtime_result(result));
        }
        result
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

    pub(super) fn control_update_due_at(&self, now_us: u64) -> bool {
        let rate_hz = self.control_loop_rates.control_hz;
        if rate_hz == 0 {
            return false;
        }

        let interval_us = 1_000_000_u64 / rate_hz as u64;
        now_us.saturating_sub(self.last_control_update_us) >= interval_us
    }

    pub(super) fn consume_control_update_deadline(&mut self, now_us: u64) {
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
        #[cfg(feature = "timing-diagnostics")]
        {
            self.realtime_cadence_diagnostics.control_deadline_consumed = self
                .realtime_cadence_diagnostics
                .control_deadline_consumed
                .saturating_add(1);
        }
    }

    pub(super) fn control_update_can_run_at(&self, now_us: u64) -> bool {
        self.control_update_due_at(now_us) && self.control_imu_accumulator.has_samples()
    }

    #[cfg(feature = "timing-diagnostics")]
    pub(super) fn record_realtime_control_gate(&mut self, now_us: u64) {
        if !self.control_update_due_at(now_us) {
            return;
        }
        self.realtime_cadence_diagnostics.control_due = self
            .realtime_cadence_diagnostics
            .control_due
            .saturating_add(1);
        if !self.control_imu_accumulator.has_samples() {
            self.realtime_cadence_diagnostics.control_due_no_sample = self
                .realtime_cadence_diagnostics
                .control_due_no_sample
                .saturating_add(1);
        }
    }

    pub(super) fn realtime_service_has_control_slack(&self, now_us: u64) -> bool {
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

    pub(super) fn realtime_service_can_continue(&self) -> bool {
        if self.imu_pending() {
            return false;
        }
        let now_us = self.board.clock_micros();
        !self.control_update_can_run_at(now_us) && self.realtime_service_has_control_slack(now_us)
    }

    pub(super) fn run_control_and_mixing_stage_if_control_due(&mut self, now_us: u64) -> bool {
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

    pub(super) fn run_control_and_mixing_stage_if_control_due_measured(
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

    pub(super) fn run_control_and_mixing_stage_with_accumulated_imu(&mut self) -> bool {
        let Some(averaged_imu) = self.control_imu_accumulator.take_average() else {
            return false;
        };
        let latest_imu = self.processed_sensors.imu;
        self.processed_sensors.imu = Some(averaged_imu);
        let ran_control = self.run_control_and_mixing_stage_if_new_imu();
        self.processed_sensors.imu = latest_imu;
        ran_control
    }

    pub(super) fn run_control_and_mixing_stage_with_accumulated_imu_measured(
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
        self.run_control_and_mixing_stage_if_new_imu_with_timing(None)
    }

    pub(super) fn run_control_and_mixing_stage_if_new_imu_measured(
        &mut self,
        timing: &mut ControlPipelineTiming,
    ) -> bool {
        self.run_control_and_mixing_stage_if_new_imu_with_timing(Some(timing))
    }

    pub(super) fn run_control_and_mixing_stage_if_new_imu_with_timing(
        &mut self,
        timing: Option<&mut ControlPipelineTiming>,
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
            timing,
        })
    }
    pub(super) fn update_board_leds(&mut self, now_ms: u32) {
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
