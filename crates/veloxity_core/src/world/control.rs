use super::*;

#[cfg(feature = "runtime-diagnostics")]
use core::sync::atomic::{AtomicU32, Ordering};

#[cfg(feature = "runtime-diagnostics")]
macro_rules! diagnostic_counter {
    ($name:ident) => {
        #[unsafe(no_mangle)]
        pub static $name: AtomicU32 = AtomicU32::new(0);
    };
}

#[cfg(feature = "runtime-diagnostics")]
diagnostic_counter!(VELOXITY_DIAG_IMU_TICK_COUNT);
#[cfg(feature = "runtime-diagnostics")]
diagnostic_counter!(VELOXITY_DIAG_IMU_TICK_SUM_US);
#[cfg(feature = "runtime-diagnostics")]
diagnostic_counter!(VELOXITY_DIAG_IMU_TICK_MAX_US);
#[cfg(feature = "runtime-diagnostics")]
diagnostic_counter!(VELOXITY_DIAG_POST_IMU_SERVICE_AVAILABLE);
#[cfg(feature = "runtime-diagnostics")]
diagnostic_counter!(VELOXITY_DIAG_POST_IMU_BLOCKED_PENDING_IMU);
#[cfg(feature = "runtime-diagnostics")]
diagnostic_counter!(VELOXITY_DIAG_POST_IMU_BLOCKED_CONTROL_DUE);
#[cfg(feature = "runtime-diagnostics")]
diagnostic_counter!(VELOXITY_DIAG_POST_IMU_BLOCKED_GUARD);

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
        self.run_communication_and_parameter_service_stage();
        self.run_sensor_ingestion_and_health_stage();
        let fresh_rc = self.processed_sensors.rc;
        self.run_rc_command_state_stages(fresh_rc);
        self.run_control_and_mixing_stage_if_new_imu();
        self.run_telemetry_stage();
        self.board.serial_flush();
        self.board.run_deferred_board_actions();
        true
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
        #[cfg(feature = "runtime-diagnostics")]
        {
            let finished_us = self.board.clock_micros();
            let elapsed_us = finished_us.saturating_sub(now_us).min(u32::MAX as u64) as u32;
            VELOXITY_DIAG_IMU_TICK_COUNT.fetch_add(1, Ordering::Relaxed);
            VELOXITY_DIAG_IMU_TICK_SUM_US.fetch_add(elapsed_us, Ordering::Relaxed);
            VELOXITY_DIAG_IMU_TICK_MAX_US.fetch_max(elapsed_us, Ordering::Relaxed);

            if self.imu_pending() {
                VELOXITY_DIAG_POST_IMU_BLOCKED_PENDING_IMU.fetch_add(1, Ordering::Relaxed);
            } else if self.control_update_can_run_at(finished_us) {
                VELOXITY_DIAG_POST_IMU_BLOCKED_CONTROL_DUE.fetch_add(1, Ordering::Relaxed);
            } else if !self.realtime_service_has_control_slack(finished_us) {
                VELOXITY_DIAG_POST_IMU_BLOCKED_GUARD.fetch_add(1, Ordering::Relaxed);
            } else {
                VELOXITY_DIAG_POST_IMU_SERVICE_AVAILABLE.fetch_add(1, Ordering::Relaxed);
            }
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

    pub fn run_rc_command_state_stages(&mut self, fresh_rc: Option<crate::packets::RcPacket>) {
        let now_ms = self.board.clock_millis();

        run_rc_command_state(RcCommandStateCtx {
            now_ms,
            fresh_rc,
            rc: &mut self.rc,
            command: &mut self.command,
            state: &mut self.state,
            params: &mut self.params,
            param_events: Some(&mut self.param_events),
        });
        self.run_pwm_output_stage();
        self.update_board_leds(now_ms);
    }

    pub fn run_pwm_output_stage(&mut self) -> bool {
        let channel_outputs_disabled = matches!(
            self.params
                .get_by_id(crate::params::ParamId::PARAM_CHANNEL_OUTPUT_MASK),
            crate::params::ParamValue::Int(0)
        );
        sync_pwm_output_state(PwmSyncCtx {
            board: &mut self.board,
            pwm: &mut self.pwm,
            output: &mut self.pwm_output,
            output_kill_active: channel_outputs_disabled
                || (self.rc.switch_mapped(crate::rc::Switch::OutputKill)
                    && self.rc.switch_on(crate::rc::Switch::OutputKill)),
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
        let elapsed_intervals = now_us.saturating_sub(self.last_control_update_us) / interval_us;
        self.last_control_update_us = self
            .last_control_update_us
            .saturating_add(elapsed_intervals.saturating_mul(interval_us));
    }

    pub(super) fn control_update_can_run_at(&self, now_us: u64) -> bool {
        self.control_update_due_at(now_us) && self.control_imu_accumulator.has_samples()
    }

    pub(super) fn realtime_service_has_control_slack(&self, now_us: u64) -> bool {
        let rate_hz = self.control_loop_rates.control_hz;
        if rate_hz == 0 {
            return true;
        }
        // A fixed-rate control update cannot run without accumulated IMU data.
        // Do not reserve the nominal deadline's guard window for impossible
        // work: service can use that time and will stop as soon as a new IMU
        // interrupt makes control work possible again.
        if !self.control_imu_accumulator.has_samples() {
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

    pub fn run_control_and_mixing_stage_if_new_imu(&mut self) -> bool {
        self.run_control_and_mixing_stage_if_new_imu_with_timing(None)
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
