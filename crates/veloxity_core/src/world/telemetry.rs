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
    pub fn run_telemetry_stage(&mut self) {
        let now_us = self.board.clock_micros();
        if !self
            .comm
            .named_telemetry_due(now_us, &self.processed_sensors)
        {
            return;
        }

        let sensor_error_count = self.board.sensors_errors_count();
        self.comm.send_named_telemetry_streams(TelemetryCtx {
            board: &mut self.board,
            now_us,
            state: &self.state,
            command: &self.command,
            params: &self.params,
            estimator_state: &self.control_pipeline.latest_estimator_state,
            sensors: &self.processed_sensors,
            actuator_commands: &self.control_pipeline.latest_pwm_outputs,
            sensor_error_count,
            loop_time_us: self.control_pipeline.latest_loop_time_us,
        });
    }

    /// Sends up to `max_streams` currently due named telemetry streams.
    ///
    /// Board realtime loops may call this after a completed control update when hardware timing
    /// shows enough post-control slack and service phases alone do not provide enough telemetry
    /// scheduling opportunities. Keep the budget board-specific and validate the resulting control
    /// p99/max timing with scope pins or timing diagnostics before making it a production default.
    pub fn run_realtime_telemetry_stage_budgeted(&mut self, max_streams: usize) -> usize {
        let mut sent = 0;
        while sent < max_streams && self.send_realtime_telemetry_stream() {
            sent += 1;
        }
        sent
    }

    /// Sends board-selected priority streams first, then fills the remaining budget normally.
    ///
    /// `priority_streams` is a board policy list. Core applies each stream's configured priority
    /// gate, so boards can choose normal due-deadline behavior or a stricter freshness gate for
    /// streams paced by the control tick.
    pub fn run_realtime_telemetry_stage_prioritized(
        &mut self,
        priority_streams: &[RealtimeTelemetryPriority],
        max_streams: usize,
    ) -> usize {
        let mut sent = 0;
        for priority in priority_streams.iter().copied() {
            if sent >= max_streams {
                return sent;
            }
            if self.send_realtime_telemetry_stream_by_priority(priority) {
                sent += 1;
            }
        }
        while sent < max_streams && self.send_realtime_telemetry_stream() {
            sent += 1;
        }
        sent
    }

    pub(super) fn send_realtime_telemetry_stream(&mut self) -> bool {
        let now_us = self.board.clock_micros();
        let sensor_error_count = self.board.sensors_errors_count();
        self.comm.send_one_named_telemetry_stream(TelemetryCtx {
            board: &mut self.board,
            now_us,
            state: &self.state,
            command: &self.command,
            params: &self.params,
            estimator_state: &self.control_pipeline.latest_estimator_state,
            sensors: &self.processed_sensors,
            actuator_commands: &self.control_pipeline.latest_pwm_outputs,
            sensor_error_count,
            loop_time_us: self.control_pipeline.latest_loop_time_us,
        })
    }

    pub(super) fn send_realtime_telemetry_stream_by_priority(
        &mut self,
        priority: RealtimeTelemetryPriority,
    ) -> bool {
        let now_us = self.board.clock_micros();
        #[cfg(feature = "timing-diagnostics")]
        let imu_readiness = if priority.stream == crate::comm::NamedTelemetryStream::Imu {
            self.realtime_cadence_diagnostics.priority_imu_attempt = self
                .realtime_cadence_diagnostics
                .priority_imu_attempt
                .saturating_add(1);
            Some(self.comm.imu_telemetry_readiness_for_gate(
                now_us,
                &self.processed_sensors,
                priority.gate,
            ))
        } else {
            None
        };
        let sensor_error_count = self.board.sensors_errors_count();
        let sent = self.comm.send_named_telemetry_stream_with_gate(
            priority,
            TelemetryCtx {
                board: &mut self.board,
                now_us,
                state: &self.state,
                command: &self.command,
                params: &self.params,
                estimator_state: &self.control_pipeline.latest_estimator_state,
                sensors: &self.processed_sensors,
                actuator_commands: &self.control_pipeline.latest_pwm_outputs,
                sensor_error_count,
                loop_time_us: self.control_pipeline.latest_loop_time_us,
            },
        );
        #[cfg(feature = "timing-diagnostics")]
        if let Some(readiness) = imu_readiness {
            if sent {
                self.realtime_cadence_diagnostics.priority_imu_sent = self
                    .realtime_cadence_diagnostics
                    .priority_imu_sent
                    .saturating_add(1);
                if let Some(packet) = self.processed_sensors.imu {
                    self.realtime_cadence_diagnostics
                        .record_imu_telemetry_timestamp(packet.header.timestamp);
                }
            } else {
                match readiness {
                    ImuTelemetryReadiness::Due => {}
                    ImuTelemetryReadiness::NotDue => {
                        self.realtime_cadence_diagnostics.priority_imu_not_due = self
                            .realtime_cadence_diagnostics
                            .priority_imu_not_due
                            .saturating_add(1);
                    }
                    ImuTelemetryReadiness::Stale => {
                        self.realtime_cadence_diagnostics.priority_imu_stale = self
                            .realtime_cadence_diagnostics
                            .priority_imu_stale
                            .saturating_add(1);
                    }
                    ImuTelemetryReadiness::NoImu => {
                        self.realtime_cadence_diagnostics.priority_imu_no_imu = self
                            .realtime_cadence_diagnostics
                            .priority_imu_no_imu
                            .saturating_add(1);
                    }
                }
            }
        }
        sent
    }
}
