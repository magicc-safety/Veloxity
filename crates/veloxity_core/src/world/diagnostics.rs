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
    #[cfg(feature = "timing-diagnostics")]
    pub(super) fn record_timing_diagnostics(&mut self, stats: WorldRunStats) {
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

        if let Some(text) = self.comm.telemetry_scheduler_diagnostic_text() {
            self.comm_events.responses.push_or_log(
                CommResponse::Statustext(StatustextMsg {
                    severity: Severity::Debug,
                    text,
                }),
                "telemetry scheduler diagnostics",
            );
        }

        self.push_timing_diagnostic_text(format_realtime_control_cadence(
            self.realtime_cadence_diagnostics,
        ));
        self.push_timing_diagnostic_text(format_realtime_imu_telemetry(
            self.realtime_cadence_diagnostics,
        ));
        self.push_timing_diagnostic_text(format_realtime_gap_diagnostics(
            self.realtime_cadence_diagnostics,
        ));
        self.realtime_cadence_diagnostics.reset_interval();
        self.timing_diagnostics.reset(now_us);
    }

    #[cfg(feature = "timing-diagnostics")]
    pub(super) fn push_timing_diagnostic_text(&mut self, text: String<50>) {
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

#[cfg(feature = "timing-diagnostics")]
fn format_realtime_control_cadence(diag: RealtimeCadenceDiagnostics) -> String<50> {
    let mut text = String::<50>::new();
    let _ = write!(
        text,
        "RTC d{} n{} r{} c{} i{} t{}",
        diag.control_due,
        diag.control_due_no_sample,
        diag.control_ran,
        diag.control_deadline_consumed,
        diag.imu_packet_taken,
        diag.imu_timestamp_changed,
    );
    text
}

#[cfg(feature = "timing-diagnostics")]
fn format_realtime_imu_telemetry(diag: RealtimeCadenceDiagnostics) -> String<50> {
    let mut text = String::<50>::new();
    let _ = write!(
        text,
        "RTI a{} ok{} nd{} st{} ni{}",
        diag.priority_imu_attempt,
        diag.priority_imu_sent,
        diag.priority_imu_not_due,
        diag.priority_imu_stale,
        diag.priority_imu_no_imu,
    );
    text
}

#[cfg(feature = "timing-diagnostics")]
fn format_realtime_gap_diagnostics(diag: RealtimeCadenceDiagnostics) -> String<50> {
    let mut text = String::<50>::new();
    let _ = write!(
        text,
        "RTG ig{} cg{} sg{}",
        diag.max_processed_imu_gap_us, diag.max_control_gap_us, diag.max_imu_telemetry_gap_us,
    );
    text
}
