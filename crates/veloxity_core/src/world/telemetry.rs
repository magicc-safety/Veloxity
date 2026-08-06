use super::*;

#[cfg(feature = "runtime-diagnostics")]
use core::sync::atomic::{AtomicU32, Ordering};

#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_TELEMETRY_DRAIN_COUNT: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_TELEMETRY_DRAIN_MESSAGES: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_TELEMETRY_DRAIN_SUM_US: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "runtime-diagnostics")]
#[unsafe(no_mangle)]
pub static VELOXITY_DIAG_TELEMETRY_DRAIN_MAX_US: AtomicU32 = AtomicU32::new(0);

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
    /// p99/max timing with scope pins before making it a production default.
    pub fn run_realtime_telemetry_stage_budgeted(&mut self, max_streams: usize) -> usize {
        let mut sent = 0;
        while sent < max_streams && self.send_realtime_telemetry_stream() {
            sent += 1;
        }
        sent
    }

    /// Sends due streams until there is no work left or the measured control
    /// slack reaches the board's safety margin. The slack check occurs between
    /// individual streams, so this never turns an arbitrary stream count into
    /// a scheduling bottleneck.
    pub fn run_realtime_telemetry_stage_with_available_slack(&mut self) -> usize {
        #[cfg(feature = "runtime-diagnostics")]
        let started_us = self.board.clock_micros();
        let mut sent = 0;
        while self.realtime_service_can_continue() && self.send_realtime_telemetry_stream() {
            sent += 1;
        }
        #[cfg(feature = "runtime-diagnostics")]
        if sent != 0 {
            let elapsed_us = self
                .board
                .clock_micros()
                .saturating_sub(started_us)
                .min(u32::MAX as u64) as u32;
            VELOXITY_DIAG_TELEMETRY_DRAIN_COUNT.fetch_add(1, Ordering::Relaxed);
            VELOXITY_DIAG_TELEMETRY_DRAIN_MESSAGES.fetch_add(sent as u32, Ordering::Relaxed);
            VELOXITY_DIAG_TELEMETRY_DRAIN_SUM_US.fetch_add(elapsed_us, Ordering::Relaxed);
            VELOXITY_DIAG_TELEMETRY_DRAIN_MAX_US.fetch_max(elapsed_us, Ordering::Relaxed);
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
        let sensor_error_count = self.board.sensors_errors_count();
        self.comm.send_named_telemetry_stream_with_gate(
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
        )
    }
}
