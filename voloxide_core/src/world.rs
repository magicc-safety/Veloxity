use crate::{
    board::BoardIo,
    comm::messages::messages::RosflightHardErrorMsg,
    comm::{CommManager, interface::CommInterface},
    command::CommandManager,
    command::service::{
        self as command_service, BoardCommandCtx, CalibrationRequestCtx, ConfigInfoCtx,
        OffboardControlCtx, ParamDefaultsCtx, ResetOriginCtx, VersionRequestCtx,
    },
    companion::{
        self, AuxCommandCtx, AuxCommandState, CompanionHeartbeatCtx, CompanionLinkState,
        ExternalAttitudeCtx, ExternalAttitudeState,
    },
    control::{ControlPipelineCtx, ControlPipelineResource, run_control_pipeline_if_new_imu},
    controller::{Controller, RcTrimCalibrator},
    estimator::{AttitudeEstimate, Estimator},
    events::{CommEventQueues, CommandEventQueues, CompanionEventQueues, ParamEventQueues},
    log::drain::{self as log_drain, LogDrainCtx},
    mixer::Mixer,
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
    sensors::ingestion::{SensorProcessorSet, process_sensor_bus},
    sensors::processors::CalibrationFlags,
    sensors::{ProcessedSensors, SensorBus},
    state_machine::{ErrorFlag, Event, StateManager},
};

const IMU_TIMEOUT_US: u64 = 100_000;

pub struct World<B, E, C, M, CI, PD>
where
    B: BoardIo,
    E: Estimator,
    C: Controller<State = E::State> + RcTrimCalibrator,
    M: crate::mixer::Mixer<MixerInput = C::ControlOutput>,
    M::ActuatorCommands: AsRef<[f64]> + Copy,
    E::State: Copy + Default,
    CI: CommInterface<B>,
    PD: PwmDriver,
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
    raw_sensors: SensorBus,
    processed_sensors: ProcessedSensors,
    sensor_processors: SensorProcessorSet,
    rc: Rc,
    command: CommandManager,
    state: StateManager,
    cal_flags: CalibrationFlags,
    estimator: E,
    controller: C,
    mixer: M,
    control_pipeline: ControlPipelineResource<E::State, M::ActuatorCommands>,
    pwm_output: PwmOutputState,
    pwm: PD,
    last_imu_seen: u64,
}

impl<B, E, C, M, CI, PD> World<B, E, C, M, CI, PD>
where
    B: BoardIo,
    E: Estimator,
    C: Controller<State = E::State> + RcTrimCalibrator,
    M: crate::mixer::Mixer<MixerInput = C::ControlOutput>,
    M::ActuatorCommands: AsRef<[f64]> + Copy,
    E::State: Copy + Default,
    CI: CommInterface<B>,
    PD: PwmDriver,
{
    const ESTIMATOR_DT: f64 = 1.0 / 400.0;

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
        if pending_hard_error
            .map(|msg| msg.do_rearm != 0)
            .unwrap_or(false)
        {
            state.update(Event::REQUEST_ARM, &params);
        }

        Self {
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
        }
    }

    pub fn run_once(&mut self) -> bool {
        self.run_communication_and_parameter_service_stage();
        self.run_sensor_ingestion_and_health_stage();
        self.run_rc_command_state_stages();
        self.run_control_and_mixing_stage_if_new_imu();
        self.run_telemetry_stage();
        true
    }

    pub fn run_comm_param_sensor_stages(&mut self) {
        self.run_communication_and_parameter_service_stage();
        self.run_sensor_ingestion_and_health_stage();
    }

    pub fn run_communication_and_parameter_service_stage(&mut self) {
        self.process_comm_stage();
        self.apply_companion_events();
        self.apply_command_events();
        self.service_param_events();
        self.apply_param_reactions();
        self.request_gyro_calibration_if_needed();
    }

    pub fn run_sensor_ingestion_and_health_stage(&mut self) {
        let now_us = self.board.clock_micros();

        self.run_sensor_ingestion_stage();
        self.update_sensor_health_and_calibration(now_us);
        self.drain_logs_and_send_responses();
    }

    fn process_comm_stage(&mut self) {
        self.comm.process_incoming_messages(&mut self.board);
        self.comm.act_on_messages(
            &mut self.param_events,
            &mut self.comm_events,
            &mut self.command_events,
            &mut self.companion_events,
            &mut self.board,
        );
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
        let started_calibration =
            command_service::apply_calibration_requests(CalibrationRequestCtx {
                requests: EventDrainPort::new(&mut self.command_events.calibration_requests),
                responses: EventEmitPort::new(&mut self.comm_events.responses),
                state: &self.state,
                flags: &mut self.cal_flags,
                params: &mut self.params,
            });
        self.comm.set_pending_calibration_ack(started_calibration);
        command_service::apply_offboard_control_requests(OffboardControlCtx {
            requests: EventDrainPort::new(&mut self.command_events.offboard_control_requests),
            command: &mut self.command,
            state: &self.state,
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

        self.param_events.changes.clear();
    }

    fn request_gyro_calibration_if_needed(&mut self) {
        if self.state.is_calibrating() && !self.cal_flags.contains(CalibrationFlags::GYRO) {
            self.cal_flags.insert(CalibrationFlags::GYRO);
        }
    }

    fn run_sensor_ingestion_stage(&mut self) {
        self.board.update_sensor_bus(&mut self.raw_sensors);
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
        {
            self.estimator.reset_adaptive_bias();
        }
        if calibration_flags_before.contains(CalibrationFlags::ACCEL)
            && !self.cal_flags.contains(CalibrationFlags::ACCEL)
        {
            self.estimator.reset();
            self.control_pipeline = ControlPipelineResource::default();
        }
    }

    fn drain_logs_and_send_responses(&mut self) {
        log_drain::drain_logs_to_comm_responses(LogDrainCtx {
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            connected: self.companion_link.connected,
        });
        self.comm
            .send_comm_responses(&mut self.board, &mut self.comm_events);
        self.board.serial_flush();
        self.board.run_deferred_board_actions();
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

        let completed_arm_calibration =
            self.state.is_calibrating() && !self.cal_flags.contains(CalibrationFlags::GYRO);

        if completed_arm_calibration {
            self.state.update(Event::CALIBRATION_COMPLETE, &self.params);
        }
        self.comm
            .queue_completed_calibration_ack(&mut self.comm_events, self.cal_flags);
    }

    pub fn run_rc_command_state_stages(&mut self) {
        let now_ms = self.board.clock_millis();

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
            dt: Self::ESTIMATOR_DT,
        })
    }

    pub fn run_telemetry_stage(&mut self) {
        let now_us = self.board.clock_micros();
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
        packets::{ImuPacket, RC_PACKET_CHANNELS, RcPacket, RosflightPacketHeader},
        params::{ParamId, ParamValue},
        pwm::{PwmDriver, PwmError},
        test_support::{RecordingCommLink, TestBoard},
        vehicle::quadrotor,
    };

    #[derive(Default)]
    struct SensorStageBoard {
        current_time_us: u64,
        imu: Option<ImuPacket>,
        rc: Option<RcPacket>,
        update_count: usize,
    }

    impl BoardIo for SensorStageBoard {
        fn update_sensor_bus(&mut self, sensors: &mut SensorBus) {
            sensors.clear();
            self.update_count += 1;
            if let Some(imu) = self.imu.take() {
                sensors.imu = Some(Ok(imu));
            }
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

        fn clock_millis(&self) -> u32 {
            (self.current_time_us / 1000) as u32
        }

        fn clock_micros(&self) -> u64 {
            self.current_time_us
        }
    }

    #[derive(Default)]
    struct SensorStageCommLink;

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

    impl PwmDriver for TestPwm {
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
        quadrotor::Estimator,
        quadrotor::Controller,
        quadrotor::Mixer,
        RecordingCommLink,
        TestPwm,
    >;

    fn test_world_with_params(params: Params) -> TestWorld {
        let mixer = quadrotor::mixer(&params);

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

    fn armed_test_world_with_params(params: Params) -> TestWorld {
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
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

        world.comm.msgs.param_set = Some(ParamSetMsg {
            target_system: 1,
            target_component: 1,
            param_id: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
            param_value: ParamValue::Int(42),
        });

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
        };
        let state = StateManager::new();
        let mixer = quadrotor::mixer(&params);

        let mut world = World::<
            SensorStageBoard,
            quadrotor::Estimator,
            quadrotor::Controller,
            quadrotor::Mixer,
            SensorStageCommLink,
            TestPwm,
        >::init(
            board,
            params,
            SensorStageCommLink,
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

        assert!(!world.run_control_stages_if_new_imu());
        assert_eq!(world.pwm.send_count, 1);
        assert_eq!(world.comm.comm_link().output_raw_count, 1);

        world
            .processed_sensors
            .imu
            .as_mut()
            .unwrap()
            .header
            .timestamp = 2;

        assert!(world.run_control_stages_if_new_imu());
        assert_eq!(world.pwm.send_count, 2);
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

        assert!(world.run_control_stages_if_new_imu());

        world.processed_sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 10,
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

        assert!(world.run_control_stages_if_new_imu());

        assert_eq!(world.pwm.configure_count, 1);
        assert_eq!(world.pwm.last_rate_len, 10);
        assert_eq!(world.pwm.last_rates, [0.0; 10]);
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
    fn world_sends_calibration_ack_after_calibration_flag_clears() {
        let mut world = test_world();

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::GyroCalibration,
        });

        world.run_comm_param_sensor_stages();

        assert!(world.cal_flags.contains(CalibrationFlags::GYRO));
        assert_eq!(world.comm.comm_link().cmd_ack_count, 0);

        world.cal_flags.remove(CalibrationFlags::GYRO);
        world.update_sensor_health_and_calibration(world.board.clock_micros());

        assert_eq!(world.comm.comm_link().cmd_ack_count, 0);
        world
            .comm
            .send_comm_responses(&mut world.board, &mut world.comm_events);

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
