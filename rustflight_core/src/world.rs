use core::marker::PhantomData;

use crate::{
    board::BoardIo,
    bodytype::BodyType,
    comm_manager::{CommManager, comm_link_trait::CommInterface},
    command_manager::CommandManager,
    command_system::{
        self, BoardCommandCtx, CalibrationRequestCtx, ConfigInfoCtx, OffboardControlCtx,
        ParamDefaultsCtx, ResetOriginCtx,
    },
    controller::Controller,
    events::{CommEventQueues, CommandEventQueues, ParamEventQueues},
    estimator::{AttitudeStateTrait, NamedEstimator},
    mixer::Mixer,
    param_reactions::{self, CommandParamChangedCtx, RcParamChangedCtx},
    param_system::{self, ParamApplyCtx, ParamListCtx, ParamListState, ParamReadCtx},
    params2::Params,
    ports::{EventDrainPort, EventEmitPort, EventReadPort, ParamsReadPort, ParamsWritePort},
    pwm::PwmDriver,
    pwm_system::{PwmOutputState, sync_pwm_output_state, write_pwm_commands},
    rc::Rc,
    sensor_systems::{SensorProcessorSet, process_sensor_bus},
    sensorprocessors::CalibrationFlags,
    sensors::{ProcessedSensors, SensorBus},
    state_machine::{ErrorFlag, Event, StateManager},
};

const IMU_TIMEOUT_US: u64 = 100_000;

pub struct World<B, BT, CI, PD>
where
    B: BoardIo,
    BT: BodyType,
    BT::Estimator: NamedEstimator,
    BT::Controller: Controller<State = <BT::Estimator as NamedEstimator>::State>,
    BT::Mixer: crate::mixer::Mixer<MixerInput = <BT::Controller as Controller>::ControlOutput>,
    <BT::Mixer as crate::mixer::Mixer>::ActuatorCommands: AsRef<[f64]> + Copy,
    <BT::Estimator as NamedEstimator>::State: Copy + Default,
    CI: CommInterface<B>,
    PD: PwmDriver,
{
    pub board: B,
    pub params: Params,
    pub param_list_state: ParamListState,
    pub param_events: ParamEventQueues,
    pub comm_events: CommEventQueues,
    pub command_events: CommandEventQueues,
    pub comm: CommManager<B, CI>,
    pub raw_sensors: SensorBus,
    pub processed_sensors: ProcessedSensors,
    pub sensor_processors: SensorProcessorSet,
    pub rc: Rc,
    pub command: CommandManager,
    pub state: StateManager,
    pub cal_flags: CalibrationFlags,
    pub estimator: BT::Estimator,
    pub controller: BT::Controller,
    pub mixer: BT::Mixer,
    pub latest_state: <BT::Estimator as NamedEstimator>::State,
    pub latest_actuator_commands: Option<<BT::Mixer as crate::mixer::Mixer>::ActuatorCommands>,
    pub pwm_output: PwmOutputState,
    pub pwm: PD,
    last_imu_time: u64,
    last_imu_seen: u64,
    _body_type: PhantomData<BT>,
}

impl<B, BT, CI, PD> World<B, BT, CI, PD>
where
    B: BoardIo,
    BT: BodyType,
    BT::Estimator: NamedEstimator,
    BT::Controller: Controller<State = <BT::Estimator as NamedEstimator>::State>,
    BT::Mixer: crate::mixer::Mixer<MixerInput = <BT::Controller as Controller>::ControlOutput>,
    <BT::Mixer as crate::mixer::Mixer>::ActuatorCommands: AsRef<[f64]> + Copy,
    <BT::Estimator as NamedEstimator>::State: Copy + Default,
    CI: CommInterface<B>,
    PD: PwmDriver,
{
    const ESTIMATOR_DT: f64 = 1.0 / 400.0;

    pub fn init(
        mut board: B,
        mut params: Params,
        comm_link: CI,
        mut state: StateManager,
        estimator: BT::Estimator,
        controller: BT::Controller,
        mixer: BT::Mixer,
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

        Self {
            board,
            params,
            param_list_state: ParamListState::default(),
            param_events: ParamEventQueues::default(),
            comm_events: CommEventQueues::default(),
            command_events: CommandEventQueues::default(),
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
            latest_state: Default::default(),
            latest_actuator_commands: None,
            pwm_output,
            pwm,
            last_imu_time: 0,
            last_imu_seen: now_us,
            _body_type: PhantomData,
        }
    }

    pub fn run_comm_param_sensor_stages(&mut self) -> bool {
        self.run_comm_param_sensor_stages_only();
        self.run_rc_command_state_stages();
        self.run_control_stages_if_new_imu();
        true
    }

    pub fn run_comm_param_sensor_stages_only(&mut self) {
        let now_us = self.board.clock_micros();

        self.comm.process_incoming_messages(&mut self.board);
        self.comm.act_on_messages(
            &mut self.param_events,
            &mut self.comm_events,
            &mut self.command_events,
            &mut self.board,
        );

        command_system::apply_calibration_requests(CalibrationRequestCtx {
            requests: EventDrainPort::new(&mut self.command_events.calibration_requests),
            flags: &mut self.cal_flags,
        });
        command_system::apply_offboard_control_requests(OffboardControlCtx {
            requests: EventDrainPort::new(&mut self.command_events.offboard_control_requests),
            command: &mut self.command,
            params: &self.params,
        });
        let applied_defaults = command_system::apply_param_defaults_requests(ParamDefaultsCtx {
            requests: EventDrainPort::new(&mut self.command_events.param_defaults_requests),
            params: &mut self.params,
        });
        self.comm
            .send_completed_param_defaults_ack(&mut self.board, applied_defaults);

        command_system::apply_rc_trim_calibration_requests(command_system::RcTrimCalibrationCtx {
            requests: EventDrainPort::new(&mut self.command_events.rc_trim_calibration_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            rc: &self.rc,
            params: &mut self.params,
        });

        command_system::apply_board_command_requests(BoardCommandCtx {
            requests: EventDrainPort::new(&mut self.command_events.board_command_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
            board: &mut self.board,
            params: &mut self.params,
        });

        command_system::apply_reset_origin_requests(ResetOriginCtx {
            requests: EventDrainPort::new(&mut self.command_events.reset_origin_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });

        command_system::apply_config_info_requests(ConfigInfoCtx {
            requests: EventDrainPort::new(&mut self.command_events.config_info_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });

        param_system::service_param_read_requests(ParamReadCtx {
            params: ParamsReadPort::new(&self.params),
            requests: EventDrainPort::new(&mut self.param_events.read_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });

        param_system::service_param_list_requests(ParamListCtx {
            params: ParamsReadPort::new(&self.params),
            state: &mut self.param_list_state,
            requests: EventDrainPort::new(&mut self.param_events.list_requests),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });

        param_system::apply_param_requests(ParamApplyCtx {
            params: ParamsWritePort::new(&mut self.params),
            requests: EventDrainPort::new(&mut self.param_events.set_requests),
            changes: EventEmitPort::new(&mut self.param_events.changes),
            responses: EventEmitPort::new(&mut self.comm_events.responses),
        });

        param_reactions::rc_on_param_changed(RcParamChangedCtx {
            rc: &mut self.rc,
            params: ParamsReadPort::new(&self.params),
            changes: EventReadPort::new(&self.param_events.changes),
        });

        param_reactions::command_on_param_changed(CommandParamChangedCtx {
            command: &mut self.command,
            state: &mut self.state,
            params: ParamsReadPort::new(&self.params),
            changes: EventReadPort::new(&self.param_events.changes),
        });

        self.comm
            .send_comm_responses(&mut self.board, &mut self.comm_events);
        self.param_events.changes.clear();

        if self.state.is_calibrating() && !self.cal_flags.contains(CalibrationFlags::GYRO) {
            self.cal_flags.insert(CalibrationFlags::GYRO);
        }

        self.board.update_sensor_bus(&mut self.raw_sensors);
        process_sensor_bus(
            &mut self.raw_sensors,
            &mut self.processed_sensors,
            &mut self.sensor_processors,
            &mut self.cal_flags,
            &mut self.params,
        );
        self.update_sensor_health_and_calibration(now_us);
    }

    fn update_sensor_health_and_calibration(&mut self, now_us: u64) {
        if self.processed_sensors.imu.is_some() {
            self.last_imu_seen = now_us;
            self.state.update(
                Event::ERROR_CLEARED(ErrorFlag::IMU_NOT_RESPONDING),
                &self.params,
            );
        } else if now_us > self.last_imu_seen + IMU_TIMEOUT_US {
            self.state.update(
                Event::ERROR_OCCURRED(ErrorFlag::IMU_NOT_RESPONDING),
                &self.params,
            );
        }

        if self.state.is_calibrating() && !self.cal_flags.contains(CalibrationFlags::GYRO) {
            self.state.update(Event::CALIBRATION_COMPLETE, &self.params);
        }
        self.comm
            .send_completed_calibration_ack(&mut self.board, self.cal_flags);
    }

    pub fn run_rc_command_state_stages(&mut self) {
        let now_ms = self.board.clock_millis();

        if let Some(rc_packet) = self.processed_sensors.rc {
            self.rc.receive(&rc_packet, &self.params, &mut self.state);
        }

        self.rc.run(now_ms, &self.params, &mut self.state);
        self.command
            .run(now_ms, &self.params, &mut self.rc, &mut self.state);
        self.state.run(&self.params);
        self.run_pwm_output_stage();
    }

    pub fn run_pwm_output_stage(&mut self) -> bool {
        sync_pwm_output_state(
            &mut self.board,
            &mut self.pwm,
            &mut self.pwm_output,
            &self.state,
        )
        .unwrap_or(false)
    }

    pub fn run_control_stages_if_new_imu(&mut self) -> bool {
        let Some(imu_packet) = self.processed_sensors.imu else {
            return false;
        };

        let current_time = imu_packet.header.timestamp;
        if current_time == self.last_imu_time {
            return false;
        }
        self.last_imu_time = current_time;

        let state =
            self.estimator
                .estimate_named(&self.processed_sensors, &self.params, Self::ESTIMATOR_DT);

        if state.is_healthy() {
            self.state.update(
                crate::state_machine::Event::ERROR_CLEARED(
                    crate::state_machine::ErrorFlag::UNHEALTHY_ESTIMATOR,
                ),
                &self.params,
            );
        } else {
            self.state.update(
                crate::state_machine::Event::ERROR_OCCURRED(
                    crate::state_machine::ErrorFlag::UNHEALTHY_ESTIMATOR,
                ),
                &self.params,
            );
        }

        let controls = self.controller.control(
            &state,
            &mut self.state,
            self.command.combined_control(),
            &self.params,
            Self::ESTIMATOR_DT,
        );
        let actuator_commands = self.mixer.mix(&controls, &self.state);
        write_pwm_commands(
            &mut self.board,
            &mut self.pwm,
            &self.pwm_output,
            actuator_commands.as_ref(),
        );
        let now_us = self.board.clock_micros();
        self.comm.send_named_telemetry_streams(
            &mut self.board,
            now_us,
            &self.state,
            &self.command,
            &state,
            &self.processed_sensors,
            &actuator_commands,
        );

        self.latest_state = state;
        self.latest_actuator_commands = Some(actuator_commands);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        bodytype::quadrotor::Quadrotor,
        comm_messages::{
            enums::{
                OffboardControlIgnore, OffboardControlMode, ParamIdentifier, RosflightCmd,
                RosflightCmdResponse,
            },
            messages::{
                OffboardControlMsg, ParamRequestListMsg, ParamRequestReadMsg, ParamSetMsg,
                RosflightCmdMsg,
            },
        },
        params2::{ParamId, ParamValue},
        pwm::{PwmDriver, PwmError},
        test_support::{RecordingCommLink, TestBoard},
    };

    pub struct TestPwm {
        enabled: bool,
        enable_all_count: usize,
        disable_all_count: usize,
        flush_count: usize,
        send_count: usize,
        last_commands: [f64; 8],
        last_command_len: usize,
    }

    impl TestPwm {
        fn new() -> Self {
            Self {
                enabled: false,
                enable_all_count: 0,
                disable_all_count: 0,
                flush_count: 0,
                send_count: 0,
                last_commands: [0.0; 8],
                last_command_len: 0,
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

        fn send_commands<Board: BoardIo>(&mut self, _board: &mut Board, commands: &[f64]) {
            self.send_count += 1;
            self.last_command_len = commands.len().min(self.last_commands.len());
            self.last_commands[..self.last_command_len]
                .copy_from_slice(&commands[..self.last_command_len]);
        }
    }

    #[test]
    fn world_scheduler_runs_deferred_param_pipeline() {
        let board = TestBoard::default();
        let params = Params::new();
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        world.comm.msgs.param_set = Some(ParamSetMsg {
            target_system: 1,
            target_component: 1,
            param_id: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
            param_value: ParamValue::Int(42),
        });

        assert!(world.run_comm_param_sensor_stages());

        assert_eq!(
            world.params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );
        assert_eq!(world.comm.sysid, 42);
    }

    #[test]
    fn world_scheduler_streams_param_request_list_through_param_system() {
        let board = TestBoard::default();
        let params = Params::new();
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        world.comm.msgs.param_request_list = Some(ParamRequestListMsg {
            target_system: 1,
            target_component: 1,
        });

        world.run_comm_param_sensor_stages_only();

        assert!(world.param_events.list_requests.is_empty());
        assert!(world.param_list_state.is_active());
        assert_eq!(world.comm.comm_link().sent_param_value_count, 1);
        let first = world.comm.comm_link().sent_param_values[0].unwrap();
        assert_eq!(first.param_index, ParamId::PARAM_BAUD_RATE as u16);
        assert_eq!(first.param_value, ParamValue::Int(921600));

        world.run_comm_param_sensor_stages_only();

        assert_eq!(world.comm.comm_link().sent_param_value_count, 2);
        let second = world.comm.comm_link().sent_param_values[1].unwrap();
        assert_eq!(second.param_index, ParamId::PARAM_SERIAL_DEVICE as u16);
    }

    #[test]
    fn world_scheduler_answers_param_request_read_through_param_system() {
        let board = TestBoard::default();
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        world.comm.msgs.param_request_read = Some(ParamRequestReadMsg {
            target_system: 1,
            target_component: 1,
            param_identifier: ParamIdentifier::ID(*b"SYS_ID\0\0\0\0\0\0\0\0\0\0"),
        });

        world.run_comm_param_sensor_stages_only();

        assert!(world.param_events.read_requests.is_empty());
        assert_eq!(world.comm.comm_link().sent_param_value_count, 1);
        let response = world.comm.comm_link().sent_param_values[0].unwrap();
        assert_eq!(response.param_index, ParamId::PARAM_SYSTEM_ID as u16);
        assert_eq!(response.param_value, ParamValue::Int(42));
    }

    #[test]
    fn world_scheduler_processes_named_rc_packet() {
        let board = TestBoard::default();
        let mut params = Params::new();
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

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

        assert!(!world.state.get_errors().contains(crate::state_machine::ErrorFlag::RC_LOST));
    }

    #[test]
    fn world_control_stage_runs_once_per_imu_timestamp() {
        let board = TestBoard::default();
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_FAILSAFE_THROTTLE, ParamValue::Float(0.0));
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        world
            .state
            .update(crate::state_machine::Event::REQUEST_ARM, &world.params);
        assert!(world.run_pwm_output_stage());

        world.board.current_time_us = 1_100_000;
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
        assert_eq!(world.pwm.send_count, 1);
        assert_eq!(world.comm.comm_link().heartbeat_count, 1);
        assert_eq!(world.comm.comm_link().status_count, 1);
        assert_eq!(world.comm.comm_link().imu_count, 1);
        assert_eq!(world.comm.comm_link().attitude_count, 1);
        assert_eq!(world.comm.comm_link().output_raw_count, 1);
        assert!(world.latest_actuator_commands.is_some());

        assert!(!world.run_control_stages_if_new_imu());
        assert_eq!(world.pwm.send_count, 1);
        assert_eq!(world.comm.comm_link().output_raw_count, 1);

        world.processed_sensors.imu.as_mut().unwrap().header.timestamp = 2;

        assert!(world.run_control_stages_if_new_imu());
        assert_eq!(world.pwm.send_count, 2);
        assert_eq!(world.comm.comm_link().output_raw_count, 2);
    }

    #[test]
    fn world_sensor_health_sets_and_clears_imu_timeout() {
        let mut board = TestBoard::default();
        board.current_time_us = 0;
        let params = Params::new();
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

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
    fn world_sends_calibration_ack_after_calibration_flag_clears() {
        let board = TestBoard::default();
        let params = Params::new();
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::GyroCalibration,
        });

        world.run_comm_param_sensor_stages_only();

        assert!(world.cal_flags.contains(CalibrationFlags::GYRO));
        assert_eq!(world.comm.comm_link().cmd_ack_count, 0);

        world.cal_flags.remove(CalibrationFlags::GYRO);
        world.update_sensor_health_and_calibration(world.board.clock_micros());

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
        let board = TestBoard::default();
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_FAILSAFE_THROTTLE, ParamValue::Float(0.0));
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

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
        let board = TestBoard::default();
        let params = Params::new();
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        world.comm.msgs.offboard_control = Some(OffboardControlMsg {
            mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
            ignore: OffboardControlIgnore::IGNORE_QY,
            qx: 0.1,
            qy: 0.2,
            qz: 0.3,
            fx: 0.4,
            fy: 0.5,
            fz: 0.6,
        });

        world.run_comm_param_sensor_stages_only();

        assert!(world.command.is_offboard_active());
        assert!(world.command_events.offboard_control_requests.is_empty());
    }

    #[test]
    fn world_applies_param_defaults_and_sends_ack_after_apply() {
        let board = TestBoard::default();
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::SetParamDefaults,
        });

        world.run_comm_param_sensor_stages_only();

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
        let board = TestBoard::default();
        let params = Params::new();
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::WriteParams,
        });

        world.run_comm_param_sensor_stages_only();

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
    fn world_routes_rc_trim_calibration_and_sets_equilibrium_torques() {
        let board = TestBoard::default();
        let params = Params::new();
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        let mut channels = [0.5; crate::packets::RC_PACKET_CHANNELS];
        channels[0] = 0.55;
        channels[1] = 0.45;
        channels[3] = 0.60;
        world.rc.receive(
            &crate::packets::RcPacket {
                header: crate::packets::RosflightPacketHeader {
                    timestamp: 1,
                    status: 0,
                },
                n_chan: 4,
                chan: channels,
                lol: false,
            },
            &world.params,
            &mut world.state,
        );

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::RcCalibration,
        });

        world.run_comm_param_sensor_stages_only();

        assert!(world.command_events.rc_trim_calibration_requests.is_empty());
        assert_eq!(
            world.params.get_by_id(ParamId::PARAM_X_EQ_TORQUE),
            ParamValue::Float(0.100000024)
        );
        assert_eq!(
            world.params.get_by_id(ParamId::PARAM_Y_EQ_TORQUE),
            ParamValue::Float(-0.100000024)
        );
        assert_eq!(
            world.params.get_by_id(ParamId::PARAM_Z_EQ_TORQUE),
            ParamValue::Float(0.20000005)
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
        let board = TestBoard::default();
        let params = Params::new();
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::ResetOrigin,
        });

        world.run_comm_param_sensor_stages_only();

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
        let board = TestBoard::default();
        let params = Params::new();
        let comm_link = RecordingCommLink::new();
        let state = StateManager::new();
        let mixer = <Quadrotor as BodyType>::Mixer::new(&params);

        let mut world = World::<TestBoard, Quadrotor, RecordingCommLink, TestPwm>::init(
            board,
            params,
            comm_link,
            state,
            Default::default(),
            Default::default(),
            mixer,
            TestPwm::new(),
        );

        world.comm.msgs.cmd = Some(RosflightCmdMsg {
            command: RosflightCmd::SendAllConfigInfos,
        });

        world.run_comm_param_sensor_stages_only();

        assert!(world.command_events.config_info_requests.is_empty());
        assert_eq!(world.comm.comm_link().cmd_ack_count, 1);
        let ack = world.comm.comm_link().last_cmd_ack.unwrap();
        assert!(matches!(ack.command, RosflightCmd::SendAllConfigInfos));
        assert!(matches!(
            ack.success,
            RosflightCmdResponse::RosflightCmdFailed
        ));
    }

}
