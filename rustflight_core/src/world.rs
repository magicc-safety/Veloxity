use core::marker::PhantomData;

use crate::{
    board::BoardIo,
    bodytype::BodyType,
    comm_manager::{CommManager, comm_link_trait::CommInterface},
    command_manager::CommandManager,
    controller::Controller,
    events::ParamEventQueues,
    estimator::{AttitudeStateTrait, NamedEstimator},
    mixer::Mixer,
    param_reactions::{self, CommandParamChangedCtx, RcParamChangedCtx},
    param_system::{self, ParamApplyCtx},
    params2::{ParamIter, Params},
    ports::{EventDrainPort, EventEmitPort, EventReadPort, ParamsReadPort, ParamsWritePort},
    pwm::PwmDriver,
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
    pub params_iter: Option<ParamIter>,
    pub param_events: ParamEventQueues,
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

        Self {
            board,
            params,
            params_iter: None,
            param_events: ParamEventQueues::default(),
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
            &mut self.params_iter,
            &mut self.params,
            &mut self.param_events,
            &mut self.cal_flags,
            &mut self.board,
            &mut self.command,
        );

        param_system::apply_param_requests(ParamApplyCtx {
            params: ParamsWritePort::new(&mut self.params),
            requests: EventDrainPort::new(&mut self.param_events.set_requests),
            changes: EventEmitPort::new(&mut self.param_events.changes),
            responses: EventEmitPort::new(&mut self.param_events.comm_responses),
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
            .send_comm_responses(&mut self.board, &mut self.param_events);
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
        self.pwm
            .send_commands(&mut self.board, actuator_commands.as_ref());
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
        comm_messages::messages::ParamSetMsg,
        params2::{ParamId, ParamValue},
        pwm::{PwmDriver, PwmError},
        test_support::{RecordingCommLink, TestBoard},
    };

    pub struct TestPwm {
        enabled: bool,
        send_count: usize,
        last_commands: [f64; 8],
        last_command_len: usize,
    }

    impl TestPwm {
        fn new() -> Self {
            Self {
                enabled: false,
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
            Ok(())
        }

        fn disable_all(&mut self) {
            self.enabled = false;
        }

        fn set_duty_cycle(&mut self, _channel: usize, _duty: u16) -> Result<(), PwmError> {
            Ok(())
        }

        fn flush<Board: BoardIo>(&mut self, _board: &mut Board) {}

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
}
