use core::marker::PhantomData;

use crate::{
    board::BoardTrait,
    bodytype::BodyType,
    comm_manager::{CommManager, comm_link_trait::CommInterface},
    command_manager::CommandManager,
    events::ParamEventQueues,
    param_reactions::{self, CommandParamChangedCtx, RcParamChangedCtx},
    param_system::{self, ParamApplyCtx},
    params2::{ParamIter, Params},
    ports::{EventDrainPort, EventEmitPort, EventReadPort, ParamsReadPort, ParamsWritePort},
    pwm::PwmDriver,
    rc::Rc,
    sensor_systems::{SensorProcessorSet, process_sensor_bus},
    sensorprocessors::CalibrationFlags,
    sensors::{ProcessedSensors, SensorBus},
    state_machine::{Event, StateManager},
};

pub struct World<B, BT, CI, PD>
where
    B: BoardTrait,
    BT: BodyType,
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
    pub pwm: PD,
    _body_type: PhantomData<BT>,
}

impl<B, BT, CI, PD> World<B, BT, CI, PD>
where
    B: BoardTrait,
    BT: BodyType,
    CI: CommInterface<B>,
    PD: PwmDriver,
{
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
            pwm,
            _body_type: PhantomData,
        }
    }

    pub fn run_comm_param_sensor_stages(&mut self) -> bool {
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

        self.board.update_sensor_bus(&mut self.raw_sensors);
        process_sensor_bus(
            &mut self.raw_sensors,
            &mut self.processed_sensors,
            &mut self.sensor_processors,
            &mut self.cal_flags,
            &mut self.params,
        );

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
    }

    impl TestPwm {
        fn new() -> Self {
            Self { enabled: false }
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

        fn flush<Board: BoardTrait>(&mut self, _board: &mut Board) {}

        fn send_commands<Board: BoardTrait>(&mut self, _board: &mut Board, _commands: &[f64]) {}
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
}
