use crate::{
    board::BoardIo,
    comm_messages::{
        enums::{RosflightCmd, RosflightCmdResponse},
        messages::{RosflightCmdAckMsg, RosflightVersionMsg},
    },
    command_manager::CommandManager,
    controller::RcTrimCalibrator,
    events::{
        BoardCommandRequested, COMM_RESPONSE_QUEUE_CAPACITY, CalibrationRequested, CommResponse,
        ConfigInfoRequested, OffboardControlRequested, ParamDefaultsRequested,
        RcTrimCalibrationRequested, ResetOriginRequested, VersionRequested,
    },
    params2::{ParamId, ParamValue, Params},
    ports::{EventDrainPort, EventEmitPort},
    sensorprocessors::CalibrationFlags,
    state_machine::StateManager,
};

fn emit_cmd_ack<const N: usize>(
    responses: &mut EventEmitPort<'_, CommResponse, N>,
    command: RosflightCmd,
    success: RosflightCmdResponse,
) {
    let _ = responses.emit(CommResponse::CmdAck(RosflightCmdAckMsg {
        command,
        success,
    }));
}

pub struct CalibrationRequestCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, CalibrationRequested, N>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
    pub state: &'a StateManager,
    pub flags: &'a mut CalibrationFlags,
}

pub fn apply_calibration_requests<const N: usize>(
    mut ctx: CalibrationRequestCtx<'_, N>,
) -> Option<RosflightCmd> {
    let mut started = None;
    while let Some(request) = ctx.requests.next() {
        if ctx.state.is_armed() {
            emit_cmd_ack(
                &mut ctx.responses,
                request.command,
                RosflightCmdResponse::RosflightCmdFailed,
            );
            continue;
        }

        match request.command {
            RosflightCmd::AccelCalibration => ctx.flags.insert(CalibrationFlags::ACCEL),
            RosflightCmd::GyroCalibration => ctx.flags.insert(CalibrationFlags::GYRO),
            RosflightCmd::BaroCalibration => ctx.flags.insert(CalibrationFlags::BARO),
            RosflightCmd::AirspeedCalibration => ctx.flags.insert(CalibrationFlags::PITOT),
            _ => {}
        }
        started = Some(request.command);
    }
    started
}

pub struct OffboardControlCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, OffboardControlRequested, N>,
    pub command: &'a mut CommandManager,
    pub params: &'a Params,
}

pub fn apply_offboard_control_requests<const N: usize>(mut ctx: OffboardControlCtx<'_, N>) {
    while let Some(request) = ctx.requests.next() {
        ctx.command
            .set_new_offboard_command(request.now_us, &request.msg, ctx.params);
    }
}

pub struct ParamDefaultsCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, ParamDefaultsRequested, N>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
    pub state: &'a StateManager,
    pub params: &'a mut Params,
}

pub fn apply_param_defaults_requests<const N: usize>(mut ctx: ParamDefaultsCtx<'_, N>) {
    while let Some(request) = ctx.requests.next() {
        let success = if ctx.state.is_armed() {
            RosflightCmdResponse::RosflightCmdFailed
        } else {
            ctx.params.set_defaults();
            RosflightCmdResponse::RosflightCmdSuccess
        };
        emit_cmd_ack(&mut ctx.responses, request.command, success);
    }
}

pub struct BoardCommandCtx<'a, B, const N: usize>
where
    B: BoardIo,
{
    pub requests: EventDrainPort<'a, BoardCommandRequested, N>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
    pub state: &'a StateManager,
    pub board: &'a mut B,
    pub params: &'a mut Params,
}

pub fn apply_board_command_requests<B, const N: usize>(mut ctx: BoardCommandCtx<'_, B, N>)
where
    B: BoardIo,
{
    while let Some(request) = ctx.requests.next() {
        let completed = if ctx.state.is_armed() {
            false
        } else {
            match request.command {
                RosflightCmd::ReadParams => ctx.board.read_params(ctx.params),
                RosflightCmd::WriteParams => ctx.board.write_params(ctx.params),
                RosflightCmd::Reboot => ctx.board.reboot(),
                RosflightCmd::RebootToBootloader => ctx.board.reboot_to_bootloader(),
                _ => false,
            }
        };

        let success = if completed {
            RosflightCmdResponse::RosflightCmdSuccess
        } else {
            RosflightCmdResponse::RosflightCmdFailed
        };
        let _ = ctx.responses.emit(CommResponse::CmdAck(RosflightCmdAckMsg {
            command: request.command,
            success,
        }));
    }
}

pub struct RcTrimCalibrationCtx<'a, C, const N: usize>
where
    C: RcTrimCalibrator,
{
    pub requests: EventDrainPort<'a, RcTrimCalibrationRequested, N>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
    pub state: &'a StateManager,
    pub command: &'a CommandManager,
    pub controller: &'a mut C,
    pub params: &'a mut Params,
}

pub fn apply_rc_trim_calibration_requests<C, const N: usize>(
    mut ctx: RcTrimCalibrationCtx<'_, C, N>,
) where
    C: RcTrimCalibrator,
{
    while let Some(request) = ctx.requests.next() {
        if !ctx.state.is_armed() {
            let torques = ctx
                .controller
                .calculate_equilibrium_torques_from_rc(ctx.command.rc_control(), ctx.params);
            ctx.params.set_by_id(
                ParamId::PARAM_X_EQ_TORQUE,
                ParamValue::Float(param_float(ctx.params, ParamId::PARAM_X_EQ_TORQUE) + torques[0]),
            );
            ctx.params.set_by_id(
                ParamId::PARAM_Y_EQ_TORQUE,
                ParamValue::Float(param_float(ctx.params, ParamId::PARAM_Y_EQ_TORQUE) + torques[1]),
            );
            ctx.params.set_by_id(
                ParamId::PARAM_Z_EQ_TORQUE,
                ParamValue::Float(param_float(ctx.params, ParamId::PARAM_Z_EQ_TORQUE) + torques[2]),
            );
        }

        let success = if ctx.state.is_armed() {
            RosflightCmdResponse::RosflightCmdFailed
        } else {
            RosflightCmdResponse::RosflightCmdSuccess
        };
        let _ = ctx.responses.emit(CommResponse::CmdAck(RosflightCmdAckMsg {
            command: request.command,
            success,
        }));
    }
}

fn param_float(params: &Params, id: ParamId) -> f32 {
    match params.get_by_id(id) {
        ParamValue::Float(value) => value,
        _ => 0.0,
    }
}

pub struct VersionRequestCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, VersionRequested, N>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
    pub state: &'a StateManager,
}

pub fn apply_version_requests<const N: usize>(mut ctx: VersionRequestCtx<'_, N>) {
    while let Some(request) = ctx.requests.next() {
        if ctx.state.is_armed() {
            emit_cmd_ack(
                &mut ctx.responses,
                request.command,
                RosflightCmdResponse::RosflightCmdFailed,
            );
            continue;
        }

        let version_str = "RustFlight Alpha 0.1";
        let mut version_bytes = [0u8; 50];
        let len = version_str.len().min(version_bytes.len());
        version_bytes[..len].copy_from_slice(version_str.as_bytes());
        let _ = ctx.responses.emit(CommResponse::Version(RosflightVersionMsg {
            version: version_bytes,
        }));
        emit_cmd_ack(
            &mut ctx.responses,
            request.command,
            RosflightCmdResponse::RosflightCmdSuccess,
        );
    }
}

pub struct ResetOriginCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, ResetOriginRequested, N>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
}

pub fn apply_reset_origin_requests<const N: usize>(mut ctx: ResetOriginCtx<'_, N>) {
    while let Some(request) = ctx.requests.next() {
        let _ = ctx.responses.emit(CommResponse::CmdAck(RosflightCmdAckMsg {
            command: request.command,
            success: RosflightCmdResponse::RosflightCmdFailed,
        }));
    }
}

pub struct ConfigInfoCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, ConfigInfoRequested, N>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
}

pub fn apply_config_info_requests<const N: usize>(mut ctx: ConfigInfoCtx<'_, N>) {
    while let Some(request) = ctx.requests.next() {
        let _ = ctx.responses.emit(CommResponse::CmdAck(RosflightCmdAckMsg {
            command: request.command,
            success: RosflightCmdResponse::RosflightCmdFailed,
        }));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        controller::quad_controller::QuadController,
        comm_messages::{
            enums::{OffboardControlIgnore, OffboardControlMode},
            messages::OffboardControlMsg,
        },
        events::{
            BOARD_COMMAND_REQUEST_QUEUE_CAPACITY, CALIBRATION_REQUEST_QUEUE_CAPACITY,
            COMM_RESPONSE_QUEUE_CAPACITY, CONFIG_INFO_REQUEST_QUEUE_CAPACITY, EventQueue,
            OFFBOARD_CONTROL_REQUEST_QUEUE_CAPACITY, PARAM_DEFAULTS_REQUEST_QUEUE_CAPACITY,
            RC_TRIM_CALIBRATION_REQUEST_QUEUE_CAPACITY, RESET_ORIGIN_REQUEST_QUEUE_CAPACITY,
            VERSION_REQUEST_QUEUE_CAPACITY,
        },
        packets::{RcPacket, RosflightPacketHeader},
        rc::Rc,
        state_machine::{Event, StateManager},
        test_support::TestBoard,
    };

    fn initialized_state() -> StateManager {
        let params = Params::new();
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state
    }

    fn armed_state() -> StateManager {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        let mut state = initialized_state();
        state.update(Event::REQUEST_ARM, &params);
        assert!(state.is_armed());
        state
    }

    #[test]
    fn apply_calibration_requests_sets_requested_flags() {
        let mut requests =
            EventQueue::<CalibrationRequested, CALIBRATION_REQUEST_QUEUE_CAPACITY>::new();
        let mut flags = CalibrationFlags::empty();

        let _ = requests.push(CalibrationRequested {
            command: RosflightCmd::GyroCalibration,
        });
        let _ = requests.push(CalibrationRequested {
            command: RosflightCmd::BaroCalibration,
        });
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();
        let state = initialized_state();

        let started = apply_calibration_requests(CalibrationRequestCtx {
            requests: EventDrainPort::new(&mut requests),
            responses: EventEmitPort::new(&mut responses),
            state: &state,
            flags: &mut flags,
        });

        assert!(matches!(started, Some(RosflightCmd::BaroCalibration)));
        assert!(flags.contains(CalibrationFlags::GYRO));
        assert!(flags.contains(CalibrationFlags::BARO));
        assert!(responses.is_empty());
        assert!(requests.is_empty());
    }

    #[test]
    fn apply_offboard_control_requests_updates_command_manager() {
        let params = Params::new();
        let mut command = CommandManager::new();
        let mut requests =
            EventQueue::<OffboardControlRequested, OFFBOARD_CONTROL_REQUEST_QUEUE_CAPACITY>::new();

        let _ = requests.push(OffboardControlRequested {
            now_us: 42_000,
            msg: OffboardControlMsg {
                mode: OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
                ignore: OffboardControlIgnore::IGNORE_QY,
                qx: 0.1,
                qy: 0.2,
                qz: 0.3,
                fx: 0.4,
                fy: 0.5,
                fz: 0.6,
            },
        });

        apply_offboard_control_requests(OffboardControlCtx {
            requests: EventDrainPort::new(&mut requests),
            command: &mut command,
            params: &params,
        });

        assert!(command.is_offboard_active());
        assert!(requests.is_empty());
    }

    #[test]
    fn apply_param_defaults_requests_resets_params_and_reports_command() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let mut requests =
            EventQueue::<ParamDefaultsRequested, PARAM_DEFAULTS_REQUEST_QUEUE_CAPACITY>::new();

        let _ = requests.push(ParamDefaultsRequested {
            command: RosflightCmd::SetParamDefaults,
        });
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();
        let state = initialized_state();

        apply_param_defaults_requests(ParamDefaultsCtx {
            requests: EventDrainPort::new(&mut requests),
            responses: EventEmitPort::new(&mut responses),
            state: &state,
            params: &mut params,
        });

        assert_eq!(params.get_by_id(ParamId::PARAM_SYSTEM_ID), ParamValue::Int(1));
        match responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::SetParamDefaults));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdSuccess
                ));
            }
            _ => panic!("expected command ack response"),
        }
        assert!(requests.is_empty());
    }

    #[test]
    fn apply_board_command_requests_reports_unsupported_as_failed_ack() {
        let mut board = TestBoard::default();
        let mut params = Params::new();
        let mut requests =
            EventQueue::<BoardCommandRequested, BOARD_COMMAND_REQUEST_QUEUE_CAPACITY>::new();
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();

        let _ = requests.push(BoardCommandRequested {
            command: RosflightCmd::WriteParams,
        });

        apply_board_command_requests(BoardCommandCtx {
            requests: EventDrainPort::new(&mut requests),
            responses: EventEmitPort::new(&mut responses),
            state: &initialized_state(),
            board: &mut board,
            params: &mut params,
        });

        match responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::WriteParams));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdFailed
                ));
            }
            _ => panic!("expected command ack response"),
        }
        assert!(requests.is_empty());
    }

    #[test]
    fn apply_rc_trim_calibration_requests_sets_equilibrium_torques_and_acks() {
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
        let mut rc = Rc::new();
        let mut state = crate::state_machine::StateManager::new();
        state.update(Event::INITIALIZED, &params);
        rc.init(&mut (), &params);
        let mut channels = [0.5; crate::packets::RC_PACKET_CHANNELS];
        channels[0] = 0.55;
        channels[1] = 0.45;
        channels[3] = 0.60;
        rc.receive(
            &RcPacket {
                header: RosflightPacketHeader {
                    timestamp: 1,
                    status: 0,
                },
                n_chan: 4,
                chan: channels,
                lol: false,
            },
            &params,
            &mut state,
        );
        let mut command = CommandManager::new();
        command.run(0, &params, &mut rc, &mut state);
        let mut controller = QuadController::default();
        let mut requests = EventQueue::<
            RcTrimCalibrationRequested,
            RC_TRIM_CALIBRATION_REQUEST_QUEUE_CAPACITY,
        >::new();
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();

        let _ = requests.push(RcTrimCalibrationRequested {
            command: RosflightCmd::RcCalibration,
        });

        apply_rc_trim_calibration_requests(RcTrimCalibrationCtx {
            requests: EventDrainPort::new(&mut requests),
            responses: EventEmitPort::new(&mut responses),
            state: &initialized_state(),
            command: &command,
            controller: &mut controller,
            params: &mut params,
        });

        assert_eq!(
            params.get_by_id(ParamId::PARAM_X_EQ_TORQUE),
            ParamValue::Float(0.70000005)
        );
        assert_eq!(
            params.get_by_id(ParamId::PARAM_Y_EQ_TORQUE),
            ParamValue::Float(-0.8000001)
        );
        assert_eq!(
            params.get_by_id(ParamId::PARAM_Z_EQ_TORQUE),
            ParamValue::Float(1.0500002)
        );

        match responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::RcCalibration));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdSuccess
                ));
            }
            _ => panic!("expected command ack response"),
        }
        assert!(requests.is_empty());
    }

    #[test]
    fn command_requests_fail_without_mutation_when_armed() {
        let armed = armed_state();
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();
        let mut flags = CalibrationFlags::empty();
        let mut calibration_requests =
            EventQueue::<CalibrationRequested, CALIBRATION_REQUEST_QUEUE_CAPACITY>::new();
        let _ = calibration_requests.push(CalibrationRequested {
            command: RosflightCmd::GyroCalibration,
        });

        let started = apply_calibration_requests(CalibrationRequestCtx {
            requests: EventDrainPort::new(&mut calibration_requests),
            responses: EventEmitPort::new(&mut responses),
            state: &armed,
            flags: &mut flags,
        });

        assert!(started.is_none());
        assert!(!flags.contains(CalibrationFlags::GYRO));
        match responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::GyroCalibration));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdFailed
                ));
            }
            _ => panic!("expected command ack response"),
        }

        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let mut default_requests =
            EventQueue::<ParamDefaultsRequested, PARAM_DEFAULTS_REQUEST_QUEUE_CAPACITY>::new();
        let _ = default_requests.push(ParamDefaultsRequested {
            command: RosflightCmd::SetParamDefaults,
        });

        apply_param_defaults_requests(ParamDefaultsCtx {
            requests: EventDrainPort::new(&mut default_requests),
            responses: EventEmitPort::new(&mut responses),
            state: &armed,
            params: &mut params,
        });

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );
        match responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::SetParamDefaults));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdFailed
                ));
            }
            _ => panic!("expected command ack response"),
        }
    }

    #[test]
    fn apply_version_requests_sends_version_only_when_disarmed() {
        let state = initialized_state();
        let mut requests = EventQueue::<VersionRequested, VERSION_REQUEST_QUEUE_CAPACITY>::new();
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();

        let _ = requests.push(VersionRequested {
            command: RosflightCmd::SendVersion,
        });

        apply_version_requests(VersionRequestCtx {
            requests: EventDrainPort::new(&mut requests),
            responses: EventEmitPort::new(&mut responses),
            state: &state,
        });

        assert!(matches!(responses.pop(), Some(CommResponse::Version(_))));
        match responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::SendVersion));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdSuccess
                ));
            }
            _ => panic!("expected command ack response"),
        }

        let armed = armed_state();
        let mut requests = EventQueue::<VersionRequested, VERSION_REQUEST_QUEUE_CAPACITY>::new();
        let _ = requests.push(VersionRequested {
            command: RosflightCmd::SendVersion,
        });

        apply_version_requests(VersionRequestCtx {
            requests: EventDrainPort::new(&mut requests),
            responses: EventEmitPort::new(&mut responses),
            state: &armed,
        });

        match responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::SendVersion));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdFailed
                ));
            }
            _ => panic!("expected command ack response"),
        }
        assert!(responses.is_empty());
    }

    #[test]
    fn apply_reset_origin_requests_reports_unsupported_as_failed_ack() {
        let mut requests =
            EventQueue::<ResetOriginRequested, RESET_ORIGIN_REQUEST_QUEUE_CAPACITY>::new();
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();

        let _ = requests.push(ResetOriginRequested {
            command: RosflightCmd::ResetOrigin,
        });

        apply_reset_origin_requests(ResetOriginCtx {
            requests: EventDrainPort::new(&mut requests),
            responses: EventEmitPort::new(&mut responses),
        });

        match responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::ResetOrigin));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdFailed
                ));
            }
            _ => panic!("expected command ack response"),
        }
        assert!(requests.is_empty());
    }

    #[test]
    fn apply_config_info_requests_reports_unsupported_as_failed_ack() {
        let mut requests =
            EventQueue::<ConfigInfoRequested, CONFIG_INFO_REQUEST_QUEUE_CAPACITY>::new();
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();

        let _ = requests.push(ConfigInfoRequested {
            command: RosflightCmd::SendAllConfigInfos,
        });

        apply_config_info_requests(ConfigInfoCtx {
            requests: EventDrainPort::new(&mut requests),
            responses: EventEmitPort::new(&mut responses),
        });

        match responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::SendAllConfigInfos));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdFailed
                ));
            }
            _ => panic!("expected command ack response"),
        }
        assert!(requests.is_empty());
    }
}
