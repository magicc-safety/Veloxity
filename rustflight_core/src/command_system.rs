use crate::{
    board::BoardIo,
    comm_messages::{
        enums::{RosflightCmd, RosflightCmdResponse},
        messages::RosflightCmdAckMsg,
    },
    command_manager::CommandManager,
    events::{
        BoardCommandRequested, COMM_RESPONSE_QUEUE_CAPACITY, CalibrationRequested, CommResponse,
        ConfigInfoRequested, OffboardControlRequested, ParamDefaultsRequested,
        RcTrimCalibrationRequested, ResetOriginRequested,
    },
    params2::{ParamId, ParamValue, Params},
    ports::{EventDrainPort, EventEmitPort},
    rc::{Rc, Stick},
    sensorprocessors::CalibrationFlags,
};

pub struct CalibrationRequestCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, CalibrationRequested, N>,
    pub flags: &'a mut CalibrationFlags,
}

pub fn apply_calibration_requests<const N: usize>(mut ctx: CalibrationRequestCtx<'_, N>) {
    while let Some(request) = ctx.requests.next() {
        match request.command {
            RosflightCmd::AccelCalibration => ctx.flags.insert(CalibrationFlags::ACCEL),
            RosflightCmd::GyroCalibration => ctx.flags.insert(CalibrationFlags::GYRO),
            RosflightCmd::BaroCalibration => ctx.flags.insert(CalibrationFlags::BARO),
            RosflightCmd::AirspeedCalibration => ctx.flags.insert(CalibrationFlags::PITOT),
            _ => {}
        }
    }
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
    pub params: &'a mut Params,
}

pub fn apply_param_defaults_requests<const N: usize>(
    mut ctx: ParamDefaultsCtx<'_, N>,
) -> Option<RosflightCmd> {
    let mut applied = None;
    while let Some(request) = ctx.requests.next() {
        ctx.params.set_defaults();
        applied = Some(request.command);
    }
    applied
}

pub struct BoardCommandCtx<'a, B, const N: usize>
where
    B: BoardIo,
{
    pub requests: EventDrainPort<'a, BoardCommandRequested, N>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
    pub board: &'a mut B,
    pub params: &'a mut Params,
}

pub fn apply_board_command_requests<B, const N: usize>(mut ctx: BoardCommandCtx<'_, B, N>)
where
    B: BoardIo,
{
    while let Some(request) = ctx.requests.next() {
        let completed = match request.command {
            RosflightCmd::ReadParams => ctx.board.read_params(ctx.params),
            RosflightCmd::WriteParams => ctx.board.write_params(ctx.params),
            RosflightCmd::Reboot => ctx.board.reboot(),
            RosflightCmd::RebootToBootloader => ctx.board.reboot_to_bootloader(),
            _ => false,
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

pub struct RcTrimCalibrationCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, RcTrimCalibrationRequested, N>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
    pub rc: &'a Rc,
    pub params: &'a mut Params,
}

pub fn apply_rc_trim_calibration_requests<const N: usize>(
    mut ctx: RcTrimCalibrationCtx<'_, N>,
) {
    while let Some(request) = ctx.requests.next() {
        let has_rc = ctx.rc.get_rc_struct().num_channels > 0;
        if has_rc {
            ctx.params.set_by_id(
                ParamId::PARAM_X_EQ_TORQUE,
                ParamValue::Float(ctx.rc.stick(Stick::X)),
            );
            ctx.params.set_by_id(
                ParamId::PARAM_Y_EQ_TORQUE,
                ParamValue::Float(ctx.rc.stick(Stick::Y)),
            );
            ctx.params.set_by_id(
                ParamId::PARAM_Z_EQ_TORQUE,
                ParamValue::Float(ctx.rc.stick(Stick::Z)),
            );
        }

        let success = if has_rc {
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
        comm_messages::{
            enums::{OffboardControlIgnore, OffboardControlMode},
            messages::OffboardControlMsg,
        },
        events::{
            BOARD_COMMAND_REQUEST_QUEUE_CAPACITY, CALIBRATION_REQUEST_QUEUE_CAPACITY,
            COMM_RESPONSE_QUEUE_CAPACITY, CONFIG_INFO_REQUEST_QUEUE_CAPACITY, EventQueue,
            OFFBOARD_CONTROL_REQUEST_QUEUE_CAPACITY, PARAM_DEFAULTS_REQUEST_QUEUE_CAPACITY,
            RC_TRIM_CALIBRATION_REQUEST_QUEUE_CAPACITY, RESET_ORIGIN_REQUEST_QUEUE_CAPACITY,
        },
        packets::{RcPacket, RosflightPacketHeader},
        test_support::TestBoard,
    };

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

        apply_calibration_requests(CalibrationRequestCtx {
            requests: EventDrainPort::new(&mut requests),
            flags: &mut flags,
        });

        assert!(flags.contains(CalibrationFlags::GYRO));
        assert!(flags.contains(CalibrationFlags::BARO));
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

        let applied = apply_param_defaults_requests(ParamDefaultsCtx {
            requests: EventDrainPort::new(&mut requests),
            params: &mut params,
        });

        assert!(matches!(applied, Some(RosflightCmd::SetParamDefaults)));
        assert_eq!(params.get_by_id(ParamId::PARAM_SYSTEM_ID), ParamValue::Int(1));
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
        let mut rc = Rc::new();
        let mut state = crate::state_machine::StateManager::new();
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
            rc: &rc,
            params: &mut params,
        });

        assert_eq!(
            params.get_by_id(ParamId::PARAM_X_EQ_TORQUE),
            ParamValue::Float(0.100000024)
        );
        assert_eq!(
            params.get_by_id(ParamId::PARAM_Y_EQ_TORQUE),
            ParamValue::Float(-0.100000024)
        );
        assert_eq!(
            params.get_by_id(ParamId::PARAM_Z_EQ_TORQUE),
            ParamValue::Float(0.20000005)
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
