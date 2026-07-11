use crate::{
    board::BoardIo,
    comm::messages::{
        enums::{RosflightCmd, RosflightCmdResponse},
        messages::{RosflightCmdAckMsg, RosflightVersionMsg},
    },
    command::CommandManager,
    controller::RcTrimCalibrator,
    events::{CommEventQueues, CommResponse, CommandEventQueues, ParamEventQueues},
    params::service::{mark_all_params_changed, set_param_and_emit_change},
    params::{ParamId, ParamValue, Params},
    sensors::processors::CalibrationFlags,
    state_machine::StateManager,
};

fn emit_cmd_ack(
    comm_events: &mut CommEventQueues,
    command: RosflightCmd,
    success: RosflightCmdResponse,
) {
    comm_events.responses.push_or_log(
        CommResponse::CmdAck(RosflightCmdAckMsg { command, success }),
        "command ack response",
    );
}

pub struct CommandRequestCtx<'a, B, C>
where
    B: BoardIo,
    C: RcTrimCalibrator,
{
    pub requests: &'a mut CommandEventQueues,
    pub param_events: &'a mut ParamEventQueues,
    pub comm_events: &'a mut CommEventQueues,
    pub state: &'a StateManager,
    pub command: &'a mut CommandManager,
    pub controller: &'a mut C,
    pub board: &'a mut B,
    pub flags: &'a mut CalibrationFlags,
    pub params: &'a mut Params,
}

pub fn apply_command_requests<B, C>(ctx: &mut CommandRequestCtx<'_, B, C>)
where
    B: BoardIo,
    C: RcTrimCalibrator,
{
    apply_calibration_requests(ctx);
    apply_offboard_control_requests(ctx);
    apply_param_defaults_requests(ctx);
    apply_rc_trim_calibration_requests(ctx);
    apply_board_command_requests(ctx);
    apply_version_requests(ctx);
    apply_reset_origin_requests(ctx);
    apply_config_info_requests(ctx);
}

pub fn apply_calibration_requests<B, C>(ctx: &mut CommandRequestCtx<'_, B, C>)
where
    B: BoardIo,
    C: RcTrimCalibrator,
{
    while let Some(request) = ctx.requests.calibration_requests.pop() {
        if ctx.state.is_armed() {
            emit_cmd_ack(
                ctx.comm_events,
                request.command,
                RosflightCmdResponse::RosflightCmdFailed,
            );
            continue;
        }

        match request.command {
            RosflightCmd::AccelCalibration => {
                ctx.flags
                    .remove(CalibrationFlags::GYRO_FAILED | CalibrationFlags::ACCEL_FAILED);
                ctx.flags.insert(CalibrationFlags::IMU);
                zero_gyro_biases(ctx.params, ctx.param_events);
                zero_accel_biases(ctx.params, ctx.param_events);
            }
            RosflightCmd::GyroCalibration => {
                ctx.flags.remove(CalibrationFlags::GYRO_FAILED);
                ctx.flags.insert(CalibrationFlags::GYRO);
                zero_gyro_biases(ctx.params, ctx.param_events);
            }
            RosflightCmd::BaroCalibration => {
                ctx.flags.remove(CalibrationFlags::BARO_FAILED);
                ctx.flags.insert(CalibrationFlags::BARO);
            }
            RosflightCmd::AirspeedCalibration => {
                ctx.flags.remove(CalibrationFlags::PITOT_FAILED);
                ctx.flags.insert(CalibrationFlags::PITOT);
                set_param_and_emit_change(
                    ctx.params,
                    &mut ctx.param_events.changes,
                    ParamId::PARAM_DIFF_PRESS_BIAS,
                    ParamValue::Float(0.0),
                );
            }
            _ => {}
        }
        emit_cmd_ack(
            ctx.comm_events,
            request.command,
            RosflightCmdResponse::RosflightCmdSuccess,
        );
    }
}

fn zero_gyro_biases(params: &mut Params, events: &mut ParamEventQueues) {
    set_param_and_emit_change(
        params,
        &mut events.changes,
        ParamId::PARAM_GYRO_X_BIAS,
        ParamValue::Float(0.0),
    );
    set_param_and_emit_change(
        params,
        &mut events.changes,
        ParamId::PARAM_GYRO_Y_BIAS,
        ParamValue::Float(0.0),
    );
    set_param_and_emit_change(
        params,
        &mut events.changes,
        ParamId::PARAM_GYRO_Z_BIAS,
        ParamValue::Float(0.0),
    );
}

fn zero_accel_biases(params: &mut Params, events: &mut ParamEventQueues) {
    set_param_and_emit_change(
        params,
        &mut events.changes,
        ParamId::PARAM_ACC_X_BIAS,
        ParamValue::Float(0.0),
    );
    set_param_and_emit_change(
        params,
        &mut events.changes,
        ParamId::PARAM_ACC_Y_BIAS,
        ParamValue::Float(0.0),
    );
    set_param_and_emit_change(
        params,
        &mut events.changes,
        ParamId::PARAM_ACC_Z_BIAS,
        ParamValue::Float(0.0),
    );
}

pub fn apply_offboard_control_requests<B, C>(ctx: &mut CommandRequestCtx<'_, B, C>)
where
    B: BoardIo,
    C: RcTrimCalibrator,
{
    while let Some(request) = ctx.requests.offboard_control_requests.pop() {
        ctx.command
            .set_new_offboard_command(request.now_us, &request.msg, ctx.params);
    }
}

pub fn apply_param_defaults_requests<B, C>(ctx: &mut CommandRequestCtx<'_, B, C>)
where
    B: BoardIo,
    C: RcTrimCalibrator,
{
    while let Some(request) = ctx.requests.param_defaults_requests.pop() {
        let success = if ctx.state.is_armed() {
            RosflightCmdResponse::RosflightCmdFailed
        } else {
            ctx.params.set_defaults();
            mark_all_params_changed(ctx.param_events);
            RosflightCmdResponse::RosflightCmdSuccess
        };
        emit_cmd_ack(ctx.comm_events, request.command, success);
    }
}

pub fn apply_board_command_requests<B, C>(ctx: &mut CommandRequestCtx<'_, B, C>)
where
    B: BoardIo,
    C: RcTrimCalibrator,
{
    while let Some(request) = ctx.requests.board_command_requests.pop() {
        if !ctx.state.is_armed()
            && matches!(
                request.command,
                RosflightCmd::Reboot | RosflightCmd::RebootToBootloader
            )
        {
            emit_cmd_ack(
                ctx.comm_events,
                request.command,
                RosflightCmdResponse::RosflightCmdSuccess,
            );
            match request.command {
                RosflightCmd::Reboot => {
                    let _ = ctx.board.reboot();
                }
                RosflightCmd::RebootToBootloader => {
                    let _ = ctx.board.reboot_to_bootloader();
                }
                _ => {}
            }
            continue;
        }

        let completed = if ctx.state.is_armed() {
            false
        } else {
            match request.command {
                RosflightCmd::ReadParams => ctx.board.read_params(ctx.params),
                RosflightCmd::WriteParams => ctx.board.write_params(ctx.params),
                _ => false,
            }
        };
        if completed && matches!(request.command, RosflightCmd::ReadParams) {
            mark_all_params_changed(ctx.param_events);
        }

        let success = if completed {
            RosflightCmdResponse::RosflightCmdSuccess
        } else {
            RosflightCmdResponse::RosflightCmdFailed
        };
        ctx.comm_events.responses.push_or_log(
            CommResponse::CmdAck(RosflightCmdAckMsg {
                command: request.command,
                success,
            }),
            "board command ack response",
        );
    }
}

pub fn apply_rc_trim_calibration_requests<B, C>(ctx: &mut CommandRequestCtx<'_, B, C>)
where
    B: BoardIo,
    C: RcTrimCalibrator,
{
    while let Some(request) = ctx.requests.rc_trim_calibration_requests.pop() {
        if !ctx.state.is_armed() {
            let torques = ctx
                .controller
                .calculate_equilibrium_torques_from_rc(ctx.command.rc_control(), ctx.params);
            set_param_and_emit_change(
                ctx.params,
                &mut ctx.param_events.changes,
                ParamId::PARAM_X_EQ_TORQUE,
                ParamValue::Float(param_float(ctx.params, ParamId::PARAM_X_EQ_TORQUE) + torques[0]),
            );
            set_param_and_emit_change(
                ctx.params,
                &mut ctx.param_events.changes,
                ParamId::PARAM_Y_EQ_TORQUE,
                ParamValue::Float(param_float(ctx.params, ParamId::PARAM_Y_EQ_TORQUE) + torques[1]),
            );
            set_param_and_emit_change(
                ctx.params,
                &mut ctx.param_events.changes,
                ParamId::PARAM_Z_EQ_TORQUE,
                ParamValue::Float(param_float(ctx.params, ParamId::PARAM_Z_EQ_TORQUE) + torques[2]),
            );
        }

        let success = if ctx.state.is_armed() {
            RosflightCmdResponse::RosflightCmdFailed
        } else {
            RosflightCmdResponse::RosflightCmdSuccess
        };
        ctx.comm_events.responses.push_or_log(
            CommResponse::CmdAck(RosflightCmdAckMsg {
                command: request.command,
                success,
            }),
            "rc trim ack response",
        );
    }
}

fn param_float(params: &Params, id: ParamId) -> f32 {
    match params.get_by_id(id) {
        ParamValue::Float(value) => value,
        _ => 0.0,
    }
}

pub fn apply_version_requests<B, C>(ctx: &mut CommandRequestCtx<'_, B, C>)
where
    B: BoardIo,
    C: RcTrimCalibrator,
{
    while let Some(request) = ctx.requests.version_requests.pop() {
        if ctx.state.is_armed() {
            emit_cmd_ack(
                ctx.comm_events,
                request.command,
                RosflightCmdResponse::RosflightCmdFailed,
            );
            continue;
        }

        let version_str = "Veloxity 1.0";
        let mut version_bytes = [0u8; 50];
        let len = version_str.len().min(version_bytes.len());
        version_bytes[..len].copy_from_slice(version_str.as_bytes());
        ctx.comm_events.responses.push_or_log(
            CommResponse::Version(RosflightVersionMsg {
                version: version_bytes,
            }),
            "version response",
        );
        emit_cmd_ack(
            ctx.comm_events,
            request.command,
            RosflightCmdResponse::RosflightCmdSuccess,
        );
    }
}

pub fn apply_reset_origin_requests<B, C>(ctx: &mut CommandRequestCtx<'_, B, C>)
where
    B: BoardIo,
    C: RcTrimCalibrator,
{
    while let Some(request) = ctx.requests.reset_origin_requests.pop() {
        ctx.comm_events.responses.push_or_log(
            CommResponse::CmdAck(RosflightCmdAckMsg {
                command: request.command,
                success: RosflightCmdResponse::RosflightCmdFailed,
            }),
            "reset origin ack response",
        );
    }
}

pub fn apply_config_info_requests<B, C>(ctx: &mut CommandRequestCtx<'_, B, C>)
where
    B: BoardIo,
    C: RcTrimCalibrator,
{
    while let Some(request) = ctx.requests.config_info_requests.pop() {
        ctx.comm_events.responses.push_or_log(
            CommResponse::CmdAck(RosflightCmdAckMsg {
                command: request.command,
                success: RosflightCmdResponse::RosflightCmdFailed,
            }),
            "config info ack response",
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        board::BoardIo,
        comm::messages::{
            enums::{OffboardControlIgnore, OffboardControlMode},
            messages::OffboardControlMsg,
        },
        controller::quad::QuadController,
        errors,
        events::{
            BoardCommandRequested, CalibrationRequested, CommEventQueues, CommandEventQueues,
            ConfigInfoRequested, OffboardControlRequested, ParamDefaultsRequested,
            ParamEventQueues, RcTrimCalibrationRequested, ResetOriginRequested, VersionRequested,
        },
        packets::{RcPacket, RosflightPacketHeader},
        rc::Rc,
        state_machine::{Event, StateManager},
        test_support::TestBoard,
    };

    #[derive(Default)]
    struct PersistBoard {
        read_count: usize,
        write_count: usize,
        stored_system_id: i32,
    }

    impl BoardIo for PersistBoard {
        fn serial_rx_read(&mut self, _buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
            None
        }

        fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
            Some(Ok(bytes.len()))
        }

        fn clock_millis(&self) -> u32 {
            0
        }

        fn clock_micros(&self) -> u64 {
            0
        }

        fn read_params(&mut self, params: &mut Params) -> bool {
            self.read_count += 1;
            params.set_by_id(
                ParamId::PARAM_SYSTEM_ID,
                ParamValue::Int(self.stored_system_id),
            );
            true
        }

        fn write_params(&mut self, params: &Params) -> bool {
            self.write_count += 1;
            self.stored_system_id = match params.get_by_id(ParamId::PARAM_SYSTEM_ID) {
                ParamValue::Int(value) => value,
                _ => 0,
            };
            true
        }
    }

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
        state.update_arming_safety(true, true);
        state.update(Event::REQUEST_ARM, &params);
        assert!(state.is_armed());
        state
    }

    fn test_ctx<'a, B, C>(
        requests: &'a mut CommandEventQueues,
        param_events: &'a mut ParamEventQueues,
        comm_events: &'a mut CommEventQueues,
        state: &'a StateManager,
        command: &'a mut CommandManager,
        controller: &'a mut C,
        board: &'a mut B,
        flags: &'a mut CalibrationFlags,
        params: &'a mut Params,
    ) -> CommandRequestCtx<'a, B, C>
    where
        B: BoardIo,
        C: RcTrimCalibrator,
    {
        CommandRequestCtx {
            requests,
            param_events,
            comm_events,
            state,
            command,
            controller,
            board,
            flags,
            params,
        }
    }

    #[test]
    fn apply_calibration_requests_sets_requested_flags() {
        let mut requests = CommandEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut param_events = ParamEventQueues::default();
        let mut flags = CalibrationFlags::empty();
        let mut params = Params::new();
        let mut command = CommandManager::new();
        let mut controller = QuadController::<f64>::default();
        let mut board = TestBoard::default();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.4));
        params.set_by_id(ParamId::PARAM_BARO_BIAS, ParamValue::Float(1000.0));

        let _ = requests.calibration_requests.push(CalibrationRequested {
            command: RosflightCmd::GyroCalibration,
        });
        let _ = requests.calibration_requests.push(CalibrationRequested {
            command: RosflightCmd::BaroCalibration,
        });
        let state = initialized_state();

        apply_calibration_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &state,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

        assert!(flags.contains(CalibrationFlags::GYRO));
        assert!(flags.contains(CalibrationFlags::BARO));
        assert_eq!(
            params.get_by_id(ParamId::PARAM_GYRO_X_BIAS),
            ParamValue::Float(0.0)
        );
        // Barometer calibration updates BARO_BIAS only after its sample
        // window finishes; the command acknowledgement merely accepts it.
        assert_eq!(
            params.get_by_id(ParamId::PARAM_BARO_BIAS),
            ParamValue::Float(1000.0)
        );
        assert_eq!(comm_events.responses.len(), 2);
        for expected in [RosflightCmd::GyroCalibration, RosflightCmd::BaroCalibration] {
            match comm_events.responses.pop().unwrap() {
                CommResponse::CmdAck(ack) => {
                    assert!(matches!(ack.command, command if command == expected));
                    assert!(matches!(
                        ack.success,
                        RosflightCmdResponse::RosflightCmdSuccess
                    ));
                }
                _ => panic!("expected command ack response"),
            }
        }
        assert!(requests.calibration_requests.is_empty());
    }

    #[test]
    fn accel_calibration_starts_full_imu_calibration_and_zeros_biases() {
        let mut requests = CommandEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut param_events = ParamEventQueues::default();
        let mut flags = CalibrationFlags::empty();
        let mut params = Params::new();
        let mut command = CommandManager::new();
        let mut controller = QuadController::<f64>::default();
        let mut board = TestBoard::default();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.4));
        params.set_by_id(ParamId::PARAM_ACC_Z_BIAS, ParamValue::Float(-0.2));
        let _ = requests.calibration_requests.push(CalibrationRequested {
            command: RosflightCmd::AccelCalibration,
        });
        let state = initialized_state();

        apply_calibration_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &state,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

        assert!(flags.contains(CalibrationFlags::IMU));
        assert_eq!(
            params.get_by_id(ParamId::PARAM_GYRO_X_BIAS),
            ParamValue::Float(0.0)
        );
        assert_eq!(
            params.get_by_id(ParamId::PARAM_ACC_Z_BIAS),
            ParamValue::Float(0.0)
        );
        match comm_events.responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::AccelCalibration));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdSuccess
                ));
            }
            _ => panic!("expected command ack response"),
        }
    }

    #[test]
    fn apply_offboard_control_requests_updates_command_manager() {
        let mut params = Params::new();
        let mut command = CommandManager::new();
        let mut requests = CommandEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut param_events = ParamEventQueues::default();
        let state = initialized_state();
        let mut flags = CalibrationFlags::empty();
        let mut controller = QuadController::<f64>::default();
        let mut board = TestBoard::default();

        let _ = requests
            .offboard_control_requests
            .push(OffboardControlRequested {
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
                    passthrough: [0.0; 4],
                },
            });

        apply_offboard_control_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &state,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

        assert!(command.is_offboard_active());
        assert!(requests.offboard_control_requests.is_empty());
    }

    #[test]
    fn apply_offboard_control_requests_stores_requests_while_disarmed() {
        let mut params = Params::new();
        let mut command = CommandManager::new();
        let mut requests = CommandEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut param_events = ParamEventQueues::default();
        let state = initialized_state();
        let mut flags = CalibrationFlags::empty();
        let mut controller = QuadController::<f64>::default();
        let mut board = TestBoard::default();

        let _ = requests
            .offboard_control_requests
            .push(OffboardControlRequested {
                now_us: 42_000,
                msg: OffboardControlMsg {
                    mode: OffboardControlMode::ModePassThrough,
                    ignore: OffboardControlIgnore::empty(),
                    qx: 0.1,
                    qy: 0.2,
                    qz: 0.3,
                    fx: 0.4,
                    fy: 0.5,
                    fz: 30.0,
                    passthrough: [0.0; 4],
                },
            });

        apply_offboard_control_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &state,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

        assert!(command.is_offboard_active());
        assert!(requests.offboard_control_requests.is_empty());
    }

    #[test]
    fn apply_param_defaults_requests_resets_params_and_reports_command() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let mut requests = CommandEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut param_events = ParamEventQueues::default();
        let mut command = CommandManager::new();
        let mut controller = QuadController::<f64>::default();
        let mut board = TestBoard::default();
        let mut flags = CalibrationFlags::empty();

        let _ = requests
            .param_defaults_requests
            .push(ParamDefaultsRequested {
                command: RosflightCmd::SetParamDefaults,
            });
        let state = initialized_state();

        apply_param_defaults_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &state,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(1)
        );
        assert!(param_events.full_refresh);
        match comm_events.responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::SetParamDefaults));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdSuccess
                ));
            }
            _ => panic!("expected command ack response"),
        }
        assert!(requests.param_defaults_requests.is_empty());
    }

    #[test]
    fn apply_board_command_requests_reports_unsupported_as_failed_ack() {
        let mut board = TestBoard::default();
        let mut params = Params::new();
        let mut requests = CommandEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut param_events = ParamEventQueues::default();
        let mut command = CommandManager::new();
        let mut controller = QuadController::<f64>::default();
        let mut flags = CalibrationFlags::empty();
        let state = initialized_state();

        let _ = requests.board_command_requests.push(BoardCommandRequested {
            command: RosflightCmd::WriteParams,
        });

        apply_board_command_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &state,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

        match comm_events.responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::WriteParams));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdFailed
                ));
            }
            _ => panic!("expected command ack response"),
        }
        assert!(requests.board_command_requests.is_empty());
    }

    #[test]
    fn apply_board_command_requests_round_trips_persistent_params_when_disarmed() {
        let mut board = PersistBoard {
            stored_system_id: 77,
            ..Default::default()
        };
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let mut requests = CommandEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut param_events = ParamEventQueues::default();
        let mut command = CommandManager::new();
        let mut controller = QuadController::<f64>::default();
        let mut flags = CalibrationFlags::empty();
        let state = initialized_state();

        let _ = requests.board_command_requests.push(BoardCommandRequested {
            command: RosflightCmd::WriteParams,
        });
        let _ = requests.board_command_requests.push(BoardCommandRequested {
            command: RosflightCmd::ReadParams,
        });

        apply_board_command_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &state,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

        assert_eq!(board.write_count, 1);
        assert_eq!(board.read_count, 1);
        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );
        for expected in [RosflightCmd::WriteParams, RosflightCmd::ReadParams] {
            match comm_events.responses.pop().unwrap() {
                CommResponse::CmdAck(ack) => {
                    assert!(matches!(ack.command, command if command == expected));
                    assert!(matches!(
                        ack.success,
                        RosflightCmdResponse::RosflightCmdSuccess
                    ));
                }
                _ => panic!("expected command ack response"),
            }
        }
        assert!(requests.board_command_requests.is_empty());
    }

    #[test]
    fn apply_rc_trim_calibration_requests_sets_equilibrium_torques_and_acks() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_RC_ATTITUDE_MODE, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(4));
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
        rc.init(&params);
        let mut channels = [0.5; crate::packets::RC_PACKET_CHANNELS];
        channels[0] = 0.55;
        channels[1] = 0.45;
        channels[3] = 0.60;
        rc.receive(&RcPacket {
            header: RosflightPacketHeader {
                timestamp: 1,
                status: 0,
            },
            n_chan: 4,
            chan: channels,
            lol: false,
        });
        rc.run(0, &params, &mut state);
        let mut command = CommandManager::new();
        command.run(0, &params, &mut rc, &mut state);
        let mut controller = QuadController::<f64>::default();
        let mut requests = CommandEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut param_events = ParamEventQueues::default();
        let mut flags = CalibrationFlags::empty();
        let mut board = TestBoard::default();
        let state = initialized_state();

        let _ = requests
            .rc_trim_calibration_requests
            .push(RcTrimCalibrationRequested {
                command: RosflightCmd::RcCalibration,
            });

        apply_rc_trim_calibration_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &state,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

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

        match comm_events.responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::RcCalibration));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdSuccess
                ));
            }
            _ => panic!("expected command ack response"),
        }
        assert!(requests.rc_trim_calibration_requests.is_empty());
    }

    #[test]
    fn command_requests_fail_without_mutation_when_armed() {
        let armed = armed_state();
        let mut comm_events = CommEventQueues::default();
        let mut param_events = ParamEventQueues::default();
        let mut flags = CalibrationFlags::empty();
        let mut params = Params::new();
        let mut requests = CommandEventQueues::default();
        let mut command = CommandManager::new();
        let mut controller = QuadController::<f64>::default();
        let mut board = TestBoard::default();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.4));
        let _ = requests.calibration_requests.push(CalibrationRequested {
            command: RosflightCmd::GyroCalibration,
        });

        apply_calibration_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &armed,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

        assert!(!flags.contains(CalibrationFlags::GYRO));
        assert_eq!(
            params.get_by_id(ParamId::PARAM_GYRO_X_BIAS),
            ParamValue::Float(0.4)
        );
        match comm_events.responses.pop().unwrap() {
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
        let mut requests = CommandEventQueues::default();
        let _ = requests
            .param_defaults_requests
            .push(ParamDefaultsRequested {
                command: RosflightCmd::SetParamDefaults,
            });

        apply_param_defaults_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &armed,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );
        match comm_events.responses.pop().unwrap() {
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
        let mut requests = CommandEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut param_events = ParamEventQueues::default();
        let mut command = CommandManager::new();
        let mut controller = QuadController::<f64>::default();
        let mut board = TestBoard::default();
        let mut flags = CalibrationFlags::empty();
        let mut params = Params::new();

        let _ = requests.version_requests.push(VersionRequested {
            command: RosflightCmd::SendVersion,
        });

        apply_version_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &state,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

        match comm_events.responses.pop().unwrap() {
            CommResponse::Version(version) => {
                assert_eq!(&version.version[..12], b"Veloxity 1.0");
            }
            _ => panic!("expected version response"),
        }
        match comm_events.responses.pop().unwrap() {
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
        let mut requests = CommandEventQueues::default();
        let _ = requests.version_requests.push(VersionRequested {
            command: RosflightCmd::SendVersion,
        });

        apply_version_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &armed,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

        match comm_events.responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::SendVersion));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdFailed
                ));
            }
            _ => panic!("expected command ack response"),
        }
        assert!(comm_events.responses.is_empty());
    }

    #[test]
    fn apply_reset_origin_requests_reports_unsupported_as_failed_ack() {
        let mut requests = CommandEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut param_events = ParamEventQueues::default();
        let state = initialized_state();
        let mut command = CommandManager::new();
        let mut controller = QuadController::<f64>::default();
        let mut board = TestBoard::default();
        let mut flags = CalibrationFlags::empty();
        let mut params = Params::new();

        let _ = requests.reset_origin_requests.push(ResetOriginRequested {
            command: RosflightCmd::ResetOrigin,
        });

        apply_reset_origin_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &state,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

        match comm_events.responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::ResetOrigin));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdFailed
                ));
            }
            _ => panic!("expected command ack response"),
        }
        assert!(requests.reset_origin_requests.is_empty());
    }

    #[test]
    fn apply_config_info_requests_reports_unsupported_as_failed_ack() {
        let mut requests = CommandEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut param_events = ParamEventQueues::default();
        let state = initialized_state();
        let mut command = CommandManager::new();
        let mut controller = QuadController::<f64>::default();
        let mut board = TestBoard::default();
        let mut flags = CalibrationFlags::empty();
        let mut params = Params::new();

        let _ = requests.config_info_requests.push(ConfigInfoRequested {
            command: RosflightCmd::SendAllConfigInfos,
        });

        apply_config_info_requests(&mut test_ctx(
            &mut requests,
            &mut param_events,
            &mut comm_events,
            &state,
            &mut command,
            &mut controller,
            &mut board,
            &mut flags,
            &mut params,
        ));

        match comm_events.responses.pop().unwrap() {
            CommResponse::CmdAck(ack) => {
                assert!(matches!(ack.command, RosflightCmd::SendAllConfigInfos));
                assert!(matches!(
                    ack.success,
                    RosflightCmdResponse::RosflightCmdFailed
                ));
            }
            _ => panic!("expected command ack response"),
        }
        assert!(requests.config_info_requests.is_empty());
    }
}
