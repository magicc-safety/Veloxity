use crate::{
    command::{ATTITUDE_RATE_MODE, CommandManager},
    events::ParamEventQueues,
    packets::RcPacket,
    params::{ParamId, ParamValue, Params},
    rc::Rc,
    state_machine::StateManager,
};

pub struct RcCommandStateCtx<'a> {
    pub now_ms: u32,
    /// A newly ingested RC sample for this system pass. The long-lived RC
    /// resource retains interpreted stick/switch state; cached sensor state is
    /// never replayed as a fresh command.
    pub fresh_rc: Option<RcPacket>,
    pub rc: &'a mut Rc,
    pub command: &'a mut CommandManager,
    pub state: &'a mut StateManager,
    pub params: &'a mut Params,
    pub param_events: Option<&'a mut ParamEventQueues>,
}

pub fn run_rc_command_state(ctx: RcCommandStateCtx<'_>) {
    if let Some(rc_packet) = ctx.fresh_rc {
        ctx.rc.receive(&rc_packet);
        ctx.rc.run(ctx.now_ms, ctx.params, ctx.state);
    }

    let command_result = ctx.command.run(ctx.now_ms, ctx.params, ctx.rc, ctx.state);
    if command_result.force_rc_attitude_mode_rate {
        if let Some(param_events) = ctx.param_events {
            crate::params::service::set_param_and_emit_change(
                ctx.params,
                &mut param_events.changes,
                ParamId::PARAM_RC_ATTITUDE_MODE,
                ParamValue::Int(ATTITUDE_RATE_MODE),
            );
        } else {
            ctx.params.set_by_id(
                ParamId::PARAM_RC_ATTITUDE_MODE,
                ParamValue::Int(ATTITUDE_RATE_MODE),
            );
        }
    }
    ctx.state.run(ctx.params);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        events::ParamEventQueues,
        packets::{RC_PACKET_CHANNELS, RcPacket, RosflightPacketHeader},
        params::{ParamId, ParamValue},
        state_machine::ErrorFlag,
    };

    #[test]
    fn rc_command_state_consumes_named_rc_packet() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));
        let fresh_rc = Some(RcPacket {
            header: RosflightPacketHeader {
                timestamp: 1,
                status: 0,
            },
            n_chan: 1,
            chan: [0.5; RC_PACKET_CHANNELS],
            lol: false,
        });
        let mut rc = Rc::new();
        let mut command = CommandManager::new();
        let mut state = StateManager::new();

        run_rc_command_state(RcCommandStateCtx {
            now_ms: 1,
            fresh_rc,
            rc: &mut rc,
            command: &mut command,
            state: &mut state,
            params: &mut params,
            param_events: None,
        });

        assert!(!state.get_errors().contains(ErrorFlag::RC_LOST));
    }

    #[test]
    fn rc_command_state_does_not_publish_command_from_lost_frame() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));
        let fresh_rc = Some(RcPacket {
            header: RosflightPacketHeader {
                timestamp: 1,
                status: 1,
            },
            n_chan: 1,
            chan: [0.5; RC_PACKET_CHANNELS],
            lol: false,
        });
        let mut rc = Rc::new();
        let mut command = CommandManager::new();
        let mut state = StateManager::new();

        run_rc_command_state(RcCommandStateCtx {
            now_ms: 1,
            fresh_rc,
            rc: &mut rc,
            command: &mut command,
            state: &mut state,
            params: &mut params,
            param_events: None,
        });

        assert!(state.get_errors().contains(ErrorFlag::RC_LOST));
        assert!(!rc.new_command());
    }

    #[test]
    fn rc_command_state_emits_param_change_when_lockout_forces_rate_mode() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_RC_ATTITUDE_MODE, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_EST_ANGLE_LOCKOUT, ParamValue::Int(1));
        let fresh_rc = Some(RcPacket {
            header: RosflightPacketHeader {
                timestamp: 1,
                status: 0,
            },
            n_chan: 8,
            chan: [0.5; RC_PACKET_CHANNELS],
            lol: false,
        });
        let mut rc = Rc::new();
        rc.init(&params);
        let mut command = CommandManager::new();
        let mut state = StateManager::new();
        state.set_error_flag(ErrorFlag::UNHEALTHY_ESTIMATOR, true, &params);
        let mut param_events = ParamEventQueues::default();

        run_rc_command_state(RcCommandStateCtx {
            now_ms: 1,
            fresh_rc,
            rc: &mut rc,
            command: &mut command,
            state: &mut state,
            params: &mut params,
            param_events: Some(&mut param_events),
        });

        assert_eq!(
            params.get_by_id(ParamId::PARAM_RC_ATTITUDE_MODE),
            ParamValue::Int(ATTITUDE_RATE_MODE)
        );
        let change = param_events.changes.pop().unwrap();
        assert_eq!(change.id, ParamId::PARAM_RC_ATTITUDE_MODE);
        assert_eq!(change.old, ParamValue::Int(1));
        assert_eq!(change.new, ParamValue::Int(ATTITUDE_RATE_MODE));
    }

    #[test]
    fn rc_command_state_does_not_force_rate_mode_when_lockout_disabled() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_RC_ATTITUDE_MODE, ParamValue::Int(1));
        let fresh_rc = Some(RcPacket {
            header: RosflightPacketHeader {
                timestamp: 1,
                status: 0,
            },
            n_chan: 8,
            chan: [0.5; RC_PACKET_CHANNELS],
            lol: false,
        });
        let mut rc = Rc::new();
        rc.init(&params);
        let mut command = CommandManager::new();
        let mut state = StateManager::new();
        state.set_error_flag(ErrorFlag::UNHEALTHY_ESTIMATOR, true, &params);
        let mut param_events = ParamEventQueues::default();

        run_rc_command_state(RcCommandStateCtx {
            now_ms: 1,
            fresh_rc,
            rc: &mut rc,
            command: &mut command,
            state: &mut state,
            params: &mut params,
            param_events: Some(&mut param_events),
        });

        assert_eq!(
            params.get_by_id(ParamId::PARAM_RC_ATTITUDE_MODE),
            ParamValue::Int(1)
        );
        assert!(param_events.changes.pop().is_none());
    }
}
