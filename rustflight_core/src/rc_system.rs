use crate::{
    command_manager::CommandManager, params::Params, rc::Rc, sensors::ProcessedSensors,
    state_machine::StateManager,
};

pub struct RcCommandStateCtx<'a> {
    pub now_ms: u32,
    pub sensors: &'a ProcessedSensors,
    pub rc: &'a mut Rc,
    pub command: &'a mut CommandManager,
    pub state: &'a mut StateManager,
    pub params: &'a Params,
}

pub fn run_rc_command_state(mut ctx: RcCommandStateCtx<'_>) {
    if let Some(rc_packet) = ctx.sensors.rc {
        ctx.rc.receive(&rc_packet, ctx.params, ctx.state);
    }

    ctx.rc.run(ctx.now_ms, ctx.params, ctx.state);
    ctx.command.run(ctx.now_ms, ctx.params, ctx.rc, ctx.state);
    ctx.state.run(ctx.params);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        packets::{RC_PACKET_CHANNELS, RcPacket, RosflightPacketHeader},
        params::{ParamId, ParamValue},
        state_machine::ErrorFlag,
    };

    #[test]
    fn rc_command_state_consumes_named_rc_packet() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(1));
        let sensors = ProcessedSensors {
            rc: Some(RcPacket {
                header: RosflightPacketHeader {
                    timestamp: 1,
                    status: 0,
                },
                n_chan: 1,
                chan: [0.5; RC_PACKET_CHANNELS],
                lol: false,
            }),
            ..ProcessedSensors::default()
        };
        let mut rc = Rc::new();
        let mut command = CommandManager::new();
        let mut state = StateManager::new();

        run_rc_command_state(RcCommandStateCtx {
            now_ms: 1,
            sensors: &sensors,
            rc: &mut rc,
            command: &mut command,
            state: &mut state,
            params: &params,
        });

        assert!(!state.get_errors().contains(ErrorFlag::RC_LOST));
    }
}
