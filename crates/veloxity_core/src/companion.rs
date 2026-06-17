use crate::{
    comm::messages::messages::RosflightHardErrorMsg,
    comm::messages::messages::{ExternalAttitudeMsg, HeartbeatMsg, RosflightAuxCmdMsg},
    events::{CommEventQueues, CommResponse, CompanionEventQueues},
};

#[derive(Default)]
pub struct CompanionLinkState {
    pub connected: bool,
    pub last_heartbeat: Option<HeartbeatMsg>,
}

#[derive(Default)]
pub struct AuxCommandState {
    pub latest: Option<RosflightAuxCmdMsg>,
}

#[derive(Default)]
pub struct ExternalAttitudeState {
    pub latest: Option<ExternalAttitudeMsg>,
}

pub struct CompanionInputCtx<'a> {
    pub events: &'a mut CompanionEventQueues,
    pub comm_events: &'a mut CommEventQueues,
    pub link: &'a mut CompanionLinkState,
    pub aux_commands: &'a mut AuxCommandState,
    pub external_attitude: &'a mut ExternalAttitudeState,
    pub pending_hard_error: &'a mut Option<RosflightHardErrorMsg>,
}

pub fn apply_companion_inputs(ctx: &mut CompanionInputCtx<'_>) {
    apply_companion_heartbeats(ctx);
    emit_pending_hard_error_if_connected(ctx);
    apply_aux_commands(ctx);
    apply_external_attitudes(ctx);
}

pub fn apply_companion_heartbeats(ctx: &mut CompanionInputCtx<'_>) {
    while let Some(event) = ctx.events.heartbeats.pop() {
        ctx.link.connected = true;
        ctx.link.last_heartbeat = Some(event.msg);
    }
}

fn emit_pending_hard_error_if_connected(ctx: &mut CompanionInputCtx<'_>) {
    if ctx.link.connected
        && let Some(msg) = ctx.pending_hard_error.take()
    {
        ctx.comm_events
            .responses
            .push_or_log(CommResponse::HardError(msg), "hard error");
    }
}

pub fn apply_aux_commands(ctx: &mut CompanionInputCtx<'_>) {
    while let Some(event) = ctx.events.aux_commands.pop() {
        ctx.aux_commands.latest = Some(event.msg);
    }
}

pub fn apply_external_attitudes(ctx: &mut CompanionInputCtx<'_>) {
    while let Some(event) = ctx.events.external_attitudes.pop() {
        ctx.external_attitude.latest = Some(event.msg);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        comm::messages::{
            enums::RosflightAuxCmdType,
            messages::{ExternalAttitudeMsg, HeartbeatMsg, RosflightAuxCmdMsg},
        },
        events::{
            AuxCommandReceived, CommEventQueues, CompanionEventQueues, CompanionHeartbeatReceived,
            ExternalAttitudeReceived,
        },
    };

    fn test_ctx<'a>(
        events: &'a mut CompanionEventQueues,
        comm_events: &'a mut CommEventQueues,
        link: &'a mut CompanionLinkState,
        aux_commands: &'a mut AuxCommandState,
        external_attitude: &'a mut ExternalAttitudeState,
        pending_hard_error: &'a mut Option<RosflightHardErrorMsg>,
    ) -> CompanionInputCtx<'a> {
        CompanionInputCtx {
            events,
            comm_events,
            link,
            aux_commands,
            external_attitude,
            pending_hard_error,
        }
    }

    #[test]
    fn companion_heartbeat_marks_link_connected_and_records_latest() {
        let mut events = CompanionEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut link = CompanionLinkState::default();
        let mut aux_commands = AuxCommandState::default();
        let mut external_attitude = ExternalAttitudeState::default();
        let mut pending_hard_error = None;
        let heartbeat = HeartbeatMsg {
            type_: 1,
            autopilot: 2,
            base_mode: 3,
            custom_mode: 4,
            system_status: 5,
            mavlink_version: 6,
        };

        let _ = events
            .heartbeats
            .push(CompanionHeartbeatReceived { msg: heartbeat });

        apply_companion_heartbeats(&mut test_ctx(
            &mut events,
            &mut comm_events,
            &mut link,
            &mut aux_commands,
            &mut external_attitude,
            &mut pending_hard_error,
        ));

        assert!(link.connected);
        assert_eq!(link.last_heartbeat.unwrap().system_status, 5);
        assert!(events.heartbeats.is_empty());
    }

    #[test]
    fn aux_command_records_latest_command() {
        let mut events = CompanionEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut link = CompanionLinkState::default();
        let mut aux_commands = AuxCommandState::default();
        let mut external_attitude = ExternalAttitudeState::default();
        let mut pending_hard_error = None;
        let mut msg = RosflightAuxCmdMsg {
            type_array: [RosflightAuxCmdType::Disabled; 14],
            aux_cmd_array: [0.0; 14],
        };
        msg.type_array[2] = RosflightAuxCmdType::Servo;
        msg.aux_cmd_array[2] = 0.75;

        let _ = events.aux_commands.push(AuxCommandReceived { msg });

        apply_aux_commands(&mut test_ctx(
            &mut events,
            &mut comm_events,
            &mut link,
            &mut aux_commands,
            &mut external_attitude,
            &mut pending_hard_error,
        ));

        let latest = aux_commands.latest.unwrap();
        assert!(matches!(latest.type_array[2], RosflightAuxCmdType::Servo));
        assert_eq!(latest.aux_cmd_array[2], 0.75);
        assert!(events.aux_commands.is_empty());
    }

    #[test]
    fn external_attitude_records_latest_attitude() {
        let mut events = CompanionEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut link = CompanionLinkState::default();
        let mut aux_commands = AuxCommandState::default();
        let mut external_attitude = ExternalAttitudeState::default();
        let mut pending_hard_error = None;

        let _ = events.external_attitudes.push(ExternalAttitudeReceived {
            msg: ExternalAttitudeMsg {
                qw: 1.0,
                qx: 0.1,
                qy: 0.2,
                qz: 0.3,
            },
        });

        apply_external_attitudes(&mut test_ctx(
            &mut events,
            &mut comm_events,
            &mut link,
            &mut aux_commands,
            &mut external_attitude,
            &mut pending_hard_error,
        ));

        let latest = external_attitude.latest.unwrap();
        assert_eq!(latest.qw, 1.0);
        assert_eq!(latest.qz, 0.3);
        assert!(events.external_attitudes.is_empty());
    }
}
