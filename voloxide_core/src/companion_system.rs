use crate::{
    comm_messages::messages::{ExternalAttitudeMsg, HeartbeatMsg, RosflightAuxCmdMsg},
    events::{AuxCommandReceived, CompanionHeartbeatReceived, ExternalAttitudeReceived},
    ports::EventDrainPort,
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

pub struct CompanionHeartbeatCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, CompanionHeartbeatReceived, N>,
    pub state: &'a mut CompanionLinkState,
}

pub fn apply_companion_heartbeats<const N: usize>(mut ctx: CompanionHeartbeatCtx<'_, N>) {
    while let Some(event) = ctx.requests.next() {
        ctx.state.connected = true;
        ctx.state.last_heartbeat = Some(event.msg);
    }
}

pub struct AuxCommandCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, AuxCommandReceived, N>,
    pub state: &'a mut AuxCommandState,
}

pub fn apply_aux_commands<const N: usize>(mut ctx: AuxCommandCtx<'_, N>) {
    while let Some(event) = ctx.requests.next() {
        ctx.state.latest = Some(event.msg);
    }
}

pub struct ExternalAttitudeCtx<'a, const N: usize> {
    pub requests: EventDrainPort<'a, ExternalAttitudeReceived, N>,
    pub state: &'a mut ExternalAttitudeState,
}

pub fn apply_external_attitudes<const N: usize>(mut ctx: ExternalAttitudeCtx<'_, N>) {
    while let Some(event) = ctx.requests.next() {
        ctx.state.latest = Some(event.msg);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        comm_messages::{
            enums::RosflightAuxCmdType,
            messages::{ExternalAttitudeMsg, HeartbeatMsg, RosflightAuxCmdMsg},
        },
        events::{
            AUX_COMMAND_QUEUE_CAPACITY, COMPANION_HEARTBEAT_QUEUE_CAPACITY,
            EXTERNAL_ATTITUDE_QUEUE_CAPACITY, EventQueue,
        },
    };

    #[test]
    fn companion_heartbeat_marks_link_connected_and_records_latest() {
        let mut requests =
            EventQueue::<CompanionHeartbeatReceived, COMPANION_HEARTBEAT_QUEUE_CAPACITY>::new();
        let mut state = CompanionLinkState::default();
        let heartbeat = HeartbeatMsg {
            type_: 1,
            autopilot: 2,
            base_mode: 3,
            custom_mode: 4,
            system_status: 5,
            mavlink_version: 6,
        };

        let _ = requests.push(CompanionHeartbeatReceived { msg: heartbeat });

        apply_companion_heartbeats(CompanionHeartbeatCtx {
            requests: EventDrainPort::new(&mut requests),
            state: &mut state,
        });

        assert!(state.connected);
        assert_eq!(state.last_heartbeat.unwrap().system_status, 5);
        assert!(requests.is_empty());
    }

    #[test]
    fn aux_command_records_latest_command() {
        let mut requests = EventQueue::<AuxCommandReceived, AUX_COMMAND_QUEUE_CAPACITY>::new();
        let mut state = AuxCommandState::default();
        let mut msg = RosflightAuxCmdMsg {
            type_array: [RosflightAuxCmdType::Disabled; 14],
            aux_cmd_array: [0.0; 14],
        };
        msg.type_array[2] = RosflightAuxCmdType::Servo;
        msg.aux_cmd_array[2] = 0.75;

        let _ = requests.push(AuxCommandReceived { msg });

        apply_aux_commands(AuxCommandCtx {
            requests: EventDrainPort::new(&mut requests),
            state: &mut state,
        });

        let latest = state.latest.unwrap();
        assert!(matches!(latest.type_array[2], RosflightAuxCmdType::Servo));
        assert_eq!(latest.aux_cmd_array[2], 0.75);
        assert!(requests.is_empty());
    }

    #[test]
    fn external_attitude_records_latest_attitude() {
        let mut requests =
            EventQueue::<ExternalAttitudeReceived, EXTERNAL_ATTITUDE_QUEUE_CAPACITY>::new();
        let mut state = ExternalAttitudeState::default();

        let _ = requests.push(ExternalAttitudeReceived {
            msg: ExternalAttitudeMsg {
                qw: 1.0,
                qx: 0.1,
                qy: 0.2,
                qz: 0.3,
            },
        });

        apply_external_attitudes(ExternalAttitudeCtx {
            requests: EventDrainPort::new(&mut requests),
            state: &mut state,
        });

        let latest = state.latest.unwrap();
        assert_eq!(latest.qw, 1.0);
        assert_eq!(latest.qz, 0.3);
        assert!(requests.is_empty());
    }
}
