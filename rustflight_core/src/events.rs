use crate::{
    comm_messages::{
        enums::{ParamIdentifier, RosflightCmd},
        messages::{
            ExternalAttitudeMsg, HeartbeatMsg, OffboardControlMsg, ParamValueMsg,
            RosflightAuxCmdMsg, RosflightCmdAckMsg, RosflightVersionMsg, StatustextMsg,
        },
    },
    params::{ParamId, ParamValue},
};

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum EventQueueError {
    Full,
}

pub struct EventQueue<T: Copy, const N: usize> {
    items: [Option<T>; N],
    head: usize,
    len: usize,
}

impl<T: Copy, const N: usize> EventQueue<T, N> {
    pub const fn new() -> Self {
        Self {
            items: [None; N],
            head: 0,
            len: 0,
        }
    }

    pub fn push(&mut self, event: T) -> Result<(), EventQueueError> {
        if self.len == N {
            return Err(EventQueueError::Full);
        }

        let idx = (self.head + self.len) % N;
        self.items[idx] = Some(event);
        self.len += 1;
        Ok(())
    }

    pub fn push_or_log(&mut self, event: T, label: &str) -> bool {
        if self.push(event).is_ok() {
            true
        } else {
            crate::log_warn!("event queue full: {}", label);
            false
        }
    }

    pub fn pop(&mut self) -> Option<T> {
        if self.len == 0 {
            return None;
        }

        let event = self.items[self.head].take();
        self.head = (self.head + 1) % N;
        self.len -= 1;
        event
    }

    pub fn iter(&self) -> EventQueueIter<'_, T, N> {
        EventQueueIter {
            queue: self,
            offset: 0,
        }
    }

    pub fn clear(&mut self) {
        while self.pop().is_some() {}
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl<T: Copy, const N: usize> Default for EventQueue<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct EventQueueIter<'a, T: Copy, const N: usize> {
    queue: &'a EventQueue<T, N>,
    offset: usize,
}

impl<'a, T: Copy, const N: usize> Iterator for EventQueueIter<'a, T, N> {
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.offset >= self.queue.len {
            return None;
        }

        let idx = (self.queue.head + self.offset) % N;
        self.offset += 1;
        self.queue.items[idx]
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamSetRequested {
    pub value: ParamValue,
    pub param_id_bytes: [u8; 16],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamReadRequested {
    pub identifier: ParamIdentifier,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamChanged {
    pub id: ParamId,
    pub old: ParamValue,
    pub new: ParamValue,
    pub param_id_bytes: [u8; 16],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamListRequested;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CalibrationRequested {
    pub command: RosflightCmd,
}

#[derive(Debug, Clone, Copy)]
pub struct OffboardControlRequested {
    pub now_us: u64,
    pub msg: OffboardControlMsg,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamDefaultsRequested {
    pub command: RosflightCmd,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoardCommandRequested {
    pub command: RosflightCmd,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RcTrimCalibrationRequested {
    pub command: RosflightCmd,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VersionRequested {
    pub command: RosflightCmd,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResetOriginRequested {
    pub command: RosflightCmd,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConfigInfoRequested {
    pub command: RosflightCmd,
}

#[derive(Debug, Clone, Copy)]
pub struct CompanionHeartbeatReceived {
    pub msg: HeartbeatMsg,
}

#[derive(Debug, Clone, Copy)]
pub struct AuxCommandReceived {
    pub msg: RosflightAuxCmdMsg,
}

#[derive(Debug, Clone, Copy)]
pub struct ExternalAttitudeReceived {
    pub msg: ExternalAttitudeMsg,
}

#[derive(Debug, Clone, Copy)]
pub enum CommResponse {
    ParamValue(ParamValueMsg),
    CmdAck(RosflightCmdAckMsg),
    Version(RosflightVersionMsg),
    Statustext(StatustextMsg),
}

pub const PARAM_SET_REQUEST_QUEUE_CAPACITY: usize = 4;
pub const PARAM_READ_REQUEST_QUEUE_CAPACITY: usize = 4;
pub const PARAM_LIST_REQUEST_QUEUE_CAPACITY: usize = 2;
pub const PARAM_CHANGED_QUEUE_CAPACITY: usize = 8;
pub const COMM_RESPONSE_QUEUE_CAPACITY: usize = 8;
pub const CALIBRATION_REQUEST_QUEUE_CAPACITY: usize = 4;
pub const OFFBOARD_CONTROL_REQUEST_QUEUE_CAPACITY: usize = 4;
pub const PARAM_DEFAULTS_REQUEST_QUEUE_CAPACITY: usize = 2;
pub const BOARD_COMMAND_REQUEST_QUEUE_CAPACITY: usize = 4;
pub const RC_TRIM_CALIBRATION_REQUEST_QUEUE_CAPACITY: usize = 2;
pub const VERSION_REQUEST_QUEUE_CAPACITY: usize = 2;
pub const RESET_ORIGIN_REQUEST_QUEUE_CAPACITY: usize = 2;
pub const CONFIG_INFO_REQUEST_QUEUE_CAPACITY: usize = 2;
pub const COMPANION_HEARTBEAT_QUEUE_CAPACITY: usize = 2;
pub const AUX_COMMAND_QUEUE_CAPACITY: usize = 2;
pub const EXTERNAL_ATTITUDE_QUEUE_CAPACITY: usize = 2;

#[derive(Default)]
pub struct ParamEventQueues {
    pub set_requests: EventQueue<ParamSetRequested, PARAM_SET_REQUEST_QUEUE_CAPACITY>,
    pub read_requests: EventQueue<ParamReadRequested, PARAM_READ_REQUEST_QUEUE_CAPACITY>,
    pub list_requests: EventQueue<ParamListRequested, PARAM_LIST_REQUEST_QUEUE_CAPACITY>,
    pub changes: EventQueue<ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>,
}

#[derive(Default)]
pub struct CommEventQueues {
    pub responses: EventQueue<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
}

#[derive(Default)]
pub struct CompanionEventQueues {
    pub heartbeats: EventQueue<CompanionHeartbeatReceived, COMPANION_HEARTBEAT_QUEUE_CAPACITY>,
    pub aux_commands: EventQueue<AuxCommandReceived, AUX_COMMAND_QUEUE_CAPACITY>,
    pub external_attitudes: EventQueue<ExternalAttitudeReceived, EXTERNAL_ATTITUDE_QUEUE_CAPACITY>,
}

#[derive(Default)]
pub struct CommandEventQueues {
    pub calibration_requests: EventQueue<CalibrationRequested, CALIBRATION_REQUEST_QUEUE_CAPACITY>,
    pub offboard_control_requests:
        EventQueue<OffboardControlRequested, OFFBOARD_CONTROL_REQUEST_QUEUE_CAPACITY>,
    pub param_defaults_requests:
        EventQueue<ParamDefaultsRequested, PARAM_DEFAULTS_REQUEST_QUEUE_CAPACITY>,
    pub board_command_requests:
        EventQueue<BoardCommandRequested, BOARD_COMMAND_REQUEST_QUEUE_CAPACITY>,
    pub rc_trim_calibration_requests:
        EventQueue<RcTrimCalibrationRequested, RC_TRIM_CALIBRATION_REQUEST_QUEUE_CAPACITY>,
    pub version_requests: EventQueue<VersionRequested, VERSION_REQUEST_QUEUE_CAPACITY>,
    pub reset_origin_requests:
        EventQueue<ResetOriginRequested, RESET_ORIGIN_REQUEST_QUEUE_CAPACITY>,
    pub config_info_requests: EventQueue<ConfigInfoRequested, CONFIG_INFO_REQUEST_QUEUE_CAPACITY>,
}

impl ParamEventQueues {
    pub fn clear_loop_events(&mut self) {
        self.set_requests.clear();
        self.read_requests.clear();
        self.list_requests.clear();
        self.changes.clear();
    }
}

impl CommEventQueues {
    pub fn clear_loop_events(&mut self) {
        self.responses.clear();
    }
}

impl CompanionEventQueues {
    pub fn clear_loop_events(&mut self) {
        self.heartbeats.clear();
        self.aux_commands.clear();
        self.external_attitudes.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::logger::Logger;

    #[test]
    fn event_queue_preserves_fifo_order_across_wraparound() {
        let mut queue = EventQueue::<u8, 3>::new();

        assert_eq!(queue.push(1), Ok(()));
        assert_eq!(queue.push(2), Ok(()));
        assert_eq!(queue.pop(), Some(1));
        assert_eq!(queue.push(3), Ok(()));
        assert_eq!(queue.push(4), Ok(()));
        assert_eq!(queue.push(5), Err(EventQueueError::Full));

        assert_eq!(queue.pop(), Some(2));
        assert_eq!(queue.pop(), Some(3));
        assert_eq!(queue.pop(), Some(4));
        assert_eq!(queue.pop(), None);
    }

    #[test]
    fn event_queue_iter_reads_without_draining() {
        let mut queue = EventQueue::<u8, 3>::new();

        let _ = queue.push(7);
        let _ = queue.push(8);

        let mut iter = queue.iter();
        assert_eq!(iter.next(), Some(7));
        assert_eq!(iter.next(), Some(8));
        assert_eq!(iter.next(), None);

        assert_eq!(queue.pop(), Some(7));
        assert_eq!(queue.pop(), Some(8));
    }

    #[test]
    fn push_or_log_drops_new_event_when_queue_is_full() {
        let mut queue = EventQueue::<u8, 1>::new();

        assert!(queue.push_or_log(1, "test event"));
        assert!(!queue.push_or_log(2, "test event"));

        assert_eq!(queue.pop(), Some(1));
        assert_eq!(queue.pop(), None);

        while Logger::pop().is_some() {}
    }
}
