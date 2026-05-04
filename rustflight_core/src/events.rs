use crate::{
    comm_messages::{enums::RosflightCmd, messages::ParamValueMsg},
    params2::{ParamId, ParamValue},
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
    pub id: ParamId,
    pub value: ParamValue,
    pub param_id_bytes: [u8; 16],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamChanged {
    pub id: ParamId,
    pub old: ParamValue,
    pub new: ParamValue,
    pub param_id_bytes: [u8; 16],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CalibrationRequested {
    pub command: RosflightCmd,
}

#[derive(Debug, Clone, Copy)]
pub enum CommResponse {
    ParamValue(ParamValueMsg),
}

pub const PARAM_SET_REQUEST_QUEUE_CAPACITY: usize = 4;
pub const PARAM_CHANGED_QUEUE_CAPACITY: usize = 8;
pub const COMM_RESPONSE_QUEUE_CAPACITY: usize = 8;
pub const CALIBRATION_REQUEST_QUEUE_CAPACITY: usize = 4;

#[derive(Default)]
pub struct ParamEventQueues {
    pub set_requests: EventQueue<ParamSetRequested, PARAM_SET_REQUEST_QUEUE_CAPACITY>,
    pub changes: EventQueue<ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>,
    pub comm_responses: EventQueue<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
}

#[derive(Default)]
pub struct CommandEventQueues {
    pub calibration_requests: EventQueue<CalibrationRequested, CALIBRATION_REQUEST_QUEUE_CAPACITY>,
}

impl ParamEventQueues {
    pub fn clear_loop_events(&mut self) {
        self.set_requests.clear();
        self.changes.clear();
        self.comm_responses.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
