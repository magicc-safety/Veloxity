use crate::{
    events::{EventQueue, EventQueueError},
    params::{ParamId, ParamValue, Params},
};

pub struct ParamsReadPort<'a> {
    params: &'a Params,
}

impl<'a> ParamsReadPort<'a> {
    pub fn new(params: &'a Params) -> Self {
        Self { params }
    }

    pub fn raw(&self) -> &'a Params {
        self.params
    }

    pub fn get(&self, id: ParamId) -> ParamValue {
        self.params.get_by_id(id)
    }
}

pub struct ParamsWritePort<'a> {
    params: &'a mut Params,
}

impl<'a> ParamsWritePort<'a> {
    pub fn new(params: &'a mut Params) -> Self {
        Self { params }
    }

    pub fn get(&self, id: ParamId) -> ParamValue {
        self.params.get_by_id(id)
    }

    pub fn set(&mut self, id: ParamId, value: ParamValue) {
        self.params.set_by_id(id, value);
    }
}

pub struct EventEmitPort<'a, T: Copy, const N: usize> {
    queue: &'a mut EventQueue<T, N>,
}

impl<'a, T: Copy, const N: usize> EventEmitPort<'a, T, N> {
    pub fn new(queue: &'a mut EventQueue<T, N>) -> Self {
        Self { queue }
    }

    pub fn emit(&mut self, event: T) -> Result<(), EventQueueError> {
        self.queue.push(event)
    }

    pub fn emit_or_log(&mut self, event: T, label: &str) -> bool {
        if self.emit(event).is_ok() {
            true
        } else {
            crate::log_warn!("event queue full: {}", label);
            false
        }
    }
}

pub struct EventDrainPort<'a, T: Copy, const N: usize> {
    queue: &'a mut EventQueue<T, N>,
}

impl<'a, T: Copy, const N: usize> EventDrainPort<'a, T, N> {
    pub fn new(queue: &'a mut EventQueue<T, N>) -> Self {
        Self { queue }
    }

    pub fn next(&mut self) -> Option<T> {
        self.queue.pop()
    }
}

pub struct EventReadPort<'a, T: Copy, const N: usize> {
    queue: &'a EventQueue<T, N>,
}

impl<'a, T: Copy, const N: usize> EventReadPort<'a, T, N> {
    pub fn new(queue: &'a EventQueue<T, N>) -> Self {
        Self { queue }
    }

    pub fn iter(&self) -> impl Iterator<Item = T> + '_ {
        self.queue.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::logger::Logger;

    #[test]
    fn emit_or_log_reports_full_queue_without_overwriting_existing_event() {
        let mut queue = EventQueue::<u8, 1>::new();
        let mut port = EventEmitPort::new(&mut queue);

        assert!(port.emit_or_log(1, "test event"));
        assert!(!port.emit_or_log(2, "test event"));

        assert_eq!(queue.pop(), Some(1));
        assert_eq!(queue.pop(), None);

        while Logger::pop().is_some() {}
    }
}
