use core::cell::RefCell;

use critical_section::Mutex;
use veloxity_core::{errors::SensorError, packets::BaroPacket};

const BARO_QUEUE_CAPACITY: usize = 4;

const EMPTY_BARO_PACKET: Result<BaroPacket, SensorError> = Ok(BaroPacket {
    header: veloxity_core::packets::RosflightPacketHeader {
        timestamp: 0,
        status: 0,
    },
    altitude: 0.0,
    pressure: 0.0,
    temperature: 0.0,
});

pub static BARO_QUEUE: Mutex<RefCell<BaroQueue>> = Mutex::new(RefCell::new(BaroQueue::new()));

#[derive(Clone, Copy)]
pub struct SharedBaroQueue {
    inner: &'static Mutex<RefCell<BaroQueue>>,
}

unsafe impl Send for SharedBaroQueue {}
unsafe impl Sync for SharedBaroQueue {}

impl SharedBaroQueue {
    pub const fn new(inner: &'static Mutex<RefCell<BaroQueue>>) -> Self {
        Self { inner }
    }

    pub fn push_from_sensor_task(&self, packet: Result<BaroPacket, SensorError>) {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).push(packet));
    }

    pub fn take_latest(&self) -> Option<Result<BaroPacket, SensorError>> {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).take_latest())
    }

    pub fn has_pending(&self) -> bool {
        critical_section::with(|cs| self.inner.borrow_ref(cs).has_pending())
    }

    pub fn dropped_oldest(&self) -> u32 {
        critical_section::with(|cs| self.inner.borrow_ref(cs).dropped_oldest())
    }
}

pub const SHARED_BARO_QUEUE: SharedBaroQueue = SharedBaroQueue::new(&BARO_QUEUE);

pub struct BaroQueue {
    packets: [Result<BaroPacket, SensorError>; BARO_QUEUE_CAPACITY],
    head: usize,
    len: usize,
    dropped_oldest: u32,
}

impl BaroQueue {
    pub const fn new() -> Self {
        Self {
            packets: [EMPTY_BARO_PACKET; BARO_QUEUE_CAPACITY],
            head: 0,
            len: 0,
            dropped_oldest: 0,
        }
    }

    fn push(&mut self, packet: Result<BaroPacket, SensorError>) {
        if self.len == BARO_QUEUE_CAPACITY {
            self.head = (self.head + 1) % BARO_QUEUE_CAPACITY;
            self.len -= 1;
            self.dropped_oldest = self.dropped_oldest.wrapping_add(1);
        }

        let index = (self.head + self.len) % BARO_QUEUE_CAPACITY;
        self.packets[index] = packet;
        self.len += 1;
    }

    fn take_latest(&mut self) -> Option<Result<BaroPacket, SensorError>> {
        if self.len == 0 {
            return None;
        }

        let index = (self.head + self.len - 1) % BARO_QUEUE_CAPACITY;
        let packet = self.packets[index];
        self.head = 0;
        self.len = 0;
        Some(packet)
    }

    fn has_pending(&self) -> bool {
        self.len != 0
    }

    pub fn dropped_oldest(&self) -> u32 {
        self.dropped_oldest
    }
}

impl Default for BaroQueue {
    fn default() -> Self {
        Self::new()
    }
}
