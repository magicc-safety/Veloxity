use core::cell::RefCell;

use critical_section::Mutex;
use voloxide_core::packets::{RC_PACKET_CHANNELS, RcPacket, RosflightPacketHeader};

pub const CRSF_BAUDRATE: u32 = 420_000;
pub const CRSF_MAX_CHANNELS: usize = 16;

const RC_QUEUE_CAPACITY: usize = 4;

const EMPTY_RC_PACKET: RcPacket = RcPacket {
    header: RosflightPacketHeader {
        timestamp: 0,
        status: 0,
    },
    n_chan: 0,
    chan: [0.0; RC_PACKET_CHANNELS],
    lol: true,
};

pub static CRSF_RC_QUEUE: Mutex<RefCell<CrsfRcQueue>> =
    Mutex::new(RefCell::new(CrsfRcQueue::new()));

#[derive(Clone, Copy)]
pub struct SharedCrsfRcQueue {
    inner: &'static Mutex<RefCell<CrsfRcQueue>>,
}

unsafe impl Send for SharedCrsfRcQueue {}
unsafe impl Sync for SharedCrsfRcQueue {}

impl SharedCrsfRcQueue {
    pub const fn new(inner: &'static Mutex<RefCell<CrsfRcQueue>>) -> Self {
        Self { inner }
    }

    pub fn push_from_receiver_task(&self, packet: RcPacket) {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).push(packet));
    }

    pub fn take_latest(&self) -> Option<RcPacket> {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).take_latest())
    }
}

pub const SHARED_CRSF_RC_QUEUE: SharedCrsfRcQueue = SharedCrsfRcQueue::new(&CRSF_RC_QUEUE);

pub struct CrsfRcQueue {
    packets: [RcPacket; RC_QUEUE_CAPACITY],
    head: usize,
    len: usize,
    dropped_oldest: u32,
}

impl CrsfRcQueue {
    pub const fn new() -> Self {
        Self {
            packets: [EMPTY_RC_PACKET; RC_QUEUE_CAPACITY],
            head: 0,
            len: 0,
            dropped_oldest: 0,
        }
    }

    fn push(&mut self, packet: RcPacket) {
        if self.len == RC_QUEUE_CAPACITY {
            self.head = (self.head + 1) % RC_QUEUE_CAPACITY;
            self.len -= 1;
            self.dropped_oldest = self.dropped_oldest.wrapping_add(1);
        }

        let tail = (self.head + self.len) % RC_QUEUE_CAPACITY;
        self.packets[tail] = packet;
        self.len += 1;
    }

    fn take_latest(&mut self) -> Option<RcPacket> {
        if self.len == 0 {
            return None;
        }

        let latest = (self.head + self.len - 1) % RC_QUEUE_CAPACITY;
        let packet = self.packets[latest];
        self.head = (latest + 1) % RC_QUEUE_CAPACITY;
        self.len = 0;
        Some(packet)
    }

    pub fn dropped_oldest(&self) -> u32 {
        self.dropped_oldest
    }
}

impl Default for CrsfRcQueue {
    fn default() -> Self {
        Self::new()
    }
}
