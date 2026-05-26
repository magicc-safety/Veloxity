use core::cell::RefCell;

use critical_section::Mutex;
use voloxide_core::packets::{ImuPacket, RosflightPacketHeader};

#[cfg(feature = "ism330dhcx-driver")]
pub use ism330dhcx_rs::asynchronous as st_driver;

const IMU_QUEUE_CAPACITY: usize = 8;

const EMPTY_IMU_PACKET: ImuPacket<f32> = ImuPacket {
    header: RosflightPacketHeader {
        timestamp: 0,
        status: 0,
    },
    accel: [0.0; 3],
    gyro: [0.0; 3],
    temperature: 0.0,
    seq: 0,
};

pub static ISM330DHCX_IMU_QUEUE: Mutex<RefCell<Ism330dhcxImuQueue>> =
    Mutex::new(RefCell::new(Ism330dhcxImuQueue::new()));

#[derive(Clone, Copy)]
pub struct SharedIsm330dhcxImuQueue {
    inner: &'static Mutex<RefCell<Ism330dhcxImuQueue>>,
}

unsafe impl Send for SharedIsm330dhcxImuQueue {}
unsafe impl Sync for SharedIsm330dhcxImuQueue {}

impl SharedIsm330dhcxImuQueue {
    pub const fn new(inner: &'static Mutex<RefCell<Ism330dhcxImuQueue>>) -> Self {
        Self { inner }
    }

    pub fn push_from_interrupt(&self, packet: ImuPacket<f32>) {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).push(packet));
    }

    pub fn take_latest(&self) -> Option<ImuPacket<f32>> {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).take_latest())
    }

    pub fn has_pending(&self) -> bool {
        critical_section::with(|cs| self.inner.borrow_ref(cs).has_pending())
    }
}

pub const SHARED_ISM330DHCX_IMU_QUEUE: SharedIsm330dhcxImuQueue =
    SharedIsm330dhcxImuQueue::new(&ISM330DHCX_IMU_QUEUE);

pub struct Ism330dhcxImuQueue {
    packets: [ImuPacket<f32>; IMU_QUEUE_CAPACITY],
    head: usize,
    len: usize,
    dropped_oldest: u32,
}

impl Ism330dhcxImuQueue {
    pub const fn new() -> Self {
        Self {
            packets: [EMPTY_IMU_PACKET; IMU_QUEUE_CAPACITY],
            head: 0,
            len: 0,
            dropped_oldest: 0,
        }
    }

    fn push(&mut self, packet: ImuPacket<f32>) {
        if self.len == IMU_QUEUE_CAPACITY {
            self.head = (self.head + 1) % IMU_QUEUE_CAPACITY;
            self.len -= 1;
            self.dropped_oldest = self.dropped_oldest.wrapping_add(1);
        }

        let tail = (self.head + self.len) % IMU_QUEUE_CAPACITY;
        self.packets[tail] = packet;
        self.len += 1;
    }

    fn take_latest(&mut self) -> Option<ImuPacket<f32>> {
        if self.len == 0 {
            return None;
        }

        let latest = (self.head + self.len - 1) % IMU_QUEUE_CAPACITY;
        let packet = self.packets[latest];
        self.head = (latest + 1) % IMU_QUEUE_CAPACITY;
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

impl Default for Ism330dhcxImuQueue {
    fn default() -> Self {
        Self::new()
    }
}
