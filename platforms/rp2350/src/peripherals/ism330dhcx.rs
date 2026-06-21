use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use critical_section::Mutex;
use veloxity_core::packets::{ImuPacket, RosflightPacketHeader};

#[cfg(feature = "ism330dhcx-driver")]
pub use ism330dhcx_rs::blocking as st_driver;

#[cfg(feature = "ism330dhcx-driver")]
use crate::hal::{
    gpio::Output,
    peripherals::SPI1,
    spi::{Blocking, Spi},
};
#[cfg(feature = "ism330dhcx-driver")]
use embassy_time::Delay;
#[cfg(feature = "ism330dhcx-driver")]
use embedded_hal::spi::{ErrorType as SpiErrorType, Operation, SpiDevice};
#[cfg(feature = "ism330dhcx-driver")]
use st_driver::{Ism330dhcx, prelude::*};

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

    pub fn dropped_oldest(&self) -> u32 {
        critical_section::with(|cs| self.inner.borrow_ref(cs).dropped_oldest())
    }
}

pub const SHARED_ISM330DHCX_IMU_QUEUE: SharedIsm330dhcxImuQueue =
    SharedIsm330dhcxImuQueue::new(&ISM330DHCX_IMU_QUEUE);

static IMU_INIT_ATTEMPTS: AtomicU32 = AtomicU32::new(0);
static IMU_INIT_OK: AtomicU32 = AtomicU32::new(0);
static IMU_LAST_WHO_AM_I: AtomicU8 = AtomicU8::new(0);
static IMU_DRDY_EDGES: AtomicU32 = AtomicU32::new(0);
static IMU_READ_OK: AtomicU32 = AtomicU32::new(0);
static IMU_READ_ERRORS: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy, Default)]
pub struct Ism330dhcxStats {
    pub init_attempts: u32,
    pub init_ok: u32,
    pub last_who_am_i: u8,
    pub drdy_edges: u32,
    pub read_ok: u32,
    pub read_errors: u32,
}

pub fn record_ism330dhcx_init_attempt() {
    IMU_INIT_ATTEMPTS.fetch_add(1, Ordering::Relaxed);
}

pub fn record_ism330dhcx_init_ok(who_am_i: u8) {
    IMU_LAST_WHO_AM_I.store(who_am_i, Ordering::Relaxed);
    IMU_INIT_OK.fetch_add(1, Ordering::Relaxed);
}

pub fn record_ism330dhcx_init_failure(who_am_i: Option<u8>) {
    if let Some(who_am_i) = who_am_i {
        IMU_LAST_WHO_AM_I.store(who_am_i, Ordering::Relaxed);
    }
}

pub fn record_ism330dhcx_drdy_edge() {
    IMU_DRDY_EDGES.fetch_add(1, Ordering::Relaxed);
}

pub fn record_ism330dhcx_read_ok() {
    IMU_READ_OK.fetch_add(1, Ordering::Relaxed);
}

pub fn record_ism330dhcx_read_error() {
    IMU_READ_ERRORS.fetch_add(1, Ordering::Relaxed);
}

pub fn ism330dhcx_stats() -> Ism330dhcxStats {
    Ism330dhcxStats {
        init_attempts: IMU_INIT_ATTEMPTS.load(Ordering::Relaxed),
        init_ok: IMU_INIT_OK.load(Ordering::Relaxed),
        last_who_am_i: IMU_LAST_WHO_AM_I.load(Ordering::Relaxed),
        drdy_edges: IMU_DRDY_EDGES.load(Ordering::Relaxed),
        read_ok: IMU_READ_OK.load(Ordering::Relaxed),
        read_errors: IMU_READ_ERRORS.load(Ordering::Relaxed),
    }
}

#[cfg(feature = "ism330dhcx-driver")]
pub const ISM330DHCX_WHO_AM_I: u8 = st_driver::ISM330DHCX_ID;
#[cfg(feature = "ism330dhcx-driver")]
pub const ISM330DHCX_SPI_HZ: u32 = 10_000_000;

#[cfg(feature = "ism330dhcx-driver")]
#[derive(Clone, Copy)]
pub struct Ism330dhcxSampleConfig {
    pub accel_odr: OdrXl,
    pub gyro_odr: OdrGy,
    pub accel_scale: FsXl,
    pub gyro_scale: FsGy,
    pub period_us: u64,
}

#[cfg(feature = "ism330dhcx-driver")]
impl Ism330dhcxSampleConfig {
    pub const ODR_3333HZ_16G_2000DPS: Self = Self {
        accel_odr: OdrXl::_3332hz,
        gyro_odr: OdrGy::_3332hz,
        accel_scale: FsXl::_16g,
        gyro_scale: FsGy::_2000dps,
        period_us: 0,
    };

    pub const ODR_1666HZ_16G_2000DPS: Self = Self {
        accel_odr: OdrXl::_1666hz,
        gyro_odr: OdrGy::_1666hz,
        accel_scale: FsXl::_16g,
        gyro_scale: FsGy::_2000dps,
        period_us: 0,
    };
}

#[cfg(feature = "ism330dhcx-driver")]
#[derive(Debug)]
pub enum RpSpiDeviceError {
    Spi(crate::hal::spi::Error),
    Cs(core::convert::Infallible),
}

#[cfg(feature = "ism330dhcx-driver")]
impl embedded_hal::spi::Error for RpSpiDeviceError {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        embedded_hal::spi::ErrorKind::Other
    }
}

#[cfg(feature = "ism330dhcx-driver")]
pub struct RpBlockingSpiDevice {
    spi: Spi<'static, SPI1, Blocking>,
    cs: Output<'static>,
}

#[cfg(feature = "ism330dhcx-driver")]
impl RpBlockingSpiDevice {
    pub fn new(spi: Spi<'static, SPI1, Blocking>, mut cs: Output<'static>) -> Self {
        cs.set_high();
        Self { spi, cs }
    }
}

#[cfg(feature = "ism330dhcx-driver")]
impl SpiErrorType for RpBlockingSpiDevice {
    type Error = RpSpiDeviceError;
}

#[cfg(feature = "ism330dhcx-driver")]
impl SpiDevice<u8> for RpBlockingSpiDevice {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        self.cs.set_low();
        let result = (|| {
            for operation in operations {
                match operation {
                    Operation::Read(buf) => self.spi.blocking_read(buf),
                    Operation::Write(buf) => self.spi.blocking_write(buf),
                    Operation::Transfer(read, write) => self.spi.blocking_transfer(read, write),
                    Operation::TransferInPlace(buf) => self.spi.blocking_transfer_in_place(buf),
                    Operation::DelayNs(_) => Ok(()),
                }
                .map_err(RpSpiDeviceError::Spi)?;
            }
            Ok(())
        })();
        self.cs.set_high();
        result
    }
}

#[cfg(feature = "ism330dhcx-driver")]
pub struct Ism330dhcxImuProducer {
    sensor: Ism330dhcx<st_mems_bus::blocking::spi::SpiBus<RpBlockingSpiDevice>, Delay, MainBank>,
    config: Ism330dhcxSampleConfig,
    seq: u32,
}

#[cfg(feature = "ism330dhcx-driver")]
impl Ism330dhcxImuProducer {
    pub fn new(
        spi: Spi<'static, SPI1, Blocking>,
        cs: Output<'static>,
        config: Ism330dhcxSampleConfig,
    ) -> Self {
        let dev = RpBlockingSpiDevice::new(spi, cs);
        Self {
            sensor: Ism330dhcx::new_spi(dev, Delay),
            config,
            seq: 0,
        }
    }

    pub fn init(&mut self) -> Result<u8, ()> {
        let who_am_i = self.sensor.device_id_get().map_err(|_| ())?;
        if who_am_i != ISM330DHCX_WHO_AM_I {
            return Err(());
        }
        self.sensor
            .reset_set(st_driver::PROPERTY_ENABLE)
            .map_err(|_| ())?;
        self.sensor
            .block_data_update_set(st_driver::PROPERTY_ENABLE)
            .map_err(|_| ())?;
        self.sensor
            .xl_full_scale_set(self.config.accel_scale)
            .map_err(|_| ())?;
        self.sensor
            .gy_full_scale_set(self.config.gyro_scale)
            .map_err(|_| ())?;
        self.sensor
            .xl_data_rate_set(self.config.accel_odr)
            .map_err(|_| ())?;
        self.sensor
            .gy_data_rate_set(self.config.gyro_odr)
            .map_err(|_| ())?;
        Ok(who_am_i)
    }

    pub fn read_packet(&mut self, now_us: u64) -> Result<ImuPacket<f32>, ()> {
        let gyro_raw = self.sensor.angular_rate_raw_get().map_err(|_| ())?;
        let accel_raw = self.sensor.acceleration_raw_get().map_err(|_| ())?;
        let temperature_raw = self.sensor.temperature_raw_get().map_err(|_| ())?;
        let seq = self.seq;
        self.seq = self.seq.wrapping_add(1);

        Ok(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: now_us,
                status: 0,
            },
            accel: [
                st_driver::from_fs16g_to_mg(accel_raw[0]) * 0.009_806_65,
                st_driver::from_fs16g_to_mg(accel_raw[1]) * 0.009_806_65,
                st_driver::from_fs16g_to_mg(accel_raw[2]) * 0.009_806_65,
            ],
            gyro: [
                st_driver::from_fs2000dps_to_mdps(gyro_raw[0]) * core::f32::consts::PI / 180_000.0,
                st_driver::from_fs2000dps_to_mdps(gyro_raw[1]) * core::f32::consts::PI / 180_000.0,
                st_driver::from_fs2000dps_to_mdps(gyro_raw[2]) * core::f32::consts::PI / 180_000.0,
            ],
            temperature: 25.0 + temperature_raw as f32 / 256.0,
            seq,
        })
    }

    pub fn period_us(&self) -> u64 {
        self.config.period_us
    }
}

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
