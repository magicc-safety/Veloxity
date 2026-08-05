use core::{
    cell::RefCell,
    sync::atomic::{AtomicU8, AtomicU32, Ordering},
};

use critical_section::Mutex;
use veloxity_core::{
    errors,
    packets::{GNSSFixType, GNSSPacket, RosflightPacketHeader},
};

const GPS_QUEUE_CAPACITY: usize = 4;
const UBX_MAX_PAYLOAD_BYTES: usize = 256;
const NAV_PVT_CLASS: u8 = 0x01;
const NAV_PVT_ID: u8 = 0x07;
const NAV_PVT_LEN: usize = 92;

static GPS_TOTAL_BYTES: AtomicU32 = AtomicU32::new(0);
static GPS_UBX_SYNC: AtomicU32 = AtomicU32::new(0);
static GPS_NAV_PVT: AtomicU32 = AtomicU32::new(0);
static GPS_UBX_FRAMES: AtomicU32 = AtomicU32::new(0);
static GPS_LAST_FRAME: AtomicU32 = AtomicU32::new(0);
static GPS_LAST_BYTE: AtomicU8 = AtomicU8::new(0);

#[derive(Debug, Clone, Copy, Default)]
pub struct GpsStats {
    pub total_bytes: u32,
    pub ubx_sync: u32,
    pub ubx_frames: u32,
    pub last_frame: u32,
    pub nav_pvt: u32,
}

pub fn record_gps_byte(byte: u8) {
    let last = GPS_LAST_BYTE.swap(byte, Ordering::Relaxed);
    GPS_TOTAL_BYTES.fetch_add(1, Ordering::Relaxed);
    if last == 0xb5 && byte == 0x62 {
        GPS_UBX_SYNC.fetch_add(1, Ordering::Relaxed);
    }
}

pub fn record_nav_pvt() {
    GPS_NAV_PVT.fetch_add(1, Ordering::Relaxed);
}

pub fn record_ubx_frame(class: u8, id: u8, length: usize) {
    GPS_UBX_FRAMES.fetch_add(1, Ordering::Relaxed);
    GPS_LAST_FRAME.store(
        ((class as u32) << 24) | ((id as u32) << 16) | (length.min(0xffff) as u32),
        Ordering::Relaxed,
    );
}

pub fn gps_stats() -> GpsStats {
    GpsStats {
        total_bytes: GPS_TOTAL_BYTES.load(Ordering::Relaxed),
        ubx_sync: GPS_UBX_SYNC.load(Ordering::Relaxed),
        ubx_frames: GPS_UBX_FRAMES.load(Ordering::Relaxed),
        last_frame: GPS_LAST_FRAME.load(Ordering::Relaxed),
        nav_pvt: GPS_NAV_PVT.load(Ordering::Relaxed),
    }
}

pub static GNSS_QUEUE: Mutex<RefCell<GnssQueue>> = Mutex::new(RefCell::new(GnssQueue::new()));

#[derive(Clone, Copy)]
pub struct SharedGnssQueue {
    inner: &'static Mutex<RefCell<GnssQueue>>,
}

unsafe impl Send for SharedGnssQueue {}
unsafe impl Sync for SharedGnssQueue {}

impl SharedGnssQueue {
    pub const fn new(inner: &'static Mutex<RefCell<GnssQueue>>) -> Self {
        Self { inner }
    }

    pub fn push_from_receiver_task(&self, packet: Result<GNSSPacket, errors::SensorError>) {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).push(packet));
    }

    pub fn take_latest(&self) -> Option<Result<GNSSPacket, errors::SensorError>> {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).take_latest())
    }

    pub fn has_pending(&self) -> bool {
        critical_section::with(|cs| self.inner.borrow_ref(cs).has_pending())
    }
}

pub const SHARED_GNSS_QUEUE: SharedGnssQueue = SharedGnssQueue::new(&GNSS_QUEUE);

#[derive(Clone, Copy)]
pub struct GnssQueue {
    packets: [Result<GNSSPacket, errors::SensorError>; GPS_QUEUE_CAPACITY],
    head: usize,
    len: usize,
    dropped_oldest: u32,
}

impl GnssQueue {
    pub const fn new() -> Self {
        Self {
            packets: [Ok(GNSSPacket {
                header: RosflightPacketHeader {
                    timestamp: 0,
                    status: 0,
                },
                unix_seconds: 0,
                unix_nanos: 0,
                lat: 0.0,
                lon: 0.0,
                height: 0.0,
                vel_n: 0.0,
                vel_e: 0.0,
                vel_d: 0.0,
                h_acc: 0.0,
                v_acc: 0.0,
                s_acc: 0.0,
                month: 0,
                year: 0,
                day: 0,
                hour: 0,
                min: 0,
                sec: 0,
                nano: 0,
                fix_type: GNSSFixType::NoFix,
                num_sats: 0,
                mag_dec: 0.0,
                time_correction: 0,
            }); GPS_QUEUE_CAPACITY],
            head: 0,
            len: 0,
            dropped_oldest: 0,
        }
    }

    fn push(&mut self, packet: Result<GNSSPacket, errors::SensorError>) {
        if self.len == GPS_QUEUE_CAPACITY {
            self.head = (self.head + 1) % GPS_QUEUE_CAPACITY;
            self.len -= 1;
            self.dropped_oldest = self.dropped_oldest.wrapping_add(1);
        }

        let tail = (self.head + self.len) % GPS_QUEUE_CAPACITY;
        self.packets[tail] = packet;
        self.len += 1;
    }

    fn take_latest(&mut self) -> Option<Result<GNSSPacket, errors::SensorError>> {
        if self.len == 0 {
            return None;
        }

        let latest = (self.head + self.len - 1) % GPS_QUEUE_CAPACITY;
        let packet = self.packets[latest];
        self.head = (latest + 1) % GPS_QUEUE_CAPACITY;
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

impl Default for GnssQueue {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy)]
struct UbxFrame {
    class: u8,
    id: u8,
    length: usize,
    ck_a: u8,
    ck_b: u8,
    payload: [u8; UBX_MAX_PAYLOAD_BYTES],
}

impl UbxFrame {
    const fn new() -> Self {
        Self {
            class: 0,
            id: 0,
            length: 0,
            ck_a: 0,
            ck_b: 0,
            payload: [0; UBX_MAX_PAYLOAD_BYTES],
        }
    }

    fn reset_checksum(&mut self) {
        self.ck_a = 0;
        self.ck_b = 0;
    }

    fn checksum_byte(&mut self, byte: u8) {
        self.ck_a = self.ck_a.wrapping_add(byte);
        self.ck_b = self.ck_b.wrapping_add(self.ck_a);
    }
}

pub struct UbxNavPvtParser {
    frame: UbxFrame,
    index: usize,
}

impl UbxNavPvtParser {
    pub const fn new() -> Self {
        Self {
            frame: UbxFrame::new(),
            index: 0,
        }
    }

    pub fn feed_byte(&mut self, byte: u8, now_us: u64) -> Option<GNSSPacket> {
        if byte == 0xb5 && self.index == 1 {
            self.index = 0;
        }

        if self.index == 0 {
            self.index = usize::from(byte == 0xb5);
        } else if self.index == 1 {
            if byte == 0x62 {
                self.index += 1;
            } else if byte == 0xb5 {
                self.index = 1;
            } else {
                self.index = 0;
            }
        } else if self.index == 2 {
            self.frame.reset_checksum();
            self.frame.class = byte;
            self.frame.checksum_byte(byte);
            self.index += 1;
        } else if self.index == 3 {
            self.frame.id = byte;
            self.frame.checksum_byte(byte);
            self.index += 1;
        } else if self.index == 4 {
            self.frame.length = byte as usize;
            self.frame.checksum_byte(byte);
            self.index += 1;
        } else if self.index == 5 {
            self.frame.length |= (byte as usize) << 8;
            if self.frame.length > UBX_MAX_PAYLOAD_BYTES {
                self.index = 0;
            } else {
                self.frame.checksum_byte(byte);
                self.index += 1;
            }
        } else if self.index < self.frame.length + 6 {
            self.frame.payload[self.index - 6] = byte;
            self.frame.checksum_byte(byte);
            self.index += 1;
        } else if self.index == self.frame.length + 6 {
            if self.frame.ck_a == byte {
                self.index += 1;
            } else {
                self.index = 0;
            }
        } else {
            self.index = 0;
            if self.frame.ck_b == byte {
                record_ubx_frame(self.frame.class, self.frame.id, self.frame.length);
                if self.frame.class == NAV_PVT_CLASS
                    && self.frame.id == NAV_PVT_ID
                    && self.frame.length == NAV_PVT_LEN
                {
                    return Some(nav_pvt_packet(&self.frame.payload[..NAV_PVT_LEN], now_us));
                }
            }
        }

        None
    }
}

impl Default for UbxNavPvtParser {
    fn default() -> Self {
        Self::new()
    }
}

pub fn make_ubx_packet(class: u8, id: u8, payload: &[u8], out: &mut [u8]) -> Option<usize> {
    let len = payload.len();
    if len + 8 > out.len() {
        return None;
    }

    out[0] = 0xb5;
    out[1] = 0x62;
    out[2] = class;
    out[3] = id;
    out[4..6].copy_from_slice(&(len as u16).to_le_bytes());
    out[6..6 + len].copy_from_slice(payload);

    let (ck_a, ck_b) = checksum(&out[2..6 + len]);
    out[6 + len] = ck_a;
    out[7 + len] = ck_b;
    Some(len + 8)
}

fn checksum(bytes: &[u8]) -> (u8, u8) {
    let mut ck_a = 0_u8;
    let mut ck_b = 0_u8;

    for byte in bytes {
        ck_a = ck_a.wrapping_add(*byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }

    (ck_a, ck_b)
}

fn nav_pvt_packet(payload: &[u8], now_us: u64) -> GNSSPacket {
    let valid = u8_at(payload, 11);
    let flags = u8_at(payload, 21);
    let flags2 = u8_at(payload, 22);
    let flags3 = u16_at(payload, 78);

    GNSSPacket {
        header: RosflightPacketHeader {
            timestamp: now_us,
            status: nav_pvt_status(valid, flags, flags2, flags3),
        },
        unix_seconds: unix_seconds_from_utc(
            u16_at(payload, 4),
            u8_at(payload, 6),
            u8_at(payload, 7),
            u8_at(payload, 8),
            u8_at(payload, 9),
            u8_at(payload, 10),
        ),
        unix_nanos: i32_at(payload, 16),
        // UBX NAV-PVT latitude/longitude are signed degrees scaled by 1e-7.
        // ROSFLIGHT_GNSS also specifies decimal degrees, so do not convert to radians here.
        lat: i32_at(payload, 28) as f64 * 1.0e-7,
        lon: i32_at(payload, 24) as f64 * 1.0e-7,
        height: i32_at(payload, 32) as f32 / 1000.0,
        vel_n: i32_at(payload, 48) as f32 / 1000.0,
        vel_e: i32_at(payload, 52) as f32 / 1000.0,
        vel_d: i32_at(payload, 56) as f32 / 1000.0,
        h_acc: u32_at(payload, 40) as f32 / 1000.0,
        v_acc: u32_at(payload, 44) as f32 / 1000.0,
        s_acc: u32_at(payload, 68) as f32 / 1000.0,
        month: u8_at(payload, 6),
        year: u16_at(payload, 4),
        day: u8_at(payload, 7),
        hour: u8_at(payload, 8),
        min: u8_at(payload, 9),
        sec: u8_at(payload, 10),
        nano: i32_at(payload, 16),
        fix_type: GNSSFixType::from_u8(u8_at(payload, 20)),
        num_sats: u8_at(payload, 23),
        mag_dec: i16_at(payload, 88) as f32 * 1.745_329_251_994_329_6e-4,
        time_correction: 0,
    }
}

fn nav_pvt_status(valid: u8, flags: u8, flags2: u8, flags3: u16) -> u16 {
    let mut status = 0_u16;

    if valid & 0x01 != 0 {
        status |= 0x0001;
    }
    if valid & 0x02 != 0 {
        status |= 0x0002;
    }
    if valid & 0x04 != 0 {
        status |= 0x0004;
    }
    if valid & 0x08 != 0 {
        status |= 0x0008;
    }
    if flags & 0x01 != 0 {
        status |= 0x0010;
    }
    if flags & 0x02 != 0 {
        status |= 0x0020;
    }
    if flags & 0x10 != 0 {
        status |= 0x0020;
    }
    if flags & 0x20 != 0 {
        status |= 0x0040;
    }
    if flags & 0x80 != 0 {
        status |= 0x0080;
    }
    if flags2 & 0x20 != 0 {
        status |= 0x0100;
    }
    if flags2 & 0x40 != 0 {
        status |= 0x0200;
    }
    if flags2 & 0x80 != 0 {
        status |= 0x0400;
    }
    if flags3 & 0x0001 != 0 {
        status |= 0x0800;
    }
    if flags3 & 0x0010 != 0 {
        status |= 0x1000;
    }

    status
}

fn unix_seconds_from_utc(year: u16, month: u8, day: u8, hour: u8, min: u8, sec: u8) -> i64 {
    let mut y = year as i32;
    let m = month as i32;
    y -= (m <= 2) as i32;
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let mp = m + if m > 2 { -3 } else { 9 };
    let doy = (153 * mp + 2) / 5 + day as i32 - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146097 + doe - 719468;
    days as i64 * 86_400 + hour as i64 * 3_600 + min as i64 * 60 + sec as i64
}

fn u8_at(payload: &[u8], offset: usize) -> u8 {
    payload[offset]
}

fn u16_at(payload: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([payload[offset], payload[offset + 1]])
}

fn u32_at(payload: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        payload[offset],
        payload[offset + 1],
        payload[offset + 2],
        payload[offset + 3],
    ])
}

fn i16_at(payload: &[u8], offset: usize) -> i16 {
    i16::from_le_bytes([payload[offset], payload[offset + 1]])
}

fn i32_at(payload: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes([
        payload[offset],
        payload[offset + 1],
        payload[offset + 2],
        payload[offset + 3],
    ])
}
