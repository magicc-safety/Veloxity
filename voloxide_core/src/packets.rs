pub static PARAM_PACKET_SIZE: usize = 2048;
//pub static LOG_MESSAGE_SIZE: usize = 64;

//#[derive(Debug, Clone, Copy, Default)]
//pub enum LogLevel {
//    #[default]
//    Info,
//    Warn,
//    Error,
//}

//#[derive(Debug, Clone, Copy, Default)]
//pub struct LogPacket {
//    pub level: LogLevel,
//    pub msg: [u8; 16],
//}

//#[derive(Debug, Clone, Copy)]
//pub struct Payload(pub [u8; PAYLOAD_SIZE]);

//impl Default for Payload {
//    fn default() -> Self {
//        Self([0u8; PAYLOAD_SIZE])
//    }
//}

// --------------------- Sensor Packets ---------------------

pub const ADC_MAX_CHANNELS: usize = 21;
pub const RC_PACKET_CHANNELS: usize = 24;

//#[derive(Debug, Clone, Copy, Default, Format)]
//pub enum Qos {
//    #[default]
//    High,
//    Medium,
//    Low,
//}

//#[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub enum RangeType {
    #[default]
    Sonar,
    Lidar,
}

// #[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub enum GNSSFixType {
    #[default]
    NoFix,
    DeadReckoningOnly,
    TwoD,
    ThreeD,
    GnssPlusDeadReckoning,
    TimeFixOnly,
}

//#[cfg(feature = "nucleo")]
impl GNSSFixType {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => GNSSFixType::NoFix,
            1 => GNSSFixType::DeadReckoningOnly,
            2 => GNSSFixType::TwoD,
            3 => GNSSFixType::ThreeD,
            4 => GNSSFixType::GnssPlusDeadReckoning,
            5 => GNSSFixType::TimeFixOnly,
            _ => GNSSFixType::NoFix, // Handle unknown/invalid fix type
        }
    }
}

// #[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RosflightPacketHeader {
    // Microseconds; packets avoid board-specific timestamp types.
    pub timestamp: u64,
    pub status: u16,
}

//#[derive(Debug, Clone, Copy, Default, Format)]
//pub struct SerialTxPacket {
//    pub header: RosflightPacketHeader,
//    pub qos: Qos,
//    pub len: i16,
//    pub payload: Payload,
//}

// ------------------------------
// ADC Packet
// ------------------------------

// #[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub struct AdcPacket {
    pub header: RosflightPacketHeader,
    pub temperature: f32,
    pub v_bku: f32,
    pub v_ref: f32,
    pub volts: [f32; ADC_MAX_CHANNELS],
}

// ------------------------------
// Battery Packet
// ------------------------------

// #[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub struct BatteryPacket {
    pub header: RosflightPacketHeader,
    pub voltage: f32,
    pub current: f32,
}

// ------------------------------
// IMU Packet
// ------------------------------

// #[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuPacket {
    pub header: RosflightPacketHeader,
    pub accel: [f64; 3],
    pub gyro: [f64; 3],
    pub temperature: f32,
    pub seq: u32,
}

// ------------------------------
// Baro Packet
// ------------------------------

// #[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub struct BaroPacket {
    pub header: RosflightPacketHeader,
    pub pressure: f32,
    pub temperature: f32,
    pub altitude: f32,
}

// ------------------------------
// Pitot Packet
// ------------------------------

// #[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PitotPacket {
    pub header: RosflightPacketHeader,
    pub differential_pressure: f32,
    pub temperature: f32,
    pub indicated_airspeed: f32,
}

// ------------------------------
// Mag Packet
// ------------------------------

// #[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub struct MagPacket {
    pub header: RosflightPacketHeader,
    pub flux: [f32; 3],
    pub temperature: f32,
}

// ------------------------------
// Rc Packet
// ------------------------------

// #[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RcPacket {
    pub header: RosflightPacketHeader,
    pub n_chan: u32,
    pub chan: [f32; RC_PACKET_CHANNELS],
    pub lol: bool,
}

// ------------------------------
// Range Packet
// ------------------------------

// #[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RangePacket {
    pub header: RosflightPacketHeader,
    pub range: f32,
    pub min_range: f32,
    pub max_range: f32,
    pub range_type: RangeType,
}

// ------------------------------
// GNSS Packet
// ------------------------------

// #[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub struct GNSSPacket {
    pub header: RosflightPacketHeader, // timestamp and device specific status
    pub unix_seconds: i64,             // Unix time, in seconds
    pub unix_nanos: i32,
    pub lat: f64,    // degrees
    pub lon: f64,    // degrees
    pub height: f32, // m/s above ellipsoid
    pub vel_n: f32,  // m/s north
    pub vel_e: f32,  // m/s east
    pub vel_d: f32,  // m/s down
    pub h_acc: f32,  // m north/east
    pub v_acc: f32,  // m down
    pub s_acc: f32,  // m/s
    pub month: u8,   // 0-11
    pub year: u16,   // 0-65535 UTC
    pub day: u8,     // 0-31 UTS day of month
    pub hour: u8,    // 0-23 UTC
    pub min: u8,     // 0-59 UTC
    pub sec: u8,     // 0-59 UTC
    pub nano: i32,   // adjustment +/1 to seconds
    pub fix_type: GNSSFixType,
    pub num_sats: u8, // 0-255
    pub mag_dec: f32, // Magnetic Declination ??
    pub time_correction: u64,
}

// ------------------------------
// PPS Packet
// ------------------------------

// #[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PpsPacket {
    pub header: RosflightPacketHeader,
}

// ------------------------------
// Attitude Packet
// ------------------------------

// #[derive(Debug, Clone, Copy, Default, Format)]
#[derive(Debug, Clone, Copy, Default)]
pub struct AttitudePacket {
    pub header: RosflightPacketHeader,
    pub q: [f32; 4],
    pub rate: [f32; 3],
}

// ------------------------------
// Serial Packet
// ------------------------------

//#[derive(Debug, Clone, Copy, Default, Format)]
//pub struct SerialRxPacket {
//    pub header: RosflightPacketHeader,
//    pub qos: Qos,
//    pub len: i16,
//    pub payload: Payload,
//}

// ------------------------------
// Param Packet
// ------------------------------

// #[derive(Debug, Clone, Copy, Format)]
#[derive(Debug, Clone, Copy)]
pub struct ParamPacket {
    pub header: RosflightPacketHeader,
    pub values: [u8; PARAM_PACKET_SIZE],
}

impl Default for ParamPacket {
    fn default() -> Self {
        Self {
            header: RosflightPacketHeader::default(),
            values: [0u8; PARAM_PACKET_SIZE],
        }
    }
}
