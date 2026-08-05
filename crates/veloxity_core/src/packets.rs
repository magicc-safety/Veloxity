// ROSflight 2.0's persisted ARM `params_t` occupies 7,004 bytes. The SD driver
// appends its four-byte CRC after this payload.
pub static PARAM_PACKET_SIZE: usize = crate::params::storage::ROSFLIGHT_C_PARAM_STORAGE_SIZE;

use crate::math::FlightFloat;

pub const ADC_MAX_CHANNELS: usize = 21;
pub const RC_PACKET_CHANNELS: usize = 24;

#[derive(Debug, Clone, Copy, Default)]
pub enum RangeType {
    #[default]
    Sonar,
    Lidar,
}

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

#[derive(Debug, Clone, Copy, Default)]
pub struct RosflightPacketHeader {
    // Microseconds; packets avoid board-specific timestamp types.
    pub timestamp: u64,
    pub status: u16,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct AdcPacket {
    pub header: RosflightPacketHeader,
    pub temperature: f32,
    pub v_bku: f32,
    pub v_ref: f32,
    pub volts: [f32; ADC_MAX_CHANNELS],
}

#[derive(Debug, Clone, Copy, Default)]
pub struct BatteryPacket {
    pub header: RosflightPacketHeader,
    pub voltage: f32,
    pub current: f32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ImuPacket<R: FlightFloat> {
    pub header: RosflightPacketHeader,
    pub accel: [R; 3],
    pub gyro: [R; 3],
    pub temperature: f32,
    pub seq: u32,
}

impl<R: FlightFloat> ImuPacket<R> {
    pub fn cast<T: FlightFloat>(self) -> ImuPacket<T> {
        ImuPacket {
            header: self.header,
            accel: [
                <T as FlightFloat>::from_flight_float(self.accel[0]),
                <T as FlightFloat>::from_flight_float(self.accel[1]),
                <T as FlightFloat>::from_flight_float(self.accel[2]),
            ],
            gyro: [
                <T as FlightFloat>::from_flight_float(self.gyro[0]),
                <T as FlightFloat>::from_flight_float(self.gyro[1]),
                <T as FlightFloat>::from_flight_float(self.gyro[2]),
            ],
            temperature: self.temperature,
            seq: self.seq,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct BaroPacket {
    pub header: RosflightPacketHeader,
    pub pressure: f32,
    pub temperature: f32,
    pub altitude: f32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct PitotPacket {
    pub header: RosflightPacketHeader,
    pub differential_pressure: f32,
    pub temperature: f32,
    pub indicated_airspeed: f32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct MagPacket {
    pub header: RosflightPacketHeader,
    pub flux: [f32; 3],
    pub temperature: f32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct RcPacket {
    pub header: RosflightPacketHeader,
    pub n_chan: u32,
    pub chan: [f32; RC_PACKET_CHANNELS],
    pub lol: bool,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct RangePacket {
    pub header: RosflightPacketHeader,
    pub range: f32,
    pub min_range: f32,
    pub max_range: f32,
    pub range_type: RangeType,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct GNSSPacket {
    pub header: RosflightPacketHeader, // timestamp and device specific status
    pub unix_seconds: i64,             // Unix time, in seconds
    pub unix_nanos: i32,
    pub lat: f64,        // degrees
    pub lon: f64,        // degrees
    pub height_msl: f32, // m above mean sea level
    pub vel_n: f32,      // m/s north
    pub vel_e: f32,      // m/s east
    pub vel_d: f32,      // m/s down
    pub h_acc: f32,      // m north/east
    pub v_acc: f32,      // m down
    pub s_acc: f32,      // m/s
    pub month: u8,       // 0-11
    pub year: u16,       // 0-65535 UTC
    pub day: u8,         // 0-31 UTS day of month
    pub hour: u8,        // 0-23 UTC
    pub min: u8,         // 0-59 UTC
    pub sec: u8,         // 0-59 UTC
    pub nano: i32,       // adjustment +/1 to seconds
    pub fix_type: GNSSFixType,
    pub num_sats: u8, // 0-255
    pub mag_dec: f32, // Magnetic Declination ??
    pub time_correction: u64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct PpsPacket {
    pub header: RosflightPacketHeader,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct AttitudePacket {
    pub header: RosflightPacketHeader,
    pub q: [f32; 4],
    pub rate: [f32; 3],
}

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_packet_cast_preserves_values_across_flight_float_types() {
        let packet = ImuPacket::<f32> {
            header: RosflightPacketHeader {
                timestamp: 42,
                status: 7,
            },
            accel: [1.0, -2.0, 3.5],
            gyro: [0.1, -0.2, 0.3],
            temperature: 24.0,
            seq: 9,
        };

        let widened = packet.cast::<f64>();
        assert_eq!(widened.header.timestamp, packet.header.timestamp);
        assert_eq!(widened.header.status, packet.header.status);
        assert_eq!(widened.accel, [1.0, -2.0, 3.5]);
        assert_eq!(
            widened.gyro,
            [0.1_f32 as f64, -0.2_f32 as f64, 0.3_f32 as f64]
        );
        assert_eq!(widened.temperature, 24.0);
        assert_eq!(widened.seq, 9);
    }
}
