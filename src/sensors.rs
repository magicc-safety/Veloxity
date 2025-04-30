#[cfg(feature = "nucleo")]
pub(crate) mod adis16500;
#[cfg(feature = "nucleo")]
pub(crate) mod dlhrl20g;
#[cfg(feature = "nucleo")]
pub(crate) mod dps310;
#[cfg(feature = "nucleo")]
pub(crate) mod iis2mdc;
#[cfg(feature = "nucleo")]
pub(crate) mod telem;

// Create enum of rosflight return types
#[cfg(feature = "nucleo")]
pub use {
    embassy_time::{Instant, Duration},
    defmt::{trace, Format},
};

// ------------------------------------------------------------------------ Mock Timing on Host Computer -----------------------------------------------------------------------

#[cfg(feature = "default")]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Duration {
    micros: u64,  // Use u64 for the duration in microseconds
}

#[cfg(feature = "default")]
impl Duration {
    // Constructor that creates a Duration from micros (u64)
    pub const fn from_micros(micros: u64) -> Self {
        Duration { micros }
    }

    // Convert the custom Duration to core::time::Duration
    pub fn to_core_duration(&self) -> core::time::Duration {
        core::time::Duration::from_micros(self.micros)
    }

    // Convert from core::time::Duration to custom Duration
    pub fn from_core_duration(duration: core::time::Duration) -> Self {
        Duration {
            micros: duration.as_secs() * 1_000_000 + duration.subsec_micros() as u64,
        }
    }

    // Get the duration in microseconds (u64)
    pub fn as_micros(&self) -> u64 {
        self.micros
    }
}

#[cfg(feature = "default")]
pub use {
    mock_instant::global::Instant, // <-- relies on "duration"
    log::info,
};

#[cfg(feature = "default")]
pub trait InstantExt {
    fn as_micros(&self) -> u64;
    fn from_micros(micros: u64) -> Self;
}

#[cfg(feature = "default")]
impl InstantExt for Instant {
    fn as_micros(&self) -> u64 {
        let duration = self.elapsed(); // Get elapsed time
        duration.as_secs() as u64 * 1_000_000 + duration.subsec_micros() as u64
    }

    fn from_micros(micros: u64) -> Instant {
        let now = Instant::now();
        now + Duration::from_micros(micros).to_core_duration()
    }
}

// ---------------------------------------------------------------------- End Mock Timing on Host Computer -----------------------------------------------------------------

// ------------------------------------------------------------------------ Logging on Host Computer -----------------------------------------------------------------------

#[cfg(feature = "default")]
mod host_rtt {
    use core::fmt::{self, Write};  // Added explicit Write import
    use libc::{close, c_char, mkfifo, open, O_WRONLY, write};

    const FIFO_PATH: &[u8] = b"/tmp/rustflight_rtt\0";
    const BUF_SIZE: usize = 128;

    pub struct RttWriter {
        fd: i32,
        buffer: [u8; BUF_SIZE],
        position: usize,
    }

    impl RttWriter {
        pub fn new() -> Self {
            unsafe {
                mkfifo(FIFO_PATH.as_ptr() as *const c_char, 0o666);
                let fd = open(FIFO_PATH.as_ptr() as *const c_char, O_WRONLY);
                Self {
                    fd,
                    buffer: [0u8; BUF_SIZE],
                    position: 0,
                }
            }
        }

        fn write_bytes(&mut self, data: &[u8]) -> fmt::Result {
            let mut bytes = data;
            while !bytes.is_empty() {
                let remaining = self.buffer.len() - self.position;
                let copy_len = core::cmp::min(bytes.len(), remaining);
                
                self.buffer[self.position..self.position+copy_len]
                    .copy_from_slice(&bytes[..copy_len]);
                self.position += copy_len;
                bytes = &bytes[copy_len..];

                if self.position == self.buffer.len() {
                    self.flush()?;  // Propagate errors
                }
            }
            Ok(())
        }

        pub fn flush(&mut self) -> fmt::Result {
            if self.position > 0 {
                let written = unsafe {
                    write(
                        self.fd,
                        self.buffer.as_ptr() as *const _,
                        self.position
                    )
                };
                
                if written < 0 {
                    return Err(fmt::Error);
                }
                
                self.position = 0;
            }
            Ok(())
        }
    }

    impl Write for RttWriter {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            self.write_bytes(s.as_bytes())
        }
    }
}


// ---------------------------------------------------------------------- End Logging on Host Computer ---------------------------------------------------------------------


use crate::board::Board;

pub fn synch_at(slot_rate: Duration) -> Instant 
{
    let dt = slot_rate.as_micros();
    let now = Instant::now().as_micros();
    Instant::from_micros((now/dt+1u64)*dt)
}

pub fn synch_at_slot(slot_rate: Duration) -> Instant 
{
    let dt = slot_rate.as_micros();
    let now = Instant::now().as_micros();
    Instant::from_micros((now/dt+1u64)*dt)
}

pub fn current_slot(timestamp: Instant, sample_period: Duration, slot_period: Duration) -> u64
{
    (timestamp.as_micros()%sample_period.as_micros())/slot_period.as_micros()
}

// All packets used by sensors:
const SERIAL_MAX_PAYLOAD_SIZE:usize = 256+4;
const ADC_MAX_CHANNELS:usize = 21;
const RC_PACKET_CHANNELS:usize = 24;


#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub enum Qos
{
    High, Medium, Low,
}

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub enum RangeType
{
    Sonar, Lidar,
}

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub enum GNSSFixType
{
    NoFix, DeadReckoningOnly,TwoD, ThreeD, GnssPlusDeadReckoning, TimeFixOnly,
}

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub struct RosflightPacketHeader
{
    pub timestamp :Instant, pub status: u16
}

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub struct SerialTxPacket { pub header: RosflightPacketHeader, pub qos: Qos, pub len: i16,  pub payload: [u8;SERIAL_MAX_PAYLOAD_SIZE]}

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub struct AdcPacket {pub header : RosflightPacketHeader, pub temperature: f32, pub v_bku: f32, pub v_ref : f32, pub volts: [f32;ADC_MAX_CHANNELS]}

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub struct BatteryPacket { pub header : RosflightPacketHeader, pub voltage :f32, pub current :f32}

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub struct ImuPacket { pub header : RosflightPacketHeader, pub accel :[f64;3], pub gyro :[f64;3], pub temperature :f32, pub seq: u16 }

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub struct BaroPacket { pub header : RosflightPacketHeader, pub pressure : f32, pub temperature : f32}

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub struct PitotPacket { pub header : RosflightPacketHeader, pub pressure : f32, pub temperature : f32}

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub struct MagPacket {pub header:RosflightPacketHeader, pub flux: [f32;3], pub temperature : f32}

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub struct RcPacket { pub header: RosflightPacketHeader, pub n_chan: u32, pub chan: [f32;RC_PACKET_CHANNELS], pub frame_lost : bool, pub rc_packet_lost : bool}

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub struct RangePacket {pub header: RosflightPacketHeader, pub range : f32, pub in_range :f32, pub max_range: f32, pub range_type: RangeType }

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub struct GNSSPacket {pub header: RosflightPacketHeader, pub pps: u64, pub fix_type: GNSSFixType} // lots more parameters for later

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub struct AttitudePacket {pub header: RosflightPacketHeader, pub q: [f32;4], pub rate: [f32;3]}
// not really needed:

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug)]
pub struct SerialRxPacket { pub header: RosflightPacketHeader, pub qos: Qos, pub len: i16, pub payload: [u8;SERIAL_MAX_PAYLOAD_SIZE]}

// pub enum RosflightPacket
// {
//     SerialTx(SerialRxPacket),
//     SerialRx(AdcPacket),
//     Adc(AdcPacket),
//     Battery(BatteryPacket),
//     Imu(ImuPacket),
//     Baro(BaroPacket),
//     Pitot(PitotPacket),
//     Mag(MagPacket),
//     Rc(RcPacket),
//     Range(RangePacket),
//     GNSS(GNSSPacket),
//     Attitude(AttitudePacket),
// }

// pub enum RosflightPacket
// {
//     // These are the ones used by Varmint Board:
//     SerialTxPacket { pub header: RosflightPacketHeader, qos: Qos, len: i16,  payload: [u8;SERIAL_MAX_PAYLOAD_SIZE]},
//     AdcPacket {pub header : RosflightPacketHeader, temperature: f32, v_bku: f32, v_ref : f32, volts: [f32;ADC_MAX_CHANNELS]},
//     BatteryPacket { pub header : RosflightPacketHeader, voltage :f32, current :f32},
//     ImuPacket { pub header : RosflightPacketHeader, accel :[f32;3], gyro :[f32;3], temperature :f32 },
//     BaroPacket { pub header : RosflightPacketHeader, pressure : f32, temperature : f32},
//     PitotPacket { pub header : RosflightPacketHeader, pressure : f32, temperature : f32},
//     MagPacket {pub header:RosflightPacketHeader, flux: [f32;3], temperature : f32},
//     RcPacket { pub header: RosflightPacketHeader, n_chan: u32, chan: [f32;RC_PACKET_CHANNELS], frame_lost : bool, rc_packet_lost : bool},
//     // Additionals in rosflight pub structures
//     RangePacket {pub header: RosflightPacketHeader, range : f32, min_range :f32, max_range: f32, range_type: RangeType },
//     GNSSPacket {pub header: RosflightPacketHeader, pps: u64, fix_type: GNSSFixType}, // lots more parameters for later
//     AttitudePacket {pub header: RosflightPacketHeader,q: [f32;4], rate: [f32;3]},
//     // Maybe add
//     SerialRxPacket { pub header: RosflightPacketHeader, qos: Qos, len: i16,  payload: [u8;SERIAL_MAX_PAYLOAD_SIZE]},
// }

#[cfg_attr(feature = "nucleo", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub enum SensorError {
    GenericSensorError(&'static str),
}

pub struct Sensors {
    pub rosflight_packet_header: Option<RosflightPacketHeader>,
    pub serial_tx_packet: Option<SerialTxPacket>,
    pub adc_packet: Option<AdcPacket>,
    pub battery_packet: Option<BatteryPacket>,
    pub imu_packet: Option<ImuPacket>,
    pub baro_packet: Option<BaroPacket>,
    pub pitot_packet: Option<PitotPacket>,
    pub mag_packet: Option<MagPacket>,
    pub rc_packet: Option<RcPacket>,
    pub range_packet: Option<RangePacket>,
    pub gnss_packet: Option<GNSSPacket>,
    pub attitude_packet: Option<AttitudePacket>,
    pub serial_rx_packet: Option<SerialRxPacket>,
}

impl Sensors {
    pub fn new() -> Self {
        Self {
            rosflight_packet_header: None, // <-- when we're done pulling this value out via ownership, the estimator can replace this with "none" so that everyone else knows it's been processed... I could even have a check when we read in from the board to see: if we're not "None" here, it means the previous value wasn't processed and we can decide what to do.
            serial_tx_packet: None,
            adc_packet: None,
            battery_packet: None,
            imu_packet: None,
            baro_packet: None,
            pitot_packet: None,
            mag_packet: None,
            rc_packet: None,
            range_packet: None,
            gnss_packet: None,
            attitude_packet: None,
            serial_rx_packet: None,
        }
    }

    pub fn run<B: Board>(&mut self, board: &B) {

        #[cfg(feature = "default")]
        use core::fmt::Write;
        #[cfg(feature = "default")]
        let mut writer = host_rtt::RttWriter::new();

        match board.baro_read() {
            Some(Ok(baro_data)) => {
                self.baro_packet = Some(baro_data);

                #[cfg(feature="nucleo")]
                trace!("Baro: {} C, ({}) kPa\n",
                    self.baro_packet.as_ref().unwrap().pressure,
                    self.baro_packet.as_ref().unwrap().temperature);

                #[cfg(feature = "default")]
                write!(&mut writer, "Baro: {} C, ({}) kPa\n",
                    self.baro_packet.as_ref().unwrap().pressure,
                    self.baro_packet.as_ref().unwrap().temperature).unwrap();

            },
            Some(Err(e)) => {
                #[cfg(feature="nucleo")]
                trace!("Baro error: {:?}\n", e);

                #[cfg(feature = "default")]
                write!(&mut writer, "Baro error: {:?}\n", e).unwrap();

                self.baro_packet = None;
            },
            None => {
                self.baro_packet = None;

                #[cfg(feature = "default")]
                write!(&mut writer, "Baro not present\n").unwrap();
            }
        }

        match board.mag_read() {
            Some(Ok(mag_data)) => {
                self.mag_packet = Some(mag_data);

                #[cfg(feature="nucleo")]
                trace!("Mag: ({},{},{}) uT, Temp: {} C\n",
                    self.mag_packet.as_ref().unwrap().flux[0],
                    self.mag_packet.as_ref().unwrap().flux[1],
                    self.mag_packet.as_ref().unwrap().flux[2],
                    self.mag_packet.as_ref().unwrap().temperature);

                #[cfg(feature = "default")]
                write!(&mut writer, "Mag: ({},{},{}) uT, Temp: {} C\n",
                    self.mag_packet.as_ref().unwrap().flux[0],
                    self.mag_packet.as_ref().unwrap().flux[1],
                    self.mag_packet.as_ref().unwrap().flux[2],
                    self.mag_packet.as_ref().unwrap().temperature).unwrap();

            },
            Some(Err(e)) => {
                #[cfg(feature="nucleo")]
                trace!("Mag error: {:?}\n", e);

                #[cfg(feature = "default")]
                write!(&mut writer, "Mag error: {:?}\n", e).unwrap();

                self.mag_packet = None;
            },
            None => { // <-- if we don't have a mag, we can set the packet to None, and then we don't even need extra functions for checking if the mag is present or not...
                self.mag_packet = None;

                #[cfg(feature = "default")]
                write!(&mut writer, "Mag not present\n").unwrap();
            }
        }

        #[cfg(feature = "default")]
        writer.flush().unwrap();
    }
}
