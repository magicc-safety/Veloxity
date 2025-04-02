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
use embassy_time::Instant;
use embassy_time::Duration;
use defmt::Format;
use embassy_stm32;
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

#[derive(Format)]
pub enum Qos
{
    High, Medium, Low,
}

#[derive(Format)]
pub enum RangeType
{
    Sonar, Lidar,
}

#[derive(Format)]
pub enum GNSSFixType
{
    NoFix, DeadReckoningOnly,TwoD, ThreeD, GnssPlusDeadReckoning, TimeFixOnly,
}

#[derive(Format)]
pub struct RosflightPacketHeader
{
    pub timestamp :Instant, pub status: u16
}

#[derive(Format)]
pub struct SerialTxPacket { pub header: RosflightPacketHeader, pub qos: Qos, pub len: i16,  pub payload: [u8;SERIAL_MAX_PAYLOAD_SIZE]}

#[derive(Format)]
pub struct AdcPacket {pub header : RosflightPacketHeader, pub temperature: f32, pub v_bku: f32, pub v_ref : f32, pub volts: [f32;ADC_MAX_CHANNELS]}

#[derive(Format)]
pub struct BatteryPacket { pub header : RosflightPacketHeader, pub voltage :f32, pub current :f32}

#[derive(Format)]
pub struct ImuPacket { pub header : RosflightPacketHeader, pub accel :[f64;3], pub gyro :[f64;3], pub temperature :f32, pub seq: u16 }

#[derive(Format)]
pub struct BaroPacket { pub header : RosflightPacketHeader, pub pressure : f32, pub temperature : f32}

#[derive(Format)]
pub struct PitotPacket { pub header : RosflightPacketHeader, pub pressure : f32, pub temperature : f32}

#[derive(Format)]
pub struct MagPacket {pub header:RosflightPacketHeader, pub flux: [f32;3], pub temperature : f32}

#[derive(Format)]
pub struct RcPacket { pub header: RosflightPacketHeader, pub n_chan: u32, pub chan: [f32;RC_PACKET_CHANNELS], pub frame_lost : bool, pub rc_packet_lost : bool}

#[derive(Format)]
pub struct RangePacket {pub header: RosflightPacketHeader, pub range : f32, pub in_range :f32, pub max_range: f32, pub range_type: RangeType }

#[derive(Format)]
pub struct GNSSPacket {pub header: RosflightPacketHeader, pub pps: u64, pub fix_type: GNSSFixType} // lots more parameters for later

#[derive(Format)]
pub struct AttitudePacket {pub header: RosflightPacketHeader, pub q: [f32;4], pub rate: [f32;3]}
// not really needed:

#[derive(Format)]
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

#[derive(Debug, Clone, defmt::Format)]
pub enum SensorError {
    GenericSensorError(&'static str),
}

pub struct Sensors {
    rosflight_packet_header: Option<RosflightPacketHeader>,
    serial_tx_packet: Option<SerialTxPacket>,
    adc_packet: Option<AdcPacket>,
    battery_packet: Option<BatteryPacket>,
    imu_packet: Option<ImuPacket>,
    baro_packet: Option<BaroPacket>,
    pitot_packet: Option<PitotPacket>,
    mag_packet: Option<MagPacket>,
    rc_packet: Option<RcPacket>,
    range_packet: Option<RangePacket>,
    gnss_packet: Option<GNSSPacket>,
    attitude_packet: Option<AttitudePacket>,
    serial_rx_packet: Option<SerialRxPacket>,
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
        match board.baro_read() {
            Some(Ok(baro_data)) => {
                self.baro_packet = Some(baro_data);

                #[cfg(feature="nucleo")]
                defmt::trace!("Baro: {} C, ({}) kPa\n",
                    self.baro_packet.as_ref().unwrap().pressure,
                    self.baro_packet.as_ref().unwrap().temperature);
            },
            Some(Err(e)) => {
                defmt::trace!("Baro error: {:?}", e);
                self.baro_packet = None;
            },
            None => {
                self.baro_packet = None;
            }
        }

        match board.mag_read() {
            Some(Ok(mag_data)) => {
                self.mag_packet = Some(mag_data);

                #[cfg(feature="nucleo")]
                defmt::trace!("Mag: ({},{},{}) uT, Temp: {} C\n",
                    self.mag_packet.as_ref().unwrap().flux[0],
                    self.mag_packet.as_ref().unwrap().flux[1],
                    self.mag_packet.as_ref().unwrap().flux[2],
                    self.mag_packet.as_ref().unwrap().temperature);
            },
            Some(Err(e)) => {
                defmt::trace!("Mag error: {:?}", e);
                self.mag_packet = None;
            },
            None => {
                self.mag_packet = None;
            }
        }
    }
}
