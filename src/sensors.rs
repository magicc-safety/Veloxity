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

#[derive(Debug, Clone)]
pub enum SensorError {
    SpiError(embassy_stm32::spi::Error),
    GenericSensorError(&'static str),
}

pub struct Sensors {
    rosflight_packet_header: Result<Option<RosflightPacketHeader>, SensorError>,
    serial_tx_packet: Result<Option<SerialTxPacket>, SensorError>,
    adc_packet: Result<Option<AdcPacket>, SensorError>,
    battery_packet: Result<Option<BatteryPacket>, SensorError>,
    imu_packet: Result<Option<ImuPacket>, SensorError>,
    baro_packet: Result<Option<BaroPacket>, SensorError>,
    pitot_packet: Result<Option<PitotPacket>, SensorError>,
    mag_packet: Result<Option<MagPacket>, SensorError>,
    rc_packet: Result<Option<RcPacket>, SensorError>,
    range_packet: Result<Option<RangePacket>, SensorError>,
    gnss_packet: Result<Option<GNSSPacket>, SensorError>,
    attitude_packet: Result<Option<AttitudePacket>, SensorError>,
    serial_rx_packet: Result<Option<SerialRxPacket>, SensorError>
}

