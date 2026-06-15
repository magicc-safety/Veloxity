use std::fs;
use std::io::{self, Write};
use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use chrono::{Datelike, TimeZone, Timelike, Utc};
use veloxity_core::{
    board::BoardIo,
    controller::quad::QuadController,
    errors,
    estimator::quad::QuadEstimator,
    math::FlightFloat,
    mixer::matrix::MatrixMixer,
    packets,
    params::{PARAM_DEFINITIONS, ParamValue, Params},
    pwm::{
        PwmDriver, PwmError, PwmOutputProtocol, effective_output_rate_hz, output_protocol_for_rate,
    },
    sensors::SensorBus,
    state_machine::StateManager,
    world::World,
};
use veloxity_mavlink::MavlinkInterface;

const NUM_PWM_CHANNELS: usize = 14;
const DEFAULT_MAVLINK_BIND: &str = "127.0.0.1:14525";
const DEFAULT_MAVLINK_REMOTE: &str = "127.0.0.1:14520";
const PARAM_DIR_ENV: &str = "VELOXITY_SIM_PARAM_DIR";
const PARAM_STORE_FILE: &str = "veloxity_sim.params";

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VeloxityFfiVector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VeloxityFfiImu {
    pub timestamp_us: u64,
    pub angular_velocity: VeloxityFfiVector3,
    pub linear_acceleration: VeloxityFfiVector3,
    pub temperature_kelvin: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VeloxityFfiMag {
    pub timestamp_us: u64,
    pub magnetic_field: VeloxityFfiVector3,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VeloxityFfiBaro {
    pub timestamp_us: u64,
    pub altitude: f32,
    pub pressure: f32,
    pub temperature_kelvin: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VeloxityFfiGnss {
    pub timestamp_us: u64,
    pub fix_type: u8,
    pub num_sat: u8,
    pub lat_degrees: f64,
    pub lon_degrees: f64,
    pub alt: f32,
    pub horizontal_accuracy: f32,
    pub vertical_accuracy: f32,
    pub vel_n: f32,
    pub vel_e: f32,
    pub vel_d: f32,
    pub speed_accuracy: f32,
    pub unix_seconds: i64,
    pub unix_nanos: i32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VeloxityFfiAirspeed {
    pub timestamp_us: u64,
    pub differential_pressure: f32,
    pub temperature_kelvin: f32,
    pub indicated_airspeed: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VeloxityFfiRange {
    pub timestamp_us: u64,
    pub range: f32,
    pub min_range: f32,
    pub max_range: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VeloxityFfiBattery {
    pub timestamp_us: u64,
    pub voltage: f32,
    pub current: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct VeloxityFfiRc {
    pub timestamp_us: u64,
    pub values: [u16; 8],
}

impl Default for VeloxityFfiRc {
    fn default() -> Self {
        Self {
            timestamp_us: 0,
            values: [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000],
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct VeloxityFfiSensorSnapshot {
    pub has_imu: bool,
    pub imu: VeloxityFfiImu,
    pub has_mag: bool,
    pub mag: VeloxityFfiMag,
    pub has_baro: bool,
    pub baro: VeloxityFfiBaro,
    pub has_gnss: bool,
    pub gnss: VeloxityFfiGnss,
    pub has_airspeed: bool,
    pub airspeed: VeloxityFfiAirspeed,
    pub has_range: bool,
    pub range: VeloxityFfiRange,
    pub has_battery: bool,
    pub battery: VeloxityFfiBattery,
    pub has_rc: bool,
    pub rc: VeloxityFfiRc,
}

#[derive(Default)]
struct SharedSensors {
    snapshot: VeloxityFfiSensorSnapshot,
}

#[derive(Clone)]
struct FfiPwmDriver {
    outputs: Arc<Mutex<[u16; NUM_PWM_CHANNELS]>>,
    output_rates_hz: [f64; NUM_PWM_CHANNELS],
    output_protocols: [PwmOutputProtocol; NUM_PWM_CHANNELS],
}

impl FfiPwmDriver {
    fn new(outputs: Arc<Mutex<[u16; NUM_PWM_CHANNELS]>>) -> Self {
        Self {
            outputs,
            output_rates_hz: [50.0; NUM_PWM_CHANNELS],
            output_protocols: [PwmOutputProtocol::StandardPwm; NUM_PWM_CHANNELS],
        }
    }

    fn set_pwm(&mut self, channel: usize, pwm_us: u16) -> Result<(), PwmError> {
        if channel >= NUM_PWM_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        if let Ok(mut outputs) = self.outputs.lock() {
            outputs[channel] = pwm_us;
            Ok(())
        } else {
            Err(PwmError::GenericError)
        }
    }
}

impl PwmDriver<f64> for FfiPwmDriver {
    fn len(&self) -> usize {
        NUM_PWM_CHANNELS
    }

    fn is_enabled(&self) -> bool {
        true
    }

    fn enable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= NUM_PWM_CHANNELS {
            Err(PwmError::ChannelOutOfRange)
        } else {
            Ok(())
        }
    }

    fn disable(&mut self, channel: usize) -> Result<(), PwmError> {
        self.set_pwm(channel, 1000)
    }

    fn enable_all(&mut self) -> Result<(), PwmError> {
        Ok(())
    }

    fn disable_all(&mut self) {
        if let Ok(mut outputs) = self.outputs.lock() {
            outputs.fill(1000);
        }
    }

    fn set_duty_cycle(&mut self, channel: usize, duty: u16) -> Result<(), PwmError> {
        let normalized = duty as f32 / u16::MAX as f32;
        self.set_pwm(channel, (1000.0 + normalized * 1000.0) as u16)
    }

    fn flush<B: BoardIo>(&mut self, _board: &mut B) {}

    fn configure_output_rates(&mut self, rates_hz: &[f64]) -> Result<(), PwmError> {
        for (index, rate) in rates_hz.iter().take(NUM_PWM_CHANNELS).enumerate() {
            self.output_protocols[index] = output_protocol_for_rate(*rate)?;
            self.output_rates_hz[index] = effective_output_rate_hz(*rate)?;
        }
        Ok(())
    }

    fn send_commands<B: BoardIo>(
        &mut self,
        _board: &mut B,
        commands: &[f64],
    ) -> Result<(), PwmError> {
        for (channel, command) in commands.iter().take(NUM_PWM_CHANNELS).enumerate() {
            let pwm_us = 1000.0 + command.clamp(0.0, 1.0) * 1000.0;
            self.set_pwm(channel, pwm_us as u16)?;
        }
        Ok(())
    }
}

struct FfiBoard {
    start_time: Instant,
    mavlink_socket: UdpSocket,
    sensors: Arc<Mutex<SharedSensors>>,
    param_store_path: PathBuf,
    last_imu_timestamp_us: u64,
    last_mag_timestamp_us: u64,
    last_baro_timestamp_us: u64,
    last_gnss_timestamp_us: u64,
    last_airspeed_timestamp_us: u64,
    last_range_timestamp_us: u64,
    last_battery_timestamp_us: u64,
    last_rc_timestamp_us: u64,
    #[cfg(feature = "timing-diagnostics")]
    last_serial_rx_count: usize,
}

impl FfiBoard {
    fn new(sensors: Arc<Mutex<SharedSensors>>) -> io::Result<Self> {
        let bind_addr: SocketAddr = std::env::var("VELOXITY_MAVLINK_BIND")
            .unwrap_or_else(|_| DEFAULT_MAVLINK_BIND.into())
            .parse()
            .map_err(|err| io::Error::new(io::ErrorKind::InvalidInput, err))?;
        let remote_addr: SocketAddr = std::env::var("VELOXITY_MAVLINK_REMOTE")
            .unwrap_or_else(|_| DEFAULT_MAVLINK_REMOTE.into())
            .parse()
            .map_err(|err| io::Error::new(io::ErrorKind::InvalidInput, err))?;
        let mavlink_socket = UdpSocket::bind(bind_addr)?;
        mavlink_socket.connect(remote_addr)?;
        mavlink_socket.set_nonblocking(true)?;

        Ok(Self {
            start_time: Instant::now(),
            mavlink_socket,
            sensors,
            param_store_path: param_store_path()?,
            last_imu_timestamp_us: 0,
            last_mag_timestamp_us: 0,
            last_baro_timestamp_us: 0,
            last_gnss_timestamp_us: 0,
            last_airspeed_timestamp_us: 0,
            last_range_timestamp_us: 0,
            last_battery_timestamp_us: 0,
            last_rc_timestamp_us: 0,
            #[cfg(feature = "timing-diagnostics")]
            last_serial_rx_count: 0,
        })
    }
}

impl BoardIo for FfiBoard {
    fn update_sensor_bus<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        sensors.clear();
        let Ok(shared) = self.sensors.lock() else {
            return;
        };
        let snapshot = shared.snapshot;

        if snapshot.has_imu && snapshot.imu.timestamp_us > self.last_imu_timestamp_us {
            self.last_imu_timestamp_us = snapshot.imu.timestamp_us;
            sensors.imu = Some(Ok(packets::ImuPacket {
                header: packets::RosflightPacketHeader {
                    timestamp: snapshot.imu.timestamp_us,
                    status: 0,
                },
                accel: [
                    <R as FlightFloat>::from_f64(snapshot.imu.linear_acceleration.x),
                    <R as FlightFloat>::from_f64(snapshot.imu.linear_acceleration.y),
                    <R as FlightFloat>::from_f64(snapshot.imu.linear_acceleration.z),
                ],
                gyro: [
                    <R as FlightFloat>::from_f64(snapshot.imu.angular_velocity.x),
                    <R as FlightFloat>::from_f64(snapshot.imu.angular_velocity.y),
                    <R as FlightFloat>::from_f64(snapshot.imu.angular_velocity.z),
                ],
                temperature: snapshot.imu.temperature_kelvin,
                seq: 0,
            }));
        }

        if snapshot.has_mag && snapshot.mag.timestamp_us > self.last_mag_timestamp_us {
            self.last_mag_timestamp_us = snapshot.mag.timestamp_us;
            sensors.mag = Some(Ok(packets::MagPacket {
                header: packets::RosflightPacketHeader {
                    timestamp: snapshot.mag.timestamp_us,
                    status: 0,
                },
                flux: [
                    snapshot.mag.magnetic_field.x as f32,
                    snapshot.mag.magnetic_field.y as f32,
                    snapshot.mag.magnetic_field.z as f32,
                ],
                temperature: 25.0,
            }));
        }

        if snapshot.has_baro && snapshot.baro.timestamp_us > self.last_baro_timestamp_us {
            self.last_baro_timestamp_us = snapshot.baro.timestamp_us;
            sensors.baro = Some(Ok(packets::BaroPacket {
                header: packets::RosflightPacketHeader {
                    timestamp: snapshot.baro.timestamp_us,
                    status: 0,
                },
                pressure: snapshot.baro.pressure,
                temperature: snapshot.baro.temperature_kelvin,
                altitude: snapshot.baro.altitude,
            }));
        }

        if snapshot.has_gnss && snapshot.gnss.timestamp_us > self.last_gnss_timestamp_us {
            self.last_gnss_timestamp_us = snapshot.gnss.timestamp_us;
            let dt = Utc
                .timestamp_opt(snapshot.gnss.unix_seconds, snapshot.gnss.unix_nanos as u32)
                .latest()
                .unwrap_or_default();
            sensors.gnss = Some(Ok(packets::GNSSPacket {
                header: packets::RosflightPacketHeader {
                    timestamp: snapshot.gnss.timestamp_us,
                    status: 0,
                },
                unix_seconds: snapshot.gnss.unix_seconds,
                unix_nanos: snapshot.gnss.unix_nanos,
                lat: snapshot.gnss.lat_degrees,
                lon: snapshot.gnss.lon_degrees,
                height: snapshot.gnss.alt,
                vel_n: snapshot.gnss.vel_n,
                vel_e: snapshot.gnss.vel_e,
                vel_d: snapshot.gnss.vel_d,
                h_acc: snapshot.gnss.horizontal_accuracy,
                v_acc: snapshot.gnss.vertical_accuracy,
                s_acc: snapshot.gnss.speed_accuracy,
                month: dt.month0() as u8,
                year: dt.year() as u16,
                day: dt.day() as u8,
                hour: dt.hour() as u8,
                min: dt.minute() as u8,
                sec: dt.second() as u8,
                nano: dt.nanosecond() as i32,
                fix_type: packets::GNSSFixType::from_u8(snapshot.gnss.fix_type),
                num_sats: snapshot.gnss.num_sat,
                mag_dec: 0.0,
                time_correction: 0,
            }));
        }

        if snapshot.has_airspeed && snapshot.airspeed.timestamp_us > self.last_airspeed_timestamp_us
        {
            self.last_airspeed_timestamp_us = snapshot.airspeed.timestamp_us;
            sensors.pitot = Some(Ok(packets::PitotPacket {
                header: packets::RosflightPacketHeader {
                    timestamp: snapshot.airspeed.timestamp_us,
                    status: 0,
                },
                differential_pressure: snapshot.airspeed.differential_pressure,
                temperature: snapshot.airspeed.temperature_kelvin,
                indicated_airspeed: snapshot.airspeed.indicated_airspeed,
            }));
        }

        if snapshot.has_range && snapshot.range.timestamp_us > self.last_range_timestamp_us {
            self.last_range_timestamp_us = snapshot.range.timestamp_us;
            sensors.range = Some(Ok(packets::RangePacket {
                header: packets::RosflightPacketHeader {
                    timestamp: snapshot.range.timestamp_us,
                    status: 0,
                },
                range: snapshot.range.range,
                min_range: snapshot.range.min_range,
                max_range: snapshot.range.max_range,
                range_type: packets::RangeType::Sonar,
            }));
        }

        if snapshot.has_battery && snapshot.battery.timestamp_us > self.last_battery_timestamp_us {
            self.last_battery_timestamp_us = snapshot.battery.timestamp_us;
            sensors.battery = Some(Ok(packets::BatteryPacket {
                header: packets::RosflightPacketHeader {
                    timestamp: snapshot.battery.timestamp_us,
                    status: 0,
                },
                voltage: snapshot.battery.voltage,
                current: snapshot.battery.current,
            }));
        }

        if snapshot.has_rc && snapshot.rc.timestamp_us > self.last_rc_timestamp_us {
            self.last_rc_timestamp_us = snapshot.rc.timestamp_us;
            let mut channels = [0.0f32; packets::RC_PACKET_CHANNELS];
            for (index, value) in snapshot.rc.values.iter().enumerate() {
                channels[index] = (*value as f32 - 1000.0) / 1000.0;
            }
            sensors.rc = Some(Ok(packets::RcPacket {
                header: packets::RosflightPacketHeader {
                    timestamp: snapshot.rc.timestamp_us,
                    status: 0,
                },
                n_chan: snapshot.rc.values.len() as u32,
                chan: channels,
                lol: false,
            }));
        }
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        let result = match self.mavlink_socket.recv(buf) {
            Ok(size) => {
                #[cfg(feature = "timing-diagnostics")]
                {
                    self.last_serial_rx_count = size;
                }
                Some(Ok(size))
            }
            Err(err) if err.kind() == io::ErrorKind::WouldBlock => {
                #[cfg(feature = "timing-diagnostics")]
                {
                    self.last_serial_rx_count = 0;
                }
                None
            }
            Err(_) => {
                #[cfg(feature = "timing-diagnostics")]
                {
                    self.last_serial_rx_count = 0;
                }
                Some(Err(errors::TelemError::GenericTelemError(
                    "error reading MAVLink UDP socket",
                )))
            }
        };
        result
    }

    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        match self.mavlink_socket.send(bytes) {
            Ok(size) => Some(Ok(size)),
            Err(err) if err.kind() == io::ErrorKind::WouldBlock => Some(Err(
                errors::TelemError::GenericTelemError("MAVLink UDP socket send buffer full"),
            )),
            Err(_) => Some(Err(errors::TelemError::GenericTelemError(
                "error writing MAVLink UDP socket",
            ))),
        }
    }

    #[cfg(feature = "timing-diagnostics")]
    fn serial_rx_last_count(&self) -> usize {
        self.last_serial_rx_count
    }

    fn clock_millis(&self) -> u32 {
        self.start_time.elapsed().as_millis() as u32
    }

    fn clock_micros(&self) -> u64 {
        self.start_time.elapsed().as_micros() as u64
    }

    fn read_params(&mut self, params: &mut Params) -> bool {
        read_params_from_path(&self.param_store_path, params).is_ok()
    }

    fn write_params(&mut self, params: &Params) -> bool {
        write_params_to_path(&self.param_store_path, params).is_ok()
    }
}

type FfiWorld = World<
    FfiBoard,
    QuadEstimator<f64>,
    QuadController<f64>,
    MatrixMixer<f64>,
    MavlinkInterface,
    FfiPwmDriver,
    f64,
>;

pub struct VeloxityFfiHandle {
    sensors: Arc<Mutex<SharedSensors>>,
    outputs: Arc<Mutex<[u16; NUM_PWM_CHANNELS]>>,
    world: FfiWorld,
}

#[unsafe(no_mangle)]
pub extern "C" fn veloxity_sim_create() -> *mut VeloxityFfiHandle {
    let sensors = Arc::new(Mutex::new(SharedSensors::default()));
    let outputs = Arc::new(Mutex::new([1000; NUM_PWM_CHANNELS]));

    let Ok(mut board) = FfiBoard::new(Arc::clone(&sensors)) else {
        return std::ptr::null_mut();
    };

    let mut params = Params::new();
    let _ = board.read_params(&mut params);
    let estimator = QuadEstimator::default();
    let controller = QuadController::default();
    let mixer = MatrixMixer::new(&params);
    let mavlink = MavlinkInterface::new();
    let state = StateManager::new();
    let pwm = FfiPwmDriver::new(Arc::clone(&outputs));

    let world = FfiWorld::init(
        board, params, mavlink, state, estimator, controller, mixer, pwm,
    );

    Box::into_raw(Box::new(VeloxityFfiHandle {
        sensors,
        outputs,
        world,
    }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn veloxity_sim_destroy(handle: *mut VeloxityFfiHandle) {
    if !handle.is_null() {
        drop(unsafe { Box::from_raw(handle) });
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn veloxity_sim_set_sensors(
    handle: *mut VeloxityFfiHandle,
    snapshot: *const VeloxityFfiSensorSnapshot,
) -> bool {
    if handle.is_null() || snapshot.is_null() {
        return false;
    }

    let handle = unsafe { &mut *handle };
    let Ok(mut sensors) = handle.sensors.lock() else {
        return false;
    };
    sensors.snapshot = unsafe { *snapshot };
    true
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn veloxity_sim_run_once(handle: *mut VeloxityFfiHandle) -> bool {
    if handle.is_null() {
        return false;
    }

    let handle = unsafe { &mut *handle };
    handle.world.run_once()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn veloxity_sim_get_pwm(
    handle: *const VeloxityFfiHandle,
    output: *mut u16,
    output_len: usize,
) -> usize {
    if handle.is_null() || output.is_null() {
        return 0;
    }

    let handle = unsafe { &*handle };
    let Ok(outputs) = handle.outputs.lock() else {
        return 0;
    };
    let copy_len = output_len.min(outputs.len());
    unsafe {
        std::ptr::copy_nonoverlapping(outputs.as_ptr(), output, copy_len);
    }
    copy_len
}

fn param_store_path() -> io::Result<PathBuf> {
    let Some(dir) = std::env::var_os(PARAM_DIR_ENV) else {
        return Err(io::Error::new(
            io::ErrorKind::NotFound,
            "VELOXITY_SIM_PARAM_DIR must point to a writable runtime parameter directory",
        ));
    };
    let dir = PathBuf::from(dir);
    fs::create_dir_all(&dir)?;
    Ok(dir.join(PARAM_STORE_FILE))
}

fn write_params_to_path(path: &Path, params: &Params) -> io::Result<()> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent)?;
    }

    let mut contents = Vec::new();
    for definition in PARAM_DEFINITIONS.iter() {
        writeln!(
            contents,
            "{}={}",
            definition.name,
            format_param_value(params.get_by_id(definition.id))
        )?;
    }

    let temp_path = path.with_extension("tmp");
    fs::write(&temp_path, contents)?;
    fs::rename(temp_path, path)
}

fn read_params_from_path(path: &Path, params: &mut Params) -> io::Result<()> {
    let contents = fs::read_to_string(path)?;

    for line in contents.lines() {
        let Some((name, value)) = line.split_once('=') else {
            continue;
        };
        let Some(definition) = PARAM_DEFINITIONS
            .iter()
            .find(|definition| definition.name == name)
        else {
            continue;
        };
        let Some(parsed) = parse_param_value(value, definition.default) else {
            continue;
        };
        params.set_by_id(definition.id, parsed);
    }

    Ok(())
}

fn format_param_value(value: ParamValue) -> String {
    match value {
        ParamValue::Float(value) => value.to_string(),
        ParamValue::Int(value) => value.to_string(),
        ParamValue::Uint(value) => value.to_string(),
        ParamValue::Bool(value) => value.to_string(),
    }
}

fn parse_param_value(value: &str, default: ParamValue) -> Option<ParamValue> {
    match default {
        ParamValue::Float(_) => value.parse().ok().map(ParamValue::Float),
        ParamValue::Int(_) => value.parse().ok().map(ParamValue::Int),
        ParamValue::Uint(_) => value.parse().ok().map(ParamValue::Uint),
        ParamValue::Bool(_) => value.parse().ok().map(ParamValue::Bool),
    }
}
