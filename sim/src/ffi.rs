use std::fs;
use std::io::{self, Write};
use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Instant;

use chrono::{Datelike, TimeZone, Timelike, Utc};
use rustflight_core::{
    board::BoardIo,
    bodytype::quadrotor::Quadrotor,
    comm_manager::comm_link_trait::mavlink::MavlinkInterface,
    controller::quad_controller::QuadController,
    errors,
    estimator::quad_estimator::QuadEstimator,
    mixer::quad_mixer::QuadMixer,
    packets,
    params::{PARAM_DEFINITIONS, ParamValue, Params},
    pwm::{PwmDriver, PwmError, PwmOutputProtocol, effective_output_rate_hz, output_protocol_for_rate},
    sensors::SensorBus,
    state_machine::StateManager,
    world::World,
};

const NUM_PWM_CHANNELS: usize = 14;
const DEFAULT_MAVLINK_BIND: &str = "127.0.0.1:14525";
const DEFAULT_MAVLINK_REMOTE: &str = "127.0.0.1:14520";
const DEFAULT_PARAM_STORE: &str = "rustflight_sim.params";
const PARAM_STORE_ENV: &str = "RUSTFLIGHT_SIM_PARAM_STORE";

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RustflightFfiVector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RustflightFfiImu {
    pub timestamp_us: u64,
    pub angular_velocity: RustflightFfiVector3,
    pub linear_acceleration: RustflightFfiVector3,
    pub temperature_kelvin: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RustflightFfiMag {
    pub timestamp_us: u64,
    pub magnetic_field: RustflightFfiVector3,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RustflightFfiBaro {
    pub timestamp_us: u64,
    pub altitude: f32,
    pub pressure: f32,
    pub temperature_kelvin: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RustflightFfiGnss {
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
pub struct RustflightFfiAirspeed {
    pub timestamp_us: u64,
    pub differential_pressure: f32,
    pub temperature_kelvin: f32,
    pub indicated_airspeed: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RustflightFfiRange {
    pub timestamp_us: u64,
    pub range: f32,
    pub min_range: f32,
    pub max_range: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RustflightFfiBattery {
    pub timestamp_us: u64,
    pub voltage: f32,
    pub current: f32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct RustflightFfiRc {
    pub timestamp_us: u64,
    pub values: [u16; 8],
}

impl Default for RustflightFfiRc {
    fn default() -> Self {
        Self {
            timestamp_us: 0,
            values: [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000],
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct RustflightFfiSensorSnapshot {
    pub has_imu: bool,
    pub imu: RustflightFfiImu,
    pub has_mag: bool,
    pub mag: RustflightFfiMag,
    pub has_baro: bool,
    pub baro: RustflightFfiBaro,
    pub has_gnss: bool,
    pub gnss: RustflightFfiGnss,
    pub has_airspeed: bool,
    pub airspeed: RustflightFfiAirspeed,
    pub has_range: bool,
    pub range: RustflightFfiRange,
    pub has_battery: bool,
    pub battery: RustflightFfiBattery,
    pub has_rc: bool,
    pub rc: RustflightFfiRc,
}

#[derive(Default)]
struct SharedSensors {
    snapshot: RustflightFfiSensorSnapshot,
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

impl PwmDriver for FfiPwmDriver {
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
}

impl FfiBoard {
    fn new(sensors: Arc<Mutex<SharedSensors>>) -> io::Result<Self> {
        let bind_addr: SocketAddr = std::env::var("RUSTFLIGHT_MAVLINK_BIND")
            .unwrap_or_else(|_| DEFAULT_MAVLINK_BIND.into())
            .parse()
            .map_err(|err| io::Error::new(io::ErrorKind::InvalidInput, err))?;
        let remote_addr: SocketAddr = std::env::var("RUSTFLIGHT_MAVLINK_REMOTE")
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
            param_store_path: param_store_path(),
        })
    }
}

impl BoardIo for FfiBoard {
    fn update_sensor_bus(&mut self, sensors: &mut SensorBus) {
        sensors.clear();
        let Ok(shared) = self.sensors.lock() else {
            return;
        };
        let snapshot = shared.snapshot;

        if snapshot.has_imu {
            sensors.imu = Some(Ok(packets::ImuPacket {
                header: packets::RosflightPacketHeader {
                    timestamp: snapshot.imu.timestamp_us,
                    status: 0,
                },
                accel: [
                    snapshot.imu.linear_acceleration.x,
                    snapshot.imu.linear_acceleration.y,
                    snapshot.imu.linear_acceleration.z,
                ],
                gyro: [
                    snapshot.imu.angular_velocity.x,
                    snapshot.imu.angular_velocity.y,
                    snapshot.imu.angular_velocity.z,
                ],
                temperature: snapshot.imu.temperature_kelvin,
                seq: 0,
            }));
        }

        if snapshot.has_mag {
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

        if snapshot.has_baro {
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

        if snapshot.has_gnss {
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
                lat: snapshot.gnss.lat_degrees.to_radians(),
                lon: snapshot.gnss.lon_degrees.to_radians(),
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

        if snapshot.has_airspeed {
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

        if snapshot.has_range {
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

        if snapshot.has_battery {
            sensors.battery = Some(Ok(packets::BatteryPacket {
                header: packets::RosflightPacketHeader {
                    timestamp: snapshot.battery.timestamp_us,
                    status: 0,
                },
                voltage: snapshot.battery.voltage,
                current: snapshot.battery.current,
            }));
        }

        if snapshot.has_rc {
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
        match self.mavlink_socket.recv(buf) {
            Ok(size) => Some(Ok(size)),
            Err(err) if err.kind() == io::ErrorKind::WouldBlock => None,
            Err(_) => Some(Err(errors::TelemError::GenericTelemError(
                "error reading MAVLink UDP socket",
            ))),
        }
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

type FfiWorld = World<FfiBoard, Quadrotor, MavlinkInterface, FfiPwmDriver>;

pub struct RustflightFfiHandle {
    sensors: Arc<Mutex<SharedSensors>>,
    outputs: Arc<Mutex<[u16; NUM_PWM_CHANNELS]>>,
    world: FfiWorld,
}

#[unsafe(no_mangle)]
pub extern "C" fn rustflight_sim_create() -> *mut RustflightFfiHandle {
    let sensors = Arc::new(Mutex::new(SharedSensors::default()));
    let outputs = Arc::new(Mutex::new([1000; NUM_PWM_CHANNELS]));

    let Ok(board) = FfiBoard::new(Arc::clone(&sensors)) else {
        return std::ptr::null_mut();
    };

    let params = Params::new();
    let estimator = QuadEstimator::default();
    let controller = QuadController::default();
    let mixer = QuadMixer::new(&params);
    let mavlink = MavlinkInterface::new();
    let state = StateManager::new();
    let pwm = FfiPwmDriver::new(Arc::clone(&outputs));

    let world = World::<FfiBoard, Quadrotor, MavlinkInterface, FfiPwmDriver>::init(
        board, params, mavlink, state, estimator, controller, mixer, pwm,
    );

    Box::into_raw(Box::new(RustflightFfiHandle {
        sensors,
        outputs,
        world,
    }))
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rustflight_sim_destroy(handle: *mut RustflightFfiHandle) {
    if !handle.is_null() {
        drop(unsafe { Box::from_raw(handle) });
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rustflight_sim_set_sensors(
    handle: *mut RustflightFfiHandle,
    snapshot: *const RustflightFfiSensorSnapshot,
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
pub unsafe extern "C" fn rustflight_sim_run_once(handle: *mut RustflightFfiHandle) -> bool {
    if handle.is_null() {
        return false;
    }

    let handle = unsafe { &mut *handle };
    handle.world.run_once()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rustflight_sim_get_pwm(
    handle: *const RustflightFfiHandle,
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

fn param_store_path() -> PathBuf {
    std::env::var_os(PARAM_STORE_ENV)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(DEFAULT_PARAM_STORE))
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
