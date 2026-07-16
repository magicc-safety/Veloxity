use std::fs;
use std::io::{self, Write};
use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::sync::{
    Arc, Condvar, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use chrono::{Datelike, TimeZone, Timelike, Utc};
use veloxity_core::{
    board::BoardIo,
    controller::quad::QuadController,
    errors,
    estimator::quad::QuadEstimator,
    math::FlightFloat,
    mixer::{MixerOutputType, matrix::MatrixMixer},
    packets,
    params::{PARAM_DEFINITIONS, ParamValue, Params},
    pwm::{
        PwmDriver, PwmError, PwmOutputProtocol, effective_output_rate_hz, output_protocol_for_rate,
        safe_disarmed_command,
    },
    sensors::SensorBus,
    state_machine::StateManager,
    world::{ControlLoopRates, RealtimeSchedulerStep, RealtimeServicePolicy, World},
};
use veloxity_mavlink::MavlinkInterface;

const NUM_PWM_CHANNELS: usize = 14;
const DEFAULT_MAVLINK_BIND: &str = "127.0.0.1:14525";
const DEFAULT_MAVLINK_REMOTE: &str = "127.0.0.1:14520";
const PARAM_DIR_ENV: &str = "VELOXITY_SIM_PARAM_DIR";
const PARAM_STORE_FILE: &str = "veloxity_sim.params";
const FIRMWARE_SYNC_TIMEOUT: Duration = Duration::from_millis(5);
const SIM_CONTROL_LOOP_HZ: u16 = 400;

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
    // Mirrors the hardware Signal slots: each value remains pending until the board consumes it,
    // while a newer value replaces an older unconsumed value.
    pending: VeloxityFfiSensorSnapshot,
    latest_imu_generation: u64,
    pending_imu_generation: u64,
    consumed_imu_generation: u64,
}

impl SharedSensors {
    fn merge(&mut self, incoming: VeloxityFfiSensorSnapshot) {
        macro_rules! replace_pending {
            ($has:ident, $value:ident) => {
                if incoming.$has {
                    self.pending.$has = true;
                    self.pending.$value = incoming.$value;
                }
            };
        }

        replace_pending!(has_imu, imu);
        replace_pending!(has_mag, mag);
        replace_pending!(has_baro, baro);
        replace_pending!(has_gnss, gnss);
        replace_pending!(has_airspeed, airspeed);
        replace_pending!(has_range, range);
        replace_pending!(has_battery, battery);
        replace_pending!(has_rc, rc);

        if incoming.has_imu {
            self.latest_imu_generation = self.latest_imu_generation.wrapping_add(1).max(1);
            self.pending_imu_generation = self.latest_imu_generation;
        }
    }

    fn imu_pending(&self) -> bool {
        self.pending.has_imu
    }

    fn take_imu(&mut self) -> Option<VeloxityFfiImu> {
        if !self.pending.has_imu {
            return None;
        }
        self.pending.has_imu = false;
        self.consumed_imu_generation = self.pending_imu_generation;
        Some(self.pending.imu)
    }

    fn take_snapshot(&mut self, include_imu: bool) -> VeloxityFfiSensorSnapshot {
        let mut snapshot = self.pending;
        if include_imu && snapshot.has_imu {
            self.pending.has_imu = false;
            self.consumed_imu_generation = self.pending_imu_generation;
        } else {
            snapshot.has_imu = false;
        }
        self.pending.has_mag = false;
        self.pending.has_baro = false;
        self.pending.has_gnss = false;
        self.pending.has_airspeed = false;
        self.pending.has_range = false;
        self.pending.has_battery = false;
        self.pending.has_rc = false;
        snapshot
    }
}

struct FirmwareProgress {
    processed_imu_generation: u64,
    pwm_outputs: [u16; NUM_PWM_CHANNELS],
    worker_failed: bool,
}

impl Default for FirmwareProgress {
    fn default() -> Self {
        Self {
            processed_imu_generation: 0,
            pwm_outputs: [1000; NUM_PWM_CHANNELS],
            worker_failed: false,
        }
    }
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

    fn output_protocol(&self, channel: usize) -> Result<PwmOutputProtocol, PwmError> {
        self.output_protocols
            .get(channel)
            .copied()
            .ok_or(PwmError::ChannelOutOfRange)
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

    fn send_disarmed_commands<B: BoardIo>(
        &mut self,
        _board: &mut B,
        output_types: &[MixerOutputType],
    ) -> Result<(), PwmError> {
        for (channel, output_type) in output_types.iter().take(NUM_PWM_CHANNELS).enumerate() {
            let command = safe_disarmed_command::<f64>(*output_type);
            let output = match self.output_protocols[channel] {
                PwmOutputProtocol::StandardPwm => 1000.0 + command * 1000.0,
                PwmOutputProtocol::Dshot if *output_type == MixerOutputType::Motor => 0.0,
                PwmOutputProtocol::Dshot => command,
            };
            self.set_pwm(channel, output as u16)?;
        }
        Ok(())
    }
}

struct FfiBoard {
    start_time: Instant,
    mavlink_socket: UdpSocket,
    sensors: Arc<Mutex<SharedSensors>>,
    param_store_path: PathBuf,
    last_mag_timestamp_us: u64,
    last_baro_timestamp_us: u64,
    last_gnss_timestamp_us: u64,
    last_airspeed_timestamp_us: u64,
    last_range_timestamp_us: u64,
    last_battery_timestamp_us: u64,
    last_rc_timestamp_us: u64,
}

impl FfiBoard {
    fn new(sensors: Arc<Mutex<SharedSensors>>, start_time: Instant) -> io::Result<Self> {
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
            start_time,
            mavlink_socket,
            sensors,
            param_store_path: param_store_path()?,
            last_mag_timestamp_us: 0,
            last_baro_timestamp_us: 0,
            last_gnss_timestamp_us: 0,
            last_airspeed_timestamp_us: 0,
            last_range_timestamp_us: 0,
            last_battery_timestamp_us: 0,
            last_rc_timestamp_us: 0,
        })
    }
}

impl FfiBoard {
    fn update_sensor_bus_impl<R: FlightFloat>(
        &mut self,
        sensors: &mut SensorBus<R>,
        include_imu: bool,
    ) {
        sensors.clear();
        let Ok(mut shared) = self.sensors.lock() else {
            return;
        };
        let snapshot = shared.take_snapshot(include_imu);

        if snapshot.has_imu {
            sensors.imu = Some(Ok(ffi_imu_packet(snapshot.imu)));
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
}

impl BoardIo for FfiBoard {
    fn update_sensor_bus<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        self.update_sensor_bus_impl(sensors, true);
    }

    fn update_service_sensor_bus<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        self.update_sensor_bus_impl(sensors, false);
    }

    fn imu_pending(&self) -> bool {
        self.sensors.lock().is_ok_and(|shared| shared.imu_pending())
    }

    fn update_imu_sensor<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        sensors.clear();
        let Ok(mut shared) = self.sensors.lock() else {
            return;
        };
        if let Some(imu) = shared.take_imu() {
            sensors.imu = Some(Ok(ffi_imu_packet(imu)));
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

fn ffi_imu_packet<R: FlightFloat>(imu: VeloxityFfiImu) -> packets::ImuPacket<R> {
    packets::ImuPacket {
        header: packets::RosflightPacketHeader {
            timestamp: imu.timestamp_us,
            status: 0,
        },
        accel: [
            <R as FlightFloat>::from_f64(imu.linear_acceleration.x),
            <R as FlightFloat>::from_f64(imu.linear_acceleration.y),
            <R as FlightFloat>::from_f64(imu.linear_acceleration.z),
        ],
        gyro: [
            <R as FlightFloat>::from_f64(imu.angular_velocity.x),
            <R as FlightFloat>::from_f64(imu.angular_velocity.y),
            <R as FlightFloat>::from_f64(imu.angular_velocity.z),
        ],
        temperature: imu.temperature_kelvin,
        seq: 0,
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
    progress: Arc<(Mutex<FirmwareProgress>, Condvar)>,
    shutdown: Arc<AtomicBool>,
    start_time: Instant,
    worker: Option<JoinHandle<()>>,
}

impl Drop for VeloxityFfiHandle {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Release);
        self.progress.1.notify_all();
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn veloxity_sim_create() -> *mut VeloxityFfiHandle {
    let sensors = Arc::new(Mutex::new(SharedSensors::default()));
    let outputs = Arc::new(Mutex::new([1000; NUM_PWM_CHANNELS]));
    let progress = Arc::new((Mutex::new(FirmwareProgress::default()), Condvar::new()));
    let shutdown = Arc::new(AtomicBool::new(false));
    let start_time = Instant::now();

    let Ok(mut board) = FfiBoard::new(Arc::clone(&sensors), start_time) else {
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

    let mut world = FfiWorld::init(
        board, params, mavlink, state, estimator, controller, mixer, pwm,
    );
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(SIM_CONTROL_LOOP_HZ));

    let worker_sensors = Arc::clone(&sensors);
    let worker_outputs = Arc::clone(&outputs);
    let worker_progress = Arc::clone(&progress);
    let worker_shutdown = Arc::clone(&shutdown);
    let Ok(worker) = thread::Builder::new()
        .name("veloxity-sim-firmware".into())
        .spawn(move || {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                run_firmware_worker(
                    world,
                    worker_sensors,
                    worker_outputs,
                    Arc::clone(&worker_progress),
                    Arc::clone(&worker_shutdown),
                );
            }));
            if result.is_err() {
                if let Ok(mut progress) = worker_progress.0.lock() {
                    progress.worker_failed = true;
                }
                worker_progress.1.notify_all();
                worker_shutdown.store(true, Ordering::Release);
            }
        })
    else {
        return std::ptr::null_mut();
    };

    Box::into_raw(Box::new(VeloxityFfiHandle {
        sensors,
        progress,
        shutdown,
        start_time,
        worker: Some(worker),
    }))
}

fn run_firmware_worker(
    mut world: FfiWorld,
    sensors: Arc<Mutex<SharedSensors>>,
    outputs: Arc<Mutex<[u16; NUM_PWM_CHANNELS]>>,
    progress: Arc<(Mutex<FirmwareProgress>, Condvar)>,
    shutdown: Arc<AtomicBool>,
) {
    while !shutdown.load(Ordering::Acquire) {
        let processed_imu_generation = match world.realtime_scheduler_step() {
            RealtimeSchedulerStep::ImuControl => {
                let _ = world.run_imu_control_tick();
                sensors
                    .lock()
                    .ok()
                    .map(|sensors| sensors.consumed_imu_generation)
            }
            RealtimeSchedulerStep::ControlUpdate => {
                let _ = world.run_control_update_tick();
                None
            }
            RealtimeSchedulerStep::Service => {
                let _ = world.run_prioritized_service_steps_with_policy(
                    RealtimeServicePolicy::with_spacing(1, 1),
                );
                None
            }
            RealtimeSchedulerStep::Idle => {
                std::hint::spin_loop();
                continue;
            }
        };

        let Some(pwm_outputs) = outputs.lock().ok().map(|outputs| *outputs) else {
            continue;
        };
        let Ok(mut worker_progress) = progress.0.lock() else {
            continue;
        };
        worker_progress.pwm_outputs = pwm_outputs;
        if let Some(generation) = processed_imu_generation {
            worker_progress.processed_imu_generation =
                worker_progress.processed_imu_generation.max(generation);
        }
        drop(worker_progress);
        progress.1.notify_all();
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn veloxity_sim_destroy(handle: *mut VeloxityFfiHandle) {
    if !handle.is_null() {
        drop(unsafe { Box::from_raw(handle) });
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn veloxity_sim_set_sensors(
    handle: *const VeloxityFfiHandle,
    snapshot: *const VeloxityFfiSensorSnapshot,
) -> bool {
    if handle.is_null() || snapshot.is_null() {
        return false;
    }

    let handle = unsafe { &*handle };
    let Ok(mut sensors) = handle.sensors.lock() else {
        return false;
    };
    sensors.merge(unsafe { *snapshot });
    true
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn veloxity_sim_sync_latest_imu(handle: *const VeloxityFfiHandle) -> bool {
    if handle.is_null() {
        return false;
    }

    let handle = unsafe { &*handle };
    let target_generation = match handle.sensors.lock() {
        Ok(sensors) => sensors.latest_imu_generation,
        Err(_) => return false,
    };
    if target_generation == 0 {
        return true;
    }

    wait_for_imu_generation(
        &handle.progress,
        &handle.shutdown,
        target_generation,
        FIRMWARE_SYNC_TIMEOUT,
    )
}

fn wait_for_imu_generation(
    progress: &(Mutex<FirmwareProgress>, Condvar),
    shutdown: &AtomicBool,
    target_generation: u64,
    timeout: Duration,
) -> bool {
    let Ok(progress_guard) = progress.0.lock() else {
        return false;
    };
    let Ok((progress_guard, _)) =
        progress
            .1
            .wait_timeout_while(progress_guard, timeout, |worker_progress| {
                worker_progress.processed_imu_generation < target_generation
                    && !worker_progress.worker_failed
                    && !shutdown.load(Ordering::Acquire)
            })
    else {
        return false;
    };
    !progress_guard.worker_failed
        && !shutdown.load(Ordering::Acquire)
        && progress_guard.processed_imu_generation >= target_generation
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
    let Ok(progress) = handle.progress.0.lock() else {
        return 0;
    };
    let copy_len = output_len.min(progress.pwm_outputs.len());
    unsafe {
        std::ptr::copy_nonoverlapping(progress.pwm_outputs.as_ptr(), output, copy_len);
    }
    copy_len
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn veloxity_sim_clock_micros(handle: *const VeloxityFfiHandle) -> u64 {
    if handle.is_null() {
        return 0;
    }
    let handle = unsafe { &*handle };
    handle.start_time.elapsed().as_micros() as u64
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

#[cfg(test)]
mod tests {
    use super::*;

    static FFI_ENV_LOCK: Mutex<()> = Mutex::new(());

    fn imu_snapshot(timestamp_us: u64) -> VeloxityFfiSensorSnapshot {
        VeloxityFfiSensorSnapshot {
            has_imu: true,
            imu: VeloxityFfiImu {
                timestamp_us,
                ..VeloxityFfiImu::default()
            },
            ..VeloxityFfiSensorSnapshot::default()
        }
    }

    #[test]
    fn imu_remains_pending_until_consumed() {
        let mut sensors = SharedSensors::default();
        sensors.merge(imu_snapshot(10));
        sensors.merge(VeloxityFfiSensorSnapshot::default());

        assert!(sensors.imu_pending());
        assert_eq!(sensors.latest_imu_generation, 1);
        assert_eq!(sensors.take_imu().map(|imu| imu.timestamp_us), Some(10));
        assert_eq!(sensors.consumed_imu_generation, 1);
        assert!(!sensors.imu_pending());
        assert!(sensors.take_imu().is_none());
    }

    #[test]
    fn newer_imu_replaces_unconsumed_sample() {
        let mut sensors = SharedSensors::default();
        sensors.merge(imu_snapshot(10));
        sensors.merge(imu_snapshot(20));

        assert_eq!(sensors.take_imu().map(|imu| imu.timestamp_us), Some(20));
        assert_eq!(sensors.latest_imu_generation, 2);
        assert_eq!(sensors.consumed_imu_generation, 2);
    }

    #[test]
    fn service_snapshot_does_not_consume_pending_imu() {
        let mut sensors = SharedSensors::default();
        sensors.merge(imu_snapshot(10));
        sensors.merge(VeloxityFfiSensorSnapshot {
            has_rc: true,
            ..VeloxityFfiSensorSnapshot::default()
        });

        let service_snapshot = sensors.take_snapshot(false);

        assert!(!service_snapshot.has_imu);
        assert!(service_snapshot.has_rc);
        assert!(sensors.imu_pending());
        assert_eq!(sensors.consumed_imu_generation, 0);
    }

    #[test]
    fn generation_barrier_accepts_a_newer_processed_replacement() {
        let progress = Arc::new((Mutex::new(FirmwareProgress::default()), Condvar::new()));
        let shutdown = Arc::new(AtomicBool::new(false));
        let worker_progress = Arc::clone(&progress);
        let worker = thread::spawn(move || {
            let mut progress = worker_progress.0.lock().unwrap();
            progress.processed_imu_generation = 2;
            drop(progress);
            worker_progress.1.notify_all();
        });

        assert!(wait_for_imu_generation(
            &progress,
            &shutdown,
            1,
            Duration::from_millis(100),
        ));
        worker.join().unwrap();
    }

    #[test]
    fn generation_barrier_returns_on_shutdown() {
        let progress = (Mutex::new(FirmwareProgress::default()), Condvar::new());
        let shutdown = AtomicBool::new(true);

        assert!(!wait_for_imu_generation(
            &progress,
            &shutdown,
            1,
            Duration::from_millis(100),
        ));
    }

    #[test]
    fn firmware_worker_processes_imu_and_shuts_down_cleanly() {
        let _env_guard = FFI_ENV_LOCK.lock().unwrap();
        let bind_probe = UdpSocket::bind("127.0.0.1:0").unwrap();
        let bind_addr = bind_probe.local_addr().unwrap();
        drop(bind_probe);
        let remote = UdpSocket::bind("127.0.0.1:0").unwrap();
        let param_dir = std::env::temp_dir().join(format!(
            "veloxity-ffi-test-{}-{}",
            std::process::id(),
            bind_addr.port()
        ));
        fs::create_dir_all(&param_dir).unwrap();
        let previous_bind = std::env::var_os("VELOXITY_MAVLINK_BIND");
        let previous_remote = std::env::var_os("VELOXITY_MAVLINK_REMOTE");
        let previous_param_dir = std::env::var_os(PARAM_DIR_ENV);

        unsafe {
            std::env::set_var("VELOXITY_MAVLINK_BIND", bind_addr.to_string());
            std::env::set_var(
                "VELOXITY_MAVLINK_REMOTE",
                remote.local_addr().unwrap().to_string(),
            );
            std::env::set_var(PARAM_DIR_ENV, &param_dir);
        }

        let handle = veloxity_sim_create();
        assert!(!handle.is_null());
        let snapshot = imu_snapshot(unsafe { veloxity_sim_clock_micros(handle) }.max(1));
        assert!(unsafe { veloxity_sim_set_sensors(handle, &snapshot) });
        assert!(unsafe { veloxity_sim_sync_latest_imu(handle) });
        let mut pwm = [0_u16; NUM_PWM_CHANNELS];
        assert_eq!(
            unsafe { veloxity_sim_get_pwm(handle, pwm.as_mut_ptr(), pwm.len()) },
            NUM_PWM_CHANNELS
        );
        unsafe { veloxity_sim_destroy(handle) };

        restore_env("VELOXITY_MAVLINK_BIND", previous_bind);
        restore_env("VELOXITY_MAVLINK_REMOTE", previous_remote);
        restore_env(PARAM_DIR_ENV, previous_param_dir);
        fs::remove_dir_all(param_dir).unwrap();
    }

    fn restore_env(name: &str, value: Option<std::ffi::OsString>) {
        unsafe {
            if let Some(value) = value {
                std::env::set_var(name, value);
            } else {
                std::env::remove_var(name);
            }
        }
    }
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
