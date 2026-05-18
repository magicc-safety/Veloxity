use std::env;
use std::fs;
use std::io::{self, Write};
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::time::Instant;

use cdr::CdrLe;
use chrono::{Datelike, TimeZone, Timelike, Utc};
use tokio::io::ErrorKind;
use tokio::net::UdpSocket;
use tokio::sync::mpsc;
use voloxide_core::board::BoardIo;
use voloxide_core::errors;
use voloxide_core::packets::{self, RC_PACKET_CHANNELS};
use voloxide_core::params::{PARAM_DEFINITIONS, ParamValue, Params};
use voloxide_core::sensors::SensorBus;
use zenoh::handlers::{RingChannel, RingChannelHandler};
use zenoh::sample::Sample;
use zenoh::{Config, pubsub::Subscriber, session::Session};

use crate::ros_messages;

const DEFAULT_ZENOH_ENDPOINT: &str = "tcp/127.0.0.1:7447";
const DEFAULT_MAVLINK_BIND: &str = "127.0.0.1:14557";
const DEFAULT_MAVLINK_REMOTE: &str = "127.0.0.1:14520";
const DEFAULT_PARAM_STORE: &str = "voloxide_sim.params";
const PARAM_STORE_ENV: &str = "VOLOXIDE_SIM_PARAM_STORE";

pub struct Board {
    start_time: Instant,
    mavlink_socket: UdpSocket,
    imu_rx: mpsc::Receiver<ros_messages::ImuData>,
    mag_rx: mpsc::Receiver<ros_messages::MagneticField>,
    baro_rx: mpsc::Receiver<ros_messages::Barometer>,
    gnss_rx: mpsc::Receiver<ros_messages::GNSS>,
    rc_rx: mpsc::Receiver<ros_messages::RCRaw>,
    param_store_path: PathBuf,
}

impl Board {
    pub async fn new() -> (Self, Session) {
        let session = open_zenoh_session().await;

        let (imu_tx, imu_rx) = mpsc::channel(50);
        let (mag_tx, mag_rx) = mpsc::channel(4);
        let (baro_tx, baro_rx) = mpsc::channel(4);
        let (gnss_tx, gnss_rx) = mpsc::channel(4);
        let (rc_tx, rc_rx) = mpsc::channel(50);

        spawn_cdr_subscription::<ros_messages::ImuData>(
            &session,
            "simulated_sensors/imu/data",
            50,
            imu_tx,
        )
        .await;
        spawn_cdr_subscription::<ros_messages::MagneticField>(
            &session,
            "simulated_sensors/mag",
            4,
            mag_tx,
        )
        .await;
        spawn_cdr_subscription::<ros_messages::Barometer>(
            &session,
            "simulated_sensors/baro",
            4,
            baro_tx,
        )
        .await;
        spawn_cdr_subscription::<ros_messages::GNSS>(
            &session,
            "simulated_sensors/gnss",
            4,
            gnss_tx,
        )
        .await;
        spawn_cdr_subscription::<ros_messages::RCRaw>(&session, "sim/RC", 50, rc_tx).await;

        let mavlink_socket = open_mavlink_socket().await;

        (
            Self {
                start_time: Instant::now(),
                mavlink_socket,
                imu_rx,
                mag_rx,
                baro_rx,
                gnss_rx,
                rc_rx,
                param_store_path: param_store_path(),
            },
            session,
        )
    }
}

impl BoardIo for Board {
    fn update_sensor_bus(&mut self, sensors: &mut SensorBus) {
        sensors.clear();
        sensors.imu = recv_sensor(&mut self.imu_rx, "imu");
        sensors.mag = recv_sensor(&mut self.mag_rx, "mag");
        sensors.baro = recv_sensor(&mut self.baro_rx, "baro");
        sensors.gnss = recv_sensor(&mut self.gnss_rx, "gnss");
        sensors.rc = recv_sensor(&mut self.rc_rx, "rc");
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        match self.mavlink_socket.try_recv(buf) {
            Ok(n) => Some(Ok(n)),
            Err(e) if e.kind() == ErrorKind::WouldBlock => None,
            Err(_) => Some(Err(errors::TelemError::GenericTelemError(
                "error reading MAVLink UDP socket",
            ))),
        }
    }

    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        match self.mavlink_socket.try_send(bytes) {
            Ok(n) => Some(Ok(n)),
            Err(e) if e.kind() == ErrorKind::WouldBlock => Some(Err(
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

fn param_store_path() -> PathBuf {
    env::var_os(PARAM_STORE_ENV)
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

fn recv_sensor<M, P>(
    rx: &mut mpsc::Receiver<M>,
    name: &'static str,
) -> Option<Result<P, errors::SensorError>>
where
    M: Into<P>,
{
    match rx.try_recv() {
        Ok(msg) => Some(Ok(msg.into())),
        Err(tokio::sync::mpsc::error::TryRecvError::Empty) => None,
        Err(_) => Some(Err(errors::SensorError::GenericSensorError(name))),
    }
}

async fn open_zenoh_session() -> Session {
    let endpoint =
        env::var("VOLOXIDE_ZENOH_ENDPOINT").unwrap_or_else(|_| DEFAULT_ZENOH_ENDPOINT.into());
    let mut config = Config::default();
    config.insert_json5("mode", "\"client\"").unwrap();
    config
        .insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))
        .unwrap();
    zenoh::open(config).await.unwrap()
}

async fn open_mavlink_socket() -> UdpSocket {
    let bind_addr: SocketAddr = env::var("VOLOXIDE_MAVLINK_BIND")
        .unwrap_or_else(|_| DEFAULT_MAVLINK_BIND.into())
        .parse()
        .unwrap();
    let remote_addr: SocketAddr = env::var("VOLOXIDE_MAVLINK_REMOTE")
        .unwrap_or_else(|_| DEFAULT_MAVLINK_REMOTE.into())
        .parse()
        .unwrap();
    let socket = UdpSocket::bind(bind_addr).await.unwrap();
    socket.connect(remote_addr).await.unwrap();
    socket
}

async fn spawn_cdr_subscription<T>(
    session: &Session,
    key: &'static str,
    capacity: usize,
    sender: mpsc::Sender<T>,
) where
    T: serde::de::DeserializeOwned + Send + 'static,
{
    let subscriber = session
        .declare_subscriber(key)
        .with(RingChannel::new(capacity))
        .await
        .unwrap();
    tokio::spawn(capture_cdr_messages(subscriber, sender));
}

async fn capture_cdr_messages<T>(
    subscriber: Subscriber<RingChannelHandler<Sample>>,
    sender: mpsc::Sender<T>,
) where
    T: serde::de::DeserializeOwned + Send + 'static,
{
    while let Ok(sample) = subscriber.recv_async().await {
        if let Ok(msg) = cdr::deserialize::<T>(&sample.payload().to_bytes()) {
            if sender.send(msg).await.is_err() {
                break;
            }
        }
    }
}

fn stamp_to_micros(stamp: &ros_messages::Time) -> u64 {
    stamp.sec as u64 * 1_000_000 + stamp.nanosec as u64 / 1000
}

impl From<ros_messages::RCRaw> for packets::RcPacket {
    fn from(msg: ros_messages::RCRaw) -> Self {
        let mut channels = [0.0f32; RC_PACKET_CHANNELS];
        for (i, &value) in msg.values.iter().enumerate() {
            if i >= RC_PACKET_CHANNELS {
                break;
            }
            let normalized = (value as f32 - 1000.0) / 1000.0;
            channels[i] = normalized.clamp(0.0, 1.0);
        }
        Self {
            header: packets::RosflightPacketHeader {
                timestamp: stamp_to_micros(&msg.header.stamp),
                status: 0,
            },
            n_chan: msg.values.len() as u32,
            chan: channels,
            lol: false,
        }
    }
}

impl From<ros_messages::MagneticField> for packets::MagPacket {
    fn from(msg: ros_messages::MagneticField) -> Self {
        Self {
            header: packets::RosflightPacketHeader {
                timestamp: stamp_to_micros(&msg.header.stamp),
                status: 0,
            },
            flux: [
                msg.magnetic_field.x as f32,
                -msg.magnetic_field.y as f32,
                -msg.magnetic_field.z as f32,
            ],
            temperature: 25.0,
        }
    }
}

impl From<ros_messages::Barometer> for packets::BaroPacket {
    fn from(msg: ros_messages::Barometer) -> Self {
        Self {
            header: packets::RosflightPacketHeader {
                timestamp: stamp_to_micros(&msg.header.stamp),
                status: 0,
            },
            pressure: msg.pressure,
            temperature: msg.temperature - 273.15,
            altitude: msg.altitude,
        }
    }
}

impl From<ros_messages::GNSS> for packets::GNSSPacket {
    fn from(msg: ros_messages::GNSS) -> Self {
        let dt = Utc
            .timestamp_opt(msg.header.stamp.sec as i64, msg.header.stamp.nanosec)
            .latest()
            .unwrap_or_default();

        Self {
            header: packets::RosflightPacketHeader {
                timestamp: msg.rosflight_timestamp as u64,
                status: 0,
            },
            unix_seconds: msg.header.stamp.sec as i64,
            unix_nanos: msg.header.stamp.nanosec as i32,
            lat: msg.lat.to_radians(),
            lon: msg.lon.to_radians(),
            height: msg.alt,
            vel_n: msg.vel_n,
            vel_e: msg.vel_e,
            vel_d: msg.vel_d,
            h_acc: msg.horizontal_accuracy,
            v_acc: msg.vertical_accuracy,
            s_acc: msg.speed_accuracy,
            month: dt.month0() as u8,
            year: dt.year() as u16,
            day: dt.day() as u8,
            hour: dt.hour() as u8,
            min: dt.minute() as u8,
            sec: dt.second() as u8,
            nano: dt.nanosecond() as i32,
            fix_type: packets::GNSSFixType::from_u8(msg.fix_type),
            num_sats: msg.num_sat,
            mag_dec: 0.0,
            time_correction: 0,
        }
    }
}

impl From<ros_messages::ImuData> for packets::ImuPacket {
    fn from(msg: ros_messages::ImuData) -> Self {
        Self {
            header: packets::RosflightPacketHeader {
                timestamp: stamp_to_micros(&msg.header.stamp),
                status: 0,
            },
            accel: [
                msg.linear_acceleration.x,
                -msg.linear_acceleration.y,
                -msg.linear_acceleration.z,
            ],
            gyro: [
                msg.angular_velocity.x,
                -msg.angular_velocity.y,
                -msg.angular_velocity.z,
            ],
            temperature: 25.0,
            seq: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;
    use std::rc::Rc;
    use std::time::Instant;
    use voloxide_core::{
        controller::quad_controller::QuadController,
        estimator::quad_estimator::QuadEstimator,
        mixer::quad_mixer::QuadMixer,
        params::ParamId,
        pwm::{PwmDriver, PwmError},
        state_machine::StateManager,
        world::World,
    };
    use voloxide_mavlink::MavlinkInterface;

    fn stamp(sec: i32, nanosec: u32) -> ros_messages::Time {
        ros_messages::Time { sec, nanosec }
    }

    fn header(sec: i32, nanosec: u32) -> ros_messages::Header {
        ros_messages::Header {
            stamp: stamp(sec, nanosec),
            frame_id: "test".into(),
        }
    }

    fn vector(x: f64, y: f64, z: f64) -> ros_messages::Vector3 {
        ros_messages::Vector3 { x, y, z }
    }

    fn quaternion() -> ros_messages::Quaternion {
        ros_messages::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }

    fn test_param_path(name: &str) -> PathBuf {
        let mut path = env::temp_dir();
        path.push(format!(
            "voloxide_sim_{}_{}.params",
            name,
            std::process::id()
        ));
        let _ = fs::remove_file(&path);
        path
    }

    struct SmokePwm {
        flushed: Rc<Cell<usize>>,
        sent: Rc<Cell<usize>>,
    }

    impl PwmDriver for SmokePwm {
        fn len(&self) -> usize {
            14
        }

        fn is_enabled(&self) -> bool {
            true
        }

        fn enable(&mut self, _channel: usize) -> Result<(), PwmError> {
            Ok(())
        }

        fn disable(&mut self, _channel: usize) -> Result<(), PwmError> {
            Ok(())
        }

        fn enable_all(&mut self) -> Result<(), PwmError> {
            Ok(())
        }

        fn disable_all(&mut self) {}

        fn set_duty_cycle(&mut self, _channel: usize, _duty: u16) -> Result<(), PwmError> {
            Ok(())
        }

        fn flush<B: BoardIo>(&mut self, _board: &mut B) {
            self.flushed.set(self.flushed.get() + 1);
        }

        fn send_commands<B: BoardIo>(
            &mut self,
            _board: &mut B,
            _commands: &[f64],
        ) -> Result<(), PwmError> {
            self.sent.set(self.sent.get() + 1);
            Ok(())
        }
    }

    #[test]
    fn sim_param_store_round_trips_known_param_values() {
        let path = test_param_path("round_trip");
        let mut written = Params::new();
        written.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        written.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Int(1));
        written.set_by_id(ParamId::PARAM_X_EQ_TORQUE, ParamValue::Float(0.25));

        write_params_to_path(&path, &written).unwrap();

        let mut read = Params::new();
        read.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(7));
        read.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Int(0));
        read.set_by_id(ParamId::PARAM_X_EQ_TORQUE, ParamValue::Float(-0.5));

        read_params_from_path(&path, &mut read).unwrap();

        assert_eq!(
            read.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );
        assert_eq!(
            read.get_by_id(ParamId::PARAM_FIXED_WING),
            ParamValue::Int(1)
        );
        assert_eq!(
            read.get_by_id(ParamId::PARAM_X_EQ_TORQUE),
            ParamValue::Float(0.25)
        );

        let _ = fs::remove_file(path);
    }

    #[test]
    fn sim_param_store_ignores_unknown_and_malformed_lines() {
        let path = test_param_path("malformed");
        fs::write(
            &path,
            "SYS_ID=77\nUNKNOWN_PARAM=1\nMALFORMED\nFIXED_WING=not-a-bool\n",
        )
        .unwrap();

        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Int(1));

        read_params_from_path(&path, &mut params).unwrap();

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(77)
        );
        assert_eq!(
            params.get_by_id(ParamId::PARAM_FIXED_WING),
            ParamValue::Int(1)
        );

        let _ = fs::remove_file(path);
    }

    #[tokio::test]
    async fn sim_board_update_sensor_bus_converts_queued_messages() {
        let (imu_tx, imu_rx) = mpsc::channel(1);
        let (mag_tx, mag_rx) = mpsc::channel(1);
        let (baro_tx, baro_rx) = mpsc::channel(1);
        let (gnss_tx, gnss_rx) = mpsc::channel(1);
        let (rc_tx, rc_rx) = mpsc::channel(1);
        let socket = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        let param_store_path = test_param_path("sensor_bus");

        imu_tx
            .try_send(ros_messages::ImuData {
                header: header(1, 250_000_000),
                orientation: quaternion(),
                orientation_covariance: [0.0; 9],
                angular_velocity: vector(0.1, 0.2, 0.3),
                angular_velocity_covariance: [0.0; 9],
                linear_acceleration: vector(1.0, 2.0, 3.0),
                linear_acceleration_covariance: [0.0; 9],
            })
            .unwrap();
        mag_tx
            .try_send(ros_messages::MagneticField {
                header: header(2, 500_000),
                magnetic_field: vector(4.0, 5.0, 6.0),
                magnetic_field_covariance: [0.0; 9],
            })
            .unwrap();
        baro_tx
            .try_send(ros_messages::Barometer {
                header: header(3, 0),
                altitude: 123.0,
                pressure: 101_325.0,
                temperature: 298.15,
            })
            .unwrap();
        gnss_tx
            .try_send(ros_messages::GNSS {
                header: header(4, 0),
                fix_type: 3,
                num_sat: 11,
                lat: 40.0,
                lon: -111.0,
                alt: 1550.0,
                horizontal_accuracy: 0.7,
                vertical_accuracy: 1.2,
                vel_n: 1.0,
                vel_e: 2.0,
                vel_d: -0.5,
                speed_accuracy: 0.3,
                rosflight_timestamp: 4_200_000.0,
            })
            .unwrap();
        rc_tx
            .try_send(ros_messages::RCRaw {
                header: header(5, 125_000),
                values: [1000, 1250, 1500, 1750, 2000, 2500, 500, 1100],
            })
            .unwrap();

        let mut board = Board {
            start_time: Instant::now(),
            mavlink_socket: socket,
            imu_rx,
            mag_rx,
            baro_rx,
            gnss_rx,
            rc_rx,
            param_store_path,
        };
        let mut sensors = SensorBus::default();

        board.update_sensor_bus(&mut sensors);

        let imu = sensors.imu.unwrap().unwrap();
        assert_eq!(imu.header.timestamp, 1_250_000);
        assert_eq!(imu.accel, [1.0, -2.0, -3.0]);
        assert_eq!(imu.gyro, [0.1, -0.2, -0.3]);

        let mag = sensors.mag.unwrap().unwrap();
        assert_eq!(mag.header.timestamp, 2_000_500);
        assert_eq!(mag.flux, [4.0, -5.0, -6.0]);

        let baro = sensors.baro.unwrap().unwrap();
        assert_eq!(baro.header.timestamp, 3_000_000);
        assert_eq!(baro.pressure, 101_325.0);
        assert!((baro.temperature - 25.0).abs() < 1e-5);

        let gnss = sensors.gnss.unwrap().unwrap();
        assert_eq!(gnss.header.timestamp, 4_200_000);
        assert!((gnss.lat - 40.0_f64.to_radians()).abs() < 1e-12);
        assert!((gnss.lon - (-111.0_f64).to_radians()).abs() < 1e-12);
        assert_eq!(gnss.num_sats, 11);

        let rc = sensors.rc.unwrap().unwrap();
        assert_eq!(rc.header.timestamp, 5_000_125);
        assert_eq!(rc.n_chan, 8);
        assert_eq!(&rc.chan[..8], &[0.0, 0.25, 0.5, 0.75, 1.0, 1.0, 0.0, 0.1]);
    }

    #[tokio::test]
    async fn sim_board_runs_one_world_tick_with_queued_sensor_messages() {
        let (imu_tx, imu_rx) = mpsc::channel(1);
        let (_mag_tx, mag_rx) = mpsc::channel(1);
        let (_baro_tx, baro_rx) = mpsc::channel(1);
        let (_gnss_tx, gnss_rx) = mpsc::channel(1);
        let (rc_tx, rc_rx) = mpsc::channel(1);
        let socket = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        socket.connect("127.0.0.1:9").await.unwrap();
        let param_store_path = test_param_path("world_smoke");

        imu_tx
            .try_send(ros_messages::ImuData {
                header: header(0, 1_000_000),
                orientation: quaternion(),
                orientation_covariance: [0.0; 9],
                angular_velocity: vector(0.0, 0.0, 0.0),
                angular_velocity_covariance: [0.0; 9],
                linear_acceleration: vector(0.0, 0.0, 9.80665),
                linear_acceleration_covariance: [0.0; 9],
            })
            .unwrap();
        rc_tx
            .try_send(ros_messages::RCRaw {
                header: header(0, 1_000_000),
                values: [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000],
            })
            .unwrap();

        let board = Board {
            start_time: Instant::now(),
            mavlink_socket: socket,
            imu_rx,
            mag_rx,
            baro_rx,
            gnss_rx,
            rc_rx,
            param_store_path,
        };
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_FAILSAFE_THROTTLE, ParamValue::Float(0.0));
        params.set_by_id(ParamId::PARAM_RC_NUM_CHANNELS, ParamValue::Int(8));
        let flushed_pwm = Rc::new(Cell::new(0));
        let sent_pwm = Rc::new(Cell::new(0));
        let mixer = QuadMixer::new(&params);
        let mut world = World::<
            Board,
            QuadEstimator,
            QuadController,
            QuadMixer,
            MavlinkInterface,
            SmokePwm,
        >::init(
            board,
            params,
            MavlinkInterface::new(),
            StateManager::new(),
            QuadEstimator::default(),
            QuadController::default(),
            mixer,
            SmokePwm {
                flushed: flushed_pwm.clone(),
                sent: sent_pwm.clone(),
            },
        );

        assert!(world.run_once());
        assert_eq!(flushed_pwm.get(), 1);
        assert_eq!(sent_pwm.get(), 0);

        let _ = fs::remove_file(test_param_path("world_smoke"));
    }
}
