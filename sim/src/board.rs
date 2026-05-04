use std::env;
use std::net::SocketAddr;
use std::time::Instant;

use cdr::CdrLe;
use chrono::{Datelike, TimeZone, Timelike, Utc};
use rustflight_core::board::BoardTrait;
use rustflight_core::errors;
use rustflight_core::hlist::HNil;
use rustflight_core::packets::{self, RC_PACKET_CHANNELS};
use rustflight_core::sensors::SensorBus;
use tokio::io::ErrorKind;
use tokio::net::UdpSocket;
use tokio::sync::mpsc;
use zenoh::handlers::{RingChannel, RingChannelHandler};
use zenoh::sample::Sample;
use zenoh::{Config, pubsub::Subscriber, session::Session};

use crate::ros_messages;

const DEFAULT_ZENOH_ENDPOINT: &str = "tcp/127.0.0.1:7447";
const DEFAULT_MAVLINK_BIND: &str = "127.0.0.1:14557";
const DEFAULT_MAVLINK_REMOTE: &str = "127.0.0.1:14520";

pub struct Board {
    start_time: Instant,
    mavlink_socket: UdpSocket,
    imu_rx: mpsc::Receiver<ros_messages::ImuData>,
    mag_rx: mpsc::Receiver<ros_messages::MagneticField>,
    baro_rx: mpsc::Receiver<ros_messages::Barometer>,
    gnss_rx: mpsc::Receiver<ros_messages::GNSS>,
    rc_rx: mpsc::Receiver<ros_messages::RCRaw>,
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
            },
            session,
        )
    }
}

impl BoardTrait for Board {
    type RawSensorSet = HNil;
    type ProcessedSensorSet = HNil;
    type ProcessorHList = HNil;

    fn update_sensors(&mut self, _sensors: &mut Self::RawSensorSet) {}

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
            Err(e) if e.kind() == ErrorKind::WouldBlock => {
                Some(Err(errors::TelemError::GenericTelemError(
                    "MAVLink UDP socket send buffer full",
                )))
            }
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
        env::var("RUSTFLIGHT_ZENOH_ENDPOINT").unwrap_or_else(|_| DEFAULT_ZENOH_ENDPOINT.into());
    let mut config = Config::default();
    config.insert_json5("mode", "\"client\"").unwrap();
    config
        .insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))
        .unwrap();
    zenoh::open(config).await.unwrap()
}

async fn open_mavlink_socket() -> UdpSocket {
    let bind_addr: SocketAddr = env::var("RUSTFLIGHT_MAVLINK_BIND")
        .unwrap_or_else(|_| DEFAULT_MAVLINK_BIND.into())
        .parse()
        .unwrap();
    let remote_addr: SocketAddr = env::var("RUSTFLIGHT_MAVLINK_REMOTE")
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
