// /**
// ******************************************************************************
// * File     : board.rs
// * Date     : November 14, 2025
// ******************************************************************************
// *
// * Copyright (c) 2023, AeroVironment, Inc.
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// * 1.Redistributions of source code must retain the above copyright notice, this
// * list of conditions and the following disclaimer.
// *
// * 2.Redistributions in binary form must reproduce the above copyright notice,
// * this list of conditions and the following disclaimer in the documentation
// * and/or other materials provided with the distribution.
// *
// * 3.Neither the name of the copyright holder nor the names of its
// * contributors may be used to endorse or promote products derived from
// * this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// ******************************************************************************
// **/

use crate::ros_messages;
use rustflight_core::board::BoardTrait;
use rustflight_core::comm_manager;
use rustflight_core::errors;
use rustflight_core::hlist_type;
use rustflight_core::packets::{self, RC_PACKET_CHANNELS};
use rustflight_core::sensorprocessors;

use cdr::{CdrLe, Infinite};
use tokio::io::Empty;
use tokio::sync::mpsc;
use tokio::net::UdpSocket;
use tokio::io::ErrorKind;
use tokio::time::Instant;
use zenoh::Config;
use zenoh::bytes::{Encoding, ZBytes};
use zenoh::handlers::FifoChannelHandler;
use zenoh::handlers::RingChannelHandler;
use zenoh::pubsub::{Publisher, Subscriber};
use zenoh::sample::Sample;
use zenoh::session::Session;
use zenoh::handlers::RingChannel;
use chrono::{Datelike, Timelike, Utc, TimeZone};

impl From<ros_messages::RCRaw> for packets::RcPacket {
    fn from(msg: ros_messages::RCRaw) -> Self {
        let mut channels = [0.0f32; RC_PACKET_CHANNELS];

        // Now iterate over the fixed-size array `msg.values`
        for (i, &value) in msg.values.iter().enumerate() {
                // Ensure we don't write past the end of the `channels` buffer
                if i < RC_PACKET_CHANNELS {
                    // FIX: Normalize 1000-2000us to 0.0-1.0
                    let normalized = (value as f32 - 1000.0) / 1000.0;
                    channels[i] = normalized.clamp(0.0, 1.0);
                } else {
                     break;
                }
            }

        // --- Rest of the implementation remains the same ---
        Self {
            header: packets::RosflightPacketHeader {
                timestamp: (msg.header.stamp.sec as u64 * 1_000_000) + (msg.header.stamp.nanosec as u64 / 1000),
                status: 0x00,
            },
            // Use the fixed size from the array
            n_chan: msg.values.len() as u32, // This will always be 8
            chan: channels,
            lol: false,
        }
    }
}

impl From<ros_messages::MagneticField> for packets::MagPacket {
    fn from(msg: ros_messages::MagneticField) -> Self {
        Self {
            header: packets::RosflightPacketHeader {
                // Convert the ROS timestamp (sec, nanosec) to a single microsecond value.
                timestamp: (msg.header.stamp.sec as u64 * 1_000_000)
                    + (msg.header.stamp.nanosec as u64 / 1000),

                // The status field is not present in the ROS MagneticField message, so we default to 0.
                status: 0,
            },

            // Map the magnetic_field vector (f64) to the flux array (f32).
            // 'magnetic_field' (Tesla) and 'flux' (magnetic flux density, Tesla)
            // represent the same physical quantity here.
            flux: [
                msg.magnetic_field.x as f32,
                msg.magnetic_field.y as f32,
                msg.magnetic_field.z as f32,
            ],

            // NOTE: The standard ROS sensor_msgs/MagneticField does not contain a temperature field.
            // We are setting a default value (e.g., 25.0 C), just like in the ImuPacket.
            temperature: 25.0,
        }
    }
}

impl From<ros_messages::Barometer> for packets::BaroPacket {
    fn from(msg: ros_messages::Barometer) -> Self {
        Self {
            header: packets::RosflightPacketHeader {
                // Convert the ROS timestamp (sec, nanosec) to a single microsecond value.
                timestamp: (msg.header.stamp.sec as u64 * 1_000_000)
                    + (msg.header.stamp.nanosec as u64 / 1000),

                // The status field is not present in the ROS Barometer message, so we default to 0.
                status: 0,
            },

            // Map pressure (f32, Pascals)
            pressure: msg.pressure,

            // Map temperature (f32).
            // ROS message is in Kelvin (K), Rosflight packets likely use Celsius (C).
            // Conversion: C = K - 273.15
            temperature: msg.temperature - 273.15,

            // Map altitude (f32, meters)
            altitude: msg.altitude,
        }
    }
}

/// Converts the custom ROS GNSS message to the internal GNSSPacket
impl From<ros_messages::GNSS> for packets::GNSSPacket {
    fn from(msg: ros_messages::GNSS) -> Self {
        // The ROS message header.stamp (sec, nanosec) is a UTC timestamp.
        // We convert it to a DateTime object to extract year, month, day, etc.
        // FIX: Use .latest() to resolve the LocalResult to an Option, then unwrap_or_default.
        let dt = Utc.timestamp_opt(msg.header.stamp.sec as i64, msg.header.stamp.nanosec)
            .latest()
            .unwrap_or_default();

        Self {
            header: packets::RosflightPacketHeader {
                // Use the dedicated 'rosflight_timestamp' (in microseconds) for the packet header.
                timestamp: msg.rosflight_timestamp as u64,

                // Default status to 0, as 'fix_type' is handled in its own field.
                status: 0,
            },

            // Convert latitude from degrees (ROS) to radians (Packet)
            lat: msg.lat.to_radians(),

            // Convert longitude from degrees (ROS) to radians (Packet)
            lon: msg.lon.to_radians(),

            // Map altitude (m) to height (m)
            height: msg.alt,

            // Map NED velocities
            vel_n: msg.vel_n,
            vel_e: msg.vel_e,
            vel_d: msg.vel_d,

            // Map accuracies
            h_acc: msg.horizontal_accuracy,
            v_acc: msg.vertical_accuracy,
            s_acc: msg.speed_accuracy,

            // --- Extract time from the ROS header.stamp ---
            // Packet comment specifies 0-11
            month: dt.month0() as u8, 
            year: dt.year() as u16,
            // Packet comment specifies 0-31, dt.day() is 1-31
            day: dt.day() as u8, 
            hour: dt.hour() as u8,
            min: dt.minute() as u8,
            sec: dt.second() as u8,
            nano: dt.nanosecond() as i32,

            // Use the provided helper function to convert the fix type
            // Assumes GNSSFixType::from_u8 is available in the packets module
            fix_type: packets::GNSSFixType::from_u8(msg.fix_type),
            
            num_sats: msg.num_sat,

            // NOTE: The ROS message does not contain magnetic declination.
            // Defaulting to 0.0.
            mag_dec: 0.0,

            // NOTE: The ROS message does not contain a time_correction field.
            // Defaulting to 0.
            time_correction: 0,
        }
    }
}


impl From<ros_messages::ImuData> for packets::ImuPacket {
    fn from(msg: ros_messages::ImuData) -> Self {
        Self {
            header: packets::RosflightPacketHeader {
                // Convert the ROS timestamp (sec, nanosec) to a single microsecond value.
                timestamp: (msg.header.stamp.sec as u64 * 1_000_000)
                    + (msg.header.stamp.nanosec as u64 / 1000),
                
                // The status field is not present in the ROS Imu message, so we default to 0.
                status: 0,
            },
            
            // Map the linear_acceleration vector to the accel array.
            accel: [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            
            // Map the angular_velocity vector to the gyro array.
            gyro: [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],

            // NOTE: The standard ROS sensor_msgs/Imu does not contain a temperature field.
            // We are setting a default value of 25.0 C.
            temperature: 25.0,

            // NOTE: The ROS 2 std_msgs/Header does not have a sequence number ('seq').
            // We are setting a default value of 0.
            seq: 0,
        }
    }
}

pub struct Board {
    pub start_time: Instant,
    mavlink_socket: UdpSocket, 
    pub zenoh_session: Session,
    imu_data_chan: mpsc::Receiver<ros_messages::ImuData>,
    mag_chan: mpsc::Receiver<ros_messages::MagneticField>,
    baro_chan: mpsc::Receiver<ros_messages::Barometer>,
    gnss_chan: mpsc::Receiver<ros_messages::GNSS>,
    rc_chan: mpsc::Receiver<ros_messages::RCRaw>,
}

impl BoardTrait for Board {
    type RawSensorSet = hlist_type![
        Option<Result<packets::ImuPacket, errors::SensorError>>,
        Option<Result<packets::MagPacket, errors::SensorError>>,
        Option<Result<packets::BaroPacket, errors::SensorError>>,
        Option<Result<packets::PitotPacket, errors::SensorError>>,
        Option<Result<packets::RangePacket, errors::SensorError>>,
        Option<Result<packets::GNSSPacket, errors::SensorError>>,
        Option<Result<packets::BatteryPacket, errors::SensorError>>,
        Option<Result<packets::RcPacket, errors::SensorError>>,
        Option<Result<packets::AttitudePacket, errors::SensorError>>
    ];

    type ProcessedSensorSet = hlist_type![
        Option<packets::ImuPacket>,
        Option<packets::MagPacket>,
        Option<packets::BaroPacket>,
        Option<packets::PitotPacket>,
        Option<packets::RangePacket>,
        Option<packets::GNSSPacket>,
        Option<packets::BatteryPacket>,
        Option<packets::RcPacket>,
        Option<packets::AttitudePacket>
    ];

    type ProcessorHList = hlist_type![
        sensorprocessors::PassthroughImuProcessor,
        sensorprocessors::PassthroughMagProcessor,
        sensorprocessors::PassthroughBaroProcessor,
        sensorprocessors::PassthroughPitotProcessor,
        sensorprocessors::PassthroughRangeProcessor,
        sensorprocessors::PassthroughGNSSProcessor,
        sensorprocessors::PassthroughBatteryProcessor,
        sensorprocessors::PassthroughRcProcessor,
        sensorprocessors::PassthroughAttitudeProcessor
    ];

    fn update_sensors(&mut self, sensors: &mut Self::RawSensorSet) {
        sensors.0 = match self.imu_data_chan.try_recv() {
            Ok(imu) => Some(Ok(imu.into())),
            Err(e) => match e {
                tokio::sync::mpsc::error::TryRecvError::Empty => None,
                _ => Some(Err(errors::SensorError::GenericSensorError(
                    "generic rc error",
                ))),
            },
        };
        sensors.1.0 = match self.mag_chan.try_recv() {
            Ok(mag) => Some(Ok(mag.into())),
            Err(e) => match e {
                tokio::sync::mpsc::error::TryRecvError::Empty => None,
                _ => Some(Err(errors::SensorError::GenericSensorError(
                    "generic rc error",
                ))),
            },
        };
        sensors.1.1.0 = match self.baro_chan.try_recv() {
            Ok(baro) => Some(Ok(baro.into())),
            Err(e) => match e {
                tokio::sync::mpsc::error::TryRecvError::Empty => None,
                _ => Some(Err(errors::SensorError::GenericSensorError(
                    "generic rc error",
                ))),
            },
        };
        sensors.1.1.1.0 = None;
        sensors.1.1.1.1.0 = None;
        sensors.1.1.1.1.1.0 = match self.gnss_chan.try_recv() {
            Ok(gnss) => Some(Ok(gnss.into())),
            Err(e) => match e {
                tokio::sync::mpsc::error::TryRecvError::Empty => None,
                _ => Some(Err(errors::SensorError::GenericSensorError(
                    "generic rc error",
                ))),
            },
        };
;
        sensors.1.1.1.1.1.1.0 = None;
        sensors.1.1.1.1.1.1.1.0 = match self.rc_chan.try_recv() {
            Ok(rc) => Some(Ok(rc.into())),
            Err(e) => match e {
                tokio::sync::mpsc::error::TryRecvError::Empty => None,
                _ => Some(Err(errors::SensorError::GenericSensorError(
                    "generic rc error",
                ))),
            },
        };
        sensors.1.1.1.1.1.1.1.1.0 = None;
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        // Use the non-blocking try_recv on the Tokio socket
        match self.mavlink_socket.try_recv(buf) {
            Ok(n) => Some(Ok(n)), // Successfully read n bytes
            Err(e) if e.kind() == ErrorKind::WouldBlock => {
                //println!("No MAVLink data received!!!");
                None // No data available to read, not an error
            }
            Err(_) => {
                // A real I/O error occurred
                //println!("Real MAVLink error!!!");
                Some(Err(errors::TelemError::GenericTelemError(
                    "Error Reading From MAVLink UDP Socket",
                )))
            }
        }
    }

    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        // Use the non-blocking try_send on the Tokio socket
        match self.mavlink_socket.try_send(bytes) {
            Ok(n) => Some(Ok(n)), // Successfully wrote n bytes
            Err(e) if e.kind() == ErrorKind::WouldBlock => {
                 // OS buffer is full, treat as a transient error
                 Some(Err(errors::TelemError::GenericTelemError(
                    "MAVLink UDP Socket Send Buffer Full",
                )))
            }
            Err(_) => {
                // A real I/O error occurred
                Some(Err(errors::TelemError::GenericTelemError(
                    "Error Writing to MAVLink UDP Socket",
                )))
            }
        }
    }

    fn clock_millis(&self) -> u32 {
        //(self.current_time_us / 1000) as u32
        self.start_time.elapsed().as_millis() as u32

    }

    /// Returns the current dummy time in microseconds.
    fn clock_micros(&self) -> u64 {
        //self.current_time_us
        self.start_time.elapsed().as_micros() as u64
    }
}

impl Board {
    pub async fn new() -> Board {

        let start_time = Instant::now();

        let mut zenoh_config = Config::default();
        zenoh_config.insert_json5("mode", "\"client\"").unwrap();
        zenoh_config
            .insert_json5("connect/endpoints", r#"["tcp/127.0.0.1:7447"]"#)
            .unwrap();

        let zenoh_session = zenoh::open(zenoh_config).await.unwrap();

        println!("Zenoh sessions opened!");

        // Establish all channels for sub
        let (chan_send_imu_data, mut chan_recv_imu_data) = mpsc::channel::<ros_messages::ImuData>(1);
        let (chan_send_mag, mut chan_recv_mag) = mpsc::channel::<ros_messages::MagneticField>(1);
        let (chan_send_baro, mut chan_recv_baro) = mpsc::channel::<ros_messages::Barometer>(1);
        let (chan_send_gnss, mut chan_recv_gnss) = mpsc::channel::<ros_messages::GNSS>(1);
        let (chan_send_rc, mut chan_recv_rc) = mpsc::channel::<ros_messages::RCRaw>(1);

        let sub_imu_data = zenoh_session
            .declare_subscriber("simulated_sensors/imu/data")
            .with(zenoh::handlers::RingChannel::new(2))            
            .await
            .unwrap();
        let sub_mag = zenoh_session
            .declare_subscriber("simulated_sensors/mag")
            .with(zenoh::handlers::RingChannel::new(2))            
            .await
            .unwrap();
        let sub_baro = zenoh_session
            .declare_subscriber("simulated_sensors/baro")
            .with(zenoh::handlers::RingChannel::new(2))            
            .await
            .unwrap();
        let sub_gnss = zenoh_session
            .declare_subscriber("simulated_sensors/gnss")
            .with(zenoh::handlers::RingChannel::new(2))            
            .await
            .unwrap();
        let sub_rc = zenoh_session
            .declare_subscriber("sim/RC")
            .with(zenoh::handlers::RingChannel::new(2))            
            .await
            .unwrap();

        println!("Zenoh subscribers established");

        // establish udp socket for mavlink
        let mavlink_socket = UdpSocket::bind("0.0.0.0:0").await.unwrap();
        mavlink_socket.connect("127.0.0.1:14520").await.unwrap();

        println!("Mavlink connection established");

        // construct self for return
        let to_return = Self {
            start_time,
            mavlink_socket,
            zenoh_session,
            rc_chan: chan_recv_rc,
            mag_chan: chan_recv_mag,
            baro_chan: chan_recv_baro,
            gnss_chan: chan_recv_gnss,
            imu_data_chan: chan_recv_imu_data,
        };

        // Spin up async functions for senders and receivers
        tokio::spawn(capture_imu_data(sub_imu_data, chan_send_imu_data));
        tokio::spawn(capture_mag(sub_mag, chan_send_mag));
        tokio::spawn(capture_baro(sub_baro, chan_send_baro));
        tokio::spawn(capture_gnss(sub_gnss, chan_send_gnss));
        tokio::spawn(capture_rc(sub_rc, chan_send_rc));


        println!("Zenoh spawns finished");

        to_return
    }
}

async fn capture_imu_data(
    sub: Subscriber<RingChannelHandler<Sample>>,
    chan: mpsc::Sender<ros_messages::ImuData>,
) {
    while let Ok(sample) = sub.recv_async().await {
        match cdr::deserialize::<ros_messages::ImuData>(&sample.payload().to_bytes()) {
            Ok(data) => {
                if chan.send(data).await.is_err() {
                    println!("Error putting imu in channel!");
                } else {
                    //println!("\tgot imu data")
                }
            }
            Err(_) => {}
        }
    }
}

async fn capture_mag(
    sub: Subscriber<RingChannelHandler<Sample>>,
    chan: mpsc::Sender<ros_messages::MagneticField>,
) {
    while let Ok(sample) = sub.recv_async().await {
        match cdr::deserialize::<ros_messages::MagneticField>(&sample.payload().to_bytes()) {
            Ok(mag) => {
                if chan.send(mag).await.is_err() {
                    println!("Error putting mag in channel!");
                } else {
                    //println!("\t\t\t\tgot mag data")
                }
            }
            Err(_) => {}
        }
    }
}


async fn capture_baro(
    sub: Subscriber<RingChannelHandler<Sample>>,
    chan: mpsc::Sender<ros_messages::Barometer>,
) {
    while let Ok(sample) = sub.recv_async().await {
        match cdr::deserialize::<ros_messages::Barometer>(&sample.payload().to_bytes()) {
            Ok(baro) => {
                if chan.send(baro).await.is_err() {
                    println!("Error putting baro in channel!");
                } else {
                    //println!("\t\t\t\tgot mag data")
                }
            }
            Err(_) => {}
        }
    }
}

async fn capture_gnss(
    sub: Subscriber<RingChannelHandler<Sample>>,
    chan: mpsc::Sender<ros_messages::GNSS>,
) {
    while let Ok(sample) = sub.recv_async().await {
        match cdr::deserialize::<ros_messages::GNSS>(&sample.payload().to_bytes()) {
            Ok(gnss) => {
                if chan.send(gnss).await.is_err() {
                    println!("Error putting gnss in channel!");
                } else {
                    //println!("\t\t\t\tgot mag data")
                }
            }
            Err(_) => {}
        }
    }
}

async fn capture_rc(
    sub: Subscriber<RingChannelHandler<Sample>>,
    chan: mpsc::Sender<ros_messages::RCRaw>,
) {
    while let Ok(sample) = sub.recv_async().await {
        match cdr::deserialize::<ros_messages::RCRaw>(&sample.payload().to_bytes()) {
            Ok(rc) => {
                if chan.send(rc).await.is_err() {
                    println!("Error putting rc_raw in channel!");
                } else {
                    //println!("\t\t\t\tgot rc data")
                }
            }
            Err(_) => {}
        }
    }
}