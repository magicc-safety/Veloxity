use std::rc::Rc;

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

impl From<ros_messages::RCRaw> for packets::RcPacket {
    fn from(msg: ros_messages::RCRaw) -> Self {
        let mut channels = [0.0f32; RC_PACKET_CHANNELS];

        // Now iterate over the fixed-size array `msg.values`
        for (i, &value) in msg.values.iter().enumerate() {
            // Ensure we don't write past the end of the `channels` buffer
            if i < RC_PACKET_CHANNELS {
                 channels[i] = (value as f32 - 1500.0) / 500.0;
            } else {
                 break; // Stop if the source array is somehow larger (shouldn't happen here)
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
        sensors.0 = None;
        sensors.1.0 = None;
        sensors.1.1.0 = None;
        sensors.1.1.1.0 = None;
        sensors.1.1.1.1.0 = None;
        sensors.1.1.1.1.1.0 = None;
        // sensors.1.1.1.1.1.0 = match self.gnss_chan.try_recv() {
        //     Ok(gnss) => Some(Ok(packets::GNSSPacket::default())),
        //     Err(e) => match e 
        //         tokio::sync::mpsc::error::TryRecvError::Empty => None,
        //         _ => Some(Err(errors::SensorError::GenericSensorError(
        //             "generic gnss error",
        //         ))),
        //     },
        // };
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
        let (chan_send_rc, mut chan_recv_rc) = mpsc::channel::<ros_messages::RCRaw>(1);

        let sub_imu_data = zenoh_session
            .declare_subscriber("simulated_sensors/imu/data")
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
            imu_data_chan: chan_recv_imu_data,
        };

        // Spin up async functions for senders and receivers
        tokio::spawn(capture_imu_data(sub_imu_data, chan_send_imu_data));
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
                    println!("Error putting gnss in channel!");
                } else {
                    //println!("\tgot imu data")
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
                    println!("Error putting rc in channel!");
                } else {
                    //println!("\t\t\t\tgot rc data")
                }
            }
            Err(_) => {}
        }
    }
}