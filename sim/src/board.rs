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
use zenoh::pubsub::{Publisher, Subscriber};
use zenoh::sample::Sample;
use zenoh::session::Session;

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
    start_time: Instant,
    //pub current_time_us: u64,
    mavlink_socket: UdpSocket, 
    //pub zenoh_connect_session: Session,
    pub zenoh_listen_session: Session,
    //imu_temp_chan: mpsc::Receiver<ros_messages::Status>,
    imu_data_chan: mpsc::Receiver<ros_messages::ImuData>,
    //battery_chan: mpsc::Receiver<ros_messages::BatteryStatus>,
    //gnss_chan: mpsc::Receiver<ros_messages::GNSS>,
    //baro_chan: mpsc::Receiver<ros_messages::Barometer>,
    rc_chan: mpsc::Receiver<ros_messages::RCRaw>,
    //mag_chan: mpsc::Receiver<ros_messages::Status>,
    //diffpress_chan: mpsc::Receiver<ros_messages::Status>,
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

        // initialize the clock
        let start_time = Instant::now();

        // let mut zenoh_connect_config = Config::default();
        // zenoh_connect_config
        //     .insert_json5("mode", "\"client\"")
        //     .unwrap();
        // zenoh_connect_config
        //     .insert_json5("connect/endpoints", "[\"tcp/127.0.0.1:7447\"]")
        //     .unwrap();

        let mut zenoh_listen_config = Config::default();
        zenoh_listen_config.insert_json5("mode", "\"client\"").unwrap();
        zenoh_listen_config
            .insert_json5("connect/endpoints", r#"["tcp/127.0.0.1:7447"]"#)
            .unwrap();

        //let zenoh_connect_session = zenoh::open(zenoh_connect_config).await.unwrap();
        let zenoh_listen_session = zenoh::open(zenoh_listen_config).await.unwrap();

        println!("Zenoh sessions opened!");

        // Establish all channels for sub
        //let (chan_send_imu_temp, mut chan_recv_imu_temp) = mpsc::channel::<ros_messages::Status>(1);
        let (chan_send_imu_data, mut chan_recv_imu_data) = mpsc::channel::<ros_messages::ImuData>(1);
        //let (chan_send_battery, mut chan_recv_battery) =
        //    mpsc::channel::<ros_messages::BatteryStatus>(1);
        // let (chan_send_gnss, mut chan_recv_gnss) = mpsc::channel::<ros_messages::GNSS>(1);
        // let (chan_send_baro, mut chan_recv_baro) = mpsc::channel::<ros_messages::Barometer>(1);
        let (chan_send_rc, mut chan_recv_rc) = mpsc::channel::<ros_messages::RCRaw>(1);
        //let (send_mag, mut recv_baro) = mpsc::channel::<ros_messages::Barometer>(1);
        //let (send_diffpressure, mut recv_diffpressure) = mpsc::channel::<ros_messages::Barometer>(1);

        // Establish all subscribers
        //let sub_imu_temp = zenoh_listen_session
        //    .declare_subscriber("/rt/simulated_sensors/imu/temperature")
        //    .await
        //    .unwrap();
        let sub_imu_data = zenoh_listen_session
            .declare_subscriber("simulated_sensors/imu/data")
            .await
            .unwrap();
        //let sub_battery = zenoh_listen_session
        //    .declare_subscriber("/rt/simulated_sensors/battery")
        //    .await
        //    .unwrap();
        // let sub_gnss = zenoh_connect_session
        //     .declare_subscriber("simulated_sensors/gnss")
        //     .await
        //     .unwrap();
        // let sub_baro = zenoh_connect_session
        //     .declare_subscriber("simulated_sensors/baro")
        //     .await
        //     .unwrap();
        let sub_rc = zenoh_listen_session
            .declare_subscriber("sim/RC")
            .await
            .unwrap();
        //let sub_mag = zenoh_listen_session
        //    .declare_subscriber("/rt/simulated_sensors/mag")
        //    .await
        //    .unwrap();
        //let sub_sonar = zenoh_listen_session
        //    .declare_subscriber("/rt/simulated_sensors/sonar")
        //    .await
        //    .unwrap();
        //let sub_diff_pressure = zenoh_listen_session
        //    .declare_subscriber("/rt/simulated_sensors/diff_pressure")
        //    .await
        //    .unwrap();

        println!("Zenoh subscribers established");

        // // establish all channels for pub
        // let (chan_send_pwm, mut chan_recv_pwm) = mpsc::channel::<ros_messages::Status>(1);

        // // establish publisher
        // let pub_pwm_output = zenoh_listen_session
        //     .declare_publisher("sim/pwm_output")
        //     .encoding(Encoding::APPLICATION_OCTET_STREAM)
        //     .await
        //     .unwrap();


        // println!("Zenoh publishers established");

        // establish udp socket for mavlink
        let mavlink_socket = UdpSocket::bind("0.0.0.0:0").await.unwrap();
        mavlink_socket.connect("127.0.0.1:14520").await.unwrap();

        println!("Mavlink connection established");

        // construct self for return
        let to_return = Self {
            start_time,
            mavlink_socket,
            //zenoh_connect_session,
            zenoh_listen_session,
            //gnss_chan: chan_recv_gnss,
            //baro_chan: chan_recv_baro,
            rc_chan: chan_recv_rc,
            imu_data_chan: chan_recv_imu_data,
        };

        // Spin up async functions for senders and receivers
        //tokio::spawn(capture_imu_temp());
        tokio::spawn(capture_imu_data(sub_imu_data, chan_send_imu_data));
        tokio::spawn(capture_rc(sub_rc, chan_send_rc));
        //tokio::spawn(capture_battery(sub_battery, chan_send_battery));
        //tokio::spawn(capture_gnss(sub_gnss, chan_send_gnss));
        //tokio::spawn(capture_baro(sub_baro, chan_send_baro));
        //tokio::spawn(capture_mag());
        //tokio::spawn(capture_sonar());
        //tokio::spawn(capture:diff_pressure());
        //tokio::spawn(publish_pwm(pub_pwm_output, chan_recv_pwm));

        println!("Zenoh spawns finished");

        to_return
    }
}

//async fn capture_imu_temp(sub: Subscriber<FifoChannelHandler<Sample>>) {}

//async fn capture_imu_data(sub: Subscriber<FifoChannelHandler<Sample>>) {}

//async fn capture_battery(
//    sub: Subscriber<FifoChannelHandler<Sample>>,
//    chan: mpsc::Sender<ros_messages::BatteryStatus>,
//) {
//    while let Ok(sample) = sub.recv_async().await {
//        match cdr::deserialize::<ros_messages::BatteryStatus>(&sample.payload().to_bytes()) {
//            Ok(battery_status) => {
//                if chan.send(battery_status).await.is_err() {
//                    println!("Error putting battery status in channel!");
//                }
//            }
//            Err(_) => {}
//        }
//    }
//}

// async fn capture_gnss(
//     sub: Subscriber<FifoChannelHandler<Sample>>,
//     chan: mpsc::Sender<ros_messages::GNSS>,
// ) {
//     while let Ok(sample) = sub.recv_async().await {
//         match cdr::deserialize::<ros_messages::GNSS>(&sample.payload().to_bytes()) {
//             Ok(gnss) => {
//                 if chan.send(gnss).await.is_err() {
//                     println!("Error putting gnss in channel!");
//                 }
//             }
//             Err(_) => {}
//         }
//     }
// }

// async fn capture_baro(
//     sub: Subscriber<FifoChannelHandler<Sample>>,
//     chan: mpsc::Sender<ros_messages::Barometer>,
// ) {
//     while let Ok(sample) = sub.recv_async().await {
//         match cdr::deserialize::<ros_messages::Barometer>(&sample.payload().to_bytes()) {
//             Ok(barometer) => {
//                 if chan.send(barometer).await.is_err() {
//                     println!("Error putting barometer in channel!");
//                 }
//             }
//             Err(_) => {}
//         }
//     }
// }


async fn capture_imu_data(
    sub: Subscriber<FifoChannelHandler<Sample>>,
    chan: mpsc::Sender<ros_messages::ImuData>,
) {
    while let Ok(sample) = sub.recv_async().await {
        match cdr::deserialize::<ros_messages::ImuData>(&sample.payload().to_bytes()) {
            Ok(data) => {
                if chan.send(data).await.is_err() {
                    println!("Error putting gnss in channel!");
                }
            }
            Err(_) => {}
        }
    }
}

async fn capture_rc(
    sub: Subscriber<FifoChannelHandler<Sample>>,
    chan: mpsc::Sender<ros_messages::RCRaw>,
) {
    while let Ok(sample) = sub.recv_async().await {
        match cdr::deserialize::<ros_messages::RCRaw>(&sample.payload().to_bytes()) {
            Ok(rc) => {
                if chan.send(rc).await.is_err() {
                    println!("Error putting barometer in channel!");
                }
            }
            Err(_) => {}
        }
    }
}

//async fn capture_mag(sub: Subscriber<FifoChannelHandler<Sample>>) {}

//async fn capture_sonar(sub: Subscriber<FifoChannelHandler<Sample>>) {}

//async fn capture_diff_pressure(sub: Subscriber<FifoChannelHandler<Sample>>) {}

// async fn publish_pwm(publisher: Publisher<'_>, mut chan: mpsc::Receiver<ros_messages::Status>) {
//     if let Some(pwm) = chan.recv().await {
//         let zb = ZBytes::from(cdr::serialize::<_, _, CdrLe>(&pwm, Infinite).unwrap());
//         if publisher.put(zb).await.is_err() {
//             println!("Error sending zbytes: pwm");
//         }
//     }
// }
