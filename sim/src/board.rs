use crate::ros_messages;
use rustflight_core::board::BoardTrait;
use rustflight_core::comm_manager;
use rustflight_core::errors;
use rustflight_core::hlist_type;
use rustflight_core::packets;
use rustflight_core::sensorprocessors;

use cdr::{CdrLe, Infinite};
use tokio::io::Empty;
use tokio::sync::mpsc;
use tokio::net::UdpSocket;
use tokio::io::ErrorKind;
use zenoh::Config;
use zenoh::bytes::{Encoding, ZBytes};
use zenoh::handlers::FifoChannelHandler;
use zenoh::pubsub::{Publisher, Subscriber};
use zenoh::sample::Sample;
use zenoh::session::Session;

pub struct Board {
    mavlink_socket: UdpSocket, 
    pub zenoh_connect_session: Session,
    //pub zenoh_listen_session: Session,
    //imu_temp_chan: mpsc::Receiver<ros_messages::Status>,
    //imu_data_chan: mpsc::Receiver<ros_messages::Status>,
    //battery_chan: mpsc::Receiver<ros_messages::BatteryStatus>,
    gnss_chan: mpsc::Receiver<ros_messages::GNSS>,
    baro_chan: mpsc::Receiver<ros_messages::Barometer>,
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
        sensors.1.1.1.1.1.0 = match self.gnss_chan.try_recv() {
            Ok(gnss) => Some(Ok(packets::GNSSPacket::default())),
            Err(e) => match e {
                tokio::sync::mpsc::error::TryRecvError::Empty => None,
                _ => Some(Err(errors::SensorError::GenericSensorError(
                    "generic gnss error",
                ))),
            },
        };
        sensors.1.1.1.1.1.1.0 = None;
        sensors.1.1.1.1.1.1.1.0 = None;
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
}

impl Board {
    pub async fn new() -> Board {
        let mut zenoh_connect_config = Config::default();
        zenoh_connect_config
            .insert_json5("mode", "\"client\"")
            .unwrap();
        zenoh_connect_config
            .insert_json5("connect/endpoints", "[\"tcp/127.0.0.1:7447\"]")
            .unwrap();

        //let mut zenoh_listen_config = Config::default();
        //zenoh_listen_config
        //    .insert_json5("listen/endpoints", r#"["tcp/127.0.0.1:7447"]"#)
        //    .unwrap();

        let zenoh_connect_session = zenoh::open(zenoh_connect_config).await.unwrap();
        //let zenoh_listen_session = zenoh::open(zenoh_listen_config).await.unwrap();

        println!("Zenoh sessions opened!");

        // Establish all channels for sub
        //let (chan_send_imu_temp, mut chan_recv_imu_temp) = mpsc::channel::<ros_messages::Status>(1);
        //let (chan_send_imu_data, mut chan_recv_imu_data) = mpsc::channel::<ros_messages::Status>(1);
        //let (chan_send_battery, mut chan_recv_battery) =
        //    mpsc::channel::<ros_messages::BatteryStatus>(1);
        let (chan_send_gnss, mut chan_recv_gnss) = mpsc::channel::<ros_messages::GNSS>(1);
        let (chan_send_baro, mut chan_recv_baro) = mpsc::channel::<ros_messages::Barometer>(1);
        //let (send_mag, mut recv_baro) = mpsc::channel::<ros_messages::Barometer>(1);
        //let (send_diffpressure, mut recv_diffpressure) = mpsc::channel::<ros_messages::Barometer>(1);

        // Establish all subscribers
        //let sub_imu_temp = zenoh_listen_session
        //    .declare_subscriber("/rt/simulated_sensors/imu/temperature")
        //    .await
        //    .unwrap();
        //let sub_imu_data = zenoh_listen_session
        //    .declare_subscriber("/rt/simulated_sensors/imu/data")
        //    .await
        //    .unwrap();
        //let sub_battery = zenoh_listen_session
        //    .declare_subscriber("/rt/simulated_sensors/battery")
        //    .await
        //    .unwrap();
        let sub_gnss = zenoh_connect_session
            .declare_subscriber("simulated_sensors/gnss")
            .await
            .unwrap();
        let sub_baro = zenoh_connect_session
            .declare_subscriber("simulated_sensors/baro")
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

        // establish all channels for pub
        let (chan_send_pwm, mut chan_recv_pwm) = mpsc::channel::<ros_messages::Status>(1);

        // establish publisher
        let pub_pwm_output = zenoh_connect_session
            .declare_publisher("sim/pwm_output")
            .encoding(Encoding::APPLICATION_OCTET_STREAM)
            .await
            .unwrap();


        println!("Zenoh publishers established");

        // establish udp socket for mavlink
        let mavlink_socket = UdpSocket::bind("0.0.0.0:0").await.unwrap();
        mavlink_socket.connect("127.0.0.1:14520").await.unwrap();

        println!("Mavlink connection established");

        // construct self for return
        let to_return = Self {
            mavlink_socket,
            zenoh_connect_session,
            //zenoh_listen_session,
            gnss_chan: chan_recv_gnss,
            baro_chan: chan_recv_baro,
        };

        // Spin up async functions for senders and receivers
        //tokio::spawn(capture_imu_temp());
        //tokio::spawn(capture_imu_data());
        //tokio::spawn(capture_battery(sub_battery, chan_send_battery));
        tokio::spawn(capture_gnss(sub_gnss, chan_send_gnss));
        tokio::spawn(capture_baro(sub_baro, chan_send_baro));
        //tokio::spawn(capture_mag());
        //tokio::spawn(capture_sonar());
        //tokio::spawn(capture:diff_pressure());
        tokio::spawn(publish_pwm(pub_pwm_output, chan_recv_pwm));

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

async fn capture_gnss(
    sub: Subscriber<FifoChannelHandler<Sample>>,
    chan: mpsc::Sender<ros_messages::GNSS>,
) {
    while let Ok(sample) = sub.recv_async().await {
        match cdr::deserialize::<ros_messages::GNSS>(&sample.payload().to_bytes()) {
            Ok(gnss) => {
                if chan.send(gnss).await.is_err() {
                    println!("Error putting gnss in channel!");
                }
            }
            Err(_) => {}
        }
    }
}

async fn capture_baro(
    sub: Subscriber<FifoChannelHandler<Sample>>,
    chan: mpsc::Sender<ros_messages::Barometer>,
) {
    while let Ok(sample) = sub.recv_async().await {
        match cdr::deserialize::<ros_messages::Barometer>(&sample.payload().to_bytes()) {
            Ok(barometer) => {
                if chan.send(barometer).await.is_err() {
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

async fn publish_pwm(publisher: Publisher<'_>, mut chan: mpsc::Receiver<ros_messages::Status>) {
    if let Some(pwm) = chan.recv().await {
        let zb = ZBytes::from(cdr::serialize::<_, _, CdrLe>(&pwm, Infinite).unwrap());
        if publisher.put(zb).await.is_err() {
            println!("Error sending zbytes: pwm");
        }
    }
}
