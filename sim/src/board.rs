use crate::ros_messages;
use rustflight_core::board::BoardTrait;
use rustflight_core::comm_manager;
use rustflight_core::errors;
use rustflight_core::packets;
use rustflight_core::packets::GNSSPacket;

use cdr::{CdrLe, Infinite};
use tokio::sync::mpsc;
use zenoh::Config;
use zenoh::bytes::{Encoding, ZBytes};
use zenoh::handlers::FifoChannelHandler;
use zenoh::pubsub::{Publisher, Subscriber};
use zenoh::sample::Sample;
use zenoh::session::Session;

pub struct Board {
    zenoh_connect_session: Session,
    pub zenoh_listen_session: Session,
    //imu_temp_chan: mpsc::Receiver<ros_messages::Status>,
    //imu_data_chan: mpsc::Receiver<ros_messages::Status>,
    //battery_chan: mpsc::Receiver<ros_messages::BatteryStatus>,
    gnss_chan: mpsc::Receiver<ros_messages::GNSS>,
    baro_chan: mpsc::Receiver<ros_messages::Barometer>,
    //mag_chan: mpsc::Receiver<ros_messages::Status>,
    //diffpress_chan: mpsc::Receiver<ros_messages::Status>,
}

impl BoardTrait for Board {
    fn imu_read(&mut self) -> Option<Result<packets::ImuPacket, errors::SensorError>> {
        None
    }

    fn mag_read(&mut self) -> Option<Result<packets::MagPacket, errors::SensorError>> {
        None
    }

    fn baro_read(&mut self) -> Option<Result<packets::BaroPacket, errors::SensorError>> {
        None
    }

    fn diff_pressure_read(&mut self) -> Option<Result<packets::PitotPacket, errors::SensorError>> {
        None
    }

    fn sonar_read(&mut self) -> Option<Result<packets::RangePacket, errors::SensorError>> {
        None
    }

    //TODO getting lots of errors here...
    fn gnss_read(&mut self) -> Option<Result<packets::GNSSPacket, errors::SensorError>> {
        match self.gnss_chan.try_recv() {
            Ok(gnss) => {
                Some(Ok(GNSSPacket {
                    header: packets::RosflightPacketHeader {
                        status: 0u16,
                        timestamp: gnss.header.stamp.sec as u64,
                    },
                    lat: 0.0f64,    // radians
                    lon: 0.0f64,    // radians
                    height: 0.0f32, // m/s above ellipsoid
                    vel_n: 0.0f32,  // m/s north
                    vel_e: 0.0f32,  // m/s east
                    vel_d: 0.0f32,  // m/s down
                    h_acc: 0.0f32,  // m north/east
                    v_acc: 0.0f32,  // m down
                    s_acc: 0.0f32,  // m/s
                    month: 0u8,     // 0-11
                    year: 0u16,     // 0-65535 UTC
                    day: 0u8,       // 0-31 UTS day of month
                    hour: 0u8,      // 0-23 UTC
                    min: 0u8,       // 0-59 UTC
                    sec: 0u8,       // 0-59 UTC
                    nano: 0i32,     // adjustment +/1 to seconds
                    fix_type: packets::GNSSFixType::DeadReckoningOnly,
                    num_sats: 0u8,   // 0-255
                    mag_dec: 0.0f32, // Magnetic Declination ??
                    time_correction: 0u64,
                }))
            }
            Err(_) => None,
        }
    }

    fn battery_read(&mut self) -> Option<Result<packets::BatteryPacket, errors::SensorError>> {
        None
    }

    fn rc_read(&mut self) -> Option<Result<packets::RcPacket, errors::SensorError>> {
        None
    }

    fn attitude_read(&mut self) -> Option<Result<packets::AttitudePacket, errors::SensorError>> {
        None
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        None
    }

    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        None
    }
}

impl Board {
    pub async fn new() -> Board {
        let mut zenoh_connect_config = Config::default();
        zenoh_connect_config
            .insert_json5("connect/endpoints", r#"["tcp/127.0.0.1:7447"]"#)
            .unwrap();
        let mut zenoh_listen_config = Config::default();
        zenoh_listen_config
            .insert_json5("listen/endpoints", r#"["tcp/127.0.0.1:7447"]"#)
            .unwrap();

        let zenoh_connect_session = zenoh::open(zenoh_connect_config).await.unwrap();
        let zenoh_listen_session = zenoh::open(zenoh_listen_config).await.unwrap();

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
        let sub_gnss = zenoh_listen_session
            .declare_subscriber("rt/simulated_sensors/gnss")
            .await
            .unwrap();
        let sub_baro = zenoh_listen_session
            .declare_subscriber("rt/simulated_sensors/baro")
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
            .declare_publisher("rt/sim/pwm_output")
            .encoding(Encoding::APPLICATION_OCTET_STREAM)
            .await
            .unwrap();

        // construct self for return
        let to_return = Self {
            zenoh_connect_session,
            zenoh_listen_session,
            gnss_chan: chan_recv_gnss,
            baro_chan: chan_recv_baro,
        };

        println!("Zenoh publishers established");

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
