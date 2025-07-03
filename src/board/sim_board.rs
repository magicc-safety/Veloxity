use crate::board::sim_config;
use crate::board::Board;
use crate::comm_manager;
use crate::errors;
use crate::packets;

use cdr::Infinite;
use tokio::sync::mpsc;
use zenoh::bytes::{Encoding, ZBytes};
use zenoh::handlers::FifoChannelHandler;
use zenoh::pubsub::{Publisher, Subscriber};
use zenoh::sample::Sample;
use zenoh::session::Session;

pub struct Sim {
    zenoh_connect_session: Session,
    zenoh_listen_session: Session,
    //imu_temp_chan: mpsc::Receiver<sim_config::Status>,
    //imu_data_chan: mpsc::Receiver<sim_config::Status>,
    //battery_chan: mpsc::Receiver<sim_config::BatteryStatus>,
    gnss_chan: mpsc::Receiver<sim_config::GNSS>,
    baro_chan: mpsc::Receiver<sim_config::Barometer>,
    //mag_chan: mpsc::Receiver<sim_config::Status>,
    //diffpress_chan: mpsc::Receiver<sim_config::Status>,
}

impl Board for Sim {
    fn imu_read(&self) -> Option<Result<packets::ImuPacket, errors::SensorError>> {
        None
    }

    fn mag_read(&self) -> Option<Result<packets::MagPacket, errors::SensorError>> {
        None
    }

    fn baro_read(&self) -> Option<Result<packets::BaroPacket, errors::SensorError>> {
        None
    }

    fn diff_pressure_read(&self) -> Option<Result<packets::PitotPacket, errors::SensorError>> {
        None
    }

    fn sonar_read(&self) -> Option<Result<packets::RangePacket, errors::SensorError>> {
        None
    }

    //TODO getting lots of errors here...
    fn gnss_read(&self) -> Option<Result<packets::GNSSPacket, errors::SensorError>> {
        match self.gnss_chan.try_recv() {
            Some(gnss) => {}
            None => {}
        }
    }

    fn battery_read(&self) -> Option<Result<packets::BatteryPacket, errors::SensorError>> {
        None
    }

    fn rc_read(&self) -> Option<Result<packets::RcPacket, errors::SensorError>> {
        None
    }

    fn attitude_read(&self) -> Option<Result<packets::AttitudePacket, errors::SensorError>> {
        None
    }

    fn serial_rx_read(&self) -> Option<Result<packets::SerialRxPacket, errors::TelemError>> {
        None
    }

    fn serial_tx_write(
        &self,
        bytes: &[u8],
    ) -> Option<Result<packets::SerialTxPacket, errors::TelemError>> {
        None
    }
}

impl Sim {
    pub async fn new() -> Sim {
        let mut zenoh_connect_config = Config::default();
        config
            .insert_json5("connect/endpoints", r#"["tcp/127.0.0.1:7447"]"#)
            .unwrap();
        let mut zenoh_listen_config = Config::default();
        config
            .insert_json5("listen/endpoints", r#"["tcp/127.0.0.1:7447"]"#)
            .unwrap();

        let zenoh_connect_session = zenoh::open(zenoh_connect_config).await.unwrap();
        let zenoh_listen_session = zenoh::open(zenoh_listen_config).await.unwrap();

        // Establish all channels for sub
        //let (chan_send_imu_temp, mut chan_recv_imu_temp) = mpsc::channel::<sim_config::Status>(1);
        //let (chan_send_imu_data, mut chan_recv_imu_data) = mpsc::channel::<sim_config::Status>(1);
        //let (chan_send_battery, mut chan_recv_battery) =
        //    mpsc::channel::<sim_config::BatteryStatus>(1);
        let (chan_send_gnss, mut chan_recv_gnss) = mpsc::channel::<sim_config::GNSS>(1);
        let (chan_send_baro, mut chan_recv_baro) = mpsc::channel::<sim_config::Barometer>(1);
        //let (send_mag, mut recv_baro) = mpsc::channel::<sim_config::Barometer>(1);
        //let (send_diffpressure, mut recv_diffpressure) = mpsc::channel::<sim_config::Barometer>(1);

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
            .declare_subscriber("/rt/simulated_sensors/gnss")
            .await
            .unwrap();
        let sub_baro = zenoh_listen_session
            .declare_subscriber("/rt/simulated_sensors/baro")
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

        // establish all channels for pub
        let (chan_send_pwm, mut chan_recv_pwm) = mpsc::channel::<sim_config::Status>(1);

        // establish publisher
        let pub_pwm_output = zenoh_connect_session
            .declare_publisher("/rt/sim/pwm_output")
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
    }
}

//async fn capture_imu_temp(sub: Subscriber<FifoChannelHandler<Sample>>) {}

//async fn capture_imu_data(sub: Subscriber<FifoChannelHandler<Sample>>) {}

//async fn capture_battery(
//    sub: Subscriber<FifoChannelHandler<Sample>>,
//    chan: mpsc::Sender<sim_config::BatteryStatus>,
//) {
//    while let Ok(sample) = sub.recv_async().await {
//        match cdr::deserialize::<sim_config::BatteryStatus>(&sample.payload().to_bytes()) {
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
    chan: mpsc::Sender<sim_config::GNSS>,
) {
    while let Ok(sample) = sub.recv_async().await {
        match cdr::deserialize::<sim_config::GNSS>(&sample.payload().to_bytes()) {
            Ok(battery_status) => {
                if chan.send(battery_status).await.is_err() {
                    println!("Error putting gnss in channel!");
                }
            }
            Err(_) => {}
        }
    }
}

async fn capture_baro(
    sub: Subscriber<FifoChannelHandler<Sample>>,
    chan: mpsc::Sender<sim_config::Barometer>,
) {
    while let Ok(sample) = sub.recv_async().await {
        match cdr::deserialize::<sim_config::GNSS>(&sample.payload().to_bytes()) {
            Ok(battery_status) => {
                if chan.send(battery_status).await.is_err() {
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

async fn publish_pwm(publisher: Publisher<'_>, chan: mpsc::Receiver<sim_config::Status>) {
    if let Some(pwm) = chan.recv() {
        zb = ZBytes::from(cdr::serialize::<_, _, CdrLe>(&pwm, Infinite).unwrap());
        if publisher.put(zb).await.is_err() {
            println!("Error sending zbytes: pwm");
        }
    }
}
