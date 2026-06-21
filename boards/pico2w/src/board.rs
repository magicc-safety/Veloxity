use crate::config::Pico2WConfig;
use embassy_time::Instant;
#[cfg(feature = "scope-timing-pins")]
use rp2350_platform::hal::gpio::Output;
use rp2350_platform::{
    comms::{SHARED_MAVLINK_MAILBOX, SharedMavlinkMailbox},
    peripherals::{
        barometer::{SHARED_BARO_QUEUE, SharedBaroQueue},
        gps::{SHARED_GNSS_QUEUE, SharedGnssQueue},
        ism330dhcx::{SHARED_ISM330DHCX_IMU_QUEUE, SharedIsm330dhcxImuQueue},
        pwm::PioPwmDriver,
        rc_receiver::{SHARED_CRSF_RC_QUEUE, SharedCrsfRcQueue},
    },
};
use veloxity_core::{
    board::{BoardIo, SerialRxFrame, SerialTxPriority},
    comm::messages::messages::DownlinkMessage,
    errors,
    params::Params,
    sensors::SensorBus,
};

struct PicoSensorProducer {
    ism330dhcx_imu: SharedIsm330dhcxImuQueue,
    crsf_rc: SharedCrsfRcQueue,
    gnss: SharedGnssQueue,
    baro: SharedBaroQueue,
    last_imu_seq: Option<u32>,
    imu_seq_gaps: u32,
}

impl PicoSensorProducer {
    fn new() -> Self {
        Self {
            ism330dhcx_imu: SHARED_ISM330DHCX_IMU_QUEUE,
            crsf_rc: SHARED_CRSF_RC_QUEUE,
            gnss: SHARED_GNSS_QUEUE,
            baro: SHARED_BARO_QUEUE,
            last_imu_seq: None,
            imu_seq_gaps: 0,
        }
    }

    fn drain_into<R: veloxity_core::math::FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        self.drain_imu_into(sensors);
        self.drain_service_into(sensors);
    }

    fn drain_service_into<R: veloxity_core::math::FlightFloat>(
        &mut self,
        sensors: &mut SensorBus<R>,
    ) {
        if let Some(rc) = self.crsf_rc.take_latest() {
            sensors.rc = Some(Ok(rc));
        }
        if let Some(gnss) = self.gnss.take_latest() {
            sensors.gnss = Some(gnss);
        }
        if let Some(baro) = self.baro.take_latest() {
            sensors.baro = Some(baro);
        }
    }

    fn imu_pending(&self) -> bool {
        self.ism330dhcx_imu.has_pending()
    }

    fn drain_imu_into<R: veloxity_core::math::FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        let Some(imu) = self.ism330dhcx_imu.take_latest() else {
            return;
        };

        if let Some(last_seq) = self.last_imu_seq {
            let expected = last_seq.wrapping_add(1);
            if imu.seq != expected {
                self.imu_seq_gaps = self
                    .imu_seq_gaps
                    .wrapping_add(imu.seq.wrapping_sub(expected).max(1));
            }
        }
        self.last_imu_seq = Some(imu.seq);
        sensors.imu = Some(Ok(imu.cast()));
    }
}

pub struct Board {
    config: Pico2WConfig,
    mavlink: SharedMavlinkMailbox,
    sensors: PicoSensorProducer,
    params: Params,
    params_valid: bool,
    boot_time: Instant,
    #[cfg(feature = "scope-timing-pins")]
    deadline_scope_pin: Output<'static>,
    #[cfg(feature = "scope-timing-pins")]
    control_scope_pin: Output<'static>,
    #[cfg(all(feature = "scope-timing-pins", not(feature = "imu-producer-scope")))]
    non_control_scope_pin: Output<'static>,
}

impl Board {
    pub fn new_uart(
        config: Pico2WConfig,
        #[cfg(feature = "scope-timing-pins")] deadline_scope_pin: Output<'static>,
        #[cfg(feature = "scope-timing-pins")] control_scope_pin: Output<'static>,
        #[cfg(all(feature = "scope-timing-pins", not(feature = "imu-producer-scope")))]
        non_control_scope_pin: Output<'static>,
    ) -> (Self, PioPwmDriver) {
        (
            Self {
                config,
                mavlink: SHARED_MAVLINK_MAILBOX,
                sensors: PicoSensorProducer::new(),
                params: Params::default(),
                params_valid: false,
                boot_time: Instant::now(),
                #[cfg(feature = "scope-timing-pins")]
                deadline_scope_pin,
                #[cfg(feature = "scope-timing-pins")]
                control_scope_pin,
                #[cfg(all(feature = "scope-timing-pins", not(feature = "imu-producer-scope")))]
                non_control_scope_pin,
            },
            PioPwmDriver::new(),
        )
    }

    pub fn config(&self) -> Pico2WConfig {
        self.config
    }

    pub fn mavlink_mailbox(&self) -> SharedMavlinkMailbox {
        self.mavlink
    }
}

impl BoardIo for Board {
    fn update_sensor_bus<R: veloxity_core::math::FlightFloat>(
        &mut self,
        sensors: &mut SensorBus<R>,
    ) {
        sensors.clear();
        self.sensors.drain_into(sensors);
    }

    fn imu_pending(&self) -> bool {
        self.sensors.imu_pending()
    }

    fn update_imu_sensor<R: veloxity_core::math::FlightFloat>(
        &mut self,
        sensors: &mut SensorBus<R>,
    ) {
        sensors.clear();
        self.sensors.drain_imu_into(sensors);
    }

    fn update_service_sensor_bus<R: veloxity_core::math::FlightFloat>(
        &mut self,
        sensors: &mut SensorBus<R>,
    ) {
        sensors.clear();
        self.sensors.drain_service_into(sensors);
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        Some(Ok(self.mavlink.read_into(buf)))
    }

    fn serial_rx_frame_read(&mut self) -> Option<Result<SerialRxFrame, errors::TelemError>> {
        self.mavlink.pop_rx_frame().map(Ok)
    }

    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        self.serial_tx_write_priority(bytes, SerialTxPriority::DEFAULT)
    }

    fn serial_tx_write_priority(
        &mut self,
        bytes: &[u8],
        priority: SerialTxPriority,
    ) -> Option<Result<usize, errors::TelemError>> {
        Some(Ok(self.mavlink.write_from_priority(bytes, priority)))
    }

    fn serial_tx_enqueue_downlink(
        &mut self,
        system_id: u8,
        msg: DownlinkMessage,
        priority: SerialTxPriority,
    ) -> Option<Result<usize, errors::TelemError>> {
        Some(Ok(self.mavlink.enqueue_downlink(system_id, msg, priority)))
    }

    fn serial_rx_pending(&self) -> bool {
        self.mavlink.has_pending_rx_frame()
    }

    #[cfg(feature = "scope-timing-pins")]
    fn set_test_pin_1(&mut self, high: bool) {
        if high {
            self.deadline_scope_pin.set_high();
        } else {
            self.deadline_scope_pin.set_low();
        }
    }

    #[cfg(feature = "scope-timing-pins")]
    fn set_test_pin_2(&mut self, high: bool) {
        if high {
            self.control_scope_pin.set_high();
        } else {
            self.control_scope_pin.set_low();
        }
    }

    #[cfg(all(feature = "scope-timing-pins", not(feature = "imu-producer-scope")))]
    fn set_test_pin_3(&mut self, high: bool) {
        if high {
            self.non_control_scope_pin.set_high();
        } else {
            self.non_control_scope_pin.set_low();
        }
    }

    fn clock_millis(&self) -> u32 {
        (self.clock_micros() / 1000) as u32
    }

    fn clock_micros(&self) -> u64 {
        self.boot_time.elapsed().as_micros()
    }

    fn read_params(&mut self, params: &mut Params) -> bool {
        if !self.params_valid {
            return false;
        }
        *params = self.params;
        true
    }

    fn write_params(&mut self, params: &Params) -> bool {
        self.params = *params;
        self.params_valid = true;
        true
    }

    fn run_deferred_board_actions(&mut self) {}
}
