#[cfg(feature = "timing-diagnostics")]
use crate::gps::gps_stats;
#[cfg(all(feature = "timing-diagnostics", feature = "ism330dhcx-driver"))]
use crate::ism330dhcx::ism330dhcx_stats;
#[cfg(feature = "timing-diagnostics")]
use crate::rc_receiver::crsf_stats;
use crate::{
    barometer::{SHARED_BARO_QUEUE, SharedBaroQueue},
    comms_core::{SHARED_MAVLINK_MAILBOX, SharedMavlinkMailbox},
    config::Pico2WConfig,
    gps::{SHARED_GNSS_QUEUE, SharedGnssQueue},
    gy91::Gy91,
    ism330dhcx::{SHARED_ISM330DHCX_IMU_QUEUE, SharedIsm330dhcxImuQueue},
    pwm::PioPwmDriver,
    rc_receiver::{SHARED_CRSF_RC_QUEUE, SharedCrsfRcQueue},
};
use embassy_time::Instant;
#[cfg(feature = "scope-timing-pins")]
use rp2350_platform::hal::gpio::Output;
use voloxide_core::{
    board::{BoardIo, SerialRxFrame, SerialTxPriority},
    comm::messages::messages::DownlinkMessage,
    errors,
    packets::BaroPacket,
    params::Params,
    sensors::SensorBus,
};

struct PicoSensorProducer {
    ism330dhcx_imu: SharedIsm330dhcxImuQueue,
    crsf_rc: SharedCrsfRcQueue,
    gnss: SharedGnssQueue,
    baro: SharedBaroQueue,
    gy91_baro: Option<Gy91>,
    pending_baro: Option<Result<BaroPacket, errors::SensorError>>,
    last_imu_seq: Option<u32>,
    imu_seq_gaps: u32,
}

impl PicoSensorProducer {
    fn new(gy91_baro: Option<Gy91>) -> Self {
        Self {
            ism330dhcx_imu: SHARED_ISM330DHCX_IMU_QUEUE,
            crsf_rc: SHARED_CRSF_RC_QUEUE,
            gnss: SHARED_GNSS_QUEUE,
            baro: SHARED_BARO_QUEUE,
            gy91_baro,
            pending_baro: None,
            last_imu_seq: None,
            imu_seq_gaps: 0,
        }
    }

    fn sample_due(&mut self, now_us: u64) {
        let Some(gy91_baro) = &mut self.gy91_baro else {
            return;
        };

        match gy91_baro.sample_baro(now_us) {
            Ok(Some(baro)) => self.pending_baro = Some(Ok(baro)),
            Ok(None) => {}
            Err(err) => self.pending_baro = Some(Err(err.sensor_error())),
        }
    }

    fn drain_into<R: voloxide_core::math::FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        self.drain_imu_into(sensors);
        self.drain_service_into(sensors);
    }

    fn drain_service_into<R: voloxide_core::math::FlightFloat>(
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
        if let Some(baro) = self.pending_baro.take() {
            sensors.baro = Some(baro);
        }
    }

    fn imu_pending(&self) -> bool {
        self.ism330dhcx_imu.has_pending()
    }

    fn drain_imu_into<R: voloxide_core::math::FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
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

    #[cfg(feature = "timing-diagnostics")]
    fn imu_queue_drops(&self) -> u32 {
        self.ism330dhcx_imu.dropped_oldest()
    }

    #[cfg(feature = "timing-diagnostics")]
    fn imu_sequence_gaps(&self) -> u32 {
        self.imu_seq_gaps
    }

    #[cfg(feature = "timing-diagnostics")]
    fn baro_queue_drops(&self) -> u32 {
        self.baro.dropped_oldest()
    }
}

pub struct Board {
    config: Pico2WConfig,
    mavlink: SharedMavlinkMailbox,
    sensors: PicoSensorProducer,
    #[cfg(feature = "timing-diagnostics")]
    last_serial_rx_count: usize,
    #[cfg(feature = "timing-diagnostics")]
    diag_index: u8,
    params: Params,
    params_valid: bool,
    boot_time: Instant,
    #[cfg(feature = "scope-timing-pins")]
    control_scope_pin: Output<'static>,
    #[cfg(all(feature = "scope-timing-pins", not(feature = "imu-producer-scope")))]
    non_control_scope_pin: Output<'static>,
}

impl Board {
    pub fn new_uart(
        config: Pico2WConfig,
        gy91_baro: Option<Gy91>,
        #[cfg(feature = "scope-timing-pins")] control_scope_pin: Output<'static>,
        #[cfg(all(feature = "scope-timing-pins", not(feature = "imu-producer-scope")))]
        non_control_scope_pin: Output<'static>,
    ) -> (Self, PioPwmDriver) {
        (
            Self {
                config,
                mavlink: SHARED_MAVLINK_MAILBOX,
                sensors: PicoSensorProducer::new(gy91_baro),
                #[cfg(feature = "timing-diagnostics")]
                last_serial_rx_count: 0,
                #[cfg(feature = "timing-diagnostics")]
                diag_index: 0,
                params: Params::default(),
                params_valid: false,
                boot_time: Instant::now(),
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
    fn update_sensor_bus<R: voloxide_core::math::FlightFloat>(
        &mut self,
        sensors: &mut SensorBus<R>,
    ) {
        sensors.clear();
        self.sensors.drain_into(sensors);
    }

    fn imu_pending(&self) -> bool {
        self.sensors.imu_pending()
    }

    fn update_imu_sensor<R: voloxide_core::math::FlightFloat>(
        &mut self,
        sensors: &mut SensorBus<R>,
    ) {
        sensors.clear();
        self.sensors.drain_imu_into(sensors);
    }

    fn update_service_sensor_bus<R: voloxide_core::math::FlightFloat>(
        &mut self,
        sensors: &mut SensorBus<R>,
    ) {
        sensors.clear();
        self.sensors.drain_service_into(sensors);
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        let result = Ok(self.mavlink.read_into(buf));
        #[cfg(feature = "timing-diagnostics")]
        {
            self.last_serial_rx_count = result.as_ref().ok().copied().unwrap_or(0);
        }
        Some(result)
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

    #[cfg(feature = "timing-diagnostics")]
    fn serial_rx_last_count(&self) -> usize {
        self.last_serial_rx_count
    }

    #[cfg(feature = "timing-diagnostics")]
    fn board_diagnostic_text(&mut self) -> Option<[u8; 50]> {
        let mailbox = self.mavlink_mailbox();
        let stats = mailbox.stats();
        let mut out = [0_u8; 50];
        match self.diag_index {
            0 => {
                let mut offset = 0;
                write_diag_bytes(&mut out, &mut offset, b"PUTX p");
                write_diag_num(&mut out, &mut offset, stats.tx_pending);
                write_diag_bytes(&mut out, &mut offset, b" b");
                write_diag_num(&mut out, &mut offset, stats.uart_tx_batches);
                write_diag_bytes(&mut out, &mut offset, b" x");
                write_diag_num(&mut out, &mut offset, stats.uart_tx_bytes);
                write_diag_bytes(&mut out, &mut offset, b" m");
                write_diag_num(&mut out, &mut offset, stats.uart_tx_max_batch);
                self.diag_index = 1;
                Some(out)
            }
            1 => {
                let mut offset = 0;
                write_diag_bytes(&mut out, &mut offset, b"PURX c");
                write_diag_num(&mut out, &mut offset, stats.uart_rx_chunks);
                write_diag_bytes(&mut out, &mut offset, b" b");
                write_diag_num(&mut out, &mut offset, stats.uart_rx_bytes);
                write_diag_bytes(&mut out, &mut offset, b" te");
                write_diag_num(&mut out, &mut offset, stats.uart_tx_errors);
                write_diag_bytes(&mut out, &mut offset, b" re");
                write_diag_num(&mut out, &mut offset, stats.uart_rx_errors);
                self.diag_index = 2;
                Some(out)
            }
            2 => {
                let mut offset = 0;
                write_diag_bytes(&mut out, &mut offset, b"PUQD p");
                write_diag_num(&mut out, &mut offset, stats.tx_pending);
                write_diag_bytes(&mut out, &mut offset, b" d");
                write_diag_num(&mut out, &mut offset, stats.tx_dropped);
                write_diag_bytes(&mut out, &mut offset, b" r");
                write_diag_num(&mut out, &mut offset, stats.tx_replaced);
                write_diag_bytes(&mut out, &mut offset, b" f");
                write_diag_num(&mut out, &mut offset, stats.tx_pending_frames);
                self.diag_index = 3;
                Some(out)
            }
            3 => {
                let mut offset = 0;
                write_diag_bytes(&mut out, &mut offset, b"PUPR w");
                write_diag_num(&mut out, &mut offset, stats.tx_priority_min as u32);
                write_diag_bytes(&mut out, &mut offset, b" x");
                write_diag_num(&mut out, &mut offset, stats.tx_priority_max as u32);
                write_diag_bytes(&mut out, &mut offset, b" dw");
                write_diag_num(&mut out, &mut offset, stats.tx_drop_priority_min as u32);
                write_diag_bytes(&mut out, &mut offset, b" dx");
                write_diag_num(&mut out, &mut offset, stats.tx_drop_priority_max as u32);
                self.diag_index = 4;
                Some(out)
            }
            4 => {
                let mut offset = 0;
                write_diag_bytes(&mut out, &mut offset, b"PUC1 h");
                write_diag_num(&mut out, &mut offset, stats.core1_heartbeats);
                write_diag_bytes(&mut out, &mut offset, b" st");
                write_diag_num(&mut out, &mut offset, stats.comms_state);
                write_diag_bytes(&mut out, &mut offset, b" rf");
                write_diag_num(&mut out, &mut offset, stats.rx_pending_frames);
                self.diag_index = 5;
                Some(out)
            }
            5 => {
                let mut offset = 0;
                write_diag_bytes(&mut out, &mut offset, b"PURF p");
                write_diag_num(&mut out, &mut offset, stats.rx_frames_pushed);
                write_diag_bytes(&mut out, &mut offset, b" d");
                write_diag_num(&mut out, &mut offset, stats.rx_frames_dropped);
                write_diag_bytes(&mut out, &mut offset, b" r");
                write_diag_num(&mut out, &mut offset, stats.rx_frames_replaced);
                write_diag_bytes(&mut out, &mut offset, b" e");
                write_diag_num(&mut out, &mut offset, stats.uart_rx_parse_errors);
                self.diag_index = 6;
                Some(out)
            }
            6 => {
                let mut offset = 0;
                write_diag_bytes(&mut out, &mut offset, b"PUDQ p");
                write_diag_num(&mut out, &mut offset, stats.downlink_pending);
                write_diag_bytes(&mut out, &mut offset, b" e");
                write_diag_num(&mut out, &mut offset, stats.downlink_enqueued);
                write_diag_bytes(&mut out, &mut offset, b" o");
                write_diag_num(&mut out, &mut offset, stats.downlink_drained);
                write_diag_bytes(&mut out, &mut offset, b" d");
                write_diag_num(&mut out, &mut offset, stats.downlink_dropped);
                write_diag_bytes(&mut out, &mut offset, b" r");
                write_diag_num(&mut out, &mut offset, stats.downlink_replaced);
                self.diag_index = 7;
                Some(out)
            }
            7 => {
                let mut offset = 0;
                write_diag_bytes(&mut out, &mut offset, b"PUDP w");
                write_diag_num(&mut out, &mut offset, stats.downlink_priority_min as u32);
                write_diag_bytes(&mut out, &mut offset, b" x");
                write_diag_num(&mut out, &mut offset, stats.downlink_priority_max as u32);
                write_diag_bytes(&mut out, &mut offset, b" dw");
                write_diag_num(
                    &mut out,
                    &mut offset,
                    stats.downlink_drop_priority_min as u32,
                );
                write_diag_bytes(&mut out, &mut offset, b" dx");
                write_diag_num(
                    &mut out,
                    &mut offset,
                    stats.downlink_drop_priority_max as u32,
                );
                self.diag_index = 8;
                Some(out)
            }
            8 => {
                let stats = gps_stats();
                let mut offset = 0;
                write_diag_bytes(&mut out, &mut offset, b"GPS b");
                write_diag_num(&mut out, &mut offset, stats.total_bytes);
                write_diag_bytes(&mut out, &mut offset, b" s");
                write_diag_num(&mut out, &mut offset, stats.ubx_sync);
                write_diag_bytes(&mut out, &mut offset, b" f");
                write_diag_num(&mut out, &mut offset, stats.ubx_frames);
                write_diag_bytes(&mut out, &mut offset, b" l");
                write_diag_num(&mut out, &mut offset, stats.last_frame);
                write_diag_bytes(&mut out, &mut offset, b" p");
                write_diag_num(&mut out, &mut offset, stats.nav_pvt);
                self.diag_index = 9;
                Some(out)
            }
            9 => {
                let stats = crsf_stats();
                let mut offset = 0;
                write_diag_bytes(&mut out, &mut offset, b"CRSF b");
                write_diag_num(&mut out, &mut offset, stats.bytes);
                write_diag_bytes(&mut out, &mut offset, b" f");
                write_diag_num(&mut out, &mut offset, stats.frames);
                write_diag_bytes(&mut out, &mut offset, b" e");
                write_diag_num(&mut out, &mut offset, stats.read_errors);
                write_diag_bytes(&mut out, &mut offset, b" d");
                write_diag_num(&mut out, &mut offset, stats.queue_drops);
                self.diag_index = 10;
                Some(out)
            }
            10 => {
                #[cfg(feature = "ism330dhcx-driver")]
                let stats = ism330dhcx_stats();
                let mut offset = 0;
                write_diag_bytes(&mut out, &mut offset, b"IMU0 a");
                #[cfg(feature = "ism330dhcx-driver")]
                write_diag_num(&mut out, &mut offset, stats.init_attempts);
                #[cfg(not(feature = "ism330dhcx-driver"))]
                write_diag_num(&mut out, &mut offset, 0);
                write_diag_bytes(&mut out, &mut offset, b" o");
                #[cfg(feature = "ism330dhcx-driver")]
                write_diag_num(&mut out, &mut offset, stats.init_ok);
                #[cfg(not(feature = "ism330dhcx-driver"))]
                write_diag_num(&mut out, &mut offset, 0);
                write_diag_bytes(&mut out, &mut offset, b" w");
                #[cfg(feature = "ism330dhcx-driver")]
                write_diag_num(&mut out, &mut offset, stats.last_who_am_i as u32);
                #[cfg(not(feature = "ism330dhcx-driver"))]
                write_diag_num(&mut out, &mut offset, 0);
                write_diag_bytes(&mut out, &mut offset, b" e");
                #[cfg(feature = "ism330dhcx-driver")]
                write_diag_num(&mut out, &mut offset, stats.drdy_edges);
                #[cfg(not(feature = "ism330dhcx-driver"))]
                write_diag_num(&mut out, &mut offset, 0);
                self.diag_index = 11;
                Some(out)
            }
            11 => {
                #[cfg(feature = "ism330dhcx-driver")]
                let stats = ism330dhcx_stats();
                let mut offset = 0;
                write_diag_bytes(&mut out, &mut offset, b"IMU1 r");
                #[cfg(feature = "ism330dhcx-driver")]
                write_diag_num(&mut out, &mut offset, stats.read_ok);
                #[cfg(not(feature = "ism330dhcx-driver"))]
                write_diag_num(&mut out, &mut offset, 0);
                write_diag_bytes(&mut out, &mut offset, b" x");
                #[cfg(feature = "ism330dhcx-driver")]
                write_diag_num(&mut out, &mut offset, stats.read_errors);
                #[cfg(not(feature = "ism330dhcx-driver"))]
                write_diag_num(&mut out, &mut offset, 0);
                write_diag_bytes(&mut out, &mut offset, b" q");
                write_diag_num(&mut out, &mut offset, self.sensors.imu_queue_drops());
                write_diag_bytes(&mut out, &mut offset, b" g");
                write_diag_num(&mut out, &mut offset, self.sensors.imu_sequence_gaps());
                self.diag_index = 12;
                Some(out)
            }
            12 => {
                let mut offset = 0;
                write_diag_bytes(&mut out, &mut offset, b"BRDQ d");
                write_diag_num(&mut out, &mut offset, self.sensors.baro_queue_drops());
                self.diag_index = 13;
                Some(out)
            }
            _ => {
                self.diag_index = 0;
                None
            }
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

    fn run_deferred_board_actions(&mut self) {
        let now_us = self.clock_micros();
        self.sensors.sample_due(now_us);
    }
}

#[cfg(feature = "timing-diagnostics")]
fn write_diag_bytes(out: &mut [u8; 50], offset: &mut usize, bytes: &[u8]) {
    for byte in bytes {
        if *offset >= out.len() {
            return;
        }
        out[*offset] = *byte;
        *offset += 1;
    }
}

#[cfg(feature = "timing-diagnostics")]
fn write_diag_num(out: &mut [u8; 50], offset: &mut usize, mut value: u32) {
    let mut digits = [0_u8; 10];
    let mut len = 0;
    loop {
        digits[len] = b'0' + (value % 10) as u8;
        len += 1;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    while len > 0 {
        len -= 1;
        write_diag_bytes(out, offset, &digits[len..=len]);
    }
}
