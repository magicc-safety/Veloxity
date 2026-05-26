use crate::{
    comms_core::{SHARED_MAVLINK_MAILBOX, SharedMavlinkMailbox},
    config::Pico2WConfig,
    gy91::Gy91,
    ism330dhcx::{SHARED_ISM330DHCX_IMU_QUEUE, SharedIsm330dhcxImuQueue},
    pwm::PioPwmDriver,
    rc_receiver::{SHARED_CRSF_RC_QUEUE, SharedCrsfRcQueue},
};
use embassy_time::Instant;
use voloxide_core::{
    board::{BoardIo, SerialRxFrame, SerialTxPriority},
    errors,
    packets::BaroPacket,
    params::Params,
    sensors::SensorBus,
};

struct PicoSensorProducer {
    ism330dhcx_imu: SharedIsm330dhcxImuQueue,
    crsf_rc: SharedCrsfRcQueue,
    gy91_baro: Option<Gy91>,
    pending_baro: Option<Result<BaroPacket, errors::SensorError>>,
}

impl PicoSensorProducer {
    fn new(gy91_baro: Option<Gy91>) -> Self {
        Self {
            ism330dhcx_imu: SHARED_ISM330DHCX_IMU_QUEUE,
            crsf_rc: SHARED_CRSF_RC_QUEUE,
            gy91_baro,
            pending_baro: None,
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
        if self.ism330dhcx_imu.has_pending()
            && let Some(imu) = self.ism330dhcx_imu.take_latest()
        {
            sensors.imu = Some(Ok(imu.cast()));
        }
        if self.crsf_rc.has_pending()
            && let Some(rc) = self.crsf_rc.take_latest()
        {
            sensors.rc = Some(Ok(rc));
        }
        if let Some(baro) = self.pending_baro.take() {
            sensors.baro = Some(baro);
        }
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
}

impl Board {
    pub fn new_uart(config: Pico2WConfig, gy91_baro: Option<Gy91>) -> (Self, PioPwmDriver) {
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

    fn serial_rx_pending(&self) -> bool {
        self.mavlink.has_pending_rx_frame()
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
