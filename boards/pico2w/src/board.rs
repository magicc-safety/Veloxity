use crate::{
    comms_core::{SHARED_MAVLINK_MAILBOX, SharedMavlinkMailbox},
    config::Pico2WConfig,
    gy91::Gy91,
    pwm::PioPwmDriver,
};
use embassy_time::Instant;
use embedded_hal_nb::serial::{Read, Write};
use rp2350_platform::hal::uart::{Blocking, Uart};
use voloxide_core::{board::BoardIo, errors, params::Params, sensors::SensorBus};

pub enum MavlinkTransport {
    WifiMailbox(SharedMavlinkMailbox),
    Uart(Uart<'static, Blocking>),
}

pub struct Board {
    config: Pico2WConfig,
    mavlink: MavlinkTransport,
    gy91: Option<Gy91>,
    params: Params,
    params_valid: bool,
    boot_time: Instant,
}

impl Board {
    pub fn new_wifi(config: Pico2WConfig, gy91: Option<Gy91>) -> (Self, PioPwmDriver) {
        (
            Self {
                config,
                mavlink: MavlinkTransport::WifiMailbox(SHARED_MAVLINK_MAILBOX),
                gy91,
                params: Params::default(),
                params_valid: false,
                boot_time: Instant::now(),
            },
            PioPwmDriver::new(),
        )
    }

    pub fn new_uart(
        config: Pico2WConfig,
        uart: Uart<'static, Blocking>,
        gy91: Option<Gy91>,
    ) -> (Self, PioPwmDriver) {
        (
            Self {
                config,
                mavlink: MavlinkTransport::Uart(uart),
                gy91,
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

    pub fn mavlink_mailbox(&self) -> Option<SharedMavlinkMailbox> {
        match self.mavlink {
            MavlinkTransport::WifiMailbox(mailbox) => Some(mailbox),
            MavlinkTransport::Uart(_) => None,
        }
    }

    fn uart_rx_read(
        uart: &mut Uart<'static, Blocking>,
        buf: &mut [u8],
    ) -> Result<usize, errors::TelemError> {
        let mut n = 0;
        while n < buf.len() {
            match Read::read(uart) {
                Ok(byte) => {
                    buf[n] = byte;
                    n += 1;
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) if n > 0 => break,
                Err(nb::Error::Other(_)) => {
                    return Err(errors::TelemError::GenericTelemError("uart read error"));
                }
            }
        }
        Ok(n)
    }

    fn uart_tx_write(
        uart: &mut Uart<'static, Blocking>,
        bytes: &[u8],
    ) -> Result<usize, errors::TelemError> {
        let mut n = 0;
        while n < bytes.len() {
            match Write::write(uart, bytes[n]) {
                Ok(()) => n += 1,
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) if n > 0 => break,
                Err(nb::Error::Other(_)) => {
                    return Err(errors::TelemError::GenericTelemError("uart write error"));
                }
            }
        }
        let _ = Write::flush(uart);
        Ok(n)
    }
}

impl BoardIo for Board {
    fn update_sensor_bus(&mut self, sensors: &mut SensorBus) {
        sensors.clear();
        let now_us = self.clock_micros();
        if let Some(gy91) = &mut self.gy91 {
            sensors.imu = Some(gy91.sample_imu(now_us).map_err(|err| err.sensor_error()));
            match gy91.sample_baro(now_us) {
                Ok(Some(baro)) => sensors.baro = Some(Ok(baro)),
                Ok(None) => {}
                Err(err) => sensors.baro = Some(Err(err.sensor_error())),
            }
            match gy91.sample_mag(now_us) {
                Ok(Some(mag)) => sensors.mag = Some(Ok(mag)),
                Ok(None) => {}
                Err(err) => sensors.mag = Some(Err(err.sensor_error())),
            }
        }
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        Some(match &mut self.mavlink {
            MavlinkTransport::WifiMailbox(mailbox) => Ok(mailbox.read_into(buf)),
            MavlinkTransport::Uart(uart) => Self::uart_rx_read(uart, buf),
        })
    }

    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        Some(match &mut self.mavlink {
            MavlinkTransport::WifiMailbox(mailbox) => Ok(mailbox.write_from(bytes)),
            MavlinkTransport::Uart(uart) => Self::uart_tx_write(uart, bytes),
        })
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
}
