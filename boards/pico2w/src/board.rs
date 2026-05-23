use crate::{
    comms_core::{SHARED_MAVLINK_MAILBOX, SharedMavlinkMailbox},
    config::Pico2WConfig,
    gy91::Gy91,
    pwm::PioPwmDriver,
};
use embassy_time::Instant;
use embedded_hal_nb::serial::{Read, Write};
use rp2350_platform::hal::uart::{Blocking, Uart};
use voloxide_core::{
    board::{BoardIo, SerialTxPriority},
    errors,
    params::Params,
    sensors::SensorBus,
};

const UART_TX_QUEUE_CAPACITY: usize = 4096;

pub enum MavlinkTransport {
    WifiMailbox(SharedMavlinkMailbox),
    Uart(Uart<'static, Blocking>),
}

struct UartTxQueue {
    bytes: [u8; UART_TX_QUEUE_CAPACITY],
    head: usize,
    tail: usize,
    len: usize,
}

impl UartTxQueue {
    const fn new() -> Self {
        Self {
            bytes: [0; UART_TX_QUEUE_CAPACITY],
            head: 0,
            tail: 0,
            len: 0,
        }
    }

    fn clear(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.len = 0;
    }

    fn free(&self) -> usize {
        UART_TX_QUEUE_CAPACITY - self.len
    }

    fn push_slice(&mut self, bytes: &[u8]) -> bool {
        if bytes.len() > self.free() {
            return false;
        }

        for byte in bytes {
            self.bytes[self.tail] = *byte;
            self.tail = (self.tail + 1) % UART_TX_QUEUE_CAPACITY;
            self.len += 1;
        }
        true
    }

    fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }

        let byte = self.bytes[self.head];
        self.head = (self.head + 1) % UART_TX_QUEUE_CAPACITY;
        self.len -= 1;
        Some(byte)
    }
}

pub struct Board {
    config: Pico2WConfig,
    mavlink: MavlinkTransport,
    uart_tx_queue: UartTxQueue,
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
                uart_tx_queue: UartTxQueue::new(),
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
                uart_tx_queue: UartTxQueue::new(),
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

    fn uart_tx_drain(
        uart: &mut Uart<'static, Blocking>,
        queue: &mut UartTxQueue,
    ) -> Result<(), errors::TelemError> {
        while let Some(byte) = queue.pop() {
            match Write::write(uart, byte) {
                Ok(()) => {}
                Err(nb::Error::WouldBlock) => {
                    queue.head = if queue.head == 0 {
                        UART_TX_QUEUE_CAPACITY - 1
                    } else {
                        queue.head - 1
                    };
                    queue.len += 1;
                    break;
                }
                Err(nb::Error::Other(_)) => {
                    return Err(errors::TelemError::GenericTelemError("uart write error"));
                }
            }
        }
        Ok(())
    }

    fn uart_tx_enqueue(
        uart: &mut Uart<'static, Blocking>,
        queue: &mut UartTxQueue,
        bytes: &[u8],
        priority: SerialTxPriority,
    ) -> Result<usize, errors::TelemError> {
        Self::uart_tx_drain(uart, queue)?;

        if bytes.len() > queue.free() {
            if priority >= SerialTxPriority::HIGH {
                queue.clear();
            } else {
                return Ok(0);
            }
        }

        if !queue.push_slice(bytes) {
            return Ok(0);
        }

        Self::uart_tx_drain(uart, queue)?;
        Ok(bytes.len())
    }
}

impl BoardIo for Board {
    fn update_sensor_bus<R: voloxide_core::math::FlightFloat>(
        &mut self,
        sensors: &mut SensorBus<R>,
    ) {
        sensors.clear();
        let now_us = self.clock_micros();
        if let Some(gy91) = &mut self.gy91 {
            match gy91.sample_imu(now_us) {
                Ok(Some(imu)) => sensors.imu = Some(Ok(imu.cast())),
                Ok(None) => {}
                Err(err) => sensors.imu = Some(Err(err.sensor_error())),
            }
            match gy91.sample_baro(now_us) {
                Ok(Some(baro)) => sensors.baro = Some(Ok(baro)),
                Ok(None) => {}
                Err(err) => sensors.baro = Some(Err(err.sensor_error())),
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
        self.serial_tx_write_priority(bytes, SerialTxPriority::NORMAL)
    }

    fn serial_tx_write_priority(
        &mut self,
        bytes: &[u8],
        priority: SerialTxPriority,
    ) -> Option<Result<usize, errors::TelemError>> {
        Some(match &mut self.mavlink {
            MavlinkTransport::WifiMailbox(mailbox) => {
                Ok(mailbox.write_from_priority(bytes, priority))
            }
            MavlinkTransport::Uart(uart) => {
                Self::uart_tx_enqueue(uart, &mut self.uart_tx_queue, bytes, priority)
            }
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
