use crate::{comms_core::MavlinkMailbox, config::Pico2WConfig, pwm::PioPwmDriver};
use voloxide_core::{board::BoardIo, errors, params::Params, sensors::SensorBus};

pub struct Board {
    config: Pico2WConfig,
    mailbox: MavlinkMailbox,
    params: Params,
    params_valid: bool,
    micros: u64,
}

impl Board {
    pub fn new(config: Pico2WConfig) -> (Self, PioPwmDriver) {
        (
            Self {
                config,
                mailbox: MavlinkMailbox::new(),
                params: Params::default(),
                params_valid: false,
                micros: 0,
            },
            PioPwmDriver::new(),
        )
    }

    pub fn config(&self) -> Pico2WConfig {
        self.config
    }

    pub fn mavlink_mailbox_mut(&mut self) -> &mut MavlinkMailbox {
        &mut self.mailbox
    }
}

impl BoardIo for Board {
    fn update_sensor_bus(&mut self, sensors: &mut SensorBus) {
        sensors.clear();
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        Some(Ok(self.mailbox.read_into(buf)))
    }

    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        Some(Ok(self.mailbox.write_from(bytes)))
    }

    fn clock_millis(&self) -> u32 {
        (self.micros / 1000) as u32
    }

    fn clock_micros(&self) -> u64 {
        self.micros
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
