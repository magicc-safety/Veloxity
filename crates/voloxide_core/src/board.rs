use crate::{errors, params::Params, sensors::SensorBus};
pub mod dummy;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BackupData {
    pub error_code: u32,
    pub pc: u32,
    pub reset_count: u32,
    pub do_rearm: u32,
}

pub trait BoardIo {
    fn update_sensor_bus(&mut self, sensors: &mut SensorBus) {
        sensors.clear();
    }
    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>>;
    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>>;

    fn clock_millis(&self) -> u32;
    fn clock_micros(&self) -> u64;
    fn set_test_pin_1(&mut self, _high: bool) {}
    fn set_test_pin_2(&mut self, _high: bool) {}
    fn read_params(&mut self, _params: &mut Params) -> bool {
        false
    }
    fn write_params(&mut self, _params: &Params) -> bool {
        false
    }
    fn sensors_errors_count(&self) -> u16 {
        0
    }
    fn sensors_errors_message(&self, _index: u16) -> Option<[u8; 50]> {
        None
    }
    fn serial_flush(&mut self) {}
    fn led0_on(&mut self) {}
    fn led0_off(&mut self) {}
    fn led0_toggle(&mut self) {}
    fn led1_on(&mut self) {}
    fn led1_off(&mut self) {}
    fn led1_toggle(&mut self) {}
    fn backup_memory_read(&mut self) -> Option<BackupData> {
        None
    }
    fn backup_memory_write(&mut self, _data: BackupData) -> bool {
        false
    }
    fn backup_memory_clear(&mut self) -> bool {
        false
    }
    fn reboot(&mut self) -> bool {
        false
    }
    fn reboot_to_bootloader(&mut self) -> bool {
        false
    }
    fn run_deferred_board_actions(&mut self) {}
}
