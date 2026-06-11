use crate::{
    comm::messages::messages::DownlinkMessage, errors, math::FlightFloat, params::Params,
    sensors::SensorBus,
};
pub mod dummy;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BackupData {
    pub error_code: u32,
    pub pc: u32,
    pub reset_count: u32,
    pub do_rearm: u32,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct SerialTxPriority(pub u8);

impl SerialTxPriority {
    pub const REPLACEABLE_TELEMETRY: Self = Self(32);
    pub const DEFAULT: Self = Self(128);
    pub const CRITICAL: Self = Self(224);
}

pub type SerialRxPriority = SerialTxPriority;

pub const SERIAL_RX_FRAME_MAX_BYTES: usize = 280;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SerialRxFrame {
    pub data: [u8; SERIAL_RX_FRAME_MAX_BYTES],
    pub len: usize,
}

impl SerialRxFrame {
    pub const fn empty() -> Self {
        Self {
            data: [0; SERIAL_RX_FRAME_MAX_BYTES],
            len: 0,
        }
    }
}

impl Default for SerialRxFrame {
    fn default() -> Self {
        Self::empty()
    }
}

pub trait BoardIo {
    fn update_sensor_bus<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        sensors.clear();
    }
    fn imu_pending(&self) -> bool {
        false
    }
    fn update_imu_sensor<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        self.update_sensor_bus(sensors);
    }
    fn update_service_sensor_bus<R: FlightFloat>(&mut self, sensors: &mut SensorBus<R>) {
        self.update_sensor_bus(sensors);
    }
    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>>;
    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>>;
    fn serial_tx_write_priority(
        &mut self,
        bytes: &[u8],
        _priority: SerialTxPriority,
    ) -> Option<Result<usize, errors::TelemError>> {
        self.serial_tx_write(bytes)
    }
    fn serial_tx_enqueue_downlink(
        &mut self,
        _system_id: u8,
        _msg: DownlinkMessage,
        _priority: SerialTxPriority,
    ) -> Option<Result<usize, errors::TelemError>> {
        None
    }
    fn serial_rx_frame_read(&mut self) -> Option<Result<SerialRxFrame, errors::TelemError>> {
        None
    }
    fn serial_rx_pending(&self) -> bool {
        false
    }
    #[cfg(feature = "timing-diagnostics")]
    fn serial_rx_last_count(&self) -> usize {
        0
    }
    #[cfg(feature = "timing-diagnostics")]
    fn board_diagnostic_text(&mut self) -> Option<[u8; 50]> {
        None
    }

    fn clock_millis(&self) -> u32;
    fn clock_micros(&self) -> u64;
    fn set_test_pin_1(&mut self, _high: bool) {}
    fn set_test_pin_2(&mut self, _high: bool) {}
    fn set_test_pin_3(&mut self, _high: bool) {}
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
    fn serial_flush_budgeted(&mut self, _max_units: usize) {
        self.serial_flush();
    }
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
