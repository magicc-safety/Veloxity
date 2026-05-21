use crate::board::BoardIo;
use crate::errors;
use crate::packets;
use crate::sensors::SensorBus;

#[derive(Default)]
pub struct DummyBoard {
    pub current_time_us: u64,
}

impl BoardIo for DummyBoard {
    fn update_sensor_bus(&mut self, sensors: &mut SensorBus) {
        sensors.clear();
        sensors.imu = Some(Ok(packets::ImuPacket::default()));
        sensors.mag = Some(Ok(packets::MagPacket::default()));
        sensors.baro = Some(Ok(packets::BaroPacket::default()));
        sensors.pitot = Some(Ok(packets::PitotPacket::default()));
        sensors.range = Some(Ok(packets::RangePacket::default()));
        sensors.gnss = Some(Ok(packets::GNSSPacket::default()));
        sensors.battery = Some(Ok(packets::BatteryPacket::default()));
        sensors.rc = Some(Ok(packets::RcPacket::default()));
        sensors.attitude = Some(Ok(packets::AttitudePacket::default()));
    }

    fn serial_rx_read(&mut self, _buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        None // pretend we never receive any data
    }
    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        Some(Ok(bytes.len())) // pretend we've written the bytes successfully
    }

    fn clock_millis(&self) -> u32 {
        (self.current_time_us / 1000) as u32
    }

    /// Returns the current dummy time in microseconds.
    fn clock_micros(&self) -> u64 {
        self.current_time_us
    }

    fn set_test_pin_1(&mut self, _high: bool) {
        // Dummy implementation does nothing
    }
    fn set_test_pin_2(&mut self, _high: bool) {
        // Dummy implementation does nothing
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::BoardIo;

    #[test]
    fn dummy_board_populates_named_sensor_bus() {
        let mut board = DummyBoard::default();
        let mut sensors = SensorBus::default();

        BoardIo::update_sensor_bus(&mut board, &mut sensors);

        assert!(sensors.imu.as_ref().is_some_and(Result::is_ok));
        assert!(sensors.mag.as_ref().is_some_and(Result::is_ok));
        assert!(sensors.baro.as_ref().is_some_and(Result::is_ok));
        assert!(sensors.pitot.as_ref().is_some_and(Result::is_ok));
        assert!(sensors.range.as_ref().is_some_and(Result::is_ok));
        assert!(sensors.gnss.as_ref().is_some_and(Result::is_ok));
        assert!(sensors.battery.as_ref().is_some_and(Result::is_ok));
        assert!(sensors.rc.as_ref().is_some_and(Result::is_ok));
        assert!(sensors.attitude.as_ref().is_some_and(Result::is_ok));
    }
}
