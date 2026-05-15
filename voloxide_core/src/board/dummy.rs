// /**
// ******************************************************************************
// * File     : dummy.rs
// * Date     : May 8, 2025
// ******************************************************************************
// *
// * Copyright (c) 2023, AeroVironment, Inc.
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// * 1.Redistributions of source code must retain the above copyright notice, this
// * list of conditions and the following disclaimer.
// *
// * 2.Redistributions in binary form must reproduce the above copyright notice,
// * this list of conditions and the following disclaimer in the documentation
// * and/or other materials provided with the distribution.
// *
// * 3.Neither the name of the copyright holder nor the names of its
// * contributors may be used to endorse or promote products derived from
// * this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// ******************************************************************************
// **/
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

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
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

    fn set_test_pin_1(&mut self, high: bool) {
        // Dummy implementation does nothing
    }
    fn set_test_pin_2(&mut self, high: bool) {
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
