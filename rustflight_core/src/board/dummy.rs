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
use crate::board::BoardTrait;
use crate::errors;
use crate::hlist_type;
use crate::packets;
use crate::sensorprocessors;

#[derive(Default)]
pub struct DummyBoard;

impl BoardTrait for DummyBoard {
    type RawSensorSet = hlist_type![
        Option<Result<packets::ImuPacket, errors::SensorError>>,
        Option<Result<packets::MagPacket, errors::SensorError>>,
        Option<Result<packets::BaroPacket, errors::SensorError>>,
        Option<Result<packets::PitotPacket, errors::SensorError>>,
        Option<Result<packets::RangePacket, errors::SensorError>>,
        Option<Result<packets::GNSSPacket, errors::SensorError>>,
        Option<Result<packets::BatteryPacket, errors::SensorError>>,
        Option<Result<packets::RcPacket, errors::SensorError>>,
        Option<Result<packets::AttitudePacket, errors::SensorError>>
    ];

    type ProcessedSensorSet = hlist_type![
        Option<packets::ImuPacket>,
        Option<packets::MagPacket>,
        Option<packets::BaroPacket>,
        Option<packets::PitotPacket>,
        Option<packets::RangePacket>,
        Option<packets::GNSSPacket>,
        Option<packets::BatteryPacket>,
        Option<packets::RcPacket>,
        Option<packets::AttitudePacket>
    ];

    type ProcessorHList = hlist_type![
        sensorprocessors::PassthroughImuProcessor,
        sensorprocessors::PassthroughMagProcessor,
        sensorprocessors::PassthroughBaroProcessor,
        sensorprocessors::PassthroughPitotProcessor,
        sensorprocessors::PassthroughRangeProcessor,
        sensorprocessors::PassthroughGNSSProcessor,
        sensorprocessors::PassthroughBatteryProcessor,
        sensorprocessors::PassthroughRcProcessor,
        sensorprocessors::PassthroughAttitudeProcessor
    ];

    fn update_sensors(&mut self, sensors: &mut Self::RawSensorSet) {
        sensors.0 = Some(Ok(packets::ImuPacket::default()));
        sensors.1.0 = Some(Ok(packets::MagPacket::default()));
        sensors.1.1.0 = Some(Ok(packets::BaroPacket::default()));
        sensors.1.1.1.0 = Some(Ok(packets::PitotPacket::default()));
        sensors.1.1.1.1.0 = Some(Ok(packets::RangePacket::default()));
        sensors.1.1.1.1.1.0 = Some(Ok(packets::GNSSPacket::default()));
        sensors.1.1.1.1.1.1.0 = Some(Ok(packets::BatteryPacket::default()));
        sensors.1.1.1.1.1.1.1.0 = Some(Ok(packets::RcPacket::default()));
        sensors.1.1.1.1.1.1.1.1.0 = Some(Ok(packets::AttitudePacket::default()));
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        None
    }
    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        None
    }
}
