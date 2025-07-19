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
use crate::packets;
use crate::params::Params;
use crate::sensors;

#[derive(Default)]
pub struct DummyBoard;

#[derive(Default, Copy, Clone)]
struct ImuProcessor;
impl<'a> packets::Func<&'a mut Option<packets::ImuPacket>> for ImuProcessor {
    type Output = Result<Option<packets::ImuPacket>>;
    fn call(
        &self,
        arg: &'a mut Option<packets::ImuPacket>,
        state: &packets::SystemState,
    ) -> Self::Output {
        match state {
            packets::SystemState::CalibratingImu => {
                if let Some(_packet) = arg.take() {
                    // do your processing on it and return it.
                }
                None // Don't pass data to estimator during IMU calibration
            }
            _ => {
                // In all other states, process normally <-- this is like passthrough
                arg.take().map(|p| p);
            }
        }
    }
}

impl BoardTrait for DummyBoard {
    type RawSensorSet = packets::HCons<
        Option<Result<packets::ImuPacket, errors::SensorError>>,
        packets::HCons<
            Option<Result<packets::MagPacket, errors::SensorError>>,
            packets::HCons<
                Option<Result<packets::BaroPacket, errors::SensorError>>,
                packets::HCons<
                    Option<Result<packets::PitotPacket, errors::SensorError>>,
                    packets::HCons<
                        Option<Result<packets::RangePacket, errors::SensorError>>,
                        packets::HCons<
                            Option<Result<packets::GNSSPacket, errors::SensorError>>,
                            packets::HCons<
                                Option<Result<packets::BatteryPacket, errors::SensorError>>,
                                packets::HCons<
                                    Option<Result<packets::RcPacket, errors::SensorError>>,
                                    packets::HCons<
                                        Option<
                                            Result<packets::AttitudePacket, errors::SensorError>,
                                        >,
                                        packets::HCons<
                                            Option<Result<usize, errors::SensorError>>,
                                            packets::HCons<
                                                Option<Result<usize, errors::SensorError>>,
                                                packets::HNil,
                                            >,
                                        >,
                                    >,
                                >,
                            >,
                        >,
                    >,
                >,
            >,
        >,
    >;

    // ultimately you're goin to need to replace the packet types with new packet types for the
    // processed sensors if their information changes... if not you can just put it back in the
    // packets
    type ProcessedSensorSet = packets::HCons<
        Option<Result<packets::ImuPacket, errors::SensorError>>,
        packets::HCons<
            Option<Result<packets::MagPacket, errors::SensorError>>,
            packets::HCons<
                Option<Result<packets::BaroPacket, errors::SensorError>>,
                packets::HCons<
                    Option<Result<packets::PitotPacket, errors::SensorError>>,
                    packets::HCons<
                        Option<Result<packets::RangePacket, errors::SensorError>>,
                        packets::HCons<
                            Option<Result<packets::GNSSPacket, errors::SensorError>>,
                            packets::HCons<
                                Option<Result<packets::BatteryPacket, errors::SensorError>>,
                                packets::HCons<
                                    Option<Result<packets::RcPacket, errors::SensorError>>,
                                    packets::HCons<
                                        Option<
                                            Result<packets::AttitudePacket, errors::SensorError>,
                                        >,
                                        packets::HCons<
                                            Option<Result<usize, errors::SensorError>>,
                                            packets::HCons<
                                                Option<Result<usize, errors::SensorError>>,
                                                packets::HNil,
                                            >,
                                        >,
                                    >,
                                >,
                            >,
                        >,
                    >,
                >,
            >,
        >,
    >;
}

/*
impl BoardTrait for DummyBoard {
    fn imu_read(&mut self) -> Option<Result<packets::ImuPacket, errors::SensorError>> {
        None
    }

    fn mag_read(&mut self) -> Option<Result<packets::MagPacket, errors::SensorError>> {
        None
    }

    fn baro_read(&mut self) -> Option<Result<packets::BaroPacket, errors::SensorError>> {
        None
    }

    fn diff_pressure_read(&mut self) -> Option<Result<packets::PitotPacket, errors::SensorError>> {
        None
    }

    fn sonar_read(&mut self) -> Option<Result<packets::RangePacket, errors::SensorError>> {
        None
    }

    fn gnss_read(&mut self) -> Option<Result<packets::GNSSPacket, errors::SensorError>> {
        None
    }

    fn battery_read(&mut self) -> Option<Result<packets::BatteryPacket, errors::SensorError>> {
        None
    }

    fn rc_read(&mut self) -> Option<Result<packets::RcPacket, errors::SensorError>> {
        None
    }

    fn attitude_read(&mut self) -> Option<Result<packets::AttitudePacket, errors::SensorError>> {
        None
    }

    fn serial_rx_read(&mut self, buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
        None
    }

    fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
        None
    }
}
*/
