// /**
// ******************************************************************************
// * File     : sensorprocessors.rs
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
// **

use core::default;

use crate::errors;
use crate::log_info;
use crate::packets::*;
use crate::params2::{ParamId, ParamValue, Params};
use bitflags::bitflags;
use num_traits::Float;

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct CalibrationFlags: u16 {
        const GYRO = 1 << 0;
        const ACCEL = 1 << 1;
        const BARO = 1 << 2;
        const PITOT = 1 << 3;

        // Create a convenient combination for a full IMU calibration
        const IMU = Self::GYRO.bits() | Self::ACCEL.bits();
    }
}

pub trait SensorPacketProcessor<P> {
    fn process(
        &mut self,
        packet: &mut Option<Result<P, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<P>;
}

fn take_ok_packet<P>(packet: &mut Option<Result<P, errors::SensorError>>) -> Option<P> {
    match packet.take() {
        Some(Ok(packet)) => Some(packet),
        _ => None,
    }
}

macro_rules! impl_passthrough_sensor_packet_processor {
    ($processor:ty, $packet:ty) => {
        impl SensorPacketProcessor<$packet> for $processor {
            fn process(
                &mut self,
                packet: &mut Option<Result<$packet, errors::SensorError>>,
                _flags: &mut CalibrationFlags,
                _params: &mut Params,
            ) -> Option<$packet> {
                take_ok_packet(packet)
            }
        }
    };
}

// ------------------------------
// Battery Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughBatteryProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughBatteryProcessor, BatteryPacket);

// ------------------------------
// IMU Packet
// ------------------------------

const GYRO_MAX_CALIBRATION_DELTA: f64 = 0.1; // (rad/s) Threshold for movement... we won't calibrate if it's above this...

#[derive(Default, Copy, Clone)]
pub struct PassthroughImuProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughImuProcessor, ImuPacket);

#[derive(Default, Copy, Clone)]
pub struct ImuCalibrationState {
    gyro_sum: [f64; 3],
    gyro_calibration_count: u16,
    accel_sum: [f64; 3],
    accel_temp_sum: f64,
    accel_calibration_count: u16,
    max_accel: [f64; 3],
    min_accel: [f64; 3],
    max_gyro: [f64; 3],
    min_gyro: [f64; 3],
}

#[derive(Copy, Clone)]
pub struct ImuProcessor {
    calibration_state: ImuCalibrationState,
}

impl Default for ImuProcessor {
    fn default() -> Self {
        ImuProcessor::new()
    }
}

impl ImuProcessor {
    pub fn new() -> Self {
        Self {
            calibration_state: ImuCalibrationState {
                max_accel: [-1000.0, -1000.0, -1000.0],
                min_accel: [1000.0, 1000.0, 1000.0],
                ..Default::default()
            },
        }
    }

    fn process_packet(
        &mut self,
        packet: &mut Option<Result<ImuPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<ImuPacket> {
        if let Some(Ok(mut packet)) = packet.take() {
            let is_calibrating = flags.intersects(CalibrationFlags::IMU);

            if is_calibrating {
                // defmt::info!("Gyro Cal Count at beginning: {}", self.calibration_state.gyro_calibration_count);
                if flags.contains(CalibrationFlags::GYRO) {
                    // defmt::info!("Calibrating Gyro...");
                    self.calibration_state.gyro_sum[0] += packet.gyro[0] as f64;
                    self.calibration_state.gyro_sum[1] += packet.gyro[1] as f64;
                    self.calibration_state.gyro_sum[2] += packet.gyro[2] as f64;
                    self.calibration_state.gyro_calibration_count += 1;

                    // defmt::info!("Gyro Cal Count: {}", self.calibration_state.gyro_calibration_count);
                    if self.calibration_state.gyro_calibration_count > 1000 {
                        let count = self.calibration_state.gyro_calibration_count as f64;
                        let bias_x = self.calibration_state.gyro_sum[0] / count;
                        let bias_y = self.calibration_state.gyro_sum[1] / count;
                        let bias_z = self.calibration_state.gyro_sum[2] / count;

                        params.set_by_id(
                            ParamId::PARAM_GYRO_X_BIAS,
                            ParamValue::Float(bias_x as f32),
                        );
                        params.set_by_id(
                            ParamId::PARAM_GYRO_Y_BIAS,
                            ParamValue::Float(bias_y as f32),
                        );
                        params.set_by_id(
                            ParamId::PARAM_GYRO_Z_BIAS,
                            ParamValue::Float(bias_z as f32),
                        );

                        self.calibration_state.gyro_sum = [0.0; 3];
                        self.calibration_state.gyro_calibration_count = 0;
                        flags.remove(CalibrationFlags::GYRO);
                        log_info!("Gyro Calibration complete!");
                        //defmt::info!("Gyro calibration complete.")
                    }
                }

                if flags.contains(CalibrationFlags::ACCEL) {
                    // defmt::info!("Calibrating Accel...");
                    const GRAVITY: f64 = 9.80665;
                    self.calibration_state.accel_sum[0] += packet.accel[0] as f64;
                    self.calibration_state.accel_sum[1] += packet.accel[1] as f64;
                    self.calibration_state.accel_sum[2] += packet.accel[2] as f64 - GRAVITY;
                    self.calibration_state.accel_temp_sum += packet.temperature as f64;
                    self.calibration_state.accel_calibration_count += 1;

                    self.calibration_state.max_accel[0] =
                        self.calibration_state.max_accel[0].max(packet.accel[0] as f64);
                    self.calibration_state.min_accel[0] =
                        self.calibration_state.min_accel[0].min(packet.accel[0] as f64);
                    self.calibration_state.max_accel[1] =
                        self.calibration_state.max_accel[1].max(packet.accel[1] as f64);
                    self.calibration_state.min_accel[1] =
                        self.calibration_state.min_accel[1].min(packet.accel[1] as f64);
                    self.calibration_state.max_accel[2] =
                        self.calibration_state.max_accel[2].max(packet.accel[2] as f64);
                    self.calibration_state.min_accel[2] =
                        self.calibration_state.min_accel[2].min(packet.accel[2] as f64);

                    // defmt::info!("Accel Cal Count: {}", self.calibration_state.accel_calibration_count);
                    if self.calibration_state.accel_calibration_count > 1000 {
                        let max_delta = ((self.calibration_state.max_accel[0]
                            - self.calibration_state.min_accel[0])
                            .powi(2)
                            + (self.calibration_state.max_accel[1]
                                - self.calibration_state.min_accel[1])
                                .powi(2)
                            + (self.calibration_state.max_accel[2]
                                - self.calibration_state.min_accel[2])
                                .powi(2))
                        .sqrt();

                        if max_delta < 1.0 {
                            let count = self.calibration_state.accel_calibration_count as f64;
                            let temp_comp_x = if let ParamValue::Float(v) =
                                params.get_by_id(ParamId::PARAM_ACC_X_TEMP_COMP)
                            {
                                v as f64
                            } else {
                                0.0
                            };
                            let temp_comp_y = if let ParamValue::Float(v) =
                                params.get_by_id(ParamId::PARAM_ACC_Y_TEMP_COMP)
                            {
                                v as f64
                            } else {
                                0.0
                            };
                            let temp_comp_z = if let ParamValue::Float(v) =
                                params.get_by_id(ParamId::PARAM_ACC_Z_TEMP_COMP)
                            {
                                v as f64
                            } else {
                                0.0
                            };

                            let bias_x = (self.calibration_state.accel_sum[0]
                                - temp_comp_x * self.calibration_state.accel_temp_sum)
                                / count;
                            let bias_y = (self.calibration_state.accel_sum[1]
                                - temp_comp_y * self.calibration_state.accel_temp_sum)
                                / count;
                            let bias_z = (self.calibration_state.accel_sum[2]
                                - temp_comp_z * self.calibration_state.accel_temp_sum)
                                / count;

                            params.set_by_id(
                                ParamId::PARAM_ACC_X_BIAS,
                                ParamValue::Float(bias_x as f32),
                            );
                            params.set_by_id(
                                ParamId::PARAM_ACC_Y_BIAS,
                                ParamValue::Float(bias_y as f32),
                            );
                            params.set_by_id(
                                ParamId::PARAM_ACC_Z_BIAS,
                                ParamValue::Float(bias_z as f32),
                            );
                        } else {
                            //defmt::warn!("Too much movement for IMU calibration.");
                            // defmt::info!("Max Accel Delta: {}", max_delta);
                        }

                        self.calibration_state.accel_sum = [0.0; 3];
                        self.calibration_state.accel_calibration_count = 0;
                        self.calibration_state.accel_temp_sum = 0.0;
                        self.calibration_state.max_accel = [-1000.0, -1000.0, -1000.0];
                        self.calibration_state.min_accel = [1000.0, 1000.0, 1000.0];
                        flags.remove(CalibrationFlags::ACCEL);
                        log_info!("Accelerometer Calibration Complete!");
                        //defmt::info!("IMU calibration complete.")
                    }
                }
                // defmt::info!("Gyro Cal Count at end of cal: {}", self.calibration_state.gyro_calibration_count);
                // defmt::info!("IMU data used for calibration.");
                // return None; // Removed this so we don't get IMU errors while calibrating
            }
            // defmt::info!("Did I make it to here?");
            // --- Correction Logic (Not Calibrating) ---
            if let ParamValue::Float(bias) = params.get_by_id(ParamId::PARAM_GYRO_X_BIAS) {
                packet.gyro[0] -= bias as f64;
            }
            if let ParamValue::Float(bias) = params.get_by_id(ParamId::PARAM_GYRO_Y_BIAS) {
                packet.gyro[1] -= bias as f64;
            }
            if let ParamValue::Float(bias) = params.get_by_id(ParamId::PARAM_GYRO_Z_BIAS) {
                packet.gyro[2] -= bias as f64;
            }

            let temp = packet.temperature as f64;
            if let (ParamValue::Float(bias), ParamValue::Float(comp)) = (
                params.get_by_id(ParamId::PARAM_ACC_X_BIAS),
                params.get_by_id(ParamId::PARAM_ACC_X_TEMP_COMP),
            ) {
                packet.accel[0] -= (comp as f64 * temp + bias as f64);
            }
            if let (ParamValue::Float(bias), ParamValue::Float(comp)) = (
                params.get_by_id(ParamId::PARAM_ACC_Y_BIAS),
                params.get_by_id(ParamId::PARAM_ACC_Y_TEMP_COMP),
            ) {
                packet.accel[1] -= (comp as f64 * temp + bias as f64);
            }
            if let (ParamValue::Float(bias), ParamValue::Float(comp)) = (
                params.get_by_id(ParamId::PARAM_ACC_Z_BIAS),
                params.get_by_id(ParamId::PARAM_ACC_Z_TEMP_COMP),
            ) {
                packet.accel[2] -= (comp as f64 * temp + bias as f64);
            }

            // defmt::info!("IMU data processed");
            Some(packet)
        } else {
            // defmt::info!("No IMU data this cycle.");
            None
        }
    }
}

impl SensorPacketProcessor<ImuPacket> for ImuProcessor {
    fn process(
        &mut self,
        packet: &mut Option<Result<ImuPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<ImuPacket> {
        self.process_packet(packet, flags, params)
    }
}

// ------------------------------
// Baro Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughBaroProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughBaroProcessor, BaroPacket);

const SENSOR_CAL_DELAY_CYCLES: u16 = 128;
const SENSOR_CAL_CYCLES: u16 = 127;
const BARO_MAX_CALIBRATION_VARIANCE: f64 = 25.0;

#[derive(Default, Copy, Clone)]
pub struct BaroCalibrationState {
    mean: f64,
    m2: f64, // Sum of squares of differences from the current mean
    count: u16,
}

#[derive(Default, Copy, Clone)]
pub struct BaroProcessor {
    calibration_state: BaroCalibrationState,
}

impl BaroProcessor {
    pub fn new() -> Self {
        Self::default()
    }

    fn process_packet(
        &mut self,
        packet: &mut Option<Result<BaroPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<BaroPacket> {
        if let Some(Ok(mut packet)) = packet.take() {
            // If the BARO flag is set, we are calibrating.
            if flags.contains(CalibrationFlags::BARO) {
                self.calibration_state.count += 1;
                let total_cycles = SENSOR_CAL_DELAY_CYCLES + SENSOR_CAL_CYCLES;

                if self.calibration_state.count > total_cycles {
                    // Finalize calibration
                    let variance = self.calibration_state.m2 / (SENSOR_CAL_CYCLES - 1) as f64;
                    if variance < BARO_MAX_CALIBRATION_VARIANCE {
                        params.set_by_id(
                            ParamId::PARAM_BARO_BIAS,
                            ParamValue::Float(self.calibration_state.mean as f32),
                        );
                        let ground_alt = pressure_to_altitude(self.calibration_state.mean);
                        params.set_by_id(
                            ParamId::PARAM_GROUND_LEVEL,
                            ParamValue::Float(ground_alt as f32),
                        );
                        //defmt::info!("Barometer calibration successful!");
                    } else {
                        //defmt::warn!("Too much movement for barometer calibration.");
                    }

                    // Reset state and remove the flag to end the calibration process.
                    self.calibration_state = BaroCalibrationState::default();
                    flags.remove(CalibrationFlags::BARO);
                } else if self.calibration_state.count > SENSOR_CAL_DELAY_CYCLES {
                    // Accumulate data
                    let n = (self.calibration_state.count - SENSOR_CAL_DELAY_CYCLES) as f64;
                    let delta = packet.pressure as f64 - self.calibration_state.mean;
                    self.calibration_state.mean += delta / n;
                    let delta2 = packet.pressure as f64 - self.calibration_state.mean;
                    self.calibration_state.m2 += delta * delta2;
                }

                // We are calibrating, so don't return a packet with a value.
                None
            } else {
                // --- Correction Logic (Not Calibrating) ---
                if let ParamValue::Float(bias) = params.get_by_id(ParamId::PARAM_BARO_BIAS) {
                    packet.pressure -= bias;
                }
                packet.altitude = pressure_to_altitude(packet.pressure as f64) as f32;
                Some(packet)
            }
        } else {
            None
        }
    }
}

fn pressure_to_altitude(pressure: f64) -> f64 {
    44330.0 * (1.0 - (pressure / 101325.0).powf(0.190295))
}

impl SensorPacketProcessor<BaroPacket> for BaroProcessor {
    fn process(
        &mut self,
        packet: &mut Option<Result<BaroPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<BaroPacket> {
        self.process_packet(packet, flags, params)
    }
}

// ------------------------------
// Pitot Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughPitotProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughPitotProcessor, PitotPacket);

const PITOT_MAX_CALIBRATION_VARIANCE: f64 = 100.0;

#[derive(Default, Copy, Clone)]
pub struct PitotCalibrationState {
    mean: f64,
    m2: f64,
    count: u16,
    calibrated: bool,
}

#[derive(Default, Copy, Clone)]
pub struct PitotProcessor {
    calibration_state: PitotCalibrationState,
}

impl PitotProcessor {
    pub fn new() -> Self {
        Self::default()
    }

    fn process_packet(
        &mut self,
        packet: &mut Option<Result<PitotPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<PitotPacket> {
        if let Some(Ok(mut packet)) = packet.take() {
            if flags.contains(CalibrationFlags::PITOT) && !self.calibration_state.calibrated {
                self.calibration_state.count += 1;
                let total_cycles = SENSOR_CAL_DELAY_CYCLES + SENSOR_CAL_CYCLES;

                if self.calibration_state.count > total_cycles {
                    let variance = self.calibration_state.m2 / (SENSOR_CAL_CYCLES - 1) as f64;
                    if variance < PITOT_MAX_CALIBRATION_VARIANCE {
                        params.set_by_id(
                            ParamId::PARAM_DIFF_PRESS_BIAS,
                            ParamValue::Float(self.calibration_state.mean as f32),
                        );
                        self.calibration_state.calibrated = true;
                        //defmt::info!("Airspeed calibration successful!");
                    } else {
                        //defmt::info!("Too much movement for diff pressure calibration.");
                    }

                    self.calibration_state = PitotCalibrationState::default();
                    flags.remove(CalibrationFlags::PITOT);
                } else if self.calibration_state.count > SENSOR_CAL_DELAY_CYCLES {
                    let n = (self.calibration_state.count - SENSOR_CAL_DELAY_CYCLES) as f64;
                    let delta = packet.differential_pressure as f64 - self.calibration_state.mean;
                    self.calibration_state.mean += delta / n;
                    let delta2 = packet.differential_pressure as f64 - self.calibration_state.mean;
                    self.calibration_state.m2 += delta * delta2;
                }
                None
            } else {
                if let ParamValue::Float(bias) = params.get_by_id(ParamId::PARAM_DIFF_PRESS_BIAS) {
                    packet.differential_pressure -= bias;
                }

                const RHO: f64 = 1.225;
                let dp = packet.differential_pressure as f64;
                packet.indicated_airspeed =
                    (2.0 * dp.abs() / RHO).sqrt() as f32 * dp.signum() as f32;

                Some(packet)
            }
        } else {
            None
        }
    }
}

impl SensorPacketProcessor<PitotPacket> for PitotProcessor {
    fn process(
        &mut self,
        packet: &mut Option<Result<PitotPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<PitotPacket> {
        self.process_packet(packet, flags, params)
    }
}

// ------------------------------
// Mag Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughMagProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughMagProcessor, MagPacket);

#[derive(Default, Copy, Clone)]
pub struct MagProcessor;

impl MagProcessor {
    fn process_packet(
        &mut self,
        packet: &mut Option<Result<MagPacket, errors::SensorError>>,
        _flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<MagPacket> {
        if let Some(Ok(mut packet)) = packet.take() {
            // Apply hard-iron biases from parameters
            let mag_hard_x = packet.flux[0]
                - if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MAG_X_BIAS) {
                    v
                } else {
                    0.0
                };
            let mag_hard_y = packet.flux[1]
                - if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MAG_Y_BIAS) {
                    v
                } else {
                    0.0
                };
            let mag_hard_z = packet.flux[2]
                - if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MAG_Z_BIAS) {
                    v
                } else {
                    0.0
                };

            // Get soft-iron correction matrix parameters
            let a11 = if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MAG_A11_COMP) {
                v
            } else {
                1.0
            };
            let a12 = if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MAG_A12_COMP) {
                v
            } else {
                0.0
            };
            let a13 = if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MAG_A13_COMP) {
                v
            } else {
                0.0
            };
            let a21 = if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MAG_A21_COMP) {
                v
            } else {
                0.0
            };
            let a22 = if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MAG_A22_COMP) {
                v
            } else {
                1.0
            };
            let a23 = if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MAG_A23_COMP) {
                v
            } else {
                0.0
            };
            let a31 = if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MAG_A31_COMP) {
                v
            } else {
                0.0
            };
            let a32 = if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MAG_A32_COMP) {
                v
            } else {
                0.0
            };
            let a33 = if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MAG_A33_COMP) {
                v
            } else {
                1.0
            };

            // Apply soft-iron corrections (matrix multiplication)
            packet.flux[0] = a11 * mag_hard_x + a12 * mag_hard_y + a13 * mag_hard_z;
            packet.flux[1] = a21 * mag_hard_x + a22 * mag_hard_y + a23 * mag_hard_z;
            packet.flux[2] = a31 * mag_hard_x + a32 * mag_hard_y + a33 * mag_hard_z;

            Some(packet)
        } else {
            None
        }
    }
}

impl SensorPacketProcessor<MagPacket> for MagProcessor {
    fn process(
        &mut self,
        packet: &mut Option<Result<MagPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<MagPacket> {
        self.process_packet(packet, flags, params)
    }
}

// ------------------------------
// Rc Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughRcProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughRcProcessor, RcPacket);

// ------------------------------
// Range Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughRangeProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughRangeProcessor, RangePacket);

// ------------------------------
// GNSS Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughGNSSProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughGNSSProcessor, GNSSPacket);

// ------------------------------
// PPS Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughPpsProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughPpsProcessor, PpsPacket);

// ------------------------------
// Attitude Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughAttitudeProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughAttitudeProcessor, AttitudePacket);
