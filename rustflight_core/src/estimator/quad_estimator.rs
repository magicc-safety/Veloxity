// /**
// ******************************************************************************
// * File     : quad_estimator.rs
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

use super::AttitudeStateTrait;
use super::NamedEstimator;
use crate::comm_messages::messages::ExternalAttitudeMsg;
use crate::packets;
use crate::params2::{ParamId, ParamValue, Params};
use crate::sensors::ProcessedSensors;

use micro_algebra::stack::{quaternion::Quaternion, vector::Vector};

// Removed hardcoded DT - now using actual timestamps
// const DT: f64 = 1.0/400.0f64;

const G: f64 = 9.80665; // Gravity in m/s^2

#[derive(Debug, Clone, Copy)]
pub struct AttitudeState {
    pub q_hat: Quaternion<f64>,
    pub q_dot: Quaternion<f64>,
    pub body_rate: Vector<f64, 3>,
    pub b_hat: Vector<f64, 3>,
    pub is_healthy: bool,
}

impl Default for AttitudeState {
    fn default() -> Self {
        Self {
            q_hat: Quaternion::from_array([1.0, 0.0, 0.0, 0.0]),
            q_dot: Quaternion::from_array([0.0, 0.0, 0.0, 0.0]),
            body_rate: Vector::from_array([0.0, 0.0, 0.0]),
            b_hat: Vector::from_array([0.0, 0.0, 0.0]),
            is_healthy: false,
        }
    }
}

impl AttitudeStateTrait for AttitudeState {
    fn q(&self) -> [f32; 4] {
        [
            self.q_hat.get_w() as f32,
            self.q_hat.get_x() as f32,
            self.q_hat.get_y() as f32,
            self.q_hat.get_z() as f32,
        ]
    }

    fn q_dot(&self) -> [f32; 4] {
        [
            self.q_dot.get_w() as f32,
            self.q_dot.get_x() as f32,
            self.q_dot.get_y() as f32,
            self.q_dot.get_z() as f32,
        ]
    }

    fn is_healthy(&self) -> bool {
        self.is_healthy
    }
}

impl From<AttitudeState> for Vector<f64, 3> {
    fn from(state: AttitudeState) -> Self {
        state.q_hat.to_euler_angles()
    }
}

impl<'a> From<&'a AttitudeState> for Vector<f64, 3> {
    fn from(state: &'a AttitudeState) -> Self {
        state.q_hat.to_euler_angles()
    }
}

pub struct QuadEstimator {
    k_p: f64,
    k_i: f64,
    q_hat: Quaternion<f64>,
    q_dot: Quaternion<f64>,
    body_rate: Vector<f64, 3>,
    b_hat: Vector<f64, 3>,
    last_imu_time: u64,   // Track last IMU timestamp (microseconds)
    is_initialized: bool, // Track if we've received first IMU packet

    // Low-pass filter state
    accel_lpf: Vector<f64, 3>, // Filtered accelerometer
    gyro_lpf: Vector<f64, 3>,  // Filtered gyroscope

    // LPF parameters (EMA alpha values) - matching C defaults
    alpha_acc: f64,     // PARAM_ACC_ALPHA = 0.5 in C
    alpha_gyro_xy: f64, // PARAM_GYRO_XY_ALPHA = 0.3 in C
    alpha_gyro_z: f64,  // PARAM_GYRO_Z_ALPHA = 0.3 in C

    // Accelerometer gating
    accel_margin: f64, // PARAM_FILTER_ACCEL_MARGIN = 0.1 in C

    // Adaptive gains during initialization
    init_time_us: u64,   // PARAM_INIT_TIME = 3000ms = 3,000,000 μs in C
    first_imu_time: u64, // Track when first IMU arrived
}

impl QuadEstimator {
    pub fn new(k_p: f64, k_i: f64) -> Self {
        Self {
            k_p,
            k_i,
            q_hat: Quaternion::from_array([1.0, 0.0, 0.0, 0.0]),
            q_dot: Quaternion::from_array([0.0, 0.0, 0.0, 0.0]),
            body_rate: Vector::from_array([0.0, 0.0, 0.0]),
            b_hat: Vector::from_array([0.0, 0.0, 0.0]),
            last_imu_time: 0,
            is_initialized: false,

            // Initialize LPF state - accel starts at gravity pointing down (NED frame)
            accel_lpf: Vector::from_array([0.0, 0.0, -G]),
            gyro_lpf: Vector::from_array([0.0, 0.0, 0.0]),

            // LPF parameters matching C defaults
            alpha_acc: 0.5,
            alpha_gyro_xy: 0.3,
            alpha_gyro_z: 0.3,

            // Accelerometer gating - ±10% around 1g
            accel_margin: 0.1,

            // Adaptive gains - 3 second initialization period
            init_time_us: 3_000_000,
            first_imu_time: 0,
        }
    }

    /// Update parameters from the parameter server.
    /// Call this every loop to read fresh parameter values.
    pub fn update_params(&mut self, params: &Params) {
        // Read base gains (not the 10× boosted values)
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_FILTER_KP_ACC) {
            self.k_p = v as f64;
        }
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_FILTER_KI) {
            self.k_i = v as f64;
        }

        // Read LPF alpha values
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_ACC_ALPHA) {
            self.alpha_acc = v as f64;
        }
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_GYRO_XY_ALPHA) {
            self.alpha_gyro_xy = v as f64;
        }
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_GYRO_Z_ALPHA) {
            self.alpha_gyro_z = v as f64;
        }

        // Read accelerometer gating margin
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_FILTER_ACCEL_MARGIN) {
            self.accel_margin = v as f64;
        }

        // Read initialization time (convert milliseconds to microseconds)
        if let ParamValue::Int(v) = params.get_by_id(ParamId::PARAM_INIT_TIME) {
            self.init_time_us = (v as u64) * 1000;
        }
    }
}

impl Default for QuadEstimator {
    fn default() -> Self {
        Self::new(1.5, 0.05)
    }
}

impl QuadEstimator {
    fn apply_external_attitude(&mut self, external_attitude: ExternalAttitudeMsg) {
        let mut q = Quaternion::from_array([
            external_attitude.qw as f64,
            external_attitude.qx as f64,
            external_attitude.qy as f64,
            external_attitude.qz as f64,
        ]);
        q.normalize_fill();
        self.q_hat = q;
    }

    fn estimate_packets(
        &mut self,
        imu: Option<packets::ImuPacket>,
        _mag: Option<packets::MagPacket>,
        params: &Params,
        dt: f64,
    ) -> AttitudeState {
        // Update parameters from parameter server (matches C behavior)
        self.update_params(params);

        // Sanity check dt (reject if > 0.1s or <= 0)
        if dt <= 0.0 || dt > 0.1 {
            return AttitudeState {
                q_hat: self.q_hat,
                q_dot: self.q_dot,
                body_rate: self.body_rate,
                b_hat: self.b_hat,
                is_healthy: false,
            };
        }

        if let Some(imu_packet) = imu {
            // Get current timestamp for initialization tracking
            let current_time = imu_packet.header.timestamp; // microseconds

            // On first call, just initialize timestamp and skip update
            if !self.is_initialized {
                self.first_imu_time = current_time;
                self.is_initialized = true;
                return AttitudeState {
                    q_hat: self.q_hat,
                    q_dot: self.q_dot,
                    body_rate: self.body_rate,
                    b_hat: self.b_hat,
                    is_healthy: true,
                };
            }

            // Apply low-pass filter to raw measurements (EMA filter)
            let raw_accel = Vector::from_array(imu_packet.accel);
            self.accel_lpf[0] =
                (1.0 - self.alpha_acc) * raw_accel[0] + self.alpha_acc * self.accel_lpf[0];
            self.accel_lpf[1] =
                (1.0 - self.alpha_acc) * raw_accel[1] + self.alpha_acc * self.accel_lpf[1];
            self.accel_lpf[2] =
                (1.0 - self.alpha_acc) * raw_accel[2] + self.alpha_acc * self.accel_lpf[2];

            let raw_gyro = Vector::from_array(imu_packet.gyro);
            self.gyro_lpf[0] =
                (1.0 - self.alpha_gyro_xy) * raw_gyro[0] + self.alpha_gyro_xy * self.gyro_lpf[0];
            self.gyro_lpf[1] =
                (1.0 - self.alpha_gyro_xy) * raw_gyro[1] + self.alpha_gyro_xy * self.gyro_lpf[1];
            self.gyro_lpf[2] =
                (1.0 - self.alpha_gyro_z) * raw_gyro[2] + self.alpha_gyro_z * self.gyro_lpf[2];

            // Check if accelerometer magnitude is near 1g (gating)
            let accel_sqrd_norm = self.accel_lpf[0] * self.accel_lpf[0]
                + self.accel_lpf[1] * self.accel_lpf[1]
                + self.accel_lpf[2] * self.accel_lpf[2];

            let margin = self.accel_margin;
            let lowerbound = (1.0 - margin) * (1.0 - margin) * G * G;
            let upperbound = (1.0 + margin) * (1.0 + margin) * G * G;
            let can_use_accel = accel_sqrd_norm > lowerbound && accel_sqrd_norm < upperbound;

            // Calculate adaptive gains (10× during initialization)
            let time_since_init = current_time - self.first_imu_time;
            let (kp_base, ki_base) = if time_since_init < self.init_time_us {
                // First 3 seconds: 10× gains for fast convergence
                (self.k_p * 10.0, self.k_i * 10.0)
            } else {
                // After 3 seconds: normal gains for steady-state
                (self.k_p, self.k_i)
            };

            // Compute accelerometer correction (if gating passes)
            let (kp, e) = if can_use_accel {
                // Use filtered accelerometer for correction
                let mut v_a = self.accel_lpf;

                // Check for zero vector before normalizing
                let accel_norm = v_a.norm_2();
                if accel_norm > 1e-9 {
                    v_a.normalize_fill();

                    // Predict gravity in body frame using our latest estimate of q_hat
                    let g_intertial_q = Quaternion::from_array([0.0, 0.0, 0.0, -1.0]);
                    let q_conj = self.q_hat.conjugate();
                    let tmp = q_conj * g_intertial_q;
                    let gravity_in_body_q = tmp * self.q_hat;
                    let v_hat = Vector::from_array([
                        gravity_in_body_q.get_x(),
                        gravity_in_body_q.get_y(),
                        gravity_in_body_q.get_z(),
                    ]);

                    // Vector error (predicted x measured)
                    let error = v_hat.cross3(&v_a);
                    (kp_base, error)
                } else {
                    // Zero acceleration - skip accel correction
                    (0.0, Vector::from_array([0.0, 0.0, 0.0]))
                }
            } else {
                // Accel gating failed (high-g maneuver) - skip accel correction, use gyro only
                (0.0, Vector::from_array([0.0, 0.0, 0.0]))
            };

            // Bias integration (continues even when accel is gated)
            let b_dot = -ki_base * e;
            self.b_hat = self.b_hat + b_dot * dt;

            // Corrected angular rate using filtered gyro
            let body_rate = self.gyro_lpf - self.b_hat;
            let omega_corr = body_rate + kp * e;

            // Quaternion derivative
            let omega_q =
                Quaternion::from_array([0.0, omega_corr[0], omega_corr[1], omega_corr[2]]);
            self.q_dot = 0.5 * (self.q_hat * omega_q);

            // Quaternion integration
            self.q_hat = self.q_hat + self.q_dot * dt;
            self.q_hat.normalize_fill();

            // Store the bias-corrected body rate for the controller
            self.body_rate = body_rate;
        }

        let q = self.q_hat;
        let is_healthy = !(q.get_w().is_nan()
            || q.get_w().is_infinite()
            || q.get_x().is_nan()
            || q.get_x().is_infinite()
            || q.get_y().is_nan()
            || q.get_y().is_infinite()
            || q.get_z().is_nan()
            || q.get_z().is_infinite());

        AttitudeState {
            q_hat: self.q_hat,
            q_dot: self.q_dot,
            body_rate: self.body_rate,
            b_hat: self.b_hat,
            is_healthy,
        }
    }
}

impl NamedEstimator for QuadEstimator {
    type State = AttitudeState;

    fn estimate_named(
        &mut self,
        sensors: &ProcessedSensors,
        params: &Params,
        dt: f64,
    ) -> Self::State {
        self.estimate_packets(sensors.imu, sensors.mag, params, dt)
    }

    fn estimate_named_with_external_attitude(
        &mut self,
        sensors: &ProcessedSensors,
        params: &Params,
        dt: f64,
        external_attitude: Option<ExternalAttitudeMsg>,
    ) -> Self::State {
        if let Some(external_attitude) = external_attitude {
            self.apply_external_attitude(external_attitude);
        }
        self.estimate_packets(sensors.imu, sensors.mag, params, dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        comm_messages::messages::ExternalAttitudeMsg,
        estimator::NamedEstimator,
        packets::{ImuPacket, RosflightPacketHeader},
    };

    #[test]
    fn named_estimator_consumes_external_attitude_on_next_run() {
        let params = Params::new();
        let imu = ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -G],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        };
        let mut sensors = ProcessedSensors::default();
        sensors.imu = Some(imu);

        let mut estimator = QuadEstimator::default();
        let state = estimator.estimate_named_with_external_attitude(
            &sensors,
            &params,
            1.0 / 400.0,
            Some(ExternalAttitudeMsg {
                qw: 0.0,
                qx: 1.0,
                qy: 0.0,
                qz: 0.0,
            }),
        );

        assert_eq!(state.q(), [0.0, 1.0, 0.0, 0.0]);
        assert!(state.is_healthy());
    }
}
