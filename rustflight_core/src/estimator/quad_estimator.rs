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

use crate::hlist::*;
use crate::hlist_type;
use crate::packets;
use super::Estimator;
use super::AttitudeStateTrait;

use micro_algebra::stack::{
    quaternion::Quaternion,
    vector::Vector,
};

const DT: f64 = 1.0/400.0f64;

#[derive(Debug, Clone, Copy)]
pub struct AttitudeState {
    pub q_hat: Quaternion<f64>,
    pub q_dot: Quaternion<f64>,
    pub body_rate: Vector<f64, 3>,
    pub b_hat: Vector<f64, 3>,
    pub is_healthy: bool,
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
}

impl QuadEstimator {
    pub fn new(k_p: f64, k_i: f64) -> Self {
        Self {
            k_p,
            k_i,
            q_hat: Quaternion::from_array([1.0, 0.0, 0.0, 0.0]),
            q_dot: Quaternion::from_array([1.0, 0.0, 0.0, 0.0]),
            body_rate: Vector::from_array([0.0, 0.0, 0.0]),
            b_hat: Vector::from_array([0.0, 0.0, 0.0]),
        }
    }
}

impl Default for QuadEstimator {
    fn default() -> Self {
        Self::new(1.5, 0.05)
    }
}

impl Estimator for QuadEstimator {
    type Inputs = hlist_type![
        Option<packets::ImuPacket>,
        Option<packets::MagPacket>
    ];

    type State = AttitudeState;

    fn estimate(&mut self, inputs: &Self::Inputs) -> Self::State {

        if let Some(imu_packet) = inputs.0 {
            
            // normalize accelerometer measurement 
            let mut v_a = Vector::from_array(imu_packet.accel);

            // FIX: Check for zero vector before normalizing
            if v_a.norm_2() > 1e-9 { // Or some other small epsilon
                v_a.normalize_fill();
            } else {
                // Handle the zero-vector case. Maybe skip this update?
                // Or return the current state without updating.
                // For now, let's just skip the update logic.
                return AttitudeState {
                    q_hat: self.q_hat,
                    q_dot: self.q_dot,
                    body_rate: self.body_rate,
                    b_hat: self.b_hat,
                    is_healthy: true,
                };
            }
            
            // predict gravity in body frame using our latest estimate of q_hat
            let g_intertial_q = Quaternion::from_array([0.0f64, 0.0f64, 0.0f64, 1.0f64]);
            let q_conj = self.q_hat.conjugate();
            let tmp = q_conj * g_intertial_q;
            let gravity_in_body_q = tmp * self.q_hat;
            let v_hat = Vector::from_array([gravity_in_body_q.get_x(), gravity_in_body_q.get_y(), gravity_in_body_q.get_z()]);

            // vector error (predicted x measured)
            let e = v_hat.cross3(&v_a);

            // integral (bias) update
            let b_dot = -self.k_i * e;
            self.b_hat = self.b_hat + b_dot * DT;

            // corrected angular rate (body frame)
            // if signs are opposite in your convention, change +self.k_p*e to -self.k_p*e
            // omega_corr is a correction command... it's what we're eventually using to TELL
            // THE QUATERNION INTEGRATOR TO DO
            let body_rate = Vector::from_array(imu_packet.gyro) - self.b_hat;
            let omega_corr = body_rate + self.k_p * e;

            // quaternion derivative
            let omega_q = Quaternion::from_array([0.0, omega_corr[0], omega_corr[1], omega_corr[2]]);
            self.q_dot = 0.5 * (self.q_hat * omega_q);

            self.q_hat = self.q_hat + self.q_dot * DT;
            self.q_hat.normalize_fill();
        }

        let q = self.q_hat;
        let is_healthy = 
            !(q.get_w().is_nan() || q.get_w().is_infinite() ||
              q.get_x().is_nan() || q.get_x().is_infinite() ||
              q.get_y().is_nan() || q.get_y().is_infinite() ||
              q.get_z().is_nan() || q.get_z().is_infinite());

        AttitudeState {
            q_hat: self.q_hat,
            q_dot: self.q_dot,
            body_rate: self.body_rate,
            b_hat: self.b_hat,
            is_healthy: is_healthy
        }
    }
}
