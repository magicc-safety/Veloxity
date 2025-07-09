// /**
// ******************************************************************************
// * File     : estimator.rs
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
use crate::errors::*;
use crate::params::Params;
use crate::sensors::*;
use micro_algebra::stack::{quaternion, vector};

pub struct State {
    pub angular_velocity: vector::Vector<f64, 3>,
    pub attitude: quaternion::Quaternion<f64>,
}

pub struct Estimator {
    pub state: Result<State, EstimatorError>,
    pub bias: Result<vector::Vector<f64, 3>, EstimatorError>,
    pub accel_lpf: Result<vector::Vector<f64, 3>, EstimatorError>,
    pub gyro_lpf: Result<vector::Vector<f64, 3>, EstimatorError>,

    g: vector::Vector<f64, 3>,
    w1: vector::Vector<f64, 3>,
    w2: vector::Vector<f64, 3>,
    pub q_extatt: quaternion::Quaternion<f64>,
}

impl Estimator {
    pub fn new() -> Self {
        Self {
            state: Ok(State {
                angular_velocity: vector::Vector::zeros(),
                attitude: quaternion::Quaternion::zeros(),
            }),
            bias: Ok(vector::Vector::zeros()),
            accel_lpf: Ok(vector::Vector::zeros()),
            gyro_lpf: Ok(vector::Vector::zeros()),

            g: vector::Vector::from_array([0.0f64, 0.0f64, -1.0f64]),
            w1: vector::Vector::zeros(),
            w2: vector::Vector::zeros(),
            q_extatt: quaternion::Quaternion::zeros(),
            // TODO add in timing stuff here on initialization
        }
    }

    pub fn reset_state(&mut self) {
        if let Ok(state) = self.state.as_mut() {
            state.attitude = quaternion::Quaternion::from_array([0.0f64, 0.0f64, 0.0f64, 1.0f64]);
            state.angular_velocity = vector::Vector::zeros();
        }

        if let Ok(bias) = self.bias.as_mut() {
            *bias = vector::Vector::zeros();
        }

        if let Ok(accel_lpf) = self.accel_lpf.as_mut() {
            *accel_lpf = vector::Vector::from_array([0.0f64, 0.0f64, -9.80665]);
        }

        if let Ok(gyro_lpf) = self.gyro_lpf.as_mut() {
            *gyro_lpf = vector::Vector::zeros();
        }

        self.w1 = vector::Vector::zeros();
        self.w2 = vector::Vector::zeros();

        // also update timestamps
    }

    pub fn reset_adaptive_bias(&mut self) {
        if let Ok(bias) = self.bias.as_mut() {
            *bias = vector::Vector::zeros();
        }
    }

    pub fn run_LPF(&mut self) {}

    pub fn set_external_attitude_update(&mut self) {}

    pub fn run(&self) {}

    pub fn can_use_accel() {} // TODO rather than have a boolean to mark true or false, just use a
                              // Result<> type... then you can get rid of this function

    pub fn can_use_extatt() {} // DITTO

    pub fn accel_correction() -> vector::Vector<f64, 3> {
        vector::Vector::zeros()
    }

    pub fn extatt_correction() -> vector::Vector<f64, 3> {
        vector::Vector::zeros()
    }

    pub fn smoothed_gyro_measurement() -> vector::Vector<f64, 3> {
        vector::Vector::zeros()
    }

    pub fn integrate_angular_rate() {}

    pub fn quaternion_to_dcm() {} // TODO should this be in the math library?
}
