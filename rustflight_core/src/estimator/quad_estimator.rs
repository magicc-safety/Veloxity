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

use micro_algebra::stack::{
    quaternion::Quaternion,
    vector::Vector,
};

#[derive(Debug, Clone, Copy)]
pub struct AttitudeState {
    pub q_hat: Quaternion<f64>,
    pub b_hat: Vector<f64, 3>,
}

pub struct QuadEstimator {
     k_p: f64,
     k_i: f64,
     q_hat: Quaternion<f64>,
     b_hat: Vector<f64, 3>,
}

impl QuadEstimator {
    pub fn new(k_p: f64, k_i: f64) -> Self {
        Self {
            k_p,
            k_i,
            q_hat: Quaternion::from_array([1.0, 0.0, 0.0, 0.0]),
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
        AttitudeState {
            q_hat: self.q_hat,
            b_hat: self.b_hat,
        }
    }
}
