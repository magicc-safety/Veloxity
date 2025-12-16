
// /**
// ******************************************************************************
// * File     : quad_mixer.rs
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

use crate::controller::quad_controller::MixerInput;
use crate::mixer::Mixer;
use crate::params2::{ParamId, ParamValue, Params};
use micro_algebra::stack::matrix::Matrix;
use micro_algebra::stack::vector::Vector;
use micro_algebra::linalg::pinv;
use crate::state_machine::StateManager;
use num_traits::Float;
use libm::{sin, cos, fabs};

#[derive(Debug, Clone, Copy)]
pub struct MixerParams {
    // Safety / limits
    pub idle_throttle: f64,
    pub spin_when_armed: bool,
    pub num_motors: usize,
}

pub struct QuadMixer {
    // 4 Inputs (Fz, Tx, Ty, Tz) -> 4 Outputs (Motors)
    // Matrix Size: 4x4, Flattened: 16
    allocation_matrix: Matrix<f64, 4, 16>,
    params: MixerParams,
}

impl QuadMixer {

    pub fn new(params: &Params) -> Self {
        
        let mixer_params = MixerParams {
            num_motors: 4,
            idle_throttle: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE) { v as f64 } else { 0.1 },
            spin_when_armed: if let ParamValue::Bool(v) = params.get_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED) { v } else { true },
        };

        // degrees: 45, 135, 225, 315 degrees relative to forward x
        let pi = 3.14159265358979323846;
        let theta = pi / 4.0; // 45 degrees
        let s = sin(theta);   // ~0.707
        let c = cos(theta);   // ~0.707

        // Gemini added a comment here helping describe what this m matrix is doing: hopefully this helps Tyler!
        // M maps Motor Throttles (delta) -> Body Wrench (u).
        // u = M * delta
        //
        // Derived from ROSflight Eq (8):
        // Column i = [0, 0, 1, -sin(theta), cos(theta), d_i]^T
        //
        // We select the relevant rows for u = [Fz, Tx, Ty, Tz]^T
        //
        // Motor Configuration (Standard X):
        // M0: Front-Right (Theta=45),  CW  (d=-1) -> [-sin(45),  cos(45)]
        // M1: Rear-Right  (Theta=135), CCW (d=1)  -> [-sin(135), cos(135)] -> [-s, -c]
        // M2: Rear-Left   (Theta=225), CW  (d=-1) -> [-sin(225), cos(225)] -> [ s, -c]
        // M3: Front-Left  (Theta=315), CCW (d=1)  -> [-sin(315), cos(315)] -> [ s,  c]
        //
        // Note on Yaw (d_i):
        // CW motors (-1) create CCW reaction torque (Left/Negative).
        // CCW motors (+1) create CW reaction torque (Right/Positive).
        
        #[rustfmt::skip]
        let m_data: [f64; 16] = [
            // M0 (FR)   M1 (RR)    M2 (RL)    M3 (FL)
            // ---------------------------------------
             1.0,       1.0,       1.0,       1.0,     // Fz (Thrust)
            -s,        -s,         s,         s,       // Tx (Roll)  = -sin(theta)
             c,        -c,        -c,         c,       // Ty (Pitch) = cos(theta)
            -1.0,       1.0,      -1.0,       1.0      // Tz (Yaw)   = d_i
        ];

        let matrix_m = Matrix::<f64, 4, 16>::from_array(m_data);

        // This is my attempt at the (Pseudoinverse). The function has been tested on the other library as of a while ago
        let allocation_matrix = pinv::<4, 4, 16, 16>(&matrix_m, 1e-12, 100);

        Self {
            allocation_matrix,
            params: mixer_params,
        }
    }
}

impl Mixer for QuadMixer {
    type MixerInput = MixerInput;
    type ActuatorCommands = Vector<f64, 4>;

    fn mix(&mut self, controls: &Self::MixerInput, state_manager: &StateManager) -> Self::ActuatorCommands {
        
        if !state_manager.is_armed() {
            return Vector::<f64, 4>::zeros();
        }

        // pack input vector u = [Fz, Tx, Ty, Tz]^T am I doing this right? Idk
        let input_vector = Vector::<f64, 4>::from_array([
            controls.thrust,      // Fz
            controls.torques[0],  // Tx (Roll)
            controls.torques[1],  // Ty (Pitch)
            controls.torques[2],  // Tz (Yaw)
        ]);

        // this is supposed to take the body wrench and map it back to outputs
        let mut outputs = self.allocation_matrix.vmul::<4>(&input_vector);


        // begin idea from gemini
        // ==================================================================================================================

        // Find the maximum magnitude requested.
        let mut max_output = 1.0;

        for i in 0..4 {
            let val = fabs(outputs[i]);
            if val > max_output {
                max_output = val;
            }
        }

        // If commanding > 100% effort, scale everything down proportionally.
        // This prioritizes attitude control direction over total thrust magnitude. <-- idea from Gemini... must
        if max_output > 1.0 {
            let scale = 1.0 / max_output;
            outputs = outputs * scale; 
        }

        // ==================================================================================================================
        // end idea from gemini

        // 5. Idle and Safety Clamping
        for i in 0..4 {
            // Apply Idle Throttle if armed
            if self.params.spin_when_armed && outputs[i] < self.params.idle_throttle {
                outputs[i] = self.params.idle_throttle;
            }
            
            // Hard clamp to valid 0.0 - 1.0 range
            if outputs[i] > 1.0 {
                outputs[i] = 1.0;
            } else if outputs[i] < 0.0 {
                outputs[i] = 0.0;
            }
        }

        outputs
    }
}