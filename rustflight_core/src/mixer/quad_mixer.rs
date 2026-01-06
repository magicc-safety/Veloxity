
// // /**
// // ******************************************************************************
// // * File     : quad_mixer.rs
// // * Date     : May 8, 2025
// // ******************************************************************************
// // *
// // * Copyright (c) 2023, AeroVironment, Inc.
// // * All rights reserved.
// // *
// // * Redistribution and use in source and binary forms, with or without
// // * modification, are permitted provided that the following conditions are met:
// // *
// // * 1.Redistributions of source code must retain the above copyright notice, this
// // * list of conditions and the following disclaimer.
// // *
// // * 2.Redistributions in binary form must reproduce the above copyright notice,
// // * this list of conditions and the following disclaimer in the documentation
// // * and/or other materials provided with the distribution.
// // *
// // * 3.Neither the name of the copyright holder nor the names of its
// // * contributors may be used to endorse or promote products derived from
// // * this software without specific prior written permission.
// // *
// // * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// // * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// // * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// // * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// // * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// // * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// // * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// // * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// // * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// // * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// // *
// // ******************************************************************************
// // **

use crate::controller::quad_controller::MixerInput;
use crate::mixer::Mixer;
use crate::params2::{ParamId, ParamValue, Params};
use micro_algebra::stack::matrix::Matrix;
use micro_algebra::stack::vector::Vector;
use micro_algebra::linalg::pinv;
use crate::state_machine::StateManager;
use num_traits::Float;
use libm::{sin, cos, fabs};

// In no_std, we can use core::f64::consts or just hardcode constants.
const FRAC_PI_4: f64 = 0.78539816339744830961; // 45 degrees in radians

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
    // Stores the calculated max physical authority for each axis (Thrust, Roll, Pitch, Yaw)
    input_scale: Vector<f64, 4>,
}

impl QuadMixer {

    pub fn new(params: &Params) -> Self {
        
        let mixer_params = MixerParams {
            num_motors: 4,
            idle_throttle: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE) { v as f64 } else { 0.1 },
            spin_when_armed: if let ParamValue::Bool(v) = params.get_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED) { v } else { true },
        };

        // degrees: 45, 135, 225, 315 degrees relative to forward x
        let theta = FRAC_PI_4; // 45 degrees
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

        // ====================================================================
        // DYNAMIC SCALING CALCULATION
        // ====================================================================
        // Instead of hardcoding "4.0" or "2.0 * s", we calculate the max authority
        // directly from the geometry matrix.
        //
        // Max Authority = Sum of POSITIVE coefficients in the row.
        // (i.e., The value we get if we saturate all contributing motors to 1.0 
        // and set opposing motors to 0.0)
        
        let mut scales = [0.0; 4];
        for row in 0..4 {
            let mut max_authority = 0.0;
            for col in 0..4 {
                // Access flattened array: row * 4 + col
                let val = m_data[row * 4 + col];
                if val > 0.0 {
                    max_authority += val;
                }
            }
            scales[row] = max_authority;
        }

        Self {
            allocation_matrix,
            params: mixer_params,
            input_scale: Vector::<f64, 4>::from_array(scales),
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

        // ====================================================================
        // SCALING FACTORS (Fix for 1.0 Input != 1.0 Output)
        // ====================================================================
        // The mixer matrix expects physical units. Since our inputs are normalized (0..1),
        // we must scale them by the maximum physical authority of the frame.
        // We use the scales pre-calculated in `new()` for robustness.

        let input_vector = Vector::<f64, 4>::from_array([
            controls.thrust     * self.input_scale[0],
            controls.torques[0] * self.input_scale[1],  // Tx (Roll)
            controls.torques[1] * self.input_scale[2],  // Ty (Pitch)
            controls.torques[2] * self.input_scale[3],  // Tz (Yaw)
        ]);

        // this is supposed to take the body wrench and map it back to outputs
        let mut outputs = self.allocation_matrix.vmul::<4>(&input_vector);


        // ====================================================================
        // SATURATION HANDLING (Priority: Direction > Thrust)
        // ====================================================================
        // We use a "Sliding Window" approach. If outputs exceed [0.0, 1.0], we shift
        // the whole set up or down. This preserves the differential (steering) 
        // while sacrificing total thrust (altitude).

        // 1. Find the min and max requested motor outputs
        let mut max_val = -f64::INFINITY;
        let mut min_val = f64::INFINITY;

        for i in 0..4 {
            if outputs[i] > max_val { max_val = outputs[i]; }
            if outputs[i] < min_val { min_val = outputs[i]; }
        }

        // 2. Ceiling Violation: If any motor is > 1.0, shift EVERYTHING down.
        if max_val > 1.0 {
            let shift = max_val - 1.0;
            for i in 0..4 {
                outputs[i] -= shift;
            }
        }

        // 3. Floor Violation (Airmode): If any motor is < idle, shift EVERYTHING up.
        // We check this after the ceiling shift. This ensures steering control 
        // even at zero throttle.
        if self.params.spin_when_armed && min_val < self.params.idle_throttle {
            let shift = self.params.idle_throttle - min_val;
            for i in 0..4 {
                outputs[i] += shift;
            }
        }

        // 4. Final Safety Clamping
        // In extreme cases (Max Throttle + Max Roll), the spread might still be too wide.
        // We hard clamp to ensure safety.
        for i in 0..4 {
            if outputs[i] > 1.0 {
                outputs[i] = 1.0;
            } else if outputs[i] < 0.0 {
                outputs[i] = 0.0;
            }
        }

        // // Mixer debug output
        // crate::log_info!("Motor outputs with params: {:.3?} {:.3?} {:.3?} {:.3?}", outputs[0], outputs[1], outputs[2], outputs[3]);

        outputs
    }
}