
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

#[derive(Debug, Clone, Copy)]
pub struct MixerParams {
    // Motor physics parameters
    resistance: f64,
    kv: f64,
    no_load_current: f64,
    prop_diameter: f64,
    prop_ct: f64,
    prop_cq: f64,
    max_voltage: f64,
    // Mixer settings
    num_motors: usize,
    idle_throttle: f64,
    spin_when_armed: bool,
}

pub struct QuadMixer {
    mixing_matrix: Matrix<f64, 4, 16>,
    params: MixerParams,
    use_motor_params: bool,
}

impl QuadMixer {
    /// Creates a new mixer, configured from the parameter server.
    pub fn new(params: &Params) -> Self {
        let use_motor_params =
            if let ParamValue::Bool(val) = params.get_by_id(ParamId::PARAM_USE_MOTOR_PARAMETERS) {
                val
            } else {
                false
            };

        // This is a simplified version of the C++ init_mixing logic.
        // For now, we are hard-coding the QuadX mixer. A full implementation
        // would read `PARAM_PRIMARY_MIXER` and choose the matrix accordingly.
        let data: [f64; 16] = [
            // Thrust,  Roll,   Pitch,   Yaw
            1.0, -1.0, -1.0, -1.0, // Motor 0 (FR, CW)
            1.0, -1.0, 1.0, 1.0, // Motor 1 (RR, CCW)
            1.0, 1.0, 1.0, -1.0, // Motor 2 (RL, CW)
            1.0, 1.0, -1.0, 1.0, // Motor 3 (FL, CCW)
        ];
        let mixing_matrix = Matrix::from_array(data);

        // Safely load all required parameters with defaults.
        let mixer_params = MixerParams {
            resistance: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MOTOR_RESISTANCE) { v as f64 } else { 0.042 },
            kv: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MOTOR_KV) { v as f64 } else { 0.01706 },
            no_load_current: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_NO_LOAD_CURRENT) { v as f64 } else { 1.5 },
            prop_diameter: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_PROP_DIAMETER) { v as f64 } else { 0.381 },
            prop_ct: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_PROP_CT) { v as f64 } else { 0.075 },
            prop_cq: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_PROP_CQ) { v as f64 } else { 0.0045 },
            max_voltage: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_VOLT_MAX) { v as f64 } else { 25.0 },
            num_motors: if let ParamValue::Int(v) = params.get_by_id(ParamId::PARAM_NUM_MOTORS) { v as usize } else { 4 },
            idle_throttle: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE) { v as f64 } else { 0.1 },
            spin_when_armed: if let ParamValue::Bool(v) = params.get_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED) { v } else { true },
        };

        Self {
            mixing_matrix,
            params: mixer_params,
            use_motor_params,
        }
    }

    /// Private helper for simple matrix multiplication mixing.
    fn mix_without_motor_params(&self, controls: &MixerInput) -> Vector<f64, 4> {
        let command_vector = Vector::from_array([
            controls.thrust,
            controls.torques[0], // Roll
            controls.torques[1], // Pitch
            controls.torques[2], // Yaw
        ]);
        self.mixing_matrix.vmul(&command_vector)
    }

    /// Private helper for physics-based mixing.
    fn mix_with_motor_params(&self, controls: &MixerInput, rho: f64) -> Vector<f64, 4> {
        let mut outputs = Vector::<f64, 4>::zeros();
        let p = self.params;

        // In this mode, the mixer matrix converts desired F/T into required omega^2 for each motor
        let omega_sq_vector = self.mix_without_motor_params(controls);

        for i in 0..p.num_motors {
            let omega_sq = if omega_sq_vector[i] < 0.0 { 0.0 } else { omega_sq_vector[i] };
            let omega = omega_sq.sqrt();
            
            let v_in = (rho * p.prop_diameter.powi(5) / (4.0 * core::f64::consts::PI.powi(2)))
                * omega_sq * p.prop_cq * p.resistance / p.kv
                + p.resistance * p.no_load_current + p.kv * omega;
                
            outputs[i] = v_in / p.max_voltage;
        }
        outputs
    }
}

impl Mixer for QuadMixer {
    type MixerInput = MixerInput;
    type ActuatorCommands = Vector<f64, 4>; // Output for 4 motors

    fn mix(&mut self, controls: &Self::MixerInput) -> Self::ActuatorCommands {
        // Decide which mixing algorithm to use based on the parameter.
        // For now, rho (air density) is hardcoded. A full implementation would get this from sensors.
        let mut motor_outputs = if self.use_motor_params {
            self.mix_with_motor_params(controls, 1.225)
        } else {
            self.mix_without_motor_params(controls)
        };
        
        // --- Handle Saturation and Clamping (from C++ `mix_output`) ---
        let mut max_output = 1.0;
        for i in 0..self.params.num_motors {
            if motor_outputs[i].abs() > max_output {
                max_output = motor_outputs[i].abs();
            }
        }

        // If any motor is commanded above 100%, scale all motor commands down.
        if max_output > 1.0 {
            for i in 0..self.params.num_motors {
                motor_outputs[i] /= max_output;
            }
        }
        
        // Enforce idle throttle and clamp final outputs.
        // Assumes the vehicle is armed. A full implementation would check the state_manager.
        for i in 0..self.params.num_motors {
            if self.params.spin_when_armed && motor_outputs[i] < self.params.idle_throttle {
                motor_outputs[i] = self.params.idle_throttle;
            }
            motor_outputs[i] = motor_outputs[i].clamp(0.0, 1.0);
        }

        motor_outputs
    }
}