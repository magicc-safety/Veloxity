
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

// use crate::controller::quad_controller::MixerInput;
// use crate::mixer::Mixer;
// use crate::params2::{ParamId, ParamValue, Params};
// use micro_algebra::stack::matrix::Matrix;
// use micro_algebra::stack::vector::Vector;

// use num_traits::Float;

// #[derive(Debug, Clone, Copy)]
// pub struct MixerParams {
//     // Motor physics parameters
//     resistance: f64,
//     kv: f64,
//     no_load_current: f64,
//     prop_diameter: f64,
//     prop_ct: f64,
//     prop_cq: f64,
//     max_voltage: f64,
//     // Mixer settings
//     num_motors: usize,
//     idle_throttle: f64,
//     spin_when_armed: bool,
// }

// pub struct QuadMixer {
//     mixing_matrix: Matrix<f64, 4, 16>,
//     params: MixerParams,
//     use_motor_params: bool,
// }

// impl QuadMixer {
//     /// Creates a new mixer, configured from the parameter server.
//     pub fn new(params: &Params) -> Self {
//         let use_motor_params =
//             if let ParamValue::Bool(val) = params.get_by_id(ParamId::PARAM_USE_MOTOR_PARAMETERS) {
//                 val
//             } else {
//                 false
//             };

//         // This is a simplified version of the C++ init_mixing logic.
//         // For now, we are hard-coding the QuadX mixer. A full implementation
//         // would read `PARAM_PRIMARY_MIXER` and choose the matrix accordingly.
//         let data: [f64; 16] = [
//             // Thrust,  Roll,   Pitch,   Yaw
//             1.0, -1.0, -1.0, -1.0, // Motor 0 (FR, CW)
//             1.0, -1.0, 1.0, 1.0, // Motor 1 (RR, CCW)
//             1.0, 1.0, 1.0, -1.0, // Motor 2 (RL, CW)
//             1.0, 1.0, -1.0, 1.0, // Motor 3 (FL, CCW)
//         ];
//         let mixing_matrix = Matrix::from_array(data);

//         // Safely load all required parameters with defaults.
//         let mixer_params = MixerParams {
//             resistance: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MOTOR_RESISTANCE) { v as f64 } else { 0.042 },
//             kv: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MOTOR_KV) { v as f64 } else { 0.01706 },
//             no_load_current: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_NO_LOAD_CURRENT) { v as f64 } else { 1.5 },
//             prop_diameter: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_PROP_DIAMETER) { v as f64 } else { 0.381 },
//             prop_ct: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_PROP_CT) { v as f64 } else { 0.075 },
//             prop_cq: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_PROP_CQ) { v as f64 } else { 0.0045 },
//             max_voltage: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_VOLT_MAX) { v as f64 } else { 25.0 },
//             num_motors: if let ParamValue::Int(v) = params.get_by_id(ParamId::PARAM_NUM_MOTORS) { v as usize } else { 4 },
//             idle_throttle: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE) { v as f64 } else { 0.1 },
//             spin_when_armed: if let ParamValue::Bool(v) = params.get_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED) { v } else { true },
//         };

//         Self {
//             mixing_matrix,
//             params: mixer_params,
//             use_motor_params,
//         }
//     }

//     /// Private helper for simple matrix multiplication mixing.
//     fn mix_without_motor_params(&self, controls: &MixerInput) -> Vector<f64, 4> {
//         let command_vector = Vector::from_array([
//             controls.thrust,
//             controls.torques[0], // Roll
//             controls.torques[1], // Pitch
//             controls.torques[2], // Yaw
//         ]);
//         self.mixing_matrix.vmul(&command_vector)
//     }

//     /// Private helper for physics-based mixing.
//     fn mix_with_motor_params(&self, controls: &MixerInput, rho: f64) -> Vector<f64, 4> {
//         let mut outputs = Vector::<f64, 4>::zeros();
//         let p = self.params;

//         // In this mode, the mixer matrix converts desired F/T into required omega^2 for each motor
//         let omega_sq_vector = self.mix_without_motor_params(controls);

//         for i in 0..p.num_motors {
//             let omega_sq = if omega_sq_vector[i] < 0.0 { 0.0 } else { omega_sq_vector[i] };
//             let omega = omega_sq.sqrt();
            
//             let v_in = (rho * p.prop_diameter.powi(5) / (4.0 * core::f64::consts::PI.powi(2)))
//                 * omega_sq * p.prop_cq * p.resistance / p.kv
//                 + p.resistance * p.no_load_current + p.kv * omega;
                
//             outputs[i] = v_in / p.max_voltage;
//         }
//         outputs
//     }
// }

// impl Mixer for QuadMixer {
//     type MixerInput = MixerInput;
//     type ActuatorCommands = Vector<f64, 4>; // Output for 4 motors

//     fn mix(&mut self, controls: &Self::MixerInput) -> Self::ActuatorCommands {
//         // Decide which mixing algorithm to use based on the parameter.
//         // For now, rho (air density) is hardcoded. A full implementation would get this from sensors.
//         let mut motor_outputs = if self.use_motor_params {
//             self.mix_with_motor_params(controls, 1.225)
//         } else {
//             self.mix_without_motor_params(controls)
//         };
        
//         // --- Handle Saturation and Clamping (from C++ `mix_output`) ---
//         let mut max_output = 1.0;
//         for i in 0..self.params.num_motors {
//             if motor_outputs[i].abs() > max_output {
//                 max_output = motor_outputs[i].abs();
//             }
//         }

//         // If any motor is commanded above 100%, scale all motor commands down.
//         if max_output > 1.0 {
//             for i in 0..self.params.num_motors {
//                 motor_outputs[i] /= max_output;
//             }
//         }
        
//         // Enforce idle throttle and clamp final outputs.
//         // Assumes the vehicle is armed. A full implementation would check the state_manager.
//         for i in 0..self.params.num_motors {
//             if self.params.spin_when_armed && motor_outputs[i] < self.params.idle_throttle {
//                 motor_outputs[i] = self.params.idle_throttle;
//             }
//             motor_outputs[i] = motor_outputs[i].clamp(0.0, 1.0);
//         }

//         motor_outputs
//     }
// }

use crate::controller::quad_controller::MixerInput;
use crate::mixer::Mixer;
use crate::params2::{ParamId, ParamValue, Params};
use micro_algebra::stack::matrix::Matrix;
use micro_algebra::stack::vector::Vector;
use crate::state_machine::StateManager;
use num_traits::Float;

#[derive(Debug, Clone, Copy)]
pub struct MixerParams {
    // Geometric and Aerodynamic parameters
    pub k_t: f64,        // Thrust coefficient (C_T * rho * D^4) or simplified lumped constant
    pub k_q: f64,        // Torque coefficient (C_Q * rho * D^5) or simplified lumped constant
    pub arm_length: f64, // Distance from Center of Mass to Motor (l)
    
    // Safety / limits
    pub max_motor_speed: f64, // Max theoretical Omega (rad/s) to normalize output
    pub idle_throttle: f64,
    pub spin_when_armed: bool,
    pub num_motors: usize,
}

pub struct QuadMixer {
    // Pre-calculated Inverse Matrix (Allocation Matrix)
    // Maps [Thrust, Roll, Pitch, Yaw] -> [Omega_1^2, Omega_2^2, Omega_3^2, Omega_4^2]
    // User Note: Keeping <f64, 4, 16> as per existing library convention for this codebase.
    allocation_matrix: Matrix<f64, 4, 16>, 
    params: MixerParams,
}

impl QuadMixer {


    pub fn new(params: &Params) -> Self {
        // Load Parameters
        
        // 1. Calculate Max Motor Speed from Physics if specific param not present
        // Model: Max RPM approx = KV * Voltage
        // KV is typically RPM/Volt. 
        // Omega (rad/s) = RPM * 2*PI / 60.
        let kv = if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MOTOR_KV) { v as f64 } else { 900.0 };
        let max_volts = if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_VOLT_MAX) { v as f64 } else { 12.6 };
        
        // We calculate the theoretical max speed of the motor to normalize the mixer output (0.0 to 1.0)
        let calculated_max_omega = if kv < 50.0 {
             // Handle cases where KV might be stored in SI units (rad/s/V) or is just tiny in tests.
             let val = kv * max_volts;
             if val < 10.0 { 1000.0 } else { val }
        } else {
             // Standard RPM/V conversion
             (kv * max_volts) * (2.0 * std::f64::consts::PI / 60.0)
        };

        let mixer_params = MixerParams {
            // Approximating lumped Kt from Prop Diameter + CT if explicit Kt isn't available
            // standard approx: k_t = C_T * rho * D^4
            k_t: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_PROP_CT) { v as f64 } else { 0.000_001 }, 
            
            // standard approx: k_q = C_Q * rho * D^5
            k_q: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_PROP_CQ) { v as f64 } else { 0.000_000_1 },
            
            // Defaulting to 0.25m if not in params
            arm_length: 0.25, 
            
            max_motor_speed: calculated_max_omega,
            
            num_motors: 4,
            idle_throttle: if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE) { v as f64 } else { 0.1 },
            spin_when_armed: if let ParamValue::Bool(v) = params.get_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED) { v } else { true },
        };
        
        // --- COPY THIS SECTION ---
        let l = mixer_params.arm_length;
        let kt = mixer_params.k_t;
        let kq = mixer_params.k_q;
        
        let l_eff = l / 2.0_f64.sqrt(); 

        let f_t = 1.0 / (4.0 * kt);
        let f_r = 1.0 / (4.0 * l_eff * kt);
        let f_y = 1.0 / (4.0 * kq);

        // CORRECTED ALLOCATION MATRIX (Verified Simulator Order)
        // -----------------------------------------------------
        // Layout: 
        // 0: RR (CCW), 1: FR (CW), 2: RL (CW), 3: FL (CCW)
        
        // #[rustfmt::skip]
        // let data: [f64; 16] = [
        //     // Thrust, Roll,  Pitch, Yaw
        //     f_t,     -f_r, -f_r,    f_y, // Motor 0: Rear Right
        //     f_t,     -f_r,  f_r,   -f_y, // Motor 1: Front Right
        //     f_t,      f_r, -f_r,   -f_y, // Motor 2: Rear Left
        //     f_t,      f_r,  f_r,    f_y, // Motor 3: Front Left
        // ];

        #[rustfmt::skip]
        let data: [f64; 16] = [
            // Thrust, Roll,  Pitch, Yaw
            f_t,      f_r,  f_r,   -f_y, // Motor 0: Front Left
            f_t,      f_r, -f_r,    f_y, // Motor 1: Rear Left
            f_t,     -f_r,  f_r,    f_y, // Motor 2: Front Right
            f_t,     -f_r, -f_r,   -f_y, // Motor 3: Rear Right
        ];
        
        let allocation_matrix = Matrix::from_array(data);

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
        
        // 1. SAFETY: Disarmed Check
        // Prevents spinning on the ground or startup.
        if !state_manager.is_armed() {
            return Vector::<f64, 4>::zeros();
        }

        // 2. Calculate Physical Limits
        let max_omega_sq = self.params.max_motor_speed.powi(2);
        
        let max_motor_thrust = self.params.k_t * max_omega_sq;
        let max_motor_torque = self.params.k_q * max_omega_sq; 

        // Vehicle Totals:
        let max_thrust_total = 4.0 * max_motor_thrust;

        // l_eff = arm_length / sqrt(2)
        let l_eff = self.params.arm_length / 2.0_f64.sqrt();
        let max_moment_rp = 2.0 * max_motor_thrust * l_eff;
        let max_moment_yaw = 2.0 * max_motor_torque;

        // 3. Scale Inputs (Normalizing -> Physical Units)
        // This is CRITICAL. It maps 0.0-1.0 from the PIDs to actual Newtons/Nm.
        let scaled_thrust = controls.thrust * max_thrust_total;
        let scaled_roll   = controls.torques[0] * max_moment_rp;
        let scaled_pitch  = controls.torques[1] * max_moment_rp;
        let scaled_yaw    = controls.torques[2] * max_moment_yaw;

        // 4. Pack Input Vector
        // We use the full PID outputs now (no longer zeroed out!)
        let input_vector = Vector::from_array([
            scaled_thrust,
            scaled_roll, 
            scaled_pitch, 
            scaled_yaw,  
        ]);

        // 5. Apply Allocation Matrix
        let mut motor_squared_vels = self.allocation_matrix.vmul(&input_vector);

        // 6. Convert Omega^2 to Normalized Output (0.0 - 1.0)
        let mut outputs = Vector::<f64, 4>::zeros();
        let mut max_output = 1.0;

        for i in 0..4 {
            if motor_squared_vels[i] < 0.0 {
                motor_squared_vels[i] = 0.0;
            }

            let omega = motor_squared_vels[i].sqrt();
            
            // Normalize: output = omega / max_omega
            outputs[i] = omega / self.params.max_motor_speed;

            // Track max for desaturation
            if outputs[i].abs() > max_output {
                max_output = outputs[i].abs();
            }
        }

        // 7. Desaturation
        if max_output > 1.0 {
            for i in 0..4 {
                outputs[i] /= max_output;
            }
        }

        // 8. Idle and Safety
        // We are guaranteed to be ARMED here, so we apply idle throttle.
        for i in 0..4 {
            if self.params.spin_when_armed && outputs[i] < self.params.idle_throttle {
                outputs[i] = self.params.idle_throttle;
            }
            outputs[i] = outputs[i].clamp(0.0, 1.0);
        }

        // Optional: Keep this for a few flights to verify PIDs are doing work
        // println!("Out: {:.2}, {:.2}, {:.2}, {:.2}", outputs[0], outputs[1], outputs[2], outputs[3]);

        outputs
    }
}