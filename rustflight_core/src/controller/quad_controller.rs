// /**
// ******************************************************************************
// * File     : quad_controller.rs
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

use super::Controller;
use crate::estimator::quad_estimator::AttitudeState;
use micro_algebra::stack::vector::Vector;
use micro_algebra::stack::quaternion::Quaternion;
use crate::command_manager::{CombinedControl, ControlType};
use crate::state_machine::StateManager;
use crate::params2::Params;
use libm::{sin, cos, atan2};

// The system's fixed time step, as defined in the estimator.
const DT: f64 = 1.0 / 400.0;

/// Clamps a value between a lower and upper bound.
fn clamp(value: f64, min: f64, max: f64) -> f64 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

// ============== PID Controller (Unchanged) ==============

#[derive(Debug, Clone, Copy, Default)]
pub struct Pid {
    pub p: f64, pub i: f64, pub d: f64, pub max_i: f64, pub tau: f64,
    pub integrator: f64, pub differentiator: f64, pub prev_x: f64, pub prev_t: f64,
}

impl Pid {
    pub fn new(p: f64, i: f64, d: f64, max_i: f64, tau: f64) -> Self {
        Self { p, i, d, max_i, tau, integrator: 0.0, differentiator: 0.0, prev_x: 0.0, prev_t: -1.0 }
    }
    pub fn run(&mut self, x: f64, x_c: f64, dt: f64, enable_integrator: bool) -> f64 {
        let error = x_c - x;
        let p_term = self.p * error;
        if enable_integrator {
            self.integrator = clamp(self.integrator + error * dt, -self.max_i, self.max_i);
        }
        let i_term = self.i * self.integrator;
        let d_term = if self.prev_t < 0.0 {
            self.prev_x = x;
            self.prev_t = 0.0;
            0.0
        } else {
            // self.differentiator = ((2.0 * self.tau - dt) / (2.0 * self.tau + dt)) * self.differentiator
            //     + (2.0 / (2.0 * self.tau + dt)) * (x - self.prev_x);
            // self.prev_x = x;
            // self.d * self.differentiator

            // dirty derivative estimator as recommended by Gemini
            let alpha = (2.0 * self.tau - dt) / (2.0 * self.tau + dt);
            let beta = 2.0 / (2.0 * self.tau + dt);
            
            let derivative = alpha * self.differentiator + beta * (x - self.prev_x);
            self.differentiator = derivative;
            self.prev_x = x;
            
            self.d * derivative

        };
        p_term + i_term - d_term
    }

    pub fn run_with_derivative(&mut self, x: f64, x_c: f64, xdot: f64, dt: f64, enable_integrator: bool) -> f64 {
        let error = x_c - x;
        
        // P Term
        let p_term = self.p * error;

        // I Term
        if enable_integrator {
            self.integrator = clamp(self.integrator + error * dt, -self.max_i, self.max_i);
        }
        let i_term = self.i * self.integrator;

        let d_term = self.d * xdot; 

        p_term + i_term - d_term
    }

    pub fn reset(&mut self) {
        self.integrator = 0.0;
        self.differentiator = 0.0;
        self.prev_x = 0.0;
        self.prev_t = -1.0;
    }
}

// ============== Controller Data Structures ==============

/// **A new struct to bundle all controller inputs.**
// #[derive(Debug, Clone, Copy)]
// pub struct ControllerInput {
//     pub attitude: AttitudeState,
//     pub commanded_rates: Vector<f64, 3>,
//     pub commanded_thrust: f64,
// }

#[derive(Debug, Clone, Copy)]
pub struct MixerInput {
    pub torques: Vector<f64, 3>,
    pub thrust: f64,
}

// ============== Quadcopter Controller Implementation ==============

#[derive(Debug, Clone, Copy, Default)]
pub struct QuadController {
    roll_rate_pid: Pid,
    pitch_rate_pid: Pid,
    yaw_rate_pid: Pid,
    roll_angle_pid: Pid,
    pitch_angle_pid: Pid,
}

impl QuadController {
    pub fn new(
        roll_rate_pid: Pid,
        pitch_rate_pid: Pid,
        yaw_rate_pid: Pid,
        roll_angle_pid: Pid,
        pitch_angle_pid: Pid
    ) -> Self {
        Self {
            roll_rate_pid,
            pitch_rate_pid,
            yaw_rate_pid,
            roll_angle_pid,
            pitch_angle_pid,
        }
    }
}

impl Controller for QuadController {
    type State = AttitudeState;
    type ControlOutput = MixerInput;

    fn control(&mut self, state: &Self::State, state_manager: &mut StateManager, command: &CombinedControl, params: &Params) -> Self::ControlOutput {

        if !state_manager.is_armed() {
            self.roll_rate_pid.reset();
            self.pitch_rate_pid.reset();
            self.yaw_rate_pid.reset();
            self.roll_angle_pid.reset();
            self.pitch_angle_pid.reset();
            
            MixerInput {
                torques: Vector::from_array([
                    0.0f64, 
                    0.0f64, 
                    0.0f64]),
                thrust: 0.0f64,
            }
        } else {
            if command.qx.control_type == ControlType::Passthrough {
                MixerInput {
                    torques: Vector::from_array([
                        command.qx.value as f64, 
                        command.qy.value as f64, 
                        command.qy.value as f64]),
                    thrust: command.fz.value as f64,
                }
            } else {

                // prevent ourselves from running the integrator while we're on the ground...
                let enable_integrator = command.fz.value > 0.1;

                // --- Step 1: Extract the necessary quaternions from the input state ---
                let q_hat = state.q_hat;
                let q_dot = state.q_dot;
            
                // --- Step 2: Calculate angular velocity using the kinematic equation ---
                let q_conj = q_hat.conjugate();
                let omega_q = 2.0 * (q_conj * q_dot);
            
                // The vector part of omega_q is our current angular rate [p, q, r]
                let current_rates = Vector::from_array([
                    omega_q.get_x(),
                    omega_q.get_y(),
                    omega_q.get_z(),
                ]);

                let mut rate_setpoints = Vector::from_array([0.0, 0.0, 0.0, 0.0]);

                // run the inner loop as necessary
                if command.qx.control_type == ControlType::Angle {
                    // use commanded roll/pitch, but use current yaw so we calculate tilt error relative to where we're facing
                    let current_yaw = get_yaw(q_hat);
                    let q_target = quaternion_from_euler(
                        command.qx.value as f64,
                        command.qy.value as f64,
                        current_yaw
                    );

                    let mut q_err = q_hat.conjugate() * q_target;

                    if q_err.get_w() < 0.0 {
                        q_err = q_err * -1.0;
                    }

                    rate_setpoints[0] = self.roll_angle_pid.run_with_derivative(0.0, 2.0*q_err.get_x(), current_rates[0], DT, enable_integrator);
                    rate_setpoints[1] = self.pitch_angle_pid.run_with_derivative(0.0, 2.0*q_err.get_y(), current_rates[1], DT, enable_integrator)
                } else {
                    // Mode: RollratePitchrateYawrateThrottle (Acro)
                    rate_setpoints[0] = command.qx.value as f64;
                    rate_setpoints[1] = command.qy.value as f64;
                }

                rate_setpoints[2] = command.qz.value as f64;

                let torque_x = self.roll_rate_pid.run(current_rates[0], rate_setpoints[0], DT, enable_integrator);
                let torque_y = self.pitch_rate_pid.run(current_rates[1], rate_setpoints[1], DT, enable_integrator);
                let torque_z = self.yaw_rate_pid.run(current_rates[2], rate_setpoints[2], DT, enable_integrator);
            
                // --- Step 5: Assemble and return the final output ---
                MixerInput {
                    torques: Vector::from_array([torque_x, torque_y, torque_z]),
                    thrust: command.fz.value as f64,
                }
            }
        }
    }
}


/// Constructs a Quaternion from Euler angles (Roll, Pitch, Yaw) ZYX sequence
/// Roll and Pitch are in radians.
pub fn quaternion_from_euler(roll: f64, pitch: f64, yaw: f64) -> Quaternion<f64> {
    // libm 0.2 does not have sin_cos, so we compute them separately
    let sr = sin(roll * 0.5);
    let cr = cos(roll * 0.5);
    
    let sp = sin(pitch * 0.5);
    let cp = cos(pitch * 0.5);
    
    let sy = sin(yaw * 0.5);
    let cy = cos(yaw * 0.5);

    Quaternion::from_array([
        cr * cp * cy + sr * sp * sy, // w
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy, // z
    ])
}

/// Extracts Yaw (Z-axis rotation) from a Quaternion
pub fn get_yaw(q: Quaternion<f64>) -> f64 {
    let w = q.get_w();
    let x = q.get_x();
    let y = q.get_y();
    let z = q.get_z();
    
    // Use libm::atan2 instead of f64::atan2
    atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
}