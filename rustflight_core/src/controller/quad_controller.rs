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
use crate::command_manager::{Control, ControlType};

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
    p: f64, i: f64, d: f64, max_i: f64, tau: f64,
    integrator: f64, differentiator: f64, prev_x: f64, prev_t: f64,
}

impl Pid {
    pub fn new(p: f64, i: f64, d: f64, max_i: f64, tau: f64) -> Self {
        Self { p, i, d, max_i, tau, integrator: 0.0, differentiator: 0.0, prev_x: 0.0, prev_t: -1.0 }
    }
    pub fn run(&mut self, x: f64, x_c: f64, dt: f64) -> f64 {
        let error = x_c - x;
        let p_term = self.p * error;
        self.integrator = clamp(self.integrator + error * dt, -self.max_i, self.max_i);
        let i_term = self.i * self.integrator;
        let d_term = if self.prev_t < 0.0 {
            self.prev_x = x;
            self.prev_t = 0.0;
            0.0
        } else {
            self.differentiator = ((2.0 * self.tau - dt) / (2.0 * self.tau + dt)) * self.differentiator
                + (2.0 / (2.0 * self.tau + dt)) * (x - self.prev_x);
            self.prev_x = x;
            self.d * self.differentiator
        };
        p_term + i_term - d_term
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

#[derive(Debug, Clone, Copy, Default)]
pub struct PidParams {
    pub p: f64, pub i: f64, pub d: f64, pub i_max: f64,
}

// ============== Quadcopter Controller Implementation ==============

#[derive(Debug, Clone, Copy, Default)]
pub struct QuadController {
    roll_rate_pid: Pid,
    pitch_rate_pid: Pid,
    yaw_rate_pid: Pid,
}

impl Controller for QuadController {
    type State = AttitudeState;
    type ControlOutput = MixerInput;

    fn control(&mut self, state: &Self::State, command: &Control) -> Self::ControlOutput {

        if command.qx.control_type == ControlType::Passthrough {
            MixerInput {
                torques: Vector::from_array([
                    command.qx.value as f64, 
                    command.qy.value as f64, 
                    command.qy.value as f64]),
                thrust: command.fz.value as f64,
            }
        } else {
            // --- Step 1: Extract the necessary quaternions from the input state ---
            let q_hat = state.q_hat;
            let q_dot = state.q_dot;
        
            // --- Step 2: Calculate angular velocity using the kinematic equation ---
            let q_conj = q_hat.conjugate();
            let omega_q = 2.0 * q_conj * q_dot;
        
            // The vector part of omega_q is our current angular rate [p, q, r]
            let current_rates = Vector::from_array([
                omega_q.get_x(),
                omega_q.get_y(),
                omega_q.get_z(),
            ]);

            // --- Step 3: Get Commanded Rates from the Input State ---
            //let commanded_rates = command.commanded_rates;

            // --- Step 4: Run PID Rate Controllers with the clean rate signal ---
            const DT: f64 = 1.0 / 400.0; // DT is still needed for the PID's discrete I and D terms
            let torque_x = self.roll_rate_pid.run(current_rates[0], command.qx.value as f64, DT);
            let torque_y = self.pitch_rate_pid.run(current_rates[1], command.qy.value as f64, DT);
            let torque_z = self.yaw_rate_pid.run(current_rates[2], command.qz.value as f64, DT);
        
            // --- Step 5: Assemble and return the final output ---
            MixerInput {
                torques: Vector::from_array([torque_x, torque_y, torque_z]),
                thrust: command.fz.value as f64,
            }
        }
    }
}
