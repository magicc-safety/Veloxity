// /**
// ******************************************************************************
// * File     : controller_test.rs
// * Date     : November 14, 2025
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

use std::error::Error;
use std::fs::File;
use csv::WriterBuilder;
use serde::Serialize;

const PI: f64 = 3.14159265359;

// --- Import your actual library components ---
// NOTE: Crate name 'rustflight_core' is assumed. Adjust if needed.
use rustflight_core::{
    controller::Controller,
    estimator::quad_estimator::AttitudeState,
};
use micro_algebra::stack::{
    quaternion::Quaternion,
    vector::Vector,
};
use micro_algebra::libm::{
    sin,
    cos,
};

// ============================================================================
// HELPER STRUCTS AND LOGIC (for a self-contained test)
// In a real application, these would be in your main library.
// ============================================================================

fn clamp(value: f64, min: f64, max: f64) -> f64 {
    value.max(min).min(max)
}

#[derive(Debug, Clone, Copy)]
pub struct PidParams { pub p: f64, pub i: f64, pub d: f64, pub i_max: f64 }

#[derive(Debug, Clone, Copy)]
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
        
        // Corrected PID math with D-term acting as a damper
        p_term + i_term - d_term
    }
}

// The necessary input/output structs for the controller
#[derive(Debug, Clone, Copy)]
pub struct ControllerInput {
    pub attitude: AttitudeState,
    pub attitude_rate: Quaternion<f64>,
    pub commanded_rates: Vector<f64, 3>,
    pub commanded_thrust: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct MixerInput {
    pub torques: Vector<f64, 3>,
    pub thrust: f64,
}

// The stateless controller implementation
#[derive(Debug, Clone, Copy)]
pub struct QuadController {
    roll_rate_pid: Pid,
    pitch_rate_pid: Pid,
    yaw_rate_pid: Pid,
}

impl QuadController {
    pub fn new(rate_params: [PidParams; 3], tau: f64) -> Self {
        Self {
            roll_rate_pid: Pid::new(rate_params[0].p, rate_params[0].i, rate_params[0].d, rate_params[0].i_max, tau),
            pitch_rate_pid: Pid::new(rate_params[1].p, rate_params[1].i, rate_params[1].d, rate_params[1].i_max, tau),
            yaw_rate_pid: Pid::new(rate_params[2].p, rate_params[2].i, rate_params[2].d, rate_params[2].i_max, tau),
        }
    }
}

impl Controller for QuadController {
    type State = ControllerInput;
    type ControlOutput = MixerInput;

    fn control(&mut self, state: &Self::State) -> Self::ControlOutput {
        const DT: f64 = 0.01; // Simulation DT
        let q_hat = state.attitude.q_hat;
        let q_hat_dot = state.attitude_rate;
        let q_conj = q_hat.conjugate();
        let omega_q = 2.0 * q_conj * q_hat_dot;
        let current_rates = Vector::from_array([omega_q.get_x(), omega_q.get_y(), omega_q.get_z()]);

        let torque_x = self.roll_rate_pid.run(current_rates[0], state.commanded_rates[0], DT);
        let torque_y = self.pitch_rate_pid.run(current_rates[1], state.commanded_rates[1], DT);
        let torque_z = self.yaw_rate_pid.run(current_rates[2], state.commanded_rates[2], DT);
        
        MixerInput {
            torques: Vector::from_array([torque_x, torque_y, torque_z]),
            thrust: state.commanded_thrust,
        }
    }
}

// ============================================================================
// SIMULATION SPECIFIC HELPERS
// ============================================================================

struct QuadcopterDynamics {
    p: f64, q: f64, r: f64,
    orientation: Quaternion<f64>,
    ixx: f64, iyy: f64, izz: f64,
}

impl QuadcopterDynamics {
    fn new(ixx: f64, iyy: f64, izz: f64) -> Self {
        Self {
            p: 0.0, q: 0.0, r: 0.0,
            orientation: Quaternion::from_array([1.0, 0.0, 0.0, 0.0]),
            ixx, iyy, izz,
        }
    }

    fn update(&mut self, torques: &Vector<f64, 3>, dt: f64) {
        let p_dot = ((self.iyy - self.izz) * self.q * self.r / self.ixx) + (torques[0] / self.ixx);
        let q_dot = ((self.izz - self.ixx) * self.r * self.p / self.iyy) + (torques[1] / self.iyy);
        let r_dot = ((self.ixx - self.iyy) * self.p * self.q / self.izz) + (torques[2] / self.izz);

        self.p += p_dot * dt;
        self.q += q_dot * dt;
        self.r += r_dot * dt;

        let omega_q = Quaternion::from_array([0.0, self.p, self.q, self.r]);
        let q_dot = 0.5 * (self.orientation * omega_q);
        self.orientation = self.orientation + q_dot * dt;
        self.orientation.normalize_fill();
    }
}

// fn get_rate_commands(t: f64) -> Vector<f64, 3> {
//     let (p_c, q_c, r_c) = if (0.0..2.0).contains(&t) {
//         (30.0f64.to_radians(), 0.0, 0.0)
//     } else if (4.0..6.0).contains(&t) {
//         (0.0, -20.0f64.to_radians(), 0.0)
//     } else if (6.0..8.0).contains(&t) {
//         (0.0, 0.0, 40.0f64.to_radians())
//     } else {
//         (0.0, 0.0, 0.0)
//     };
//     Vector::from_array([p_c, q_c, r_c])
// }

/// Generates continuous, smoothly varying rate commands using sine waves.
fn get_rate_commands(t: f64) -> Vector<f64, 3> {
    // --- Parameters for easy modification ---

    // Maximum commanded rate in degrees/sec
    const ROLL_AMP_DEG: f64 = 30.0;
    const PITCH_AMP_DEG: f64 = 20.0;
    const YAW_AMP_DEG: f64 = 15.0;

    // Frequency in Hz (cycles per second)
    const ROLL_FREQ_HZ: f64 = 0.2;  // One full roll oscillation every 5 seconds
    const PITCH_FREQ_HZ: f64 = 0.3; // A bit faster
    const YAW_FREQ_HZ: f64 = 0.1;   // A slow yaw oscillation

    // --- Calculations ---

    // Convert amplitudes to radians/sec
    let roll_amp_rad = ROLL_AMP_DEG.to_radians();
    let pitch_amp_rad = PITCH_AMP_DEG.to_radians();
    let yaw_amp_rad = YAW_AMP_DEG.to_radians();

    // Calculate the current commanded rate for each axis
    let p_c = roll_amp_rad * (2.0 * PI * ROLL_FREQ_HZ * t).sin();
    let q_c = pitch_amp_rad * (2.0 * PI * PITCH_FREQ_HZ * t).cos(); // Use cosine to offset from roll
    let r_c = yaw_amp_rad * (2.0 * PI * YAW_FREQ_HZ * t).sin();

    Vector::from_array([p_c, q_c, r_c])
}

#[derive(Debug, Serialize)]
struct SimulationRecord {
    time_s: f64,
    cmd_roll_rad_s: f64, cmd_pitch_rad_s: f64, cmd_yaw_rad_s: f64,
    act_roll_rad_s: f64, act_pitch_rad_s: f64, act_yaw_rad_s: f64,
}

// ============================================================================
// THE TEST FUNCTION
// ============================================================================

#[test]
fn run_simulation_with_q_dot() -> Result<(), Box<dyn Error>> {
    // --- Setup ---
    const SIMULATION_TIME: f64 = 10.0;
    const DT: f64 = 0.01;

    let roll_params  = PidParams { p: 4.5, i: 3.5, d: 0.15, i_max: 5.0 };
    let pitch_params = PidParams { p: 4.5, i: 3.5, d: 0.15, i_max: 5.0 };
    let yaw_params   = PidParams { p: 3.0, i: 2.0, d: 0.05, i_max: 5.0 };
    let tau = 0.05;

    let mut controller = QuadController::new([roll_params, pitch_params, yaw_params], tau);
    let mut dynamics = QuadcopterDynamics::new(0.1, 0.1, 0.2);

    let output_path = "tests/rust_controller_results.csv";
    let mut wtr = WriterBuilder::new().from_path(output_path)?;
    println!("\nRunning q_dot simulation and writing to '{}'...", output_path);

    // --- Simulation Loop ---
    let num_steps = (SIMULATION_TIME / DT) as usize;
    for i in 0..num_steps {
        let t = i as f64 * DT;
        let commanded_rates = get_rate_commands(t);

        let omega_q = Quaternion::from_array([0.0, dynamics.p, dynamics.q, dynamics.r]);
        let q_dot = 0.5 * (dynamics.orientation * omega_q);

        let controller_input = ControllerInput {
            attitude: AttitudeState { q_hat: dynamics.orientation, b_hat: Vector::zeros() },
            attitude_rate: q_dot,
            commanded_rates,
            commanded_thrust: 0.5,
        };

        let mixer_input = controller.control(&controller_input);
        dynamics.update(&mixer_input.torques, DT);
        
        wtr.serialize(SimulationRecord {
            time_s: t,
            cmd_roll_rad_s: commanded_rates[0],
            cmd_pitch_rad_s: commanded_rates[1],
            cmd_yaw_rad_s: commanded_rates[2],
            act_roll_rad_s: dynamics.p,
            act_pitch_rad_s: dynamics.q,
            act_yaw_rad_s: dynamics.r,
        })?;
    }

    wtr.flush()?;
    println!("Simulation complete.");
    Ok(())
}
