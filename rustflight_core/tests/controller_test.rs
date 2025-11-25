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


// use std::error::Error;
// use csv::WriterBuilder;
// use serde::Serialize;

// // Import your library components
// use rustflight_core::{
//     controller::{Controller, quad_controller::{QuadController, Pid, MixerInput}},
//     estimator::quad_estimator::AttitudeState,
//     command_manager::{CombinedControl, ControlType, ControlChannel},
//     state_machine::{StateManager, Event},
//     params2::{Params, ParamId, ParamValue},
// };

// use micro_algebra::stack::{
//     quaternion::Quaternion,
//     vector::Vector,
// };

// use libm::{sin, cos, pow, fabs};

// const PI: f64 = 3.14159265359;

// // ============================================================================
// // HELPER STRUCTS AND MOCKS
// // ============================================================================

// fn create_mock_command() -> CombinedControl {
//     CombinedControl {
//         stamp_ms: 0,
//         qx: ControlChannel { value: 0.0, control_type: ControlType::Rate, active: true },
//         qy: ControlChannel { value: 0.0, control_type: ControlType::Rate, active: true },
//         qz: ControlChannel { value: 0.0, control_type: ControlType::Rate, active: true },
//         fx: ControlChannel { value: 0.0, control_type: ControlType::Throttle, active: true },
//         fy: ControlChannel { value: 0.0, control_type: ControlType::Throttle, active: true },
//         fz: ControlChannel { value: 0.0, control_type: ControlType::Throttle, active: true },
//     }
// }

// // ============================================================================
// // SIMULATION SPECIFIC HELPERS
// // ============================================================================

// struct QuadcopterDynamics {
//     p: f64, q: f64, r: f64,
//     orientation: Quaternion<f64>,
//     ixx: f64, iyy: f64, izz: f64,
// }

// impl QuadcopterDynamics {
//     fn new(ixx: f64, iyy: f64, izz: f64) -> Self {
//         Self {
//             p: 0.0, q: 0.0, r: 0.0,
//             orientation: Quaternion::from_array([1.0, 0.0, 0.0, 0.0]), 
//             ixx, iyy, izz,
//         }
//     }

//     fn update(&mut self, torques: &Vector<f64, 3>, dt: f64) {
//         let p_dot = ((self.iyy - self.izz) * self.q * self.r / self.ixx) + (torques[0] / self.ixx);
//         let q_dot_kin = ((self.izz - self.ixx) * self.r * self.p / self.iyy) + (torques[1] / self.iyy);
//         let r_dot = ((self.ixx - self.iyy) * self.p * self.q / self.izz) + (torques[2] / self.izz);

//         self.p += p_dot * dt;
//         self.q += q_dot_kin * dt;
//         self.r += r_dot * dt;

//         let omega_q = Quaternion::from_array([0.0, self.p, self.q, self.r]);
//         let q_dot = (self.orientation * omega_q) * 0.5; 
//         self.orientation = self.orientation + q_dot * dt;
        
//         self.orientation.normalize_fill();
//     }
// }

// /// Generates commands.
// /// For t < 5.0: Returns RATE commands (rad/s).
// /// For t >= 5.0: Returns ANGLE commands (rad).
// fn get_commands(t: f64) -> (Vector<f64, 3>, ControlType) {
//     if t < 5.0 {
//         // --- RATE MODE ---
//         let roll_rate = 1.0 * sin(2.0 * PI * 0.5 * t); // +/- 0.5 rad/s
//         let pitch_rate = 1.0 * cos(2.0 * PI * 0.5 * t);
//         let yaw_rate = 3.0 * sin(2.0 * PI * 0.2 * t);
//         (Vector::from_array([roll_rate, pitch_rate, yaw_rate]), ControlType::Rate)
//     } else {
//         // --- ANGLE MODE ---
//         // We want smooth transitions, but for this test a jump is fine to see response.
//         let roll_angle = 1.0 * sin(2.0 * PI * 0.2 * (t - 5.0)); // +/- 0.3 rad (~17 deg)
//         let pitch_angle = 1.0 * cos(2.0 * PI * 0.2 * (t - 5.0));
//         let yaw_rate = 3.0; // Keep yaw in rate mode usually, but let's zero it
//         (Vector::from_array([roll_angle, pitch_angle, yaw_rate]), ControlType::Angle)
//     }
// }

// #[derive(Debug, Serialize)]
// struct SimulationRecord {
//     time_s: f64,
//     mode_id: u8, // 0 = Rate, 1 = Angle
//     // Commands (either rate or angle depending on mode)
//     cmd_x: f64, cmd_y: f64, cmd_z: f64, 
//     // Actual State
//     act_roll_rad: f64, act_pitch_rad: f64, act_yaw_rad: f64, // Angles
//     act_p_rad_s: f64, act_q_rad_s: f64, act_r_rad_s: f64,    // Rates
//     // Outputs
//     torque_x: f64, torque_y: f64, torque_z: f64,
// }

// // ============================================================================
// // THE TEST FUNCTION
// // ============================================================================

// #[test]
// fn run_mixed_mode_simulation() -> Result<(), Box<dyn Error>> {
//     const SIMULATION_TIME: f64 = 10.0;
//     const DT: f64 = 0.01;

//     // 1. Setup State Manager (Armed)
//     let mut params = Params::new();
//     params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.001)); // Cheat calibration

//     let mut state_manager = StateManager::new();
//     state_manager.update(Event::INITIALIZED, &params);
//     state_manager.update(Event::REQUEST_ARM, &params);
//     assert!(state_manager.is_armed());

//     // 2. Initialize Controller
//     // Tuning: Angle P=6.0 provides good tracking. Rate P=4.5, I=3.5, D=0.15.
//     let mut controller = QuadController::new(
//         Pid::new(4.5, 3.5, 0.15, 5.0, 0.05), // roll_rate
//         Pid::new(4.5, 3.5, 0.15, 5.0, 0.05), // pitch_rate
//         Pid::new(3.0, 2.0, 0.05, 5.0, 0.05), // yaw_rate
//         Pid::new(6.0, 0.0, 0.0, 0.0, 0.0),   // roll_angle
//         Pid::new(6.0, 0.0, 0.0, 0.0, 0.0)    // pitch_angle
//     );

//     let mut dynamics = QuadcopterDynamics::new(0.1, 0.1, 0.2);
    
//     // Ensure directory exists
//     std::fs::create_dir_all("tests/controller")?;
//     let output_path = "tests/controller/rust_controller_results.csv";
//     let mut wtr = WriterBuilder::new().from_path(output_path)?;
//     println!("\nRunning Mixed Mode simulation (Rate -> Angle) and writing to '{}'...", output_path);

//     let num_steps = (SIMULATION_TIME / DT) as usize;
//     for i in 0..num_steps {
//         let t = i as f64 * DT;
        
//         // 3. Get Commands (Switches mode at t=5.0)
//         let (cmd_vec, mode) = get_commands(t);
        
//         let mut command = create_mock_command();
//         command.qx.value = cmd_vec[0];
//         command.qy.value = cmd_vec[1];
//         command.qz.value = cmd_vec[2]; // Yaw is always rate in this helper for simplicity
//         command.fz.value = 0.5; 
        
//         command.qx.control_type = mode;
//         command.qy.control_type = mode;
//         command.qz.control_type = ControlType::Rate; // Keep yaw as rate for now

//         // 4. Dynamics Update
//         let omega_q = Quaternion::from_array([0.0, dynamics.p, dynamics.q, dynamics.r]);
//         let q_dot = (dynamics.orientation * omega_q) * 0.5;

//         let state = AttitudeState { 
//             q_hat: dynamics.orientation, 
//             q_dot: q_dot, 
//             b_hat: Vector::zeros(),
//             is_healthy: true,
//         };

//         // 5. Run Controller
//         let mixer_input = controller.control(&state, &mut state_manager, &command, &params);
        
//         dynamics.update(&mixer_input.torques, DT);
        
//         // 6. Extract Actual Angles for logging
//         let euler = dynamics.orientation.to_euler_angles(); // [roll, pitch, yaw]

//         // 7. Log Data
//         wtr.serialize(SimulationRecord {
//             time_s: t,
//             mode_id: if mode == ControlType::Angle { 1 } else { 0 },
//             cmd_x: cmd_vec[0],
//             cmd_y: cmd_vec[1],
//             cmd_z: cmd_vec[2],
//             act_roll_rad: euler[0],
//             act_pitch_rad: euler[1],
//             act_yaw_rad: euler[2],
//             act_p_rad_s: dynamics.p,
//             act_q_rad_s: dynamics.q,
//             act_r_rad_s: dynamics.r,
//             torque_x: mixer_input.torques[0],
//             torque_y: mixer_input.torques[1],
//             torque_z: mixer_input.torques[2],
//         })?;
//     }

//     wtr.flush()?;
//     println!("Simulation complete.");
//     Ok(())
// }

use std::error::Error;
use csv::WriterBuilder;
use serde::Serialize;

// Import your library components
use rustflight_core::{
    controller::{Controller, quad_controller::{QuadController, Pid, MixerInput}},
    estimator::quad_estimator::AttitudeState,
    command_manager::{CombinedControl, ControlType, ControlChannel},
    state_machine::{StateManager, Event},
    params2::{Params, ParamId, ParamValue},
};

use micro_algebra::stack::{
    quaternion::Quaternion,
    vector::Vector,
};

use libm::{sin, cos, pow, fabs};

const PI: f64 = 3.14159265359;

// ============================================================================
// HELPER STRUCTS AND MOCKS
// ============================================================================

fn create_mock_command() -> CombinedControl {
    CombinedControl {
        stamp_ms: 0,
        qx: ControlChannel { value: 0.0, control_type: ControlType::Rate, active: true },
        qy: ControlChannel { value: 0.0, control_type: ControlType::Rate, active: true },
        qz: ControlChannel { value: 0.0, control_type: ControlType::Rate, active: true },
        fx: ControlChannel { value: 0.0, control_type: ControlType::Throttle, active: true },
        fy: ControlChannel { value: 0.0, control_type: ControlType::Throttle, active: true },
        fz: ControlChannel { value: 0.0, control_type: ControlType::Throttle, active: true },
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
        let q_dot_kin = ((self.izz - self.ixx) * self.r * self.p / self.iyy) + (torques[1] / self.iyy);
        let r_dot = ((self.ixx - self.iyy) * self.p * self.q / self.izz) + (torques[2] / self.izz);

        self.p += p_dot * dt;
        self.q += q_dot_kin * dt;
        self.r += r_dot * dt;

        let omega_q = Quaternion::from_array([0.0, self.p, self.q, self.r]);
        let q_dot = (self.orientation * omega_q) * 0.5; 
        self.orientation = self.orientation + q_dot * dt;
        
        self.orientation.normalize_fill();
    }
}

/// Generates commands.
/// 0.0 - 5.0s: RATE MODE (Sine)
/// 5.0 - 10.0s: ANGLE MODE (Sine)
/// 10.0 - 20.0s: ANGLE MODE (Square Wave)
fn get_commands(t: f64) -> (Vector<f64, 3>, ControlType) {
    if t < 5.0 {
        // --- RATE MODE (0-5s) ---
        let roll_rate = 1.0 * sin(2.0 * PI * 0.5 * t); // +/- 0.5 rad/s
        let pitch_rate = 1.0 * cos(2.0 * PI * 0.5 * t);
        let yaw_rate = 0.4 * sin(2.0 * PI * 0.2 * t);
        (Vector::from_array([roll_rate, pitch_rate, yaw_rate]), ControlType::Rate)
    } else if t < 10.0 {
        // --- ANGLE MODE SINE (5-10s) ---
        let roll_angle = 1.0 * sin(2.0 * PI * 0.2 * (t - 5.0)); 
        let pitch_angle = 1.0 * cos(2.0 * PI * 0.2 * (t - 5.0));
        let yaw_rate = 0.5; 
        (Vector::from_array([roll_angle, pitch_angle, yaw_rate]), ControlType::Angle)
    } else {
        // --- ANGLE MODE SQUARE WAVE (10-20s) ---
        // Pulse every 2.5 seconds
        // t=10..12.5 -> +0.3 rad
        // t=12.5..15 -> -0.3 rad
        // ...
        let cycle_pos = (t - 10.0) % 7.5; // 7.5 second full period
        let magnitude = 0.3;
        
        let roll_angle = if cycle_pos < 2.5 { magnitude } else { -magnitude };
        let pitch_angle = if cycle_pos < 2.5 { -magnitude } else { magnitude }; // Opposite phase
        let yaw_rate = 0.5;

        (Vector::from_array([roll_angle, pitch_angle, yaw_rate]), ControlType::Angle)
    }
}

#[derive(Debug, Serialize)]
struct SimulationRecord {
    time_s: f64,
    mode_id: u8, // 0 = Rate, 1 = Angle
    cmd_x: f64, cmd_y: f64, cmd_z: f64, 
    act_roll_rad: f64, act_pitch_rad: f64, act_yaw_rad: f64, 
    act_p_rad_s: f64, act_q_rad_s: f64, act_r_rad_s: f64,    
    torque_x: f64, torque_y: f64, torque_z: f64,
}

#[test]
fn run_mixed_mode_simulation() -> Result<(), Box<dyn Error>> {
    const SIMULATION_TIME: f64 = 20.0; // Extended to 20s
    const DT: f64 = 0.01;

    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.001)); 

    let mut state_manager = StateManager::new();
    state_manager.update(Event::INITIALIZED, &params);
    state_manager.update(Event::REQUEST_ARM, &params);
    assert!(state_manager.is_armed());

    // TUNED GAINS: Increased to generate realistic torque and tracking
    let mut controller = QuadController::new(
        // Rate PIDs (Higher P for better tracking)
        Pid::new(0.15, 0.05, 0.005, 5.0, 0.05), // roll_rate
        Pid::new(0.15, 0.05, 0.005, 5.0, 0.05), // pitch_rate
        Pid::new(0.20, 0.05, 0.0, 5.0, 0.05),   // yaw_rate
        // Angle PIDs (P gain drives the rate loop)
        Pid::new(4.0, 1.0, 0.0, 10.0, 0.0),      // roll_angle
        Pid::new(4.0, 1.0, 0.0, 10.0, 0.0)       // pitch_angle
    );

    // Standard small quad inertia
    let mut dynamics = QuadcopterDynamics::new(0.007, 0.007, 0.012); 
    
    std::fs::create_dir_all("tests/controller")?;
    let output_path = "tests/controller/rust_controller_results.csv";
    let mut wtr = WriterBuilder::new().from_path(output_path)?;
    println!("\nRunning Mixed Mode simulation (Rate -> Angle Sine -> Angle Square) and writing to '{}'...", output_path);

    let num_steps = (SIMULATION_TIME / DT) as usize;
    let (_, mut last_mode) = get_commands(0.0f64);

    for i in 0..num_steps {
        let t = i as f64 * DT;
        
        let (cmd_vec, mode) = get_commands(t);
        
        let mut command = create_mock_command();
        command.qx.value = cmd_vec[0];
        command.qy.value = cmd_vec[1];
        command.qz.value = cmd_vec[2]; 
        command.fz.value = 0.5; 
        
        command.qx.control_type = mode;
        command.qy.control_type = mode;
        command.qz.control_type = ControlType::Rate; 

        let omega_q = Quaternion::from_array([0.0, dynamics.p, dynamics.q, dynamics.r]);
        let q_dot = (dynamics.orientation * omega_q) * 0.5;

        let state = AttitudeState { 
            q_hat: dynamics.orientation, 
            q_dot: q_dot, 
            b_hat: Vector::zeros(),
            is_healthy: true,
        };

        let mixer_input = controller.control(&state, &mut state_manager, &command, &params);
        
        dynamics.update(&mixer_input.torques, DT);
        
        let euler = dynamics.orientation.to_euler_angles(); 

        wtr.serialize(SimulationRecord {
            time_s: t,
            mode_id: if mode == ControlType::Angle { 1 } else { 0 },
            cmd_x: cmd_vec[0],
            cmd_y: cmd_vec[1],
            cmd_z: cmd_vec[2],
            act_roll_rad: euler[0],
            act_pitch_rad: euler[1],
            act_yaw_rad: euler[2],
            act_p_rad_s: dynamics.p,
            act_q_rad_s: dynamics.q,
            act_r_rad_s: dynamics.r,
            torque_x: mixer_input.torques[0],
            torque_y: mixer_input.torques[1],
            torque_z: mixer_input.torques[2],
        })?;

        if last_mode == ControlType::Rate && mode == ControlType::Angle {
            controller.roll_angle_pid.reset();
            controller.pitch_angle_pid.reset();
        }
        last_mode = mode;
    }

    wtr.flush()?;
    println!("Simulation complete.");
    Ok(())
}