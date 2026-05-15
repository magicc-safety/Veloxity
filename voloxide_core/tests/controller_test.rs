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
use csv::WriterBuilder;
use serde::Serialize;
use std::error::Error;

// Import your library components
use voloxide_core::{
    command_manager::{CombinedControl, ControlChannel, ControlType},
    controller::{Controller, ControllerCtx, quad_controller::QuadController},
    estimator::quad_estimator::AttitudeState,
    params::{ParamId, ParamValue, Params},
    state_machine::{Event, StateManager},
};

use micro_algebra::stack::{quaternion::Quaternion, vector::Vector};

use libm::{cos, fabs, pow, sin};

const PI: f64 = 3.14159265359;

// ============================================================================
// HELPER STRUCTS AND MOCKS
// ============================================================================

fn create_mock_command() -> CombinedControl {
    CombinedControl {
        stamp_ms: 0,
        qx: ControlChannel {
            value: 0.0,
            control_type: ControlType::Rate,
            active: true,
        },
        qy: ControlChannel {
            value: 0.0,
            control_type: ControlType::Rate,
            active: true,
        },
        qz: ControlChannel {
            value: 0.0,
            control_type: ControlType::Rate,
            active: true,
        },
        fx: ControlChannel {
            value: 0.0,
            control_type: ControlType::Throttle,
            active: true,
        },
        fy: ControlChannel {
            value: 0.0,
            control_type: ControlType::Throttle,
            active: true,
        },
        fz: ControlChannel {
            value: 0.0,
            control_type: ControlType::Throttle,
            active: true,
        },
        passthrough: Default::default(),
    }
}

// ============================================================================
// SIMULATION SPECIFIC HELPERS
// ============================================================================

struct QuadcopterDynamics {
    p: f64,
    q: f64,
    r: f64,
    orientation: Quaternion<f64>,
    ixx: f64,
    iyy: f64,
    izz: f64,
}

impl QuadcopterDynamics {
    fn new(ixx: f64, iyy: f64, izz: f64) -> Self {
        Self {
            p: 0.0,
            q: 0.0,
            r: 0.0,
            orientation: Quaternion::from_array([1.0, 0.0, 0.0, 0.0]),
            ixx,
            iyy,
            izz,
        }
    }

    fn update(&mut self, torques: &Vector<f64, 3>, dt: f64) {
        let p_dot = ((self.iyy - self.izz) * self.q * self.r / self.ixx) + (torques[0] / self.ixx);
        let q_dot_kin =
            ((self.izz - self.ixx) * self.r * self.p / self.iyy) + (torques[1] / self.iyy);
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
        (
            Vector::from_array([roll_rate, pitch_rate, yaw_rate]),
            ControlType::Rate,
        )
    } else if t < 10.0 {
        // --- ANGLE MODE SINE (5-10s) ---
        let roll_angle = 1.0 * sin(2.0 * PI * 0.2 * (t - 5.0));
        let pitch_angle = 1.0 * cos(2.0 * PI * 0.2 * (t - 5.0));
        let yaw_rate = 0.5;
        (
            Vector::from_array([roll_angle, pitch_angle, yaw_rate]),
            ControlType::Angle,
        )
    } else {
        // --- ANGLE MODE SQUARE WAVE (10-20s) ---
        // Pulse every 2.5 seconds
        // t=10..12.5 -> +0.3 rad
        // t=12.5..15 -> -0.3 rad
        // ...
        let cycle_pos = (t - 10.0) % 7.5; // 7.5 second full period
        let magnitude = 0.3;

        let roll_angle = if cycle_pos < 2.5 {
            magnitude
        } else {
            -magnitude
        };
        let pitch_angle = if cycle_pos < 2.5 {
            -magnitude
        } else {
            magnitude
        }; // Opposite phase
        let yaw_rate = 0.5;

        (
            Vector::from_array([roll_angle, pitch_angle, yaw_rate]),
            ControlType::Angle,
        )
    }
}

#[derive(Debug, Serialize)]
struct SimulationRecord {
    time_s: f64,
    mode_id: u8, // 0 = Rate, 1 = Angle
    cmd_x: f64,
    cmd_y: f64,
    cmd_z: f64,
    act_roll_rad: f64,
    act_pitch_rad: f64,
    act_yaw_rad: f64,
    act_p_rad_s: f64,
    act_q_rad_s: f64,
    act_r_rad_s: f64,
    torque_x: f64,
    torque_y: f64,
    torque_z: f64,
}

#[test]
fn run_mixed_mode_simulation() -> Result<(), Box<dyn Error>> {
    const SIMULATION_TIME: f64 = 20.0; // Extended to 20s
    const DT: f64 = 1.0 / 400.0;

    let mut params = Params::new();
    params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.001));

    // Rate Parameters
    params.set_by_id(ParamId::PARAM_PID_ROLL_RATE_P, ParamValue::Float(0.15));
    params.set_by_id(ParamId::PARAM_PID_ROLL_RATE_I, ParamValue::Float(0.05));
    params.set_by_id(ParamId::PARAM_PID_ROLL_RATE_D, ParamValue::Float(0.005));

    params.set_by_id(ParamId::PARAM_PID_PITCH_RATE_P, ParamValue::Float(0.15));
    params.set_by_id(ParamId::PARAM_PID_PITCH_RATE_I, ParamValue::Float(0.05));
    params.set_by_id(ParamId::PARAM_PID_PITCH_RATE_D, ParamValue::Float(0.005));

    params.set_by_id(ParamId::PARAM_PID_YAW_RATE_P, ParamValue::Float(0.15));
    params.set_by_id(ParamId::PARAM_PID_YAW_RATE_I, ParamValue::Float(0.05));
    params.set_by_id(ParamId::PARAM_PID_YAW_RATE_D, ParamValue::Float(0.005));

    // Angle Parameters
    params.set_by_id(ParamId::PARAM_PID_ROLL_ANGLE_P, ParamValue::Float(6.0)); // The tuned value!
    params.set_by_id(ParamId::PARAM_PID_ROLL_ANGLE_I, ParamValue::Float(0.5));
    params.set_by_id(ParamId::PARAM_PID_ROLL_ANGLE_D, ParamValue::Float(0.0));

    params.set_by_id(ParamId::PARAM_PID_PITCH_ANGLE_P, ParamValue::Float(6.0)); // The tuned value!
    params.set_by_id(ParamId::PARAM_PID_PITCH_ANGLE_I, ParamValue::Float(0.5));
    params.set_by_id(ParamId::PARAM_PID_PITCH_ANGLE_D, ParamValue::Float(0.0));

    let mut state_manager = StateManager::new();
    state_manager.update(Event::INITIALIZED, &params);
    state_manager.update(Event::REQUEST_ARM, &params);
    assert!(state_manager.is_armed());

    let mut controller = QuadController::default();

    // Standard small quad inertia
    let mut dynamics = QuadcopterDynamics::new(0.007, 0.007, 0.012);

    std::fs::create_dir_all("tests/controller")?;
    let output_path = "tests/controller/voloxide_controller_results.csv";
    let mut wtr = WriterBuilder::new().from_path(output_path)?;
    println!(
        "\nRunning Mixed Mode simulation (Rate -> Angle Sine -> Angle Square) and writing to '{}'...",
        output_path
    );

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

        // This q_dot is correct for PHYSICS simulation (updating orientation)
        let omega_q = Quaternion::from_array([0.0, dynamics.p, dynamics.q, dynamics.r]);
        let q_dot = (dynamics.orientation * omega_q) * 0.5;

        // Initialize state with the new 'body_rates' field
        let state = AttitudeState {
            q_hat: dynamics.orientation,
            q_dot: q_dot,
            b_hat: Vector::zeros(),
            // --- NEW: Pass the CLEAN rates directly from the physics simulation ---
            body_rate: Vector::from_array([dynamics.p, dynamics.q, dynamics.r]),
            // --------------------------------------------------------------------
            is_healthy: true,
        };

        let mixer_input = controller.control(
            &state,
            ControllerCtx {
                state_manager: &mut state_manager,
                command: &command,
                params: &params,
                air_density: 1.225,
                dt: DT,
            },
        );

        let torques = mixer_input.torques();
        dynamics.update(&torques, DT);

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
            torque_x: torques[0],
            torque_y: torques[1],
            torque_z: torques[2],
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
