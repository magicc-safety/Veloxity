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

use super::{Controller, RcTrimCalibrator};
use crate::command_manager::{CombinedControl, ControlType};
use crate::estimator::quad_estimator::AttitudeState;
use crate::params::{ParamId, ParamValue, Params};
use crate::state_machine::StateManager;
use libm::{atan2, cos, sin};
use micro_algebra::stack::quaternion::Quaternion;
use micro_algebra::stack::vector::Vector;

// DT is now passed as a parameter from the main loop (matches C implementation)

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
    pub p: f64,
    pub i: f64,
    pub d: f64,
    pub max_i: f64,
    pub tau: f64,
    pub integrator: f64,
    pub differentiator: f64,
    pub prev_x: f64,
    pub prev_t: f64,
}

impl Pid {
    pub fn new(p: f64, i: f64, d: f64, max_i: f64, tau: f64) -> Self {
        Self {
            p,
            i,
            d,
            max_i,
            tau,
            integrator: 0.0,
            differentiator: 0.0,
            prev_x: 0.0,
            prev_t: -1.0,
        }
    }
    pub fn run(&mut self, x: f64, x_c: f64, dt: f64, enable_integrator: bool) -> f64 {
        let error = x_c - x;

        // P Term
        let p_term = self.p * error;

        // I Term (commented out - not currently used)
        // if enable_integrator {
        //     self.integrator = clamp(self.integrator + error * dt, -self.max_i, self.max_i);
        // }
        // let i_term = self.i * self.integrator;

        // D Term using β-filter (dirty derivative from PDF Chapter 10, pages 11-12)
        // PDF notation mapping:
        //   dt ≡ Ts (sample time)
        //   self.tau ≡ σ (filter time constant)
        //   x ≡ ξ[n] (current state)
        //   self.prev_x ≡ ξ[n-1] (previous state)
        //   self.differentiator ≡ u_D[n] (filtered derivative output)
        let d_term = if self.prev_t < 0.0 {
            // First call - initialize
            self.prev_x = x;
            self.prev_t = 0.0;
            0.0
        } else {
            // β = (2σ - Ts) / (2σ + Ts)
            let beta = (2.0 * self.tau - dt) / (2.0 * self.tau + dt);

            // (1 - β) = 2*Ts / (2σ + Ts)
            let one_minus_beta = 1.0 - beta;

            // (ξ[n] - ξ[n-1]) / Ts
            let derivative_estimate = (x - self.prev_x) / dt;

            // u_D[n] = β * u_D[n-1] + (1-β) * ((ξ[n] - ξ[n-1]) / Ts)
            self.differentiator = beta * self.differentiator + one_minus_beta * derivative_estimate;
            self.prev_x = x;

            self.d * self.differentiator
        };

        // Output (no I term currently)
        p_term - d_term
    }

    pub fn run_with_derivative(
        &mut self,
        x: f64,
        x_c: f64,
        xdot: f64,
        dt: f64,
        enable_integrator: bool,
    ) -> f64 {
        let error = x_c - x;

        // P Term
        let p_term = self.p * error;

        // I Term
        //if enable_integrator {
        //    self.integrator = clamp(self.integrator + error * dt, -self.max_i, self.max_i);
        //}
        //let i_term = self.i * self.integrator;

        let d_term = self.d * xdot;

        //p_term + i_term - d_term
        p_term - d_term
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
    pub roll_rate_pid: Pid,
    pub pitch_rate_pid: Pid,
    pub yaw_rate_pid: Pid,
    pub roll_angle_pid: Pid,
    pub pitch_angle_pid: Pid,
}

impl QuadController {
    pub fn new(
        roll_rate_pid: Pid,
        pitch_rate_pid: Pid,
        yaw_rate_pid: Pid,
        roll_angle_pid: Pid,
        pitch_angle_pid: Pid,
    ) -> Self {
        Self {
            roll_rate_pid,
            pitch_rate_pid,
            yaw_rate_pid,
            roll_angle_pid,
            pitch_angle_pid,
        }
    }

    fn reset_pids(&mut self) {
        self.roll_rate_pid.reset();
        self.pitch_rate_pid.reset();
        self.yaw_rate_pid.reset();
        self.roll_angle_pid.reset();
        self.pitch_angle_pid.reset();
    }

    fn run_pid_control(
        &mut self,
        state: &AttitudeState,
        command: &CombinedControl,
        params: &Params,
        dt: f64,
        add_equilibrium_torques: bool,
    ) -> MixerInput {
        if command.qx.control_type == ControlType::Passthrough {
            return MixerInput {
                torques: Vector::from_array([
                    command.qx.value as f64,
                    command.qy.value as f64,
                    command.qy.value as f64,
                ]),
                thrust: command.fz.value as f64,
            };
        }

        let enable_integrator = false;
        let q_hat = state.q_hat;
        let current_rates = state.body_rate;
        let mut rate_setpoints = Vector::from_array([0.0, 0.0, 0.0, 0.0]);

        if command.qx.control_type == ControlType::Angle {
            let current_yaw = get_yaw(q_hat);
            let q_target = quaternion_from_euler(
                command.qx.value as f64,
                command.qy.value as f64,
                current_yaw,
            );

            let mut q_err = q_hat.conjugate() * q_target;

            if q_err.get_w() < 0.0 {
                q_err = q_err * -1.0;
            }

            rate_setpoints[0] = self.roll_angle_pid.run_with_derivative(
                0.0,
                2.0 * q_err.get_x(),
                current_rates[0],
                dt,
                enable_integrator,
            );
            rate_setpoints[1] = self.pitch_angle_pid.run_with_derivative(
                0.0,
                2.0 * q_err.get_y(),
                current_rates[1],
                dt,
                enable_integrator,
            )
        } else {
            rate_setpoints[0] = command.qx.value as f64;
            rate_setpoints[1] = command.qy.value as f64;
        }

        rate_setpoints[2] = command.qz.value as f64;

        let mut torque_x =
            self.roll_rate_pid
                .run(current_rates[0], rate_setpoints[0], dt, enable_integrator);
        let mut torque_y =
            self.pitch_rate_pid
                .run(current_rates[1], rate_setpoints[1], dt, enable_integrator);
        let mut torque_z =
            self.yaw_rate_pid
                .run(current_rates[2], rate_setpoints[2], dt, enable_integrator);

        if add_equilibrium_torques {
            torque_x += param_float(params, ParamId::PARAM_X_EQ_TORQUE) as f64;
            torque_y += param_float(params, ParamId::PARAM_Y_EQ_TORQUE) as f64;
            torque_z += param_float(params, ParamId::PARAM_Z_EQ_TORQUE) as f64;
        }

        MixerInput {
            torques: Vector::from_array([torque_x, torque_y, torque_z]),
            thrust: command.fz.value as f64,
        }
    }
}

impl Controller for QuadController {
    type State = AttitudeState;
    type ControlOutput = MixerInput;

    fn update_gains(&mut self, params: &Params) {
        // Roll Rate
        self.roll_rate_pid.p = match params.get_by_id(ParamId::PARAM_PID_ROLL_RATE_P) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };
        self.roll_rate_pid.i = match params.get_by_id(ParamId::PARAM_PID_ROLL_RATE_I) {
            // ParamValue::Float(val) => val as f64,
            // other => {
            _ => 0.0,
        };
        self.roll_rate_pid.d = match params.get_by_id(ParamId::PARAM_PID_ROLL_RATE_D) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };
        self.roll_rate_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };

        // Pitch Rate
        self.pitch_rate_pid.p = match params.get_by_id(ParamId::PARAM_PID_PITCH_RATE_P) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.pitch_rate_pid.i = match params.get_by_id(ParamId::PARAM_PID_PITCH_RATE_I) {
            // ParamValue::Float(val) => val as f64,
            // other => {
            _ => 0.0,
        };
        self.pitch_rate_pid.d = match params.get_by_id(ParamId::PARAM_PID_PITCH_RATE_D) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };
        self.pitch_rate_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };

        // Yaw Rate
        self.yaw_rate_pid.p = match params.get_by_id(ParamId::PARAM_PID_YAW_RATE_P) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };
        self.yaw_rate_pid.i = match params.get_by_id(ParamId::PARAM_PID_YAW_RATE_I) {
            // ParamValue::Float(val) => val as f64,
            // other => {
            _ => 0.0,
        };
        self.yaw_rate_pid.d = match params.get_by_id(ParamId::PARAM_PID_YAW_RATE_D) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };
        self.yaw_rate_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };

        // Roll Angle
        self.roll_angle_pid.p = match params.get_by_id(ParamId::PARAM_PID_ROLL_ANGLE_P) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };
        self.roll_angle_pid.i = match params.get_by_id(ParamId::PARAM_PID_ROLL_ANGLE_I) {
            // ParamValue::Float(val) => val as f64,
            // other => {
            _ => 0.0,
        };
        self.roll_angle_pid.d = match params.get_by_id(ParamId::PARAM_PID_ROLL_ANGLE_D) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };
        self.roll_angle_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };

        // Pitch Angle
        self.pitch_angle_pid.p = match params.get_by_id(ParamId::PARAM_PID_PITCH_ANGLE_P) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };
        self.pitch_angle_pid.i = match params.get_by_id(ParamId::PARAM_PID_PITCH_ANGLE_I) {
            // ParamValue::Float(val) => val as f64,
            // other => {
            _ => 0.0,
        };
        self.pitch_angle_pid.d = match params.get_by_id(ParamId::PARAM_PID_PITCH_ANGLE_D) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };
        self.pitch_angle_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => val as f64,
            other => 0.0,
        };
    }

    fn control(
        &mut self,
        state: &Self::State,
        state_manager: &mut StateManager,
        command: &CombinedControl,
        params: &Params,
        dt: f64,
    ) -> Self::ControlOutput {
        self.update_gains(params);

        if !state_manager.is_armed() {
            self.reset_pids();

            MixerInput {
                torques: Vector::from_array([0.0f64, 0.0f64, 0.0f64]),
                thrust: 0.0f64,
            }
        } else {
            self.run_pid_control(state, command, params, dt, true)
        }
    }
}

impl RcTrimCalibrator for QuadController {
    fn calculate_equilibrium_torques_from_rc(
        &mut self,
        rc_control: &CombinedControl,
        params: &Params,
    ) -> [f32; 3] {
        let mut controller = *self;
        controller.update_gains(params);
        controller.reset_pids();
        let output =
            controller.run_pid_control(&AttitudeState::default(), rc_control, params, 0.0, false);

        [
            output.torques[0] as f32,
            output.torques[1] as f32,
            output.torques[2] as f32,
        ]
    }
}

fn param_float(params: &Params, id: ParamId) -> f32 {
    match params.get_by_id(id) {
        ParamValue::Float(value) => value,
        _ => 0.0,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        command_manager::{CombinedControl, ControlChannel, ControlType},
        state_machine::Event,
    };

    #[test]
    fn controller_adds_equilibrium_torque_params_to_control_output() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_X_EQ_TORQUE, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_Y_EQ_TORQUE, ParamValue::Float(-0.2));
        params.set_by_id(ParamId::PARAM_Z_EQ_TORQUE, ParamValue::Float(0.3));
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));

        let mut state_manager = StateManager::new();
        state_manager.update(Event::INITIALIZED, &params);
        state_manager.update(Event::REQUEST_ARM, &params);

        let mut controller = QuadController::default();
        let state = AttitudeState::default();
        let command = CombinedControl {
            qx: ControlChannel {
                active: true,
                control_type: ControlType::Rate,
                value: 0.0,
            },
            qy: ControlChannel {
                active: true,
                control_type: ControlType::Rate,
                value: 0.0,
            },
            qz: ControlChannel {
                active: true,
                control_type: ControlType::Rate,
                value: 0.0,
            },
            fz: ControlChannel {
                active: true,
                control_type: ControlType::Throttle,
                value: 0.4,
            },
            ..Default::default()
        };

        let output = controller.control(&state, &mut state_manager, &command, &params, 0.0025);

        assert_eq!(output.torques[0], 0.10000000149011612);
        assert_eq!(output.torques[1], -0.20000000298023224);
        assert_eq!(output.torques[2], 0.30000001192092896);
        assert_eq!(output.thrust, 0.4);
    }

    #[test]
    fn rc_trim_calibration_uses_pid_output_without_existing_equilibrium_torques() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_PID_ROLL_RATE_P, ParamValue::Float(2.0));
        params.set_by_id(ParamId::PARAM_PID_PITCH_RATE_P, ParamValue::Float(3.0));
        params.set_by_id(ParamId::PARAM_PID_YAW_RATE_P, ParamValue::Float(4.0));
        params.set_by_id(ParamId::PARAM_X_EQ_TORQUE, ParamValue::Float(0.5));
        params.set_by_id(ParamId::PARAM_Y_EQ_TORQUE, ParamValue::Float(-0.5));
        params.set_by_id(ParamId::PARAM_Z_EQ_TORQUE, ParamValue::Float(0.25));

        let command = CombinedControl {
            qx: ControlChannel {
                active: true,
                control_type: ControlType::Rate,
                value: 0.1,
            },
            qy: ControlChannel {
                active: true,
                control_type: ControlType::Rate,
                value: -0.1,
            },
            qz: ControlChannel {
                active: true,
                control_type: ControlType::Rate,
                value: 0.2,
            },
            fz: ControlChannel {
                active: true,
                control_type: ControlType::Throttle,
                value: 0.4,
            },
            ..Default::default()
        };

        let mut controller = QuadController::default();
        let torques = controller.calculate_equilibrium_torques_from_rc(&command, &params);

        assert_eq!(torques[0], 0.2);
        assert_eq!(torques[1], -0.3);
        assert_eq!(torques[2], 0.8);
    }
}
