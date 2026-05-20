use super::{Controller, RcTrimCalibrator};
use crate::command::{CombinedControl, ControlType};
use crate::controller::ControllerCtx;
use crate::estimator::quad::AttitudeState;
use crate::params::{ParamId, ParamValue, Params};
use libm::{atan2, cos, pow, sin, sqrt};
use nalgebra::Quaternion;
use nalgebra::SVector as Vector;

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

#[derive(Debug, Clone, Copy)]
pub struct Pid {
    pub p: f64,
    pub i: f64,
    pub d: f64,
    pub max: f64,
    pub min: f64,
    pub tau: f64,
    pub integrator: f64,
    pub differentiator: f64,
    pub prev_x: f64,
    pub prev_t: f64,
}

impl Default for Pid {
    fn default() -> Self {
        Self {
            p: 0.0,
            i: 0.0,
            d: 0.0,
            max: f64::INFINITY,
            min: -f64::INFINITY,
            tau: 0.05,
            integrator: 0.0,
            differentiator: 0.0,
            prev_x: 0.0,
            prev_t: -1.0,
        }
    }
}

impl Pid {
    pub fn new(p: f64, i: f64, d: f64, max_i: f64, tau: f64) -> Self {
        Self {
            p,
            i,
            d,
            max: max_i,
            min: -max_i,
            tau,
            integrator: 0.0,
            differentiator: 0.0,
            prev_x: 0.0,
            prev_t: -1.0,
        }
    }
    pub fn run(&mut self, x: f64, x_c: f64, dt: f64, enable_integrator: bool) -> f64 {
        let xdot = if dt > 0.0001 {
            self.differentiator = (2.0 * self.tau - dt) / (2.0 * self.tau + dt)
                * self.differentiator
                + 2.0 / (2.0 * self.tau + dt) * (x - self.prev_x);
            self.differentiator
        } else {
            0.0
        };
        self.prev_x = x;

        self.run_with_derivative(x, x_c, xdot, dt, enable_integrator)
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

        let p_term = self.p * error;
        let d_term = if self.d > 0.0 { self.d * xdot } else { 0.0 };

        let mut i_term = 0.0;
        if self.i > 0.0 && enable_integrator {
            self.integrator += error * dt;
            i_term = self.i * self.integrator;
        }

        let output = p_term - d_term + i_term;
        let saturated = clamp(output, self.min, self.max);

        if output != saturated && self.i > 0.0 && i_term.abs() > (output - p_term + d_term).abs() {
            self.integrator = (saturated - p_term + d_term) / self.i;
        }

        saturated
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControllerOutput {
    pub u: [f64; 10],
}

impl ControllerOutput {
    pub fn from_forces_torques_and_passthrough(
        forces: Vector<f64, 3>,
        torques: Vector<f64, 3>,
        passthrough: [f64; 4],
    ) -> Self {
        Self {
            u: [
                forces[0],
                forces[1],
                forces[2],
                torques[0],
                torques[1],
                torques[2],
                passthrough[0],
                passthrough[1],
                passthrough[2],
                passthrough[3],
            ],
        }
    }

    pub fn forces(&self) -> Vector<f64, 3> {
        Vector::from([self.u[0], self.u[1], self.u[2]])
    }

    pub fn torques(&self) -> Vector<f64, 3> {
        Vector::from([self.u[3], self.u[4], self.u[5]])
    }

    pub fn passthrough(&self) -> [f64; 4] {
        [self.u[6], self.u[7], self.u[8], self.u[9]]
    }

    pub fn legacy_quad_thrust(&self) -> f64 {
        -self.u[2]
    }
}

impl Default for ControllerOutput {
    fn default() -> Self {
        Self { u: [0.0; 10] }
    }
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
        update_integrators: bool,
        air_density: f64,
    ) -> ControllerOutput {
        let current_rates = state.body_rate;
        let euler: Vector<f64, 3> = state.into();

        let mut torque_x = match command.qx.control_type {
            ControlType::Rate => self.roll_rate_pid.run(
                current_rates[0],
                command.qx.value as f64,
                dt,
                update_integrators,
            ),
            ControlType::Angle => self.roll_angle_pid.run_with_derivative(
                euler[0],
                command.qx.value as f64,
                current_rates[0],
                dt,
                update_integrators,
            ),
            _ => command.qx.value as f64,
        };

        let mut torque_y = match command.qy.control_type {
            ControlType::Rate => self.pitch_rate_pid.run(
                current_rates[1],
                command.qy.value as f64,
                dt,
                update_integrators,
            ),
            ControlType::Angle => self.pitch_angle_pid.run_with_derivative(
                euler[1],
                command.qy.value as f64,
                current_rates[1],
                dt,
                update_integrators,
            ),
            _ => command.qy.value as f64,
        };

        let mut torque_z = match command.qz.control_type {
            ControlType::Rate => self.yaw_rate_pid.run(
                current_rates[2],
                command.qz.value as f64,
                dt,
                update_integrators,
            ),
            _ => command.qz.value as f64,
        };

        if add_equilibrium_torques {
            torque_x += param_float(params, ParamId::PARAM_X_EQ_TORQUE) as f64;
            torque_y += param_float(params, ParamId::PARAM_Y_EQ_TORQUE) as f64;
            torque_z += param_float(params, ParamId::PARAM_Z_EQ_TORQUE) as f64;
        }

        let forces = Vector::from([
            force_output(
                command.fx.value as f64,
                command.fx.control_type,
                params,
                false,
                air_density,
            ),
            force_output(
                command.fy.value as f64,
                command.fy.control_type,
                params,
                false,
                air_density,
            ),
            force_output(
                command.fz.value as f64,
                command.fz.control_type,
                params,
                true,
                air_density,
            ),
        ]);

        ControllerOutput::from_forces_torques_and_passthrough(
            forces,
            Vector::from([torque_x, torque_y, torque_z]),
            [
                command.passthrough[0].value as f64,
                command.passthrough[1].value as f64,
                command.passthrough[2].value as f64,
                command.passthrough[3].value as f64,
            ],
        )
    }
}

impl Controller for QuadController {
    type State = AttitudeState;
    type ControlOutput = ControllerOutput;

    fn update_gains(&mut self, params: &Params) {
        // Roll Rate
        self.roll_rate_pid.p = match params.get_by_id(ParamId::PARAM_PID_ROLL_RATE_P) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.roll_rate_pid.i = match params.get_by_id(ParamId::PARAM_PID_ROLL_RATE_I) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.roll_rate_pid.d = match params.get_by_id(ParamId::PARAM_PID_ROLL_RATE_D) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.roll_rate_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };

        // Pitch Rate
        self.pitch_rate_pid.p = match params.get_by_id(ParamId::PARAM_PID_PITCH_RATE_P) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.pitch_rate_pid.i = match params.get_by_id(ParamId::PARAM_PID_PITCH_RATE_I) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.pitch_rate_pid.d = match params.get_by_id(ParamId::PARAM_PID_PITCH_RATE_D) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.pitch_rate_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };

        // Yaw Rate
        self.yaw_rate_pid.p = match params.get_by_id(ParamId::PARAM_PID_YAW_RATE_P) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.yaw_rate_pid.i = match params.get_by_id(ParamId::PARAM_PID_YAW_RATE_I) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.yaw_rate_pid.d = match params.get_by_id(ParamId::PARAM_PID_YAW_RATE_D) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.yaw_rate_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };

        // Roll Angle
        self.roll_angle_pid.p = match params.get_by_id(ParamId::PARAM_PID_ROLL_ANGLE_P) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.roll_angle_pid.i = match params.get_by_id(ParamId::PARAM_PID_ROLL_ANGLE_I) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.roll_angle_pid.d = match params.get_by_id(ParamId::PARAM_PID_ROLL_ANGLE_D) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.roll_angle_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };

        // Pitch Angle
        self.pitch_angle_pid.p = match params.get_by_id(ParamId::PARAM_PID_PITCH_ANGLE_P) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.pitch_angle_pid.i = match params.get_by_id(ParamId::PARAM_PID_PITCH_ANGLE_I) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.pitch_angle_pid.d = match params.get_by_id(ParamId::PARAM_PID_PITCH_ANGLE_D) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
        self.pitch_angle_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => val as f64,
            _ => 0.0,
        };
    }

    fn control(&mut self, state: &Self::State, ctx: ControllerCtx<'_>) -> Self::ControlOutput {
        self.update_gains(ctx.params);

        let update_integrators = ctx.state_manager.is_armed()
            && controller_should_update_integrators(ctx.command, ctx.dt);
        self.run_pid_control(
            state,
            ctx.command,
            ctx.params,
            ctx.dt,
            true,
            update_integrators,
            ctx.air_density,
        )
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
        let output = controller.run_pid_control(
            &AttitudeState::default(),
            rc_control,
            params,
            0.0,
            false,
            false,
            1.225,
        );

        [output.u[3] as f32, output.u[4] as f32, output.u[5] as f32]
    }
}

fn param_float(params: &Params, id: ParamId) -> f32 {
    match params.get_by_id(id) {
        ParamValue::Float(value) => value,
        _ => 0.0,
    }
}

fn param_int(params: &Params, id: ParamId) -> i32 {
    match params.get_by_id(id) {
        ParamValue::Int(value) => value,
        _ => 0,
    }
}

fn force_output(
    value: f64,
    control_type: ControlType,
    params: &Params,
    is_fz: bool,
    air_density: f64,
) -> f64 {
    if control_type != ControlType::Throttle {
        return value;
    }

    let sign = if is_fz { -1.0 } else { 1.0 };
    let mut output = sign * value * param_float(params, ParamId::PARAM_RC_MAX_THROTTLE) as f64;

    if param_int(params, ParamId::PARAM_USE_MOTOR_PARAMETERS) != 0 {
        output *= calculate_max_thrust(params, air_density);
    }

    output
}

fn calculate_max_thrust(params: &Params, air_density: f64) -> f64 {
    let resistance = param_float(params, ParamId::PARAM_MOTOR_RESISTANCE) as f64;
    let diameter = param_float(params, ParamId::PARAM_PROP_DIAMETER) as f64;
    let cq = param_float(params, ParamId::PARAM_PROP_CQ) as f64;
    let ct = param_float(params, ParamId::PARAM_PROP_CT) as f64;
    let kv = param_float(params, ParamId::PARAM_MOTOR_KV) as f64;
    let no_load_current = param_float(params, ParamId::PARAM_NO_LOAD_CURRENT) as f64;
    let num_motors = param_int(params, ParamId::PARAM_NUM_MOTORS) as f64;
    let max_voltage = param_float(params, ParamId::PARAM_VOLT_MAX) as f64;

    let a = resistance * air_density * pow(diameter, 5.0) * cq
        / (4.0 * pow(core::f64::consts::PI, 2.0) * kv);
    let b = kv;
    let c = no_load_current * resistance - max_voltage;
    let omega = (-b + sqrt(pow(b, 2.0) - 4.0 * a * c)) / (2.0 * a);

    air_density * pow(diameter, 4.0) * ct * pow(omega, 2.0)
        / (4.0 * pow(core::f64::consts::PI, 2.0))
        * num_motors
}

fn controller_should_update_integrators(command: &CombinedControl, dt: f64) -> bool {
    dt < 0.01 && (command.fx.value > 0.1 || command.fy.value > 0.1 || command.fz.value > 0.1)
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

    Quaternion::new(
        cr * cp * cy + sr * sp * sy, // w
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy, // z
    )
}

/// Extracts Yaw (Z-axis rotation) from a Quaternion
pub fn get_yaw(q: Quaternion<f64>) -> f64 {
    let w = q.w;
    let x = q.i;
    let y = q.j;
    let z = q.k;

    // Use libm::atan2 instead of f64::atan2
    atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        command::{CombinedControl, ControlChannel, ControlType},
        state_machine::{Event, StateManager},
    };

    fn armed_state(params: &Params) -> StateManager {
        let mut state_manager = StateManager::new();
        state_manager.update(Event::INITIALIZED, params);
        state_manager.update_arming_safety(true, true);
        state_manager.update(Event::REQUEST_ARM, params);
        state_manager
    }

    fn control_with_density(
        controller: &mut QuadController,
        state: &AttitudeState,
        state_manager: &mut StateManager,
        command: &CombinedControl,
        params: &Params,
        dt: f64,
        air_density: f64,
    ) -> ControllerOutput {
        controller.control(
            state,
            ControllerCtx {
                state_manager,
                command,
                params,
                air_density,
                dt,
            },
        )
    }

    #[test]
    fn controller_adds_equilibrium_torque_params_to_control_output() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_X_EQ_TORQUE, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_Y_EQ_TORQUE, ParamValue::Float(-0.2));
        params.set_by_id(ParamId::PARAM_Z_EQ_TORQUE, ParamValue::Float(0.3));
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));

        let mut state_manager = armed_state(&params);

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

        let output = control_with_density(
            &mut controller,
            &state,
            &mut state_manager,
            &command,
            &params,
            0.0025,
            1.225,
        );

        assert_eq!(output.u[3], 0.10000000149011612);
        assert_eq!(output.u[4], -0.20000000298023224);
        assert_eq!(output.u[5], 0.30000001192092896);
        assert!((output.u[2] + 0.28).abs() < 1e-6);
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

    #[test]
    fn controller_output_preserves_rosflight_ten_channel_shape() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));

        let mut state_manager = armed_state(&params);
        let mut controller = QuadController::default();
        let state = AttitudeState::default();
        let command = CombinedControl {
            qx: ControlChannel {
                active: true,
                control_type: ControlType::Passthrough,
                value: 0.1,
            },
            qy: ControlChannel {
                active: true,
                control_type: ControlType::Passthrough,
                value: 0.2,
            },
            qz: ControlChannel {
                active: true,
                control_type: ControlType::Passthrough,
                value: 0.3,
            },
            fx: ControlChannel {
                active: true,
                control_type: ControlType::Passthrough,
                value: 0.4,
            },
            fy: ControlChannel {
                active: true,
                control_type: ControlType::Passthrough,
                value: 0.5,
            },
            fz: ControlChannel {
                active: true,
                control_type: ControlType::Passthrough,
                value: 0.6,
            },
            passthrough: [
                ControlChannel {
                    active: true,
                    control_type: ControlType::Passthrough,
                    value: 0.7,
                },
                ControlChannel {
                    active: true,
                    control_type: ControlType::Passthrough,
                    value: 0.8,
                },
                ControlChannel {
                    active: true,
                    control_type: ControlType::Passthrough,
                    value: 0.9,
                },
                ControlChannel {
                    active: true,
                    control_type: ControlType::Passthrough,
                    value: 1.0,
                },
            ],
            stamp_ms: 0,
        };

        let output = control_with_density(
            &mut controller,
            &state,
            &mut state_manager,
            &command,
            &params,
            0.0025,
            1.225,
        );

        assert_eq!(output.u, [0.4, 0.5, 0.6, 0.1, 0.2, 0.3, 0.7, 0.8, 0.9, 1.0]);
    }

    #[test]
    fn pid_integrator_updates_only_when_rosflight_gate_allows_it() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_PID_ROLL_RATE_P, ParamValue::Float(0.0));
        params.set_by_id(ParamId::PARAM_PID_ROLL_RATE_I, ParamValue::Float(2.0));
        params.set_by_id(ParamId::PARAM_PID_ROLL_RATE_D, ParamValue::Float(0.0));

        let state = AttitudeState::default();
        let command = CombinedControl {
            qx: ControlChannel {
                active: true,
                control_type: ControlType::Rate,
                value: 1.0,
            },
            fz: ControlChannel {
                active: true,
                control_type: ControlType::Throttle,
                value: 0.2,
            },
            ..Default::default()
        };

        let mut gated_out_state = armed_state(&params);
        let mut gated_out_controller = QuadController::default();
        let gated_out = control_with_density(
            &mut gated_out_controller,
            &state,
            &mut gated_out_state,
            &command,
            &params,
            0.02,
            1.225,
        );
        assert_eq!(gated_out.u[3], 0.0);

        let mut gated_in_state = armed_state(&params);
        let mut gated_in_controller = QuadController::default();
        let gated_in = control_with_density(
            &mut gated_in_controller,
            &state,
            &mut gated_in_state,
            &command,
            &params,
            0.005,
            1.225,
        );

        assert!((gated_in.u[3] - 0.01).abs() < 1e-9);
    }

    #[test]
    fn pid_derivative_integrator_and_saturation_match_rosflight_trace() {
        let mut pid = Pid::new(2.0, 3.0, 0.5, 0.25, 0.05);

        let first = pid.run(1.0, 3.0, 0.01, true);
        assert!((pid.differentiator - 18.1818181818).abs() < 1e-9);
        assert!((pid.integrator - 0.02).abs() < 1e-9);
        assert_eq!(first, -0.25);

        let second = pid.run(1.2, 3.0, 0.01, true);
        assert!((pid.differentiator - 18.5123966942).abs() < 1e-9);
        assert!((pid.integrator - 1.8020661157).abs() < 1e-9);
        assert_eq!(second, -0.25);

        let held_integrator = pid.integrator;
        let disabled = pid.run(1.2, 3.0, 0.01, false);
        assert_eq!(pid.integrator, held_integrator);
        assert_eq!(disabled, -0.25);
    }

    #[test]
    fn angle_mode_controller_trace_uses_body_rate_as_derivative_feedback() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_PID_ROLL_ANGLE_P, ParamValue::Float(4.0));
        params.set_by_id(ParamId::PARAM_PID_ROLL_ANGLE_I, ParamValue::Float(0.0));
        params.set_by_id(ParamId::PARAM_PID_ROLL_ANGLE_D, ParamValue::Float(0.5));
        params.set_by_id(ParamId::PARAM_PID_PITCH_ANGLE_P, ParamValue::Float(3.0));
        params.set_by_id(ParamId::PARAM_PID_PITCH_ANGLE_I, ParamValue::Float(0.0));
        params.set_by_id(ParamId::PARAM_PID_PITCH_ANGLE_D, ParamValue::Float(0.25));
        params.set_by_id(ParamId::PARAM_PID_YAW_RATE_P, ParamValue::Float(2.0));
        params.set_by_id(ParamId::PARAM_PID_YAW_RATE_I, ParamValue::Float(0.0));
        params.set_by_id(ParamId::PARAM_PID_YAW_RATE_D, ParamValue::Float(0.0));

        let mut state_manager = armed_state(&params);
        let mut controller = QuadController::default();
        let state = AttitudeState {
            q_hat: quaternion_from_euler(0.1, -0.2, 0.0),
            body_rate: Vector::from([0.3, -0.4, 0.5]),
            is_healthy: true,
            ..Default::default()
        };
        let command = CombinedControl {
            qx: ControlChannel {
                active: true,
                control_type: ControlType::Angle,
                value: 0.2,
            },
            qy: ControlChannel {
                active: true,
                control_type: ControlType::Angle,
                value: -0.1,
            },
            qz: ControlChannel {
                active: true,
                control_type: ControlType::Rate,
                value: 0.8,
            },
            fz: ControlChannel {
                active: true,
                control_type: ControlType::Throttle,
                value: 0.4,
            },
            ..Default::default()
        };

        let output = control_with_density(
            &mut controller,
            &state,
            &mut state_manager,
            &command,
            &params,
            0.005,
            1.225,
        );

        assert!((output.u[3] - 0.25).abs() < 1e-9);
        assert!((output.u[4] - 0.4).abs() < 1e-9);
        assert!((output.u[5] - 0.6).abs() < 1e-9);
        assert!((output.u[2] + 0.28).abs() < 1e-6);
    }

    #[test]
    fn motor_param_thrust_scaling_uses_controller_air_density_context() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_USE_MOTOR_PARAMETERS, ParamValue::Int(1));

        let state = AttitudeState::default();
        let command = CombinedControl {
            fz: ControlChannel {
                active: true,
                control_type: ControlType::Throttle,
                value: 0.4,
            },
            ..Default::default()
        };

        let mut lower_density_state = armed_state(&params);
        let mut lower_density_controller = QuadController::default();
        let lower_density_output = control_with_density(
            &mut lower_density_controller,
            &state,
            &mut lower_density_state,
            &command,
            &params,
            0.005,
            1.0,
        );

        let mut higher_density_state = armed_state(&params);
        let mut higher_density_controller = QuadController::default();
        let higher_density_output = control_with_density(
            &mut higher_density_controller,
            &state,
            &mut higher_density_state,
            &command,
            &params,
            0.005,
            1.3,
        );

        assert!(higher_density_output.u[2] < lower_density_output.u[2]);
    }
}
