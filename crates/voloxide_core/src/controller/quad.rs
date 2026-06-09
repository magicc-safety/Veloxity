use super::{Controller, RcTrimCalibrator};
use crate::command::{CombinedControl, ControlType};
use crate::controller::ControllerCtx;
use crate::estimator::quad::AttitudeState;
use crate::math::{FlightFloat, pi};
use crate::params::{ParamId, ParamValue, Params};
use nalgebra::Quaternion;
use nalgebra::SVector as Vector;

/// Clamps a value between a lower and upper bound.
fn r<R: FlightFloat>(value: f32) -> R {
    <R as FlightFloat>::from_f32(value)
}

fn clamp<R: FlightFloat>(value: R, min: R, max: R) -> R {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Pid<R: FlightFloat> {
    pub p: R,
    pub i: R,
    pub d: R,
    pub max: R,
    pub min: R,
    pub tau: R,
    pub integrator: R,
    pub differentiator: R,
    pub prev_x: R,
    pub prev_t: R,
}

impl<R: FlightFloat> Default for Pid<R> {
    fn default() -> Self {
        Self {
            p: r::<R>(0.0),
            i: r::<R>(0.0),
            d: r::<R>(0.0),
            max: R::infinity(),
            min: -R::infinity(),
            tau: r::<R>(0.05),
            integrator: r::<R>(0.0),
            differentiator: r::<R>(0.0),
            prev_x: r::<R>(0.0),
            prev_t: r::<R>(-1.0),
        }
    }
}

impl<R: FlightFloat> Pid<R> {
    pub fn new(p: R, i: R, d: R, max_i: R, tau: R) -> Self {
        Self {
            p,
            i,
            d,
            max: max_i,
            min: -max_i,
            tau,
            integrator: r::<R>(0.0),
            differentiator: r::<R>(0.0),
            prev_x: r::<R>(0.0),
            prev_t: r::<R>(-1.0),
        }
    }
    pub fn run(&mut self, x: R, x_c: R, dt: R, enable_integrator: bool) -> R {
        let xdot = if dt > r::<R>(0.0001) {
            self.differentiator = (r::<R>(2.0) * self.tau - dt) / (r::<R>(2.0) * self.tau + dt)
                * self.differentiator
                + r::<R>(2.0) / (r::<R>(2.0) * self.tau + dt) * (x - self.prev_x);
            self.differentiator
        } else {
            r::<R>(0.0)
        };
        self.prev_x = x;

        self.run_with_derivative(x, x_c, xdot, dt, enable_integrator)
    }

    pub fn run_with_derivative(
        &mut self,
        x: R,
        x_c: R,
        xdot: R,
        dt: R,
        enable_integrator: bool,
    ) -> R {
        let error = x_c - x;

        let p_term = self.p * error;
        let d_term = if self.d > r::<R>(0.0) {
            self.d * xdot
        } else {
            r::<R>(0.0)
        };

        let mut i_term = r::<R>(0.0);
        if self.i > r::<R>(0.0) && enable_integrator {
            self.integrator += error * dt;
            i_term = self.i * self.integrator;
        }

        let output = p_term - d_term + i_term;
        let saturated = clamp(output, self.min, self.max);

        if output != saturated
            && self.i > r::<R>(0.0)
            && i_term.abs() > (output - p_term + d_term).abs()
        {
            self.integrator = (saturated - p_term + d_term) / self.i;
        }

        saturated
    }

    pub fn reset(&mut self) {
        self.integrator = r::<R>(0.0);
        self.differentiator = r::<R>(0.0);
        self.prev_x = r::<R>(0.0);
        self.prev_t = r::<R>(-1.0);
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControllerOutput<R: FlightFloat> {
    pub u: [R; 10],
}

impl<R: FlightFloat> ControllerOutput<R> {
    pub fn from_forces_torques_and_passthrough(
        forces: Vector<R, 3>,
        torques: Vector<R, 3>,
        passthrough: [R; 4],
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

    pub fn forces(&self) -> Vector<R, 3> {
        Vector::from([self.u[0], self.u[1], self.u[2]])
    }

    pub fn torques(&self) -> Vector<R, 3> {
        Vector::from([self.u[3], self.u[4], self.u[5]])
    }

    pub fn passthrough(&self) -> [R; 4] {
        [self.u[6], self.u[7], self.u[8], self.u[9]]
    }

    pub fn quad_thrust_command(&self) -> R {
        -self.u[2]
    }
}

impl<R: FlightFloat> Default for ControllerOutput<R> {
    fn default() -> Self {
        Self {
            u: [r::<R>(0.0); 10],
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct QuadController<R: FlightFloat> {
    pub roll_rate_pid: Pid<R>,
    pub pitch_rate_pid: Pid<R>,
    pub yaw_rate_pid: Pid<R>,
    pub roll_angle_pid: Pid<R>,
    pub pitch_angle_pid: Pid<R>,
    equilibrium_torques: [R; 3],
    rc_max_throttle: R,
    use_motor_parameters: bool,
    gains_current: bool,
}

impl<R: FlightFloat> Default for QuadController<R> {
    fn default() -> Self {
        Self {
            roll_rate_pid: Pid::default(),
            pitch_rate_pid: Pid::default(),
            yaw_rate_pid: Pid::default(),
            roll_angle_pid: Pid::default(),
            pitch_angle_pid: Pid::default(),
            equilibrium_torques: [r::<R>(0.0); 3],
            rc_max_throttle: r::<R>(1.0),
            use_motor_parameters: false,
            gains_current: false,
        }
    }
}

impl<R: FlightFloat> QuadController<R> {
    pub fn new(
        roll_rate_pid: Pid<R>,
        pitch_rate_pid: Pid<R>,
        yaw_rate_pid: Pid<R>,
        roll_angle_pid: Pid<R>,
        pitch_angle_pid: Pid<R>,
    ) -> Self {
        Self {
            roll_rate_pid,
            pitch_rate_pid,
            yaw_rate_pid,
            roll_angle_pid,
            pitch_angle_pid,
            equilibrium_torques: [r::<R>(0.0); 3],
            rc_max_throttle: r::<R>(1.0),
            use_motor_parameters: false,
            gains_current: true,
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
        state: &AttitudeState<R>,
        command: &CombinedControl,
        params: &Params,
        dt: R,
        add_equilibrium_torques: bool,
        update_integrators: bool,
        air_density: R,
    ) -> ControllerOutput<R> {
        let current_rates = state.body_rate;
        let needs_euler = command.qx.control_type == ControlType::Angle
            || command.qy.control_type == ControlType::Angle;
        let euler = if needs_euler {
            Some(Vector::<R, 3>::from(state))
        } else {
            None
        };

        let mut torque_x = match command.qx.control_type {
            ControlType::Rate => self.roll_rate_pid.run(
                current_rates[0],
                <R as FlightFloat>::from_f32(command.qx.value),
                dt,
                update_integrators,
            ),
            ControlType::Angle => self.roll_angle_pid.run_with_derivative(
                euler.unwrap()[0],
                <R as FlightFloat>::from_f32(command.qx.value),
                current_rates[0],
                dt,
                update_integrators,
            ),
            _ => <R as FlightFloat>::from_f32(command.qx.value),
        };

        let mut torque_y = match command.qy.control_type {
            ControlType::Rate => self.pitch_rate_pid.run(
                current_rates[1],
                <R as FlightFloat>::from_f32(command.qy.value),
                dt,
                update_integrators,
            ),
            ControlType::Angle => self.pitch_angle_pid.run_with_derivative(
                euler.unwrap()[1],
                <R as FlightFloat>::from_f32(command.qy.value),
                current_rates[1],
                dt,
                update_integrators,
            ),
            _ => <R as FlightFloat>::from_f32(command.qy.value),
        };

        let mut torque_z = match command.qz.control_type {
            ControlType::Rate => self.yaw_rate_pid.run(
                current_rates[2],
                <R as FlightFloat>::from_f32(command.qz.value),
                dt,
                update_integrators,
            ),
            _ => <R as FlightFloat>::from_f32(command.qz.value),
        };

        if add_equilibrium_torques {
            torque_x += self.equilibrium_torques[0];
            torque_y += self.equilibrium_torques[1];
            torque_z += self.equilibrium_torques[2];
        }

        let forces = Vector::from([
            force_output(
                <R as FlightFloat>::from_f32(command.fx.value),
                command.fx.control_type,
                params,
                false,
                air_density,
                self.rc_max_throttle,
                self.use_motor_parameters,
            ),
            force_output(
                <R as FlightFloat>::from_f32(command.fy.value),
                command.fy.control_type,
                params,
                false,
                air_density,
                self.rc_max_throttle,
                self.use_motor_parameters,
            ),
            force_output(
                <R as FlightFloat>::from_f32(command.fz.value),
                command.fz.control_type,
                params,
                true,
                air_density,
                self.rc_max_throttle,
                self.use_motor_parameters,
            ),
        ]);

        ControllerOutput::from_forces_torques_and_passthrough(
            forces,
            Vector::from([torque_x, torque_y, torque_z]),
            [
                <R as FlightFloat>::from_f32(command.passthrough[0].value),
                <R as FlightFloat>::from_f32(command.passthrough[1].value),
                <R as FlightFloat>::from_f32(command.passthrough[2].value),
                <R as FlightFloat>::from_f32(command.passthrough[3].value),
            ],
        )
    }
}

impl<R: FlightFloat> Controller<R> for QuadController<R> {
    type State = AttitudeState<R>;
    type ControlOutput = ControllerOutput<R>;

    fn update_gains(&mut self, params: &Params) {
        // Roll Rate
        self.roll_rate_pid.p = match params.get_by_id(ParamId::PARAM_PID_ROLL_RATE_P) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.roll_rate_pid.i = match params.get_by_id(ParamId::PARAM_PID_ROLL_RATE_I) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.roll_rate_pid.d = match params.get_by_id(ParamId::PARAM_PID_ROLL_RATE_D) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.roll_rate_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };

        // Pitch Rate
        self.pitch_rate_pid.p = match params.get_by_id(ParamId::PARAM_PID_PITCH_RATE_P) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.pitch_rate_pid.i = match params.get_by_id(ParamId::PARAM_PID_PITCH_RATE_I) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.pitch_rate_pid.d = match params.get_by_id(ParamId::PARAM_PID_PITCH_RATE_D) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.pitch_rate_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };

        // Yaw Rate
        self.yaw_rate_pid.p = match params.get_by_id(ParamId::PARAM_PID_YAW_RATE_P) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.yaw_rate_pid.i = match params.get_by_id(ParamId::PARAM_PID_YAW_RATE_I) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.yaw_rate_pid.d = match params.get_by_id(ParamId::PARAM_PID_YAW_RATE_D) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.yaw_rate_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };

        // Roll Angle
        self.roll_angle_pid.p = match params.get_by_id(ParamId::PARAM_PID_ROLL_ANGLE_P) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.roll_angle_pid.i = match params.get_by_id(ParamId::PARAM_PID_ROLL_ANGLE_I) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.roll_angle_pid.d = match params.get_by_id(ParamId::PARAM_PID_ROLL_ANGLE_D) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.roll_angle_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };

        // Pitch Angle
        self.pitch_angle_pid.p = match params.get_by_id(ParamId::PARAM_PID_PITCH_ANGLE_P) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.pitch_angle_pid.i = match params.get_by_id(ParamId::PARAM_PID_PITCH_ANGLE_I) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.pitch_angle_pid.d = match params.get_by_id(ParamId::PARAM_PID_PITCH_ANGLE_D) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.pitch_angle_pid.tau = match params.get_by_id(ParamId::PARAM_PID_TAU) {
            ParamValue::Float(val) => <R as FlightFloat>::from_f32(val),
            _ => r::<R>(0.0),
        };
        self.equilibrium_torques = [
            <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_X_EQ_TORQUE)),
            <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_Y_EQ_TORQUE)),
            <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_Z_EQ_TORQUE)),
        ];
        self.rc_max_throttle =
            <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_RC_MAX_THROTTLE));
        self.use_motor_parameters = param_int(params, ParamId::PARAM_USE_MOTOR_PARAMETERS) != 0;
        self.gains_current = true;
    }

    fn control(&mut self, state: &Self::State, ctx: ControllerCtx<'_, R>) -> Self::ControlOutput {
        if !self.gains_current {
            self.update_gains(ctx.params);
        }

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

impl<R: FlightFloat> RcTrimCalibrator for QuadController<R> {
    fn calculate_equilibrium_torques_from_rc(
        &mut self,
        rc_control: &CombinedControl,
        params: &Params,
    ) -> [f32; 3] {
        let mut controller = *self;
        controller.update_gains(params);
        controller.reset_pids();
        let output = controller.run_pid_control(
            &AttitudeState::<R>::default(),
            rc_control,
            params,
            r::<R>(0.0),
            false,
            false,
            r::<R>(1.225),
        );

        [
            output.u[3].to_f32_lossy(),
            output.u[4].to_f32_lossy(),
            output.u[5].to_f32_lossy(),
        ]
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

fn force_output<R: FlightFloat>(
    value: R,
    control_type: ControlType,
    params: &Params,
    is_fz: bool,
    air_density: R,
    rc_max_throttle: R,
    use_motor_parameters: bool,
) -> R {
    if control_type != ControlType::Throttle {
        return value;
    }

    let sign = if is_fz { r::<R>(-1.0) } else { r::<R>(1.0) };
    let mut output = sign * value * rc_max_throttle;

    if use_motor_parameters {
        output *= calculate_max_thrust(params, air_density);
    }

    output
}

fn calculate_max_thrust<R: FlightFloat>(params: &Params, air_density: R) -> R {
    let resistance =
        <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_MOTOR_RESISTANCE));
    let diameter = <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_PROP_DIAMETER));
    let cq = <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_PROP_CQ));
    let ct = <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_PROP_CT));
    let kv = <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_MOTOR_KV));
    let no_load_current =
        <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_NO_LOAD_CURRENT));
    let num_motors = <R as FlightFloat>::from_i32(param_int(params, ParamId::PARAM_NUM_MOTORS));
    let max_voltage = <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_VOLT_MAX));

    let pi: R = pi();
    let a = resistance * air_density * diameter.powf(r::<R>(5.0)) * cq
        / (r::<R>(4.0) * pi.powf(r::<R>(2.0)) * kv);
    let b = kv;
    let c = no_load_current * resistance - max_voltage;
    let omega = (-b + (b.powf(r::<R>(2.0)) - r::<R>(4.0) * a * c).sqrt()) / (r::<R>(2.0) * a);

    air_density * diameter.powf(r::<R>(4.0)) * ct * omega.powf(r::<R>(2.0))
        / (r::<R>(4.0) * pi.powf(r::<R>(2.0)))
        * num_motors
}

fn controller_should_update_integrators<R: FlightFloat>(command: &CombinedControl, dt: R) -> bool {
    dt < r::<R>(0.01)
        && (command.fx.value > 0.1 || command.fy.value > 0.1 || command.fz.value > 0.1)
}

/// Constructs a Quaternion from Euler angles (Roll, Pitch, Yaw) ZYX sequence
/// Roll and Pitch are in radians.
pub fn quaternion_from_euler<R: FlightFloat>(roll: R, pitch: R, yaw: R) -> Quaternion<R> {
    // libm 0.2 does not have sin_cos, so we compute them separately
    let sr = (roll * r::<R>(0.5)).sin();
    let cr = (roll * r::<R>(0.5)).cos();

    let sp = (pitch * r::<R>(0.5)).sin();
    let cp = (pitch * r::<R>(0.5)).cos();

    let sy = (yaw * r::<R>(0.5)).sin();
    let cy = (yaw * r::<R>(0.5)).cos();

    Quaternion::new(
        cr * cp * cy + sr * sp * sy, // w
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy, // z
    )
}

/// Extracts Yaw (Z-axis rotation) from a Quaternion
pub fn get_yaw<R: FlightFloat>(q: Quaternion<R>) -> R {
    let w = q.w;
    let x = q.i;
    let y = q.j;
    let z = q.k;

    (r::<R>(2.0) * (w * z + x * y)).atan2(r::<R>(1.0) - r::<R>(2.0) * (y * y + z * z))
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
        controller: &mut QuadController<f64>,
        state: &AttitudeState<f64>,
        state_manager: &mut StateManager,
        command: &CombinedControl,
        params: &Params,
        dt: f64,
        air_density: f64,
    ) -> ControllerOutput<f64> {
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

        let mut controller = QuadController::<f64>::default();
        let state = AttitudeState::<f64>::default();
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

        let mut controller = QuadController::<f64>::default();
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
        let mut controller = QuadController::<f64>::default();
        let state = AttitudeState::<f64>::default();
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

        for (actual, expected) in output
            .u
            .iter()
            .zip([0.4, 0.5, 0.6, 0.1, 0.2, 0.3, 0.7, 0.8, 0.9, 1.0])
        {
            assert!((*actual - expected).abs() < 1e-6);
        }
    }

    #[test]
    fn pid_integrator_updates_only_when_rosflight_gate_allows_it() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_PID_ROLL_RATE_P, ParamValue::Float(0.0));
        params.set_by_id(ParamId::PARAM_PID_ROLL_RATE_I, ParamValue::Float(2.0));
        params.set_by_id(ParamId::PARAM_PID_ROLL_RATE_D, ParamValue::Float(0.0));

        let state = AttitudeState::<f64>::default();
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
        let mut gated_out_controller = QuadController::<f64>::default();
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
        let mut gated_in_controller = QuadController::<f64>::default();
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
        let mut pid = Pid::<f64>::new(2.0, 3.0, 0.5, 0.25, 0.05);

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
        let mut controller = QuadController::<f64>::default();
        let state = AttitudeState {
            q_hat: quaternion_from_euler::<f64>(0.1, -0.2, 0.0),
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

        assert!((output.u[3] - 0.25).abs() < 1e-6);
        assert!((output.u[4] - 0.4).abs() < 1e-6);
        assert!((output.u[5] - 0.6).abs() < 1e-6);
        assert!((output.u[2] + 0.28).abs() < 1e-6);
    }

    #[test]
    fn motor_param_thrust_scaling_uses_controller_air_density_context() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_USE_MOTOR_PARAMETERS, ParamValue::Int(1));

        let state = AttitudeState::<f64>::default();
        let command = CombinedControl {
            fz: ControlChannel {
                active: true,
                control_type: ControlType::Throttle,
                value: 0.4,
            },
            ..Default::default()
        };

        let mut lower_density_state = armed_state(&params);
        let mut lower_density_controller = QuadController::<f64>::default();
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
        let mut higher_density_controller = QuadController::<f64>::default();
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
