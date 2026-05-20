use super::AttitudeEstimate;
use super::Estimator;
use crate::comm::messages::messages::ExternalAttitudeMsg;
use crate::packets;
use crate::params::{ParamId, ParamValue, Params};
use crate::sensors::ProcessedSensors;
use libm::{cos, sin, sqrt};

use nalgebra::{Quaternion, SVector as Vector, UnitQuaternion};

// Removed hardcoded DT - now using actual timestamps
// const DT: f64 = 1.0/400.0f64;

const G: f64 = 9.80665; // Gravity in m/s^2

#[derive(Debug, Clone, Copy)]
pub struct AttitudeState {
    pub q_hat: Quaternion<f64>,
    pub q_dot: Quaternion<f64>,
    pub body_rate: Vector<f64, 3>,
    pub b_hat: Vector<f64, 3>,
    pub is_healthy: bool,
}

impl Default for AttitudeState {
    fn default() -> Self {
        Self {
            q_hat: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            q_dot: Quaternion::new(0.0, 0.0, 0.0, 0.0),
            body_rate: Vector::from([0.0, 0.0, 0.0]),
            b_hat: Vector::from([0.0, 0.0, 0.0]),
            is_healthy: false,
        }
    }
}

impl AttitudeEstimate for AttitudeState {
    fn q(&self) -> [f32; 4] {
        [
            self.q_hat.w as f32,
            self.q_hat.i as f32,
            self.q_hat.j as f32,
            self.q_hat.k as f32,
        ]
    }

    fn q_dot(&self) -> [f32; 4] {
        [
            self.q_dot.w as f32,
            self.q_dot.i as f32,
            self.q_dot.j as f32,
            self.q_dot.k as f32,
        ]
    }

    fn is_healthy(&self) -> bool {
        self.is_healthy
    }
}

impl From<AttitudeState> for Vector<f64, 3> {
    fn from(state: AttitudeState) -> Self {
        quaternion_to_euler(state.q_hat)
    }
}

impl<'a> From<&'a AttitudeState> for Vector<f64, 3> {
    fn from(state: &'a AttitudeState) -> Self {
        quaternion_to_euler(state.q_hat)
    }
}

pub struct QuadEstimator {
    k_p: f64,
    k_i: f64,
    q_hat: Quaternion<f64>,
    q_dot: Quaternion<f64>,
    body_rate: Vector<f64, 3>,
    b_hat: Vector<f64, 3>,
    is_initialized: bool, // Track if we've received first IMU packet
    last_acc_update_us: u64,
    last_extatt_update_us: u64,

    // Low-pass filter state
    accel_lpf: Vector<f64, 3>, // Filtered accelerometer
    gyro_lpf: Vector<f64, 3>,  // Filtered gyroscope
    w1: Vector<f64, 3>,
    w2: Vector<f64, 3>,
    q_extatt: Option<Quaternion<f64>>,

    // LPF parameters (EMA alpha values) - matching C defaults
    alpha_acc: f64,     // PARAM_ACC_ALPHA = 0.5 in C
    alpha_gyro_xy: f64, // PARAM_GYRO_XY_ALPHA = 0.3 in C
    alpha_gyro_z: f64,  // PARAM_GYRO_Z_ALPHA = 0.3 in C

    // Accelerometer gating
    accel_margin: f64, // PARAM_FILTER_ACCEL_MARGIN = 0.1 in C

    // Adaptive gains during initialization
    init_time_us: u64,   // PARAM_INIT_TIME = 3000ms = 3,000,000 μs in C
    first_imu_time: u64, // Track when first IMU arrived
}

impl QuadEstimator {
    pub fn new(k_p: f64, k_i: f64) -> Self {
        Self {
            k_p,
            k_i,
            q_hat: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            q_dot: Quaternion::new(0.0, 0.0, 0.0, 0.0),
            body_rate: Vector::from([0.0, 0.0, 0.0]),
            b_hat: Vector::from([0.0, 0.0, 0.0]),
            is_initialized: false,
            last_acc_update_us: 0,
            last_extatt_update_us: 0,

            // Initialize LPF state - accel starts at gravity pointing down (NED frame)
            accel_lpf: Vector::from([0.0, 0.0, -G]),
            gyro_lpf: Vector::from([0.0, 0.0, 0.0]),
            w1: Vector::from([0.0, 0.0, 0.0]),
            w2: Vector::from([0.0, 0.0, 0.0]),
            q_extatt: None,

            // LPF parameters matching C defaults
            alpha_acc: 0.5,
            alpha_gyro_xy: 0.3,
            alpha_gyro_z: 0.3,

            // Accelerometer gating - ±10% around 1g
            accel_margin: 0.1,

            // Adaptive gains - 3 second initialization period
            init_time_us: 3_000_000,
            first_imu_time: 0,
        }
    }

    /// Update parameters from the parameter server.
    /// Call this every loop to read fresh parameter values.
    pub fn update_params(&mut self, params: &Params) {
        // Read base gains (not the 10× boosted values)
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_FILTER_KP_ACC) {
            self.k_p = v as f64;
        }
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_FILTER_KI) {
            self.k_i = v as f64;
        }

        // Read LPF alpha values
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_ACC_ALPHA) {
            self.alpha_acc = v as f64;
        }
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_GYRO_XY_ALPHA) {
            self.alpha_gyro_xy = v as f64;
        }
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_GYRO_Z_ALPHA) {
            self.alpha_gyro_z = v as f64;
        }

        // Read accelerometer gating margin
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_FILTER_ACCEL_MARGIN) {
            self.accel_margin = v as f64;
        }

        // Read initialization time (convert milliseconds to microseconds)
        if let ParamValue::Int(v) = params.get_by_id(ParamId::PARAM_INIT_TIME) {
            self.init_time_us = (v as u64) * 1000;
        }
    }
}

impl Default for QuadEstimator {
    fn default() -> Self {
        Self::new(1.5, 0.05)
    }
}

impl QuadEstimator {
    pub fn reset_state(&mut self) {
        self.q_hat = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        self.q_dot = Quaternion::new(0.0, 0.0, 0.0, 0.0);
        self.body_rate = Vector::from([0.0, 0.0, 0.0]);
        self.b_hat = Vector::from([0.0, 0.0, 0.0]);
        self.accel_lpf = Vector::from([0.0, 0.0, -G]);
        self.gyro_lpf = Vector::from([0.0, 0.0, 0.0]);
        self.w1 = Vector::from([0.0, 0.0, 0.0]);
        self.w2 = Vector::from([0.0, 0.0, 0.0]);
        self.q_extatt = None;
        self.is_initialized = false;
        self.last_acc_update_us = 0;
        self.last_extatt_update_us = 0;
    }

    pub fn reset_adaptive_bias(&mut self) {
        self.b_hat = Vector::from([0.0, 0.0, 0.0]);
    }

    fn set_external_attitude_update(&mut self, external_attitude: ExternalAttitudeMsg) {
        let mut q = Quaternion::new(
            external_attitude.qw as f64,
            external_attitude.qx as f64,
            external_attitude.qy as f64,
            external_attitude.qz as f64,
        );
        q.normalize_mut();
        self.q_extatt = Some(q);
    }

    fn estimate_packets(
        &mut self,
        imu: Option<packets::ImuPacket>,
        _mag: Option<packets::MagPacket>,
        params: &Params,
        dt: f64,
    ) -> AttitudeState {
        // Update parameters from parameter server (matches C behavior)
        self.update_params(params);

        if dt < 0.0 {
            return AttitudeState {
                q_hat: self.q_hat,
                q_dot: self.q_dot,
                body_rate: self.body_rate,
                b_hat: self.b_hat,
                is_healthy: false,
            };
        }

        if let Some(imu_packet) = imu {
            // Get current timestamp for initialization tracking
            let current_time = imu_packet.header.timestamp; // microseconds

            // On first call, just initialize timestamp and skip update
            if !self.is_initialized {
                self.first_imu_time = current_time;
                self.last_acc_update_us = current_time;
                self.last_extatt_update_us = current_time;
                self.is_initialized = true;
                return AttitudeState {
                    q_hat: self.q_hat,
                    q_dot: self.q_dot,
                    body_rate: self.body_rate,
                    b_hat: self.b_hat,
                    is_healthy: true,
                };
            }

            // Apply low-pass filter to raw measurements (EMA filter)
            let raw_accel = Vector::from(imu_packet.accel);
            self.accel_lpf[0] =
                (1.0 - self.alpha_acc) * raw_accel[0] + self.alpha_acc * self.accel_lpf[0];
            self.accel_lpf[1] =
                (1.0 - self.alpha_acc) * raw_accel[1] + self.alpha_acc * self.accel_lpf[1];
            self.accel_lpf[2] =
                (1.0 - self.alpha_acc) * raw_accel[2] + self.alpha_acc * self.accel_lpf[2];

            let raw_gyro = Vector::from(imu_packet.gyro);
            self.gyro_lpf[0] =
                (1.0 - self.alpha_gyro_xy) * raw_gyro[0] + self.alpha_gyro_xy * self.gyro_lpf[0];
            self.gyro_lpf[1] =
                (1.0 - self.alpha_gyro_xy) * raw_gyro[1] + self.alpha_gyro_xy * self.gyro_lpf[1];
            self.gyro_lpf[2] =
                (1.0 - self.alpha_gyro_z) * raw_gyro[2] + self.alpha_gyro_z * self.gyro_lpf[2];

            let use_acc = param_int(params, ParamId::PARAM_FILTER_USE_ACC) != 0;
            let use_quad_int = param_int(params, ParamId::PARAM_FILTER_USE_QUAD_INT) != 0;
            let use_mat_exp = param_int(params, ParamId::PARAM_FILTER_USE_MAT_EXP) != 0;
            let fixed_wing = param_int(params, ParamId::PARAM_FIXED_WING) != 0;

            // Check if accelerometer magnitude is near 1g (gating)
            let accel_sqrd_norm = self.accel_lpf[0] * self.accel_lpf[0]
                + self.accel_lpf[1] * self.accel_lpf[1]
                + self.accel_lpf[2] * self.accel_lpf[2];

            let margin = self.accel_margin;
            let lowerbound = (1.0 - margin) * (1.0 - margin) * G * G;
            let upperbound = (1.0 + margin) * (1.0 + margin) * G * G;
            let can_use_accel =
                use_acc && accel_sqrd_norm > lowerbound && accel_sqrd_norm < upperbound;

            let mut kp = 0.0;
            let mut ki = self.k_i;
            let mut w_err = Vector::from([0.0, 0.0, 0.0]);

            if can_use_accel {
                w_err = accel_correction(self.q_hat, self.accel_lpf);
                kp = self.k_p;
                self.last_acc_update_us = current_time;
            }

            if let Some(q_extatt) = self.q_extatt.take() {
                w_err = extatt_correction(self.q_hat, q_extatt);
                kp = param_float(params, ParamId::PARAM_FILTER_KP_EXT);
                let extatt_dt =
                    current_time.saturating_sub(self.last_extatt_update_us) as f64 * 1e-6;
                let scale_dt = if dt > 0.0 { extatt_dt / dt } else { 0.0 };
                w_err = w_err * scale_dt;
                self.last_extatt_update_us = current_time;
            }

            if current_time < (param_int(params, ParamId::PARAM_INIT_TIME).max(0) as u64) * 1000 {
                kp = self.k_p * 10.0;
                ki = self.k_i * 10.0;
            }

            self.b_hat = self.b_hat - (ki * w_err * dt);

            let wbar = self.smoothed_gyro_measurement(use_quad_int);
            let wfinal = wbar - self.b_hat + kp * w_err;
            self.integrate_angular_rate(wfinal, dt, use_mat_exp);

            self.body_rate = self.gyro_lpf - self.b_hat;

            let unhealthy_due_to_accel =
                use_acc && current_time > self.last_acc_update_us + 500_000 && !fixed_wing;
            if unhealthy_due_to_accel {
                return AttitudeState {
                    q_hat: self.q_hat,
                    q_dot: self.q_dot,
                    body_rate: self.body_rate,
                    b_hat: self.b_hat,
                    is_healthy: false,
                };
            }
        }

        let q = self.q_hat;
        let is_healthy = !(q.w.is_nan()
            || q.w.is_infinite()
            || q.i.is_nan()
            || q.i.is_infinite()
            || q.j.is_nan()
            || q.j.is_infinite()
            || q.k.is_nan()
            || q.k.is_infinite());

        AttitudeState {
            q_hat: self.q_hat,
            q_dot: self.q_dot,
            body_rate: self.body_rate,
            b_hat: self.b_hat,
            is_healthy,
        }
    }

    fn smoothed_gyro_measurement(&mut self, use_quad_int: bool) -> Vector<f64, 3> {
        if use_quad_int {
            let wbar = (self.w2 / -12.0) + self.w1 * (8.0 / 12.0) + self.gyro_lpf * (5.0 / 12.0);
            self.w2 = self.w1;
            self.w1 = self.gyro_lpf;
            wbar
        } else {
            self.gyro_lpf
        }
    }

    fn integrate_angular_rate(&mut self, omega: Vector<f64, 3>, dt: f64, use_mat_exp: bool) {
        let sqrd_norm_w = omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2];
        if sqrd_norm_w == 0.0 {
            self.q_dot = Quaternion::new(0.0, 0.0, 0.0, 0.0);
            return;
        }

        let p = omega[0];
        let q = omega[1];
        let r = omega[2];
        let current = self.q_hat;

        self.q_dot = Quaternion::new(
            0.5 * (-p * current.i - q * current.j - r * current.k),
            0.5 * (p * current.w + r * current.j - q * current.k),
            0.5 * (q * current.w - r * current.i + p * current.k),
            0.5 * (r * current.w + q * current.i - p * current.j),
        );

        if use_mat_exp {
            let norm_w = sqrt(sqrd_norm_w);
            let t1 = cos((norm_w * dt) / 2.0);
            let t2 = sin((norm_w * dt) / 2.0) / norm_w;
            self.q_hat = Quaternion::new(
                t1 * current.w + t2 * (-p * current.i - q * current.j - r * current.k),
                t1 * current.i + t2 * (p * current.w + r * current.j - q * current.k),
                t1 * current.j + t2 * (q * current.w - r * current.i + p * current.k),
                t1 * current.k + t2 * (r * current.w + q * current.i - p * current.j),
            );
        } else {
            self.q_hat = self.q_hat + self.q_dot * dt;
        }
        self.q_hat.normalize_mut();
    }
}

impl Estimator for QuadEstimator {
    type State = AttitudeState;

    fn estimate(&mut self, sensors: &ProcessedSensors, params: &Params, dt: f64) -> Self::State {
        self.estimate_packets(sensors.imu, sensors.mag, params, dt)
    }

    fn reset(&mut self) {
        self.reset_state();
    }

    fn reset_adaptive_bias(&mut self) {
        QuadEstimator::reset_adaptive_bias(self);
    }

    fn estimate_with_external_attitude(
        &mut self,
        sensors: &ProcessedSensors,
        params: &Params,
        dt: f64,
        external_attitude: Option<ExternalAttitudeMsg>,
    ) -> Self::State {
        if let Some(external_attitude) = external_attitude {
            self.set_external_attitude_update(external_attitude);
        }
        self.estimate_packets(sensors.imu, sensors.mag, params, dt)
    }
}

fn param_float(params: &Params, param_id: ParamId) -> f64 {
    match params.get_by_id(param_id) {
        ParamValue::Float(value) => value as f64,
        _ => 0.0,
    }
}

fn param_int(params: &Params, param_id: ParamId) -> i32 {
    match params.get_by_id(param_id) {
        ParamValue::Int(value) => value,
        _ => 0,
    }
}

fn accel_correction(attitude: Quaternion<f64>, accel_lpf: Vector<f64, 3>) -> Vector<f64, 3> {
    let accel_norm = accel_lpf.norm();
    if accel_norm <= 1e-9 {
        return Vector::from([0.0, 0.0, 0.0]);
    }

    let a = accel_lpf / accel_norm;
    let q_acc_inv = quaternion_between_vectors(Vector::from([0.0, 0.0, -1.0]), a);
    let q_tilde = q_acc_inv * attitude;
    Vector::from([
        -2.0 * q_tilde.w * q_tilde.i,
        -2.0 * q_tilde.w * q_tilde.j,
        0.0,
    ])
}

fn quaternion_between_vectors(from: Vector<f64, 3>, to: Vector<f64, 3>) -> Quaternion<f64> {
    let cross = from.cross(&to);
    let dot = from[0] * to[0] + from[1] * to[1] + from[2] * to[2];
    let mut q = if dot < -0.999_999 {
        Quaternion::new(0.0, 1.0, 0.0, 0.0)
    } else {
        Quaternion::new(1.0 + dot, cross[0], cross[1], cross[2])
    };
    q.normalize_mut();
    q
}

fn extatt_correction(attitude: Quaternion<f64>, external: Quaternion<f64>) -> Vector<f64, 3> {
    let (xhat, yhat, zhat) = quaternion_to_dcm_rows(attitude);
    let (xext, yext, zext) = quaternion_to_dcm_rows(external);
    xext.cross(&xhat) + yext.cross(&yhat) + zext.cross(&zhat)
}

fn quaternion_to_euler(q: Quaternion<f64>) -> Vector<f64, 3> {
    let (roll, pitch, yaw) = UnitQuaternion::new_normalize(q).euler_angles();
    Vector::from([roll, pitch, yaw])
}

fn quaternion_to_dcm_rows(q: Quaternion<f64>) -> (Vector<f64, 3>, Vector<f64, 3>, Vector<f64, 3>) {
    let w = q.w;
    let x = q.i;
    let y = q.j;
    let z = q.k;
    (
        Vector::from([
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ]),
        Vector::from([
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ]),
        Vector::from([
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ]),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        comm::messages::messages::ExternalAttitudeMsg,
        estimator::Estimator,
        packets::{ImuPacket, RosflightPacketHeader},
    };

    #[test]
    fn estimator_applies_external_attitude_as_correction_not_replacement() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_FILTER_USE_ACC, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_FILTER_USE_QUAD_INT, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_FILTER_USE_MAT_EXP, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_INIT_TIME, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_FILTER_KP_EXT, ParamValue::Float(1.5));
        let mut sensors = ProcessedSensors::default();
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -G],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });

        let mut estimator = QuadEstimator::default();
        let _ = estimator.estimate(&sensors, &params, 1.0 / 400.0);
        sensors.imu.as_mut().unwrap().header.timestamp = 3_000;
        let state = estimator.estimate_with_external_attitude(
            &sensors,
            &params,
            1.0 / 400.0,
            Some(ExternalAttitudeMsg {
                qw: core::f32::consts::FRAC_1_SQRT_2,
                qx: core::f32::consts::FRAC_1_SQRT_2,
                qy: 0.0,
                qz: 0.0,
            }),
        );

        assert_ne!(state.q(), [0.0, 1.0, 0.0, 0.0]);
        assert!(state.q()[0] < 1.0);
        assert!(state.q()[1] > 0.0);
        assert!(state.is_healthy());
    }

    #[test]
    fn estimator_reports_unhealthy_after_accel_correction_timeout() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_FILTER_USE_ACC, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_INIT_TIME, ParamValue::Int(0));
        let mut estimator = QuadEstimator::default();
        let mut sensors = ProcessedSensors::default();
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -G],
            gyro: [0.0, 0.0, 0.0],
            ..Default::default()
        });

        let _ = estimator.estimate(&sensors, &params, 0.002);
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 601_001,
                status: 0,
            },
            accel: [20.0, 0.0, 0.0],
            gyro: [0.0, 0.0, 0.0],
            ..Default::default()
        });

        let state = estimator.estimate(&sensors, &params, 0.002);

        assert!(!state.is_healthy());
    }

    #[test]
    fn fixedwing_flag_keeps_attitude_estimator_healthy_on_accel_correction_timeout() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_FILTER_USE_ACC, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_INIT_TIME, ParamValue::Int(0));
        let mut estimator = QuadEstimator::default();
        let mut sensors = ProcessedSensors::default();
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -G],
            gyro: [0.0, 0.0, 0.0],
            ..Default::default()
        });

        let _ = estimator.estimate(&sensors, &params, 0.002);
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 601_001,
                status: 0,
            },
            accel: [20.0, 0.0, 0.0],
            gyro: [0.0, 0.0, 0.0],
            ..Default::default()
        });

        let state = estimator.estimate(&sensors, &params, 0.002);

        assert!(state.is_healthy());
    }

    #[test]
    fn reset_reinitializes_attitude_from_the_next_imu_sample() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_FILTER_USE_ACC, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_FILTER_USE_QUAD_INT, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_FILTER_USE_MAT_EXP, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_INIT_TIME, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_GYRO_Z_ALPHA, ParamValue::Float(0.0));
        let mut estimator = QuadEstimator::default();
        let mut sensors = ProcessedSensors::default();
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -G],
            gyro: [0.0, 0.0, 1.0],
            ..Default::default()
        });

        let _ = estimator.estimate(&sensors, &params, 0.1);
        sensors.imu.as_mut().unwrap().header.timestamp = 101_000;
        let drifted = estimator.estimate(&sensors, &params, 0.1);
        assert!(drifted.q()[3] > 0.0);

        estimator.reset();
        sensors.imu.as_mut().unwrap().header.timestamp = 201_000;
        let reset = estimator.estimate(&sensors, &params, 0.1);

        assert_eq!(reset.q(), [1.0, 0.0, 0.0, 0.0]);
        assert!(reset.is_healthy());
    }

    #[test]
    fn quadratic_interpolation_delays_gyro_rate_like_rosflight() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_FILTER_USE_ACC, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_FILTER_USE_QUAD_INT, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_FILTER_USE_MAT_EXP, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_INIT_TIME, ParamValue::Int(0));
        let mut estimator = QuadEstimator::default();
        let mut sensors = ProcessedSensors::default();
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -G],
            gyro: [0.0, 0.0, 1.0],
            ..Default::default()
        });

        let _ = estimator.estimate(&sensors, &params, 0.002);
        sensors.imu.as_mut().unwrap().header.timestamp = 3_000;
        let state = estimator.estimate(&sensors, &params, 0.002);

        assert!(state.q()[3] > 0.0);
        assert!(state.q()[3] < 0.001);
    }

    #[test]
    fn matrix_exponential_integration_matches_constant_yaw_rate() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_FILTER_USE_ACC, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_FILTER_USE_QUAD_INT, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_FILTER_USE_MAT_EXP, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_INIT_TIME, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_GYRO_Z_ALPHA, ParamValue::Float(0.0));
        let mut estimator = QuadEstimator::default();
        let mut sensors = ProcessedSensors::default();
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -G],
            gyro: [0.0, 0.0, 1.0],
            ..Default::default()
        });

        let _ = estimator.estimate(&sensors, &params, 0.1);
        sensors.imu.as_mut().unwrap().header.timestamp = 101_000;
        let state = estimator.estimate(&sensors, &params, 0.1);

        assert!((state.q()[0] as f64 - cos(0.05)).abs() < 1e-6);
        assert!((state.q()[3] as f64 - sin(0.05)).abs() < 1e-6);
    }
}
