use super::AttitudeEstimate;
use super::Estimator;
use super::EstimatorCtx;
use crate::comm::messages::messages::ExternalAttitudeMsg;
use crate::math::FlightFloat;
use crate::packets;
use crate::params::{ParamId, ParamValue, Params};

use nalgebra::{Quaternion, SVector as Vector};

fn gravity<R: FlightFloat>() -> R {
    <R as FlightFloat>::from_f32(9.80665)
}

#[derive(Debug, Clone, Copy)]
pub struct AttitudeState<R: FlightFloat> {
    pub q_hat: Quaternion<R>,
    pub q_dot: Quaternion<R>,
    pub body_rate: Vector<R, 3>,
    pub b_hat: Vector<R, 3>,
    pub is_healthy: bool,
}

impl<R: FlightFloat> Default for AttitudeState<R> {
    fn default() -> Self {
        Self {
            q_hat: Quaternion::new(
                <R as FlightFloat>::from_f32(1.0),
                <R as FlightFloat>::from_f32(0.0),
                <R as FlightFloat>::from_f32(0.0),
                <R as FlightFloat>::from_f32(0.0),
            ),
            q_dot: Quaternion::from(Vector::from([<R as FlightFloat>::from_f32(0.0); 4])),
            body_rate: Vector::from([<R as FlightFloat>::from_f32(0.0); 3]),
            b_hat: Vector::from([<R as FlightFloat>::from_f32(0.0); 3]),
            is_healthy: false,
        }
    }
}

impl<R: FlightFloat> AttitudeEstimate for AttitudeState<R> {
    fn q(&self) -> [f32; 4] {
        [
            self.q_hat.w.to_f32_lossy(),
            self.q_hat.i.to_f32_lossy(),
            self.q_hat.j.to_f32_lossy(),
            self.q_hat.k.to_f32_lossy(),
        ]
    }

    fn q_dot(&self) -> [f32; 4] {
        [
            self.q_dot.w.to_f32_lossy(),
            self.q_dot.i.to_f32_lossy(),
            self.q_dot.j.to_f32_lossy(),
            self.q_dot.k.to_f32_lossy(),
        ]
    }

    fn is_healthy(&self) -> bool {
        self.is_healthy
    }
}

impl<R: FlightFloat> From<AttitudeState<R>> for Vector<R, 3> {
    fn from(state: AttitudeState<R>) -> Self {
        quaternion_to_euler(state.q_hat)
    }
}

impl<'a, R: FlightFloat> From<&'a AttitudeState<R>> for Vector<R, 3> {
    fn from(state: &'a AttitudeState<R>) -> Self {
        quaternion_to_euler(state.q_hat)
    }
}

pub struct QuadEstimator<R: FlightFloat> {
    k_p: R,
    k_i: R,
    k_p_ext: R,
    q_hat: Quaternion<R>,
    q_dot: Quaternion<R>,
    body_rate: Vector<R, 3>,
    b_hat: Vector<R, 3>,
    is_initialized: bool, // Track if we've received first IMU packet
    last_acc_update_us: u64,
    last_extatt_update_us: u64,

    // Low-pass filter state
    accel_lpf: Vector<R, 3>, // Filtered accelerometer
    gyro_lpf: Vector<R, 3>,  // Filtered gyroscope
    w1: Vector<R, 3>,
    w2: Vector<R, 3>,
    q_extatt: Option<Quaternion<R>>,

    // LPF parameters (EMA alpha values) - matching C defaults
    alpha_acc: R,     // PARAM_ACC_ALPHA = 0.5 in C
    alpha_gyro_xy: R, // PARAM_GYRO_XY_ALPHA = 0.3 in C
    alpha_gyro_z: R,  // PARAM_GYRO_Z_ALPHA = 0.3 in C

    // Accelerometer gating
    accel_margin: R, // PARAM_FILTER_ACCEL_MARGIN = 0.1 in C

    // Adaptive gains during initialization
    init_time_us: u64,   // PARAM_INIT_TIME = 3000ms = 3,000,000 μs in C
    first_imu_time: u64, // Track when first IMU arrived
    use_acc: bool,
    use_quad_int: bool,
    use_mat_exp: bool,
    fixed_wing: bool,
}

impl<R: FlightFloat> QuadEstimator<R> {
    pub fn new(k_p: R, k_i: R) -> Self {
        Self {
            k_p,
            k_i,
            k_p_ext: <R as FlightFloat>::from_f32(1.5),
            q_hat: Quaternion::new(
                <R as FlightFloat>::from_f32(1.0),
                <R as FlightFloat>::from_f32(0.0),
                <R as FlightFloat>::from_f32(0.0),
                <R as FlightFloat>::from_f32(0.0),
            ),
            q_dot: Quaternion::from(Vector::from([<R as FlightFloat>::from_f32(0.0); 4])),
            body_rate: Vector::from([<R as FlightFloat>::from_f32(0.0); 3]),
            b_hat: Vector::from([<R as FlightFloat>::from_f32(0.0); 3]),
            is_initialized: false,
            last_acc_update_us: 0,
            last_extatt_update_us: 0,

            // Initialize LPF state - accel starts at gravity pointing down (NED frame)
            accel_lpf: Vector::from([
                <R as FlightFloat>::from_f32(0.0),
                <R as FlightFloat>::from_f32(0.0),
                -gravity::<R>(),
            ]),
            gyro_lpf: Vector::from([<R as FlightFloat>::from_f32(0.0); 3]),
            w1: Vector::from([<R as FlightFloat>::from_f32(0.0); 3]),
            w2: Vector::from([<R as FlightFloat>::from_f32(0.0); 3]),
            q_extatt: None,

            // LPF parameters matching C defaults
            alpha_acc: <R as FlightFloat>::from_f32(0.5),
            alpha_gyro_xy: <R as FlightFloat>::from_f32(0.3),
            alpha_gyro_z: <R as FlightFloat>::from_f32(0.3),

            // Accelerometer gating - ±10% around 1g
            accel_margin: <R as FlightFloat>::from_f32(0.1),

            // Adaptive gains - 3 second initialization period
            init_time_us: 3_000_000,
            first_imu_time: 0,
            use_acc: true,
            use_quad_int: true,
            use_mat_exp: true,
            fixed_wing: false,
        }
    }

    /// Update parameters from the parameter server.
    /// Call this every loop to read fresh parameter values.
    pub fn update_params(&mut self, params: &Params) {
        // Read base gains (not the 10× boosted values)
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_FILTER_KP_ACC) {
            self.k_p = <R as FlightFloat>::from_f32(v);
        }
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_FILTER_KI) {
            self.k_i = <R as FlightFloat>::from_f32(v);
        }

        // Read LPF alpha values
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_ACC_ALPHA) {
            self.alpha_acc = <R as FlightFloat>::from_f32(v);
        }
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_GYRO_XY_ALPHA) {
            self.alpha_gyro_xy = <R as FlightFloat>::from_f32(v);
        }
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_GYRO_Z_ALPHA) {
            self.alpha_gyro_z = <R as FlightFloat>::from_f32(v);
        }

        // Read accelerometer gating margin
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_FILTER_ACCEL_MARGIN) {
            self.accel_margin = <R as FlightFloat>::from_f32(v);
        }

        // Read initialization time (convert milliseconds to microseconds)
        if let ParamValue::Int(v) = params.get_by_id(ParamId::PARAM_INIT_TIME) {
            self.init_time_us = (v as u64) * 1000;
        }
        if let ParamValue::Float(v) = params.get_by_id(ParamId::PARAM_FILTER_KP_EXT) {
            self.k_p_ext = <R as FlightFloat>::from_f32(v);
        }
        if let ParamValue::Int(v) = params.get_by_id(ParamId::PARAM_FILTER_USE_ACC) {
            self.use_acc = v != 0;
        }
        if let ParamValue::Int(v) = params.get_by_id(ParamId::PARAM_FILTER_USE_QUAD_INT) {
            self.use_quad_int = v != 0;
        }
        if let ParamValue::Int(v) = params.get_by_id(ParamId::PARAM_FILTER_USE_MAT_EXP) {
            self.use_mat_exp = v != 0;
        }
        if let ParamValue::Int(v) = params.get_by_id(ParamId::PARAM_FIXED_WING) {
            self.fixed_wing = v != 0;
        }
    }
}

impl<R: FlightFloat> Default for QuadEstimator<R> {
    fn default() -> Self {
        Self::new(
            <R as FlightFloat>::from_f32(1.5),
            <R as FlightFloat>::from_f32(0.05),
        )
    }
}

impl<R: FlightFloat> QuadEstimator<R> {
    pub fn reset_state(&mut self) {
        self.q_hat = Quaternion::new(
            <R as FlightFloat>::from_f32(1.0),
            <R as FlightFloat>::from_f32(0.0),
            <R as FlightFloat>::from_f32(0.0),
            <R as FlightFloat>::from_f32(0.0),
        );
        self.q_dot = Quaternion::from(Vector::from([<R as FlightFloat>::from_f32(0.0); 4]));
        self.body_rate = Vector::from([<R as FlightFloat>::from_f32(0.0); 3]);
        self.b_hat = Vector::from([<R as FlightFloat>::from_f32(0.0); 3]);
        self.accel_lpf = Vector::from([
            <R as FlightFloat>::from_f32(0.0),
            <R as FlightFloat>::from_f32(0.0),
            -gravity::<R>(),
        ]);
        self.gyro_lpf = Vector::from([<R as FlightFloat>::from_f32(0.0); 3]);
        self.w1 = Vector::from([<R as FlightFloat>::from_f32(0.0); 3]);
        self.w2 = Vector::from([<R as FlightFloat>::from_f32(0.0); 3]);
        self.q_extatt = None;
        self.is_initialized = false;
        self.last_acc_update_us = 0;
        self.last_extatt_update_us = 0;
    }

    pub fn reset_adaptive_bias(&mut self) {
        self.b_hat = Vector::from([<R as FlightFloat>::from_f32(0.0); 3]);
    }

    fn set_external_attitude_update(&mut self, external_attitude: ExternalAttitudeMsg) {
        let mut q = Quaternion::new(
            <R as FlightFloat>::from_f32(external_attitude.qw),
            <R as FlightFloat>::from_f32(external_attitude.qx),
            <R as FlightFloat>::from_f32(external_attitude.qy),
            <R as FlightFloat>::from_f32(external_attitude.qz),
        );
        q.normalize_mut();
        self.q_extatt = Some(q);
    }

    fn estimate_packets(
        &mut self,
        imu: Option<packets::ImuPacket<R>>,
        _mag: Option<packets::MagPacket>,
        _params: &Params,
        dt: R,
    ) -> AttitudeState<R> {
        if dt < <R as FlightFloat>::from_f32(0.0) {
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
            let one = <R as FlightFloat>::from_f32(1.0);
            self.accel_lpf[0] =
                (one - self.alpha_acc) * raw_accel[0] + self.alpha_acc * self.accel_lpf[0];
            self.accel_lpf[1] =
                (one - self.alpha_acc) * raw_accel[1] + self.alpha_acc * self.accel_lpf[1];
            self.accel_lpf[2] =
                (one - self.alpha_acc) * raw_accel[2] + self.alpha_acc * self.accel_lpf[2];

            let raw_gyro = Vector::from(imu_packet.gyro);
            self.gyro_lpf[0] =
                (one - self.alpha_gyro_xy) * raw_gyro[0] + self.alpha_gyro_xy * self.gyro_lpf[0];
            self.gyro_lpf[1] =
                (one - self.alpha_gyro_xy) * raw_gyro[1] + self.alpha_gyro_xy * self.gyro_lpf[1];
            self.gyro_lpf[2] =
                (one - self.alpha_gyro_z) * raw_gyro[2] + self.alpha_gyro_z * self.gyro_lpf[2];

            // Check if accelerometer magnitude is near 1g (gating)
            let accel_sqrd_norm = self.accel_lpf[0] * self.accel_lpf[0]
                + self.accel_lpf[1] * self.accel_lpf[1]
                + self.accel_lpf[2] * self.accel_lpf[2];

            let margin = self.accel_margin;
            let g = gravity();
            let lowerbound = (one - margin) * (one - margin) * g * g;
            let upperbound = (one + margin) * (one + margin) * g * g;
            let can_use_accel =
                self.use_acc && accel_sqrd_norm > lowerbound && accel_sqrd_norm < upperbound;

            let mut kp = <R as FlightFloat>::from_f32(0.0);
            let mut ki = self.k_i;
            let mut w_err = Vector::from([<R as FlightFloat>::from_f32(0.0); 3]);

            if can_use_accel {
                w_err = accel_correction(self.q_hat, self.accel_lpf);
                kp = self.k_p;
                self.last_acc_update_us = current_time;
            }

            if let Some(q_extatt) = self.q_extatt.take() {
                w_err = extatt_correction(self.q_hat, q_extatt);
                kp = self.k_p_ext;
                let extatt_dt = <R as FlightFloat>::from_u64(
                    current_time.saturating_sub(self.last_extatt_update_us),
                ) * <R as FlightFloat>::from_f32(1e-6);
                let scale_dt = if dt > <R as FlightFloat>::from_f32(0.0) {
                    extatt_dt / dt
                } else {
                    <R as FlightFloat>::from_f32(0.0)
                };
                w_err = w_err * scale_dt;
                self.last_extatt_update_us = current_time;
            }

            if current_time < self.init_time_us {
                kp = self.k_p * <R as FlightFloat>::from_f32(10.0);
                ki = self.k_i * <R as FlightFloat>::from_f32(10.0);
            }

            self.b_hat -= w_err * (ki * dt);

            let wbar = self.smoothed_gyro_measurement(self.use_quad_int);
            let wfinal = wbar - self.b_hat + w_err * kp;
            self.integrate_angular_rate(wfinal, dt, self.use_mat_exp);

            self.body_rate = self.gyro_lpf - self.b_hat;

            let unhealthy_due_to_accel = self.use_acc
                && current_time > self.last_acc_update_us + 500_000
                && !self.fixed_wing;
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
        let is_healthy = q.w.is_finite_value()
            && q.i.is_finite_value()
            && q.j.is_finite_value()
            && q.k.is_finite_value();

        AttitudeState {
            q_hat: self.q_hat,
            q_dot: self.q_dot,
            body_rate: self.body_rate,
            b_hat: self.b_hat,
            is_healthy,
        }
    }

    fn smoothed_gyro_measurement(&mut self, use_quad_int: bool) -> Vector<R, 3> {
        if use_quad_int {
            let wbar = (self.w2 / <R as FlightFloat>::from_f32(-12.0))
                + self.w1 * <R as FlightFloat>::from_f32(8.0 / 12.0)
                + self.gyro_lpf * <R as FlightFloat>::from_f32(5.0 / 12.0);
            self.w2 = self.w1;
            self.w1 = self.gyro_lpf;
            wbar
        } else {
            self.gyro_lpf
        }
    }

    fn integrate_angular_rate(&mut self, omega: Vector<R, 3>, dt: R, use_mat_exp: bool) {
        let sqrd_norm_w = omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2];
        if sqrd_norm_w == <R as FlightFloat>::from_f32(0.0) {
            self.q_dot = Quaternion::from(Vector::from([<R as FlightFloat>::from_f32(0.0); 4]));
            return;
        }

        let p = omega[0];
        let q = omega[1];
        let r = omega[2];
        let current = self.q_hat;

        self.q_dot = Quaternion::new(
            <R as FlightFloat>::from_f32(0.5) * (-p * current.i - q * current.j - r * current.k),
            <R as FlightFloat>::from_f32(0.5) * (p * current.w + r * current.j - q * current.k),
            <R as FlightFloat>::from_f32(0.5) * (q * current.w - r * current.i + p * current.k),
            <R as FlightFloat>::from_f32(0.5) * (r * current.w + q * current.i - p * current.j),
        );

        if use_mat_exp {
            let norm_w = sqrd_norm_w.sqrt();
            let half_angle = (norm_w * dt) / <R as FlightFloat>::from_f32(2.0);
            let t1 = half_angle.cos();
            let t2 = half_angle.sin() / norm_w;
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

impl<R: FlightFloat> Estimator<R> for QuadEstimator<R> {
    type State = AttitudeState<R>;

    fn estimate(&mut self, ctx: EstimatorCtx<'_, R>) -> Self::State {
        if let Some(external_attitude) = ctx.external_attitude {
            self.set_external_attitude_update(external_attitude);
        }
        self.estimate_packets(ctx.sensors.imu, ctx.sensors.mag, ctx.params, ctx.dt)
    }

    fn update_params(&mut self, params: &Params) {
        QuadEstimator::update_params(self, params);
    }

    fn reset(&mut self) {
        self.reset_state();
    }

    fn reset_adaptive_bias(&mut self) {
        QuadEstimator::reset_adaptive_bias(self);
    }
}

fn accel_correction<R: FlightFloat>(
    attitude: Quaternion<R>,
    accel_lpf: Vector<R, 3>,
) -> Vector<R, 3> {
    let accel_norm =
        (accel_lpf[0] * accel_lpf[0] + accel_lpf[1] * accel_lpf[1] + accel_lpf[2] * accel_lpf[2])
            .sqrt();
    if accel_norm <= <R as FlightFloat>::from_f32(1e-9) {
        return Vector::from([<R as FlightFloat>::from_f32(0.0); 3]);
    }

    let ax = accel_lpf[0] / accel_norm;
    let ay = accel_lpf[1] / accel_norm;
    let az = accel_lpf[2] / accel_norm;

    let one = <R as FlightFloat>::from_f32(1.0);
    let mut q_acc_w = one - az;
    let mut q_acc_x = ay;
    let mut q_acc_y = -ax;
    let mut q_acc_z = <R as FlightFloat>::from_f32(0.0);
    if -az < <R as FlightFloat>::from_f32(-0.999_999) {
        q_acc_w = <R as FlightFloat>::from_f32(0.0);
        q_acc_x = one;
        q_acc_y = <R as FlightFloat>::from_f32(0.0);
    }
    let q_acc_norm =
        (q_acc_w * q_acc_w + q_acc_x * q_acc_x + q_acc_y * q_acc_y + q_acc_z * q_acc_z).sqrt();
    if q_acc_norm > <R as FlightFloat>::from_f32(0.0) {
        q_acc_w /= q_acc_norm;
        q_acc_x /= q_acc_norm;
        q_acc_y /= q_acc_norm;
        q_acc_z /= q_acc_norm;
    }

    let q_tilde_w =
        q_acc_w * attitude.w - q_acc_x * attitude.i - q_acc_y * attitude.j - q_acc_z * attitude.k;
    let q_tilde_i =
        q_acc_w * attitude.i + q_acc_x * attitude.w + q_acc_y * attitude.k - q_acc_z * attitude.j;
    let q_tilde_j =
        q_acc_w * attitude.j - q_acc_x * attitude.k + q_acc_y * attitude.w + q_acc_z * attitude.i;
    Vector::from([
        <R as FlightFloat>::from_f32(-2.0) * q_tilde_w * q_tilde_i,
        <R as FlightFloat>::from_f32(-2.0) * q_tilde_w * q_tilde_j,
        <R as FlightFloat>::from_f32(0.0),
    ])
}

fn extatt_correction<R: FlightFloat>(
    attitude: Quaternion<R>,
    external: Quaternion<R>,
) -> Vector<R, 3> {
    let (xhat, yhat, zhat) = quaternion_to_dcm_rows(attitude);
    let (xext, yext, zext) = quaternion_to_dcm_rows(external);
    xext.cross(&xhat) + yext.cross(&yhat) + zext.cross(&zhat)
}

fn quaternion_to_euler<R: FlightFloat>(q: Quaternion<R>) -> Vector<R, 3> {
    let two = <R as FlightFloat>::from_f32(2.0);
    let one = <R as FlightFloat>::from_f32(1.0);
    let minus_one = <R as FlightFloat>::from_f32(-1.0);

    let sin_roll = two * (q.w * q.i + q.j * q.k);
    let cos_roll = one - two * (q.i * q.i + q.j * q.j);
    let roll = sin_roll.atan2(cos_roll);

    let sin_pitch = two * (q.w * q.j - q.k * q.i);
    let pitch = sin_pitch.clamp(minus_one, one).asin();

    let sin_yaw = two * (q.w * q.k + q.i * q.j);
    let cos_yaw = one - two * (q.j * q.j + q.k * q.k);
    let yaw = sin_yaw.atan2(cos_yaw);

    Vector::from([roll, pitch, yaw])
}

fn quaternion_to_dcm_rows<R: FlightFloat>(
    q: Quaternion<R>,
) -> (Vector<R, 3>, Vector<R, 3>, Vector<R, 3>) {
    let w = q.w;
    let x = q.i;
    let y = q.j;
    let z = q.k;
    (
        Vector::from([
            <R as FlightFloat>::from_f32(1.0) - <R as FlightFloat>::from_f32(2.0) * (y * y + z * z),
            <R as FlightFloat>::from_f32(2.0) * (x * y - z * w),
            <R as FlightFloat>::from_f32(2.0) * (x * z + y * w),
        ]),
        Vector::from([
            <R as FlightFloat>::from_f32(2.0) * (x * y + z * w),
            <R as FlightFloat>::from_f32(1.0) - <R as FlightFloat>::from_f32(2.0) * (x * x + z * z),
            <R as FlightFloat>::from_f32(2.0) * (y * z - x * w),
        ]),
        Vector::from([
            <R as FlightFloat>::from_f32(2.0) * (x * z - y * w),
            <R as FlightFloat>::from_f32(2.0) * (y * z + x * w),
            <R as FlightFloat>::from_f32(1.0) - <R as FlightFloat>::from_f32(2.0) * (x * x + y * y),
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
        sensors::ProcessedSensors,
    };

    fn estimate(
        estimator: &mut QuadEstimator<f64>,
        sensors: &ProcessedSensors<f64>,
        params: &Params,
        dt: f64,
    ) -> AttitudeState<f64> {
        estimator.update_params(params);
        estimator.estimate(EstimatorCtx {
            sensors,
            params,
            dt,
            external_attitude: None,
        })
    }

    fn estimate_with_external_attitude(
        estimator: &mut QuadEstimator<f64>,
        sensors: &ProcessedSensors<f64>,
        params: &Params,
        dt: f64,
        external_attitude: ExternalAttitudeMsg,
    ) -> AttitudeState<f64> {
        estimator.update_params(params);
        estimator.estimate(EstimatorCtx {
            sensors,
            params,
            dt,
            external_attitude: Some(external_attitude),
        })
    }

    #[test]
    fn estimator_applies_external_attitude_as_correction_not_replacement() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_FILTER_USE_ACC, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_FILTER_USE_QUAD_INT, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_FILTER_USE_MAT_EXP, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_INIT_TIME, ParamValue::Int(0));
        params.set_by_id(ParamId::PARAM_FILTER_KP_EXT, ParamValue::Float(1.5));
        let mut sensors = ProcessedSensors::<f64>::default();
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq: 1,
        });

        let mut estimator = QuadEstimator::default();
        let _ = estimate(&mut estimator, &sensors, &params, 1.0 / 400.0);
        sensors.imu.as_mut().unwrap().header.timestamp = 3_000;
        let state = estimate_with_external_attitude(
            &mut estimator,
            &sensors,
            &params,
            1.0 / 400.0,
            ExternalAttitudeMsg {
                qw: core::f32::consts::FRAC_1_SQRT_2,
                qx: core::f32::consts::FRAC_1_SQRT_2,
                qy: 0.0,
                qz: 0.0,
            },
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
        let mut sensors = ProcessedSensors::<f64>::default();
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            ..Default::default()
        });

        let _ = estimate(&mut estimator, &sensors, &params, 0.002);
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 601_001,
                status: 0,
            },
            accel: [20.0, 0.0, 0.0],
            gyro: [0.0, 0.0, 0.0],
            ..Default::default()
        });

        let state = estimate(&mut estimator, &sensors, &params, 0.002);

        assert!(!state.is_healthy());
    }

    #[test]
    fn fixedwing_flag_keeps_attitude_estimator_healthy_on_accel_correction_timeout() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_FILTER_USE_ACC, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Int(1));
        params.set_by_id(ParamId::PARAM_INIT_TIME, ParamValue::Int(0));
        let mut estimator = QuadEstimator::default();
        let mut sensors = ProcessedSensors::<f64>::default();
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            ..Default::default()
        });

        let _ = estimate(&mut estimator, &sensors, &params, 0.002);
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 601_001,
                status: 0,
            },
            accel: [20.0, 0.0, 0.0],
            gyro: [0.0, 0.0, 0.0],
            ..Default::default()
        });

        let state = estimate(&mut estimator, &sensors, &params, 0.002);

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
        let mut sensors = ProcessedSensors::<f64>::default();
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 1.0],
            ..Default::default()
        });

        let _ = estimate(&mut estimator, &sensors, &params, 0.1);
        sensors.imu.as_mut().unwrap().header.timestamp = 101_000;
        let drifted = estimate(&mut estimator, &sensors, &params, 0.1);
        assert!(drifted.q()[3] > 0.0);

        estimator.reset();
        sensors.imu.as_mut().unwrap().header.timestamp = 201_000;
        let reset = estimate(&mut estimator, &sensors, &params, 0.1);

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
        let mut sensors = ProcessedSensors::<f64>::default();
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 1.0],
            ..Default::default()
        });

        let _ = estimate(&mut estimator, &sensors, &params, 0.002);
        sensors.imu.as_mut().unwrap().header.timestamp = 3_000;
        let state = estimate(&mut estimator, &sensors, &params, 0.002);

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
        let mut sensors = ProcessedSensors::<f64>::default();
        sensors.imu = Some(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: 1_000,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 1.0],
            ..Default::default()
        });

        let _ = estimate(&mut estimator, &sensors, &params, 0.1);
        sensors.imu.as_mut().unwrap().header.timestamp = 101_000;
        let state = estimate(&mut estimator, &sensors, &params, 0.1);

        assert!((state.q()[0] as f64 - 0.05_f64.cos()).abs() < 1e-6);
        assert!((state.q()[3] as f64 - 0.05_f64.sin()).abs() < 1e-6);
    }
}
