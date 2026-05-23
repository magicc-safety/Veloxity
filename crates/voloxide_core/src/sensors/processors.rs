use crate::errors;
use crate::math::FlightFloat;
use crate::packets::*;
use crate::params::{ParamId, ParamValue, Params};
use crate::{log_error, log_info};
use bitflags::bitflags;

fn deg_to_rad<R: FlightFloat>() -> R {
    <R as FlightFloat>::from_f32(0.017453293)
}

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct CalibrationFlags: u16 {
        const GYRO = 1 << 0;
        const ACCEL = 1 << 1;
        const BARO = 1 << 2;
        const PITOT = 1 << 3;
        const GYRO_FAILED = 1 << 4;
        const ACCEL_FAILED = 1 << 5;
        const BARO_FAILED = 1 << 6;
        const PITOT_FAILED = 1 << 7;

        // Create a convenient combination for a full IMU calibration
        const IMU = Self::GYRO.bits() | Self::ACCEL.bits();
    }
}

pub trait SensorPacketProcessor<P> {
    fn process(
        &mut self,
        packet: &mut Option<Result<P, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<P>;
}

fn take_ok_packet<P>(packet: &mut Option<Result<P, errors::SensorError>>) -> Option<P> {
    match packet.take() {
        Some(Ok(packet)) => Some(packet),
        _ => None,
    }
}

macro_rules! impl_passthrough_sensor_packet_processor {
    ($processor:ty, $packet:ty) => {
        impl SensorPacketProcessor<$packet> for $processor {
            fn process(
                &mut self,
                packet: &mut Option<Result<$packet, errors::SensorError>>,
                _flags: &mut CalibrationFlags,
                _params: &mut Params,
            ) -> Option<$packet> {
                take_ok_packet(packet)
            }
        }
    };
}

// ------------------------------
// Battery Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughBatteryProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughBatteryProcessor, BatteryPacket);

#[derive(Copy, Clone)]
pub struct BatteryProcessor {
    previous_voltage: f32,
    previous_current: f32,
    latest_packet: Option<BatteryPacket>,
    initialized: bool,
}

impl Default for BatteryProcessor {
    fn default() -> Self {
        Self {
            previous_voltage: 0.0,
            previous_current: 0.0,
            latest_packet: None,
            initialized: false,
        }
    }
}

impl BatteryProcessor {
    fn process_packet(
        &mut self,
        packet: &mut Option<Result<BatteryPacket, errors::SensorError>>,
        _flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<BatteryPacket> {
        let Some(mut packet) = take_ok_packet(packet) else {
            return self.latest_packet;
        };

        if !self.initialized {
            self.previous_voltage = param_float(params, ParamId::PARAM_VOLT_MAX);
            self.previous_current = 0.0;
            self.initialized = true;
        }

        let voltage_alpha = param_float(params, ParamId::PARAM_BATTERY_VOLTAGE_ALPHA);
        let current_alpha = param_float(params, ParamId::PARAM_BATTERY_CURRENT_ALPHA);

        packet.voltage =
            packet.voltage * (1.0 - voltage_alpha) + self.previous_voltage * voltage_alpha;
        packet.current =
            packet.current * (1.0 - current_alpha) + self.previous_current * current_alpha;

        self.previous_voltage = packet.voltage;
        self.previous_current = packet.current;
        self.latest_packet = Some(packet);

        Some(packet)
    }
}

impl SensorPacketProcessor<BatteryPacket> for BatteryProcessor {
    fn process(
        &mut self,
        packet: &mut Option<Result<BatteryPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<BatteryPacket> {
        self.process_packet(packet, flags, params)
    }
}

// ------------------------------
// IMU Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughImuProcessor;

impl<R: FlightFloat> SensorPacketProcessor<ImuPacket<R>> for PassthroughImuProcessor {
    fn process(
        &mut self,
        packet: &mut Option<Result<ImuPacket<R>, errors::SensorError>>,
        _flags: &mut CalibrationFlags,
        _params: &mut Params,
    ) -> Option<ImuPacket<R>> {
        take_ok_packet(packet)
    }
}

#[derive(Default, Copy, Clone)]
pub struct ImuCalibrationState<R: FlightFloat> {
    gyro_sum: [R; 3],
    gyro_calibration_count: u16,
    accel_sum: [R; 3],
    accel_temp_sum: R,
    accel_calibration_count: u16,
    max_accel: [R; 3],
    min_accel: [R; 3],
}

#[derive(Copy, Clone)]
pub struct ImuProcessor<R: FlightFloat> {
    calibration_state: ImuCalibrationState<R>,
}

impl<R: FlightFloat> Default for ImuProcessor<R> {
    fn default() -> Self {
        ImuProcessor::new()
    }
}

impl<R: FlightFloat> ImuProcessor<R> {
    pub fn new() -> Self {
        Self {
            calibration_state: ImuCalibrationState {
                max_accel: [<R as FlightFloat>::from_f32(-1000.0); 3],
                min_accel: [<R as FlightFloat>::from_f32(1000.0); 3],
                ..Default::default()
            },
        }
    }

    fn process_packet(
        &mut self,
        packet: &mut Option<Result<ImuPacket<R>, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<ImuPacket<R>> {
        if let Some(Ok(mut packet)) = packet.take() {
            rotate_imu_in_place(&mut packet, params);
            let is_calibrating = flags.intersects(CalibrationFlags::IMU);

            if is_calibrating {
                if flags.contains(CalibrationFlags::GYRO) {
                    self.calibration_state.gyro_sum[0] += packet.gyro[0];
                    self.calibration_state.gyro_sum[1] += packet.gyro[1];
                    self.calibration_state.gyro_sum[2] += packet.gyro[2];
                    self.calibration_state.gyro_calibration_count += 1;
                    if self.calibration_state.gyro_calibration_count > 1000 {
                        let count = <R as FlightFloat>::from_u64(
                            self.calibration_state.gyro_calibration_count as u64,
                        );
                        let bias_x = self.calibration_state.gyro_sum[0] / count;
                        let bias_y = self.calibration_state.gyro_sum[1] / count;
                        let bias_z = self.calibration_state.gyro_sum[2] / count;

                        if vector_norm([bias_x, bias_y, bias_z]) < <R as FlightFloat>::from_f32(1.0)
                        {
                            params.set_by_id(
                                ParamId::PARAM_GYRO_X_BIAS,
                                ParamValue::Float(bias_x.to_f32_lossy()),
                            );
                            params.set_by_id(
                                ParamId::PARAM_GYRO_Y_BIAS,
                                ParamValue::Float(bias_y.to_f32_lossy()),
                            );
                            params.set_by_id(
                                ParamId::PARAM_GYRO_Z_BIAS,
                                ParamValue::Float(bias_z.to_f32_lossy()),
                            );
                            log_info!("Gyro Calibration complete!");
                        } else {
                            flags.insert(CalibrationFlags::GYRO_FAILED);
                            log_error!("Gyro calibration failed");
                        }

                        self.calibration_state.gyro_sum = [<R as FlightFloat>::from_f32(0.0); 3];
                        self.calibration_state.gyro_calibration_count = 0;
                        flags.remove(CalibrationFlags::GYRO);
                    }
                }

                if flags.contains(CalibrationFlags::ACCEL) {
                    let gravity = <R as FlightFloat>::from_f32(9.80665);
                    self.calibration_state.accel_sum[0] += packet.accel[0];
                    self.calibration_state.accel_sum[1] += packet.accel[1];
                    self.calibration_state.accel_sum[2] += packet.accel[2] + gravity;
                    self.calibration_state.accel_temp_sum +=
                        <R as FlightFloat>::from_f32(packet.temperature);
                    self.calibration_state.accel_calibration_count += 1;

                    self.calibration_state.max_accel[0] =
                        self.calibration_state.max_accel[0].max(packet.accel[0]);
                    self.calibration_state.min_accel[0] =
                        self.calibration_state.min_accel[0].min(packet.accel[0]);
                    self.calibration_state.max_accel[1] =
                        self.calibration_state.max_accel[1].max(packet.accel[1]);
                    self.calibration_state.min_accel[1] =
                        self.calibration_state.min_accel[1].min(packet.accel[1]);
                    self.calibration_state.max_accel[2] =
                        self.calibration_state.max_accel[2].max(packet.accel[2]);
                    self.calibration_state.min_accel[2] =
                        self.calibration_state.min_accel[2].min(packet.accel[2]);
                    if self.calibration_state.accel_calibration_count > 1000 {
                        let accel_delta_x = self.calibration_state.max_accel[0]
                            - self.calibration_state.min_accel[0];
                        let accel_delta_y = self.calibration_state.max_accel[1]
                            - self.calibration_state.min_accel[1];
                        let accel_delta_z = self.calibration_state.max_accel[2]
                            - self.calibration_state.min_accel[2];
                        let max_delta = (accel_delta_x * accel_delta_x
                            + accel_delta_y * accel_delta_y
                            + accel_delta_z * accel_delta_z)
                            .sqrt();

                        if max_delta < <R as FlightFloat>::from_f32(1.0) {
                            let count = <R as FlightFloat>::from_u64(
                                self.calibration_state.accel_calibration_count as u64,
                            );
                            let temp_comp_x = if let ParamValue::Float(v) =
                                params.get_by_id(ParamId::PARAM_ACC_X_TEMP_COMP)
                            {
                                <R as FlightFloat>::from_f32(v)
                            } else {
                                <R as FlightFloat>::from_f32(0.0)
                            };
                            let temp_comp_y = if let ParamValue::Float(v) =
                                params.get_by_id(ParamId::PARAM_ACC_Y_TEMP_COMP)
                            {
                                <R as FlightFloat>::from_f32(v)
                            } else {
                                <R as FlightFloat>::from_f32(0.0)
                            };
                            let temp_comp_z = if let ParamValue::Float(v) =
                                params.get_by_id(ParamId::PARAM_ACC_Z_TEMP_COMP)
                            {
                                <R as FlightFloat>::from_f32(v)
                            } else {
                                <R as FlightFloat>::from_f32(0.0)
                            };

                            let bias_x = (self.calibration_state.accel_sum[0]
                                - temp_comp_x * self.calibration_state.accel_temp_sum)
                                / count;
                            let bias_y = (self.calibration_state.accel_sum[1]
                                - temp_comp_y * self.calibration_state.accel_temp_sum)
                                / count;
                            let bias_z = (self.calibration_state.accel_sum[2]
                                - temp_comp_z * self.calibration_state.accel_temp_sum)
                                / count;

                            if vector_norm([bias_x, bias_y, bias_z])
                                < <R as FlightFloat>::from_f32(3.0)
                            {
                                params.set_by_id(
                                    ParamId::PARAM_ACC_X_BIAS,
                                    ParamValue::Float(bias_x.to_f32_lossy()),
                                );
                                params.set_by_id(
                                    ParamId::PARAM_ACC_Y_BIAS,
                                    ParamValue::Float(bias_y.to_f32_lossy()),
                                );
                                params.set_by_id(
                                    ParamId::PARAM_ACC_Z_BIAS,
                                    ParamValue::Float(bias_z.to_f32_lossy()),
                                );
                                log_info!("Accelerometer Calibration Complete!");
                            } else {
                                flags.insert(CalibrationFlags::ACCEL_FAILED);
                                log_error!("Accelerometer calibration failed");
                            }
                        } else {
                            flags.insert(CalibrationFlags::ACCEL_FAILED);
                            log_error!("Accelerometer calibration failed: too much movement");
                        }

                        self.calibration_state.accel_sum = [<R as FlightFloat>::from_f32(0.0); 3];
                        self.calibration_state.accel_calibration_count = 0;
                        self.calibration_state.accel_temp_sum = <R as FlightFloat>::from_f32(0.0);
                        self.calibration_state.max_accel =
                            [<R as FlightFloat>::from_f32(-1000.0); 3];
                        self.calibration_state.min_accel =
                            [<R as FlightFloat>::from_f32(1000.0); 3];
                        flags.remove(CalibrationFlags::ACCEL);
                    }
                }
            }
            packet.gyro[0] -=
                <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_GYRO_X_BIAS));
            packet.gyro[1] -=
                <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_GYRO_Y_BIAS));
            packet.gyro[2] -=
                <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_GYRO_Z_BIAS));

            let temp = <R as FlightFloat>::from_f32(packet.temperature);
            packet.accel[0] -=
                <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_ACC_X_TEMP_COMP))
                    * temp
                    + <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_ACC_X_BIAS));
            packet.accel[1] -=
                <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_ACC_Y_TEMP_COMP))
                    * temp
                    + <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_ACC_Y_BIAS));
            packet.accel[2] -=
                <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_ACC_Z_TEMP_COMP))
                    * temp
                    + <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_ACC_Z_BIAS));
            Some(packet)
        } else {
            None
        }
    }
}

impl<R: FlightFloat> SensorPacketProcessor<ImuPacket<R>> for ImuProcessor<R> {
    fn process(
        &mut self,
        packet: &mut Option<Result<ImuPacket<R>, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<ImuPacket<R>> {
        self.process_packet(packet, flags, params)
    }
}

// ------------------------------
// Baro Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughBaroProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughBaroProcessor, BaroPacket);

const SENSOR_CAL_DELAY_CYCLES: u16 = 128;
const SENSOR_CAL_CYCLES: u16 = 127;
const BARO_MAX_CALIBRATION_VARIANCE: f32 = 25.0;

#[derive(Default, Copy, Clone)]
pub struct BaroCalibrationState {
    mean: f32,
    m2: f32, // Sum of squares of differences from the current mean
    count: u16,
    last_iter_ms: u32,
    calibrated: bool,
    request_active: bool,
}

#[derive(Default, Copy, Clone)]
pub struct BaroProcessor {
    calibration_state: BaroCalibrationState,
}

impl BaroProcessor {
    pub fn new() -> Self {
        Self::default()
    }

    fn process_packet(
        &mut self,
        packet: &mut Option<Result<BaroPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<BaroPacket> {
        if let Some(Ok(mut packet)) = packet.take() {
            if flags.contains(CalibrationFlags::BARO) && !self.calibration_state.request_active {
                self.calibration_state = BaroCalibrationState::default();
                self.calibration_state.request_active = true;
            }
            if !self.calibration_state.calibrated {
                self.calibrate(&packet, flags, params);
            }
            packet.altitude = pressure_to_altitude(packet.pressure);
            Some(packet)
        } else {
            None
        }
    }

    fn calibrate(
        &mut self,
        packet: &BaroPacket,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) {
        let now_ms = (packet.header.timestamp / 1000) as u32;
        if now_ms <= self.calibration_state.last_iter_ms + 20 {
            return;
        }

        self.calibration_state.count += 1;
        let total_cycles = SENSOR_CAL_DELAY_CYCLES + SENSOR_CAL_CYCLES;

        if self.calibration_state.count > total_cycles {
            if self.calibration_state.m2 < BARO_MAX_CALIBRATION_VARIANCE {
                params.set_by_id(
                    ParamId::PARAM_BARO_BIAS,
                    ParamValue::Float(self.calibration_state.mean),
                );
                let ground_alt = pressure_to_altitude(self.calibration_state.mean);
                params.set_by_id(ParamId::PARAM_GROUND_LEVEL, ParamValue::Float(ground_alt));
                self.calibration_state.calibrated = true;
                flags.remove(CalibrationFlags::BARO);
                self.calibration_state.request_active = false;
            } else {
                flags.insert(CalibrationFlags::BARO_FAILED);
                log_error!("Baro calibration failed");
            }

            self.calibration_state.mean = 0.0;
            self.calibration_state.m2 = 0.0;
            self.calibration_state.count = 0;
        } else if self.calibration_state.count > SENSOR_CAL_DELAY_CYCLES {
            let n = (self.calibration_state.count - SENSOR_CAL_DELAY_CYCLES) as f32;
            let delta = packet.pressure - self.calibration_state.mean;
            self.calibration_state.mean += delta / n;
            let delta2 = packet.pressure - self.calibration_state.mean;
            self.calibration_state.m2 += delta * delta2 / (SENSOR_CAL_CYCLES - 1) as f32;
        }

        self.calibration_state.last_iter_ms = now_ms;
    }
}

fn pressure_to_altitude(pressure: f32) -> f32 {
    pressure_to_altitude_real::<f32>(pressure)
}

fn pressure_to_altitude_real<R: FlightFloat>(pressure: R) -> R {
    <R as FlightFloat>::from_f32(44330.0)
        * (<R as FlightFloat>::from_f32(1.0)
            - (pressure / <R as FlightFloat>::from_f32(101325.0))
                .powf(<R as FlightFloat>::from_f32(0.190295)))
}

fn indicated_airspeed(dp: f32) -> f32 {
    indicated_airspeed_real::<f32>(dp)
}

fn indicated_airspeed_real<R: FlightFloat>(dp: R) -> R {
    const RHO: f32 = 1.225;
    (<R as FlightFloat>::from_f32(2.0) * dp.abs() / <R as FlightFloat>::from_f32(RHO)).sqrt()
        * dp.signum()
}

impl SensorPacketProcessor<BaroPacket> for BaroProcessor {
    fn process(
        &mut self,
        packet: &mut Option<Result<BaroPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<BaroPacket> {
        self.process_packet(packet, flags, params)
    }
}

// ------------------------------
// Pitot Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughPitotProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughPitotProcessor, PitotPacket);

const PITOT_MAX_CALIBRATION_VARIANCE: f32 = 100.0;

#[derive(Default, Copy, Clone)]
pub struct PitotCalibrationState {
    mean: f32,
    m2: f32,
    count: u16,
    last_iter_ms: u32,
    calibrated: bool,
    request_active: bool,
}

#[derive(Default, Copy, Clone)]
pub struct PitotProcessor {
    calibration_state: PitotCalibrationState,
}

impl PitotProcessor {
    pub fn new() -> Self {
        Self::default()
    }

    fn process_packet(
        &mut self,
        packet: &mut Option<Result<PitotPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<PitotPacket> {
        if let Some(Ok(mut packet)) = packet.take() {
            if flags.contains(CalibrationFlags::PITOT) && !self.calibration_state.request_active {
                self.calibration_state = PitotCalibrationState::default();
                self.calibration_state.request_active = true;
            }
            if !self.calibration_state.calibrated {
                self.calibrate(&packet, flags, params);
            }

            packet.differential_pressure -= param_float(params, ParamId::PARAM_DIFF_PRESS_BIAS);

            let dp = packet.differential_pressure;
            packet.indicated_airspeed = indicated_airspeed(dp);

            Some(packet)
        } else {
            None
        }
    }

    fn calibrate(
        &mut self,
        packet: &PitotPacket,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) {
        let now_ms = (packet.header.timestamp / 1000) as u32;
        if now_ms <= self.calibration_state.last_iter_ms + 20 {
            return;
        }

        self.calibration_state.count += 1;
        let total_cycles = SENSOR_CAL_DELAY_CYCLES + SENSOR_CAL_CYCLES;

        if self.calibration_state.count > total_cycles {
            if self.calibration_state.m2 < PITOT_MAX_CALIBRATION_VARIANCE {
                params.set_by_id(
                    ParamId::PARAM_DIFF_PRESS_BIAS,
                    ParamValue::Float(self.calibration_state.mean),
                );
                self.calibration_state.calibrated = true;
                flags.remove(CalibrationFlags::PITOT);
                self.calibration_state.request_active = false;
            } else {
                flags.insert(CalibrationFlags::PITOT_FAILED);
                log_error!("Airspeed calibration failed");
            }

            self.calibration_state.mean = 0.0;
            self.calibration_state.m2 = 0.0;
            self.calibration_state.count = 0;
        } else if self.calibration_state.count > SENSOR_CAL_DELAY_CYCLES {
            let n = (self.calibration_state.count - SENSOR_CAL_DELAY_CYCLES) as f32;
            let delta = packet.differential_pressure - self.calibration_state.mean;
            self.calibration_state.mean += delta / n;
            let delta2 = packet.differential_pressure - self.calibration_state.mean;
            self.calibration_state.m2 += delta * delta2 / (SENSOR_CAL_CYCLES - 1) as f32;
        }

        self.calibration_state.last_iter_ms = now_ms;
    }
}

impl SensorPacketProcessor<PitotPacket> for PitotProcessor {
    fn process(
        &mut self,
        packet: &mut Option<Result<PitotPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<PitotPacket> {
        self.process_packet(packet, flags, params)
    }
}

// ------------------------------
// Mag Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughMagProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughMagProcessor, MagPacket);

#[derive(Default, Copy, Clone)]
pub struct MagProcessor;

impl MagProcessor {
    fn process_packet(
        &mut self,
        packet: &mut Option<Result<MagPacket, errors::SensorError>>,
        _flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<MagPacket> {
        if let Some(Ok(mut packet)) = packet.take() {
            rotate_mag_in_place(&mut packet, params);
            // Apply hard-iron biases from parameters
            let mag_hard_x = packet.flux[0] - param_float(params, ParamId::PARAM_MAG_X_BIAS);
            let mag_hard_y = packet.flux[1] - param_float(params, ParamId::PARAM_MAG_Y_BIAS);
            let mag_hard_z = packet.flux[2] - param_float(params, ParamId::PARAM_MAG_Z_BIAS);

            // Get soft-iron correction matrix parameters
            let a00 = param_float(params, ParamId::PARAM_MAG_A00_COMP);
            let a01 = param_float(params, ParamId::PARAM_MAG_A01_COMP);
            let a02 = param_float(params, ParamId::PARAM_MAG_A02_COMP);
            let a10 = param_float(params, ParamId::PARAM_MAG_A10_COMP);
            let a11 = param_float(params, ParamId::PARAM_MAG_A11_COMP);
            let a12 = param_float(params, ParamId::PARAM_MAG_A12_COMP);
            let a20 = param_float(params, ParamId::PARAM_MAG_A20_COMP);
            let a21 = param_float(params, ParamId::PARAM_MAG_A21_COMP);
            let a22 = param_float(params, ParamId::PARAM_MAG_A22_COMP);

            // Apply soft-iron corrections (matrix multiplication)
            packet.flux[0] = a00 * mag_hard_x + a01 * mag_hard_y + a02 * mag_hard_z;
            packet.flux[1] = a10 * mag_hard_x + a11 * mag_hard_y + a12 * mag_hard_z;
            packet.flux[2] = a20 * mag_hard_x + a21 * mag_hard_y + a22 * mag_hard_z;

            Some(packet)
        } else {
            None
        }
    }
}

fn param_float(params: &Params, param_id: ParamId) -> f32 {
    match params.get_by_id(param_id) {
        ParamValue::Float(value) => value,
        _ => 0.0,
    }
}

fn vector_norm<R: FlightFloat>(vector: [R; 3]) -> R {
    (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt()
}

fn rotate_imu_in_place<R: FlightFloat>(packet: &mut ImuPacket<R>, params: &Params) {
    let rotation = euler_rotation_matrix(
        <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_IMU_ROLL)) * deg_to_rad(),
        <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_IMU_PITCH)) * deg_to_rad(),
        <R as FlightFloat>::from_f32(param_float(params, ParamId::PARAM_IMU_YAW)) * deg_to_rad(),
    );
    packet.accel = rotate_vector_real(rotation, packet.accel);
    packet.gyro = rotate_vector_real(rotation, packet.gyro);
}

fn rotate_mag_in_place(packet: &mut MagPacket, params: &Params) {
    let rotation = euler_rotation_matrix(
        param_float(params, ParamId::PARAM_MAG_ROLL) * deg_to_rad::<f32>(),
        param_float(params, ParamId::PARAM_MAG_PITCH) * deg_to_rad::<f32>(),
        param_float(params, ParamId::PARAM_MAG_YAW) * deg_to_rad::<f32>(),
    );
    let rotated = rotate_vector_real(rotation, [packet.flux[0], packet.flux[1], packet.flux[2]]);
    packet.flux = rotated;
}

fn euler_rotation_matrix<R: FlightFloat>(roll: R, pitch: R, yaw: R) -> [[R; 3]; 3] {
    let sr = roll.sin();
    let cr = roll.cos();
    let sp = pitch.sin();
    let cp = pitch.cos();
    let sy = yaw.sin();
    let cy = yaw.cos();

    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

fn rotate_vector_real<R: FlightFloat>(rotation: [[R; 3]; 3], vector: [R; 3]) -> [R; 3] {
    [
        rotation[0][0] * vector[0] + rotation[0][1] * vector[1] + rotation[0][2] * vector[2],
        rotation[1][0] * vector[0] + rotation[1][1] * vector[1] + rotation[1][2] * vector[2],
        rotation[2][0] * vector[0] + rotation[2][1] * vector[1] + rotation[2][2] * vector[2],
    ]
}

impl SensorPacketProcessor<MagPacket> for MagProcessor {
    fn process(
        &mut self,
        packet: &mut Option<Result<MagPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Option<MagPacket> {
        self.process_packet(packet, flags, params)
    }
}

// ------------------------------
// Rc Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughRcProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughRcProcessor, RcPacket);

// ------------------------------
// Range Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughRangeProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughRangeProcessor, RangePacket);

// ------------------------------
// GNSS Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughGNSSProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughGNSSProcessor, GNSSPacket);

// ------------------------------
// PPS Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughPpsProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughPpsProcessor, PpsPacket);

// ------------------------------
// Attitude Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughAttitudeProcessor;
impl_passthrough_sensor_packet_processor!(PassthroughAttitudeProcessor, AttitudePacket);

#[cfg(test)]
mod tests {
    use super::*;

    fn process_one<P, Proc>(processor: &mut Proc, packet: P, params: &mut Params) -> P
    where
        Proc: SensorPacketProcessor<P>,
    {
        let mut raw = Some(Ok(packet));
        let mut flags = CalibrationFlags::empty();
        processor.process(&mut raw, &mut flags, params).unwrap()
    }

    #[test]
    fn imu_processor_applies_rosflight_orientation_before_correction() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_IMU_YAW, ParamValue::Float(90.0));
        params.set_by_id(ParamId::PARAM_GYRO_Y_BIAS, ParamValue::Float(0.5));
        let mut processor = ImuProcessor::<f64>::new();

        let processed: ImuPacket<f64> = process_one(
            &mut processor,
            ImuPacket {
                accel: [1.0, 0.0, 0.0],
                gyro: [1.0, 0.0, 0.0],
                ..Default::default()
            },
            &mut params,
        );

        assert!(processed.accel[0].abs() < 1e-6);
        assert!((processed.accel[1] - 1.0).abs() < 1e-6);
        assert!(processed.gyro[0].abs() < 1e-6);
        assert!((processed.gyro[1] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn imu_processor_matches_rosflight_c_bias_and_temperature_compensation() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        params.set_by_id(ParamId::PARAM_GYRO_Y_BIAS, ParamValue::Float(-0.2));
        params.set_by_id(ParamId::PARAM_GYRO_Z_BIAS, ParamValue::Float(0.3));
        params.set_by_id(ParamId::PARAM_ACC_X_BIAS, ParamValue::Float(0.4));
        params.set_by_id(ParamId::PARAM_ACC_Y_BIAS, ParamValue::Float(-0.5));
        params.set_by_id(ParamId::PARAM_ACC_Z_BIAS, ParamValue::Float(0.6));
        params.set_by_id(ParamId::PARAM_ACC_X_TEMP_COMP, ParamValue::Float(0.01));
        params.set_by_id(ParamId::PARAM_ACC_Y_TEMP_COMP, ParamValue::Float(-0.02));
        params.set_by_id(ParamId::PARAM_ACC_Z_TEMP_COMP, ParamValue::Float(0.03));
        let mut processor = ImuProcessor::<f64>::new();

        let processed: ImuPacket<f64> = process_one(
            &mut processor,
            ImuPacket {
                accel: [1.4, -2.5, -8.00665],
                gyro: [0.6, -0.7, 1.1],
                temperature: 20.0,
                ..Default::default()
            },
            &mut params,
        );

        assert!((processed.gyro[0] - 0.5).abs() < 1e-6);
        assert!((processed.gyro[1] + 0.5).abs() < 1e-6);
        assert!((processed.gyro[2] - 0.8).abs() < 1e-6);
        assert!((processed.accel[0] - 0.8).abs() < 1e-6);
        assert!((processed.accel[1] + 1.6).abs() < 1e-6);
        assert!((processed.accel[2] + 9.20665).abs() < 1e-5);
    }

    #[test]
    fn mag_processor_applies_orientation_hard_iron_and_rosflight_matrix() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_MAG_YAW, ParamValue::Float(90.0));
        params.set_by_id(ParamId::PARAM_MAG_Y_BIAS, ParamValue::Float(1.0));
        params.set_by_id(ParamId::PARAM_MAG_A00_COMP, ParamValue::Float(2.0));
        params.set_by_id(ParamId::PARAM_MAG_A01_COMP, ParamValue::Float(3.0));
        params.set_by_id(ParamId::PARAM_MAG_A02_COMP, ParamValue::Float(5.0));
        params.set_by_id(ParamId::PARAM_MAG_A10_COMP, ParamValue::Float(7.0));
        params.set_by_id(ParamId::PARAM_MAG_A11_COMP, ParamValue::Float(11.0));
        params.set_by_id(ParamId::PARAM_MAG_A12_COMP, ParamValue::Float(13.0));
        params.set_by_id(ParamId::PARAM_MAG_A20_COMP, ParamValue::Float(17.0));
        params.set_by_id(ParamId::PARAM_MAG_A21_COMP, ParamValue::Float(19.0));
        params.set_by_id(ParamId::PARAM_MAG_A22_COMP, ParamValue::Float(23.0));
        let mut processor = MagProcessor;

        let processed: MagPacket = process_one(
            &mut processor,
            MagPacket {
                flux: [1.0, 0.0, 2.0],
                ..Default::default()
            },
            &mut params,
        );

        // Yaw rotation maps [1, 0, 2] to [0, 1, 2], then hard-iron Y bias
        // produces [0, 0, 2]. The soft-iron rows are therefore 10, 26, 46.
        assert!((processed.flux[0] - 10.0).abs() < 1e-5);
        assert!((processed.flux[1] - 26.0).abs() < 1e-5);
        assert!((processed.flux[2] - 46.0).abs() < 1e-5);
    }

    #[test]
    fn baro_processor_matches_rosflight_correction_without_subtracting_bias() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_BARO_BIAS, ParamValue::Float(1000.0));
        let mut processor = BaroProcessor::new();

        let processed: BaroPacket = process_one(
            &mut processor,
            BaroPacket {
                pressure: 80_000.0,
                ..Default::default()
            },
            &mut params,
        );

        let expected = pressure_to_altitude(80_000.0) as f32;
        assert_eq!(processed.pressure, 80_000.0);
        assert!((processed.altitude - expected).abs() < 1e-3);
    }

    #[test]
    fn baro_processor_calibration_uses_rosflight_timing_and_mean() {
        let mut params = Params::new();
        let mut processor = BaroProcessor::new();
        let mut flags = CalibrationFlags::BARO;

        for sample in 0..=SENSOR_CAL_DELAY_CYCLES + SENSOR_CAL_CYCLES {
            let mut raw = Some(Ok(BaroPacket {
                header: RosflightPacketHeader {
                    timestamp: (sample as u64 + 1) * 21_000,
                    status: 0,
                },
                pressure: 90_000.0,
                ..Default::default()
            }));
            let _ = processor.process(&mut raw, &mut flags, &mut params);
        }

        assert!(!flags.contains(CalibrationFlags::BARO));
        assert_eq!(
            params.get_by_id(ParamId::PARAM_BARO_BIAS),
            ParamValue::Float(90_000.0)
        );
    }

    #[test]
    fn pitot_processor_calibrates_then_corrects_pressure_and_airspeed() {
        let mut params = Params::new();
        let mut processor = PitotProcessor::new();
        let mut flags = CalibrationFlags::PITOT;

        for sample in 0..=SENSOR_CAL_DELAY_CYCLES + SENSOR_CAL_CYCLES {
            let mut raw = Some(Ok(PitotPacket {
                header: RosflightPacketHeader {
                    timestamp: (sample as u64 + 1) * 21_000,
                    status: 0,
                },
                differential_pressure: 4.0,
                ..Default::default()
            }));
            let _ = processor.process(&mut raw, &mut flags, &mut params);
        }

        assert!(!flags.contains(CalibrationFlags::PITOT));
        assert_eq!(
            params.get_by_id(ParamId::PARAM_DIFF_PRESS_BIAS),
            ParamValue::Float(4.0)
        );

        let processed: PitotPacket = process_one(
            &mut processor,
            PitotPacket {
                differential_pressure: 6.0,
                ..Default::default()
            },
            &mut params,
        );
        assert_eq!(processed.differential_pressure, 2.0);
        assert!((processed.indicated_airspeed - (4.0_f32 / 1.225).sqrt()).abs() < 1e-6);
    }

    #[test]
    fn battery_processor_matches_rosflight_lpf_initialization() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_VOLT_MAX, ParamValue::Float(20.0));
        params.set_by_id(ParamId::PARAM_BATTERY_VOLTAGE_ALPHA, ParamValue::Float(0.5));
        params.set_by_id(
            ParamId::PARAM_BATTERY_CURRENT_ALPHA,
            ParamValue::Float(0.25),
        );
        let mut processor = BatteryProcessor::default();

        let first = process_one(
            &mut processor,
            BatteryPacket {
                voltage: 10.0,
                current: 8.0,
                ..Default::default()
            },
            &mut params,
        );
        let second = process_one(
            &mut processor,
            BatteryPacket {
                voltage: 14.0,
                current: 4.0,
                ..Default::default()
            },
            &mut params,
        );

        assert_eq!(first.voltage, 15.0);
        assert_eq!(first.current, 6.0);
        assert_eq!(second.voltage, 14.5);
        assert_eq!(second.current, 4.5);
    }

    #[test]
    fn battery_processor_reuses_latest_sample_between_battery_updates() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_VOLT_MAX, ParamValue::Float(23.5));
        params.set_by_id(ParamId::PARAM_BATTERY_VOLTAGE_ALPHA, ParamValue::Float(0.0));
        params.set_by_id(ParamId::PARAM_BATTERY_CURRENT_ALPHA, ParamValue::Float(0.0));
        let mut processor = BatteryProcessor::default();
        let mut flags = CalibrationFlags::empty();
        let mut raw = Some(Ok(BatteryPacket {
            voltage: 22.0,
            current: 3.0,
            ..Default::default()
        }));

        let first = processor
            .process(&mut raw, &mut flags, &mut params)
            .unwrap();
        let mut no_new_packet = None;
        let second = processor
            .process(&mut no_new_packet, &mut flags, &mut params)
            .unwrap();

        assert_eq!(first.voltage, 22.0);
        assert_eq!(second.voltage, first.voltage);
        assert_eq!(second.current, first.current);
    }

    #[test]
    fn imu_calibration_uses_rosflight_gravity_sign_and_sanity_gates() {
        let mut params = Params::new();
        let mut processor = ImuProcessor::<f64>::new();
        let mut flags = CalibrationFlags::ACCEL | CalibrationFlags::GYRO;

        for seq in 0..=1000 {
            let mut raw = Some(Ok(ImuPacket {
                accel: [0.1, -0.2, -9.50665],
                gyro: [0.2, -0.1, 0.3],
                seq,
                ..Default::default()
            }));
            let _ = processor.process(&mut raw, &mut flags, &mut params);
        }

        assert!(!flags.intersects(CalibrationFlags::IMU));
        assert_eq!(
            params.get_by_id(ParamId::PARAM_ACC_X_BIAS),
            ParamValue::Float(0.1)
        );
        assert_eq!(
            params.get_by_id(ParamId::PARAM_ACC_Y_BIAS),
            ParamValue::Float(-0.2)
        );
        assert!((param_float(&params, ParamId::PARAM_ACC_Z_BIAS) - 0.3).abs() < 1e-5);
        assert_eq!(
            params.get_by_id(ParamId::PARAM_GYRO_X_BIAS),
            ParamValue::Float(0.2)
        );
    }
}
