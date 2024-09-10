
mod data;

use data::{GNSSData};
use micro_algebra::{SVector, SMatrix};
use std::time::Duration;

// Use GotFlags::default() to get default values of `false` for all fields.
#[derive(Default)]
struct GotFlags {
    imu: bool,
    gnss: bool,
    gnss_full: bool,
    baro: bool,
    mag: bool,
    diff_pressure: bool,
    sonar: bool,
    battery: bool,
    rc: bool,
}

type Vector3 = SVector<f32, 3>;
type Matrix3 = SMatrix<f32, 3, 3>;
struct Data {
    imu_temperature: f32,
    imu_time: u64,
    accel: Vector3,
    gyro: Vector3,
    gnss_data: GNSSData,
    gnss_full: bool,
    baro_pressure: f32,
    baro_temperature: f32,
    mag: Vector3,
    diff_pressure: f32,
    diff_pressure_temp: f32,
    sonar_range: f32,
    fcu_orientation: Matrix3,
    gnss_present: bool,
    baro_present: bool,
    mag_present: bool,
    diff_pressure_present: bool,
    sonar_present: bool,
}