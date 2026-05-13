// /**
// ******************************************************************************
// * File     : estimator_test.rs
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
// **/
use csv::{ReaderBuilder, WriterBuilder};
use serde::{Deserialize, Serialize};
use std::error::Error;
use std::fs::File;

// Import the necessary items from your actual library crate
use rustflight_core::{
    estimator::NamedEstimator, estimator::quad_estimator::QuadEstimator, packets::ImuPacket,
    params::Params, sensors::ProcessedSensors,
};

// Struct to deserialize a row from the input CSV (with truth values)
#[derive(Debug, Deserialize)]
struct ImuRecord {
    time: f64,
    accel_x: f64,
    accel_y: f64,
    accel_z: f64,
    gyro_x: f64,
    gyro_y: f64,
    gyro_z: f64,
    true_quat_w: f64,
    true_quat_x: f64,
    true_quat_y: f64,
    true_quat_z: f64,
    true_bias_x: f64,
    true_bias_y: f64,
    true_bias_z: f64,
    true_omega_x: f64,
    true_omega_y: f64,
    true_omega_z: f64,
}

// Struct to serialize a row to the output CSV (with truth values passed through)
#[derive(Debug, Serialize)]
struct OutputRecord {
    time: f64,
    est_roll_rad: f64,
    est_pitch_rad: f64,
    est_yaw_rad: f64,
    est_bias_x: f64,
    est_bias_y: f64,
    est_bias_z: f64,
    meas_gyro_x: f64,
    meas_gyro_y: f64,
    meas_gyro_z: f64,
    true_quat_w: f64,
    true_quat_x: f64,
    true_quat_y: f64,
    true_quat_z: f64,
    true_bias_x: f64,
    true_bias_y: f64,
    true_bias_z: f64,
    true_omega_x: f64,
    true_omega_y: f64,
    true_omega_z: f64,
}

#[test]
fn run_mahony_filter_against_python_data() -> Result<(), Box<dyn Error>> {
    // --- Setup ---
    let mut estimator = QuadEstimator::default();
    let params = Params::new(); // Create default parameters for test

    // Cargo runs tests from the project root, so paths are relative to that.
    let input_path = "tests/estimator/imu_sensor_data.csv";
    let output_path = "tests/estimator/rust_estimator_results.csv";

    let file = File::open(input_path)?;
    let mut rdr = ReaderBuilder::new().has_headers(true).from_reader(file);
    let mut wtr = WriterBuilder::new().from_path(output_path)?;

    println!(
        "\nReading from '{}' and writing to '{}'...",
        input_path, output_path
    );

    // --- Processing Loop ---
    let mut previous_time = None;
    for result in rdr.deserialize() {
        let record: ImuRecord = result?;

        let imu_packet = ImuPacket {
            accel: [record.accel_x, record.accel_y, record.accel_z],
            gyro: [record.gyro_x, record.gyro_y, record.gyro_z],
            // Add any other fields your ImuPacket needs, e.g., a header.
            ..ImuPacket::default()
        };

        let dt = previous_time
            .map(|previous| record.time - previous)
            .filter(|dt| *dt > 0.0)
            .unwrap_or(1.0 / 400.0);
        previous_time = Some(record.time);

        let mut sensors = ProcessedSensors::default();
        sensors.imu = Some(imu_packet);

        let state = estimator.estimate_named(&sensors, &params, dt);

        let euler_angles = state.q_hat.to_euler_angles();

        wtr.serialize(OutputRecord {
            time: record.time,
            est_roll_rad: euler_angles[0],
            est_pitch_rad: euler_angles[1],
            est_yaw_rad: euler_angles[2],
            est_bias_x: state.b_hat[0],
            est_bias_y: state.b_hat[1],
            est_bias_z: state.b_hat[2],
            meas_gyro_x: record.gyro_x,
            meas_gyro_y: record.gyro_y,
            meas_gyro_z: record.gyro_z,
            true_quat_w: record.true_quat_w,
            true_quat_x: record.true_quat_x,
            true_quat_y: record.true_quat_y,
            true_quat_z: record.true_quat_z,
            true_bias_x: record.true_bias_x,
            true_bias_y: record.true_bias_y,
            true_bias_z: record.true_bias_z,
            true_omega_x: record.true_omega_x,
            true_omega_y: record.true_omega_y,
            true_omega_z: record.true_omega_z,
        })?;
    }

    wtr.flush()?;
    println!("Processing complete.");

    Ok(())
}
