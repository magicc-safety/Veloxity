// /**
// ******************************************************************************
// * File     : ros_messages.rs
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

use serde::{Deserialize, Serialize};

// ============ Standard ROS Types ============

#[derive(Serialize, Deserialize)]
struct TriggerRequest;

#[derive(Serialize, Deserialize)]
struct TriggerResponse {
    success: bool,
    message: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}


// ============================ WORKING RosFlight Sensors =================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ImuData {
    pub header: Header,
    pub orientation: Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct MagneticField {
    pub header: Header,
    pub magnetic_field: Vector3,
    pub magnetic_field_covariance: [f64; 9],
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Barometer {
    pub header: Header,
    pub altitude: f32,
    pub pressure: f32,
    pub temperature: f32,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct GNSS {
    pub header: Header,
    pub fix_type: u8, 
    pub num_sat: u8,
    pub lat: f64,
    pub lon: f64,
    pub alt: f32,
    pub horizontal_accuracy: f32,
    pub vertical_accuracy: f32,
    pub vel_n: f32,
    pub vel_e: f32,
    pub vel_d: f32,
    pub speed_accuracy: f32,
    pub rosflight_timestamp: f64,
}
























































// ============================ BIG WORK IN PROGRESS DONT USE ANYTHING BELOW HERE IT DOESNT WORK =================================
// ============ ROSflight Messages ============

// Airspeed.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Airspeed {
    pub header: Header,
    pub differential_pressure: f64,
    pub temperature: f64,
}

// Attitude.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Attitude {
    pub header: Header,
    pub quaternion: Quaternion,
    pub angular_velocity: Vector3,
}

// AuxCommand.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct AuxCommand {
    pub header: Header,
    pub type_array: Vec<u8>,
    pub values: Vec<f32>,
}

impl AuxCommand {
    pub const AUX_COMMAND_DISABLED: u8 = 0;
    pub const AUX_COMMAND_SERVO: u8 = 1;
    pub const AUX_COMMAND_MOTOR: u8 = 2;
    pub const NUM_AUX_CHANNELS: u8 = 14;
}

// BatteryStatus.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct BatteryStatus {
    pub header: Header,
    pub voltage: f32,
    pub current: f32,
}

// Command.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Command {
    pub header: Header,
    pub mode: u8,
    pub ignore: u8,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub f: f32,
}

impl Command {
    pub const MODE_PASS_THROUGH: u8 = 0;
    pub const MODE_ROLL_PITCH_YAWRATE_THROTTLE: u8 = 1;
    pub const MODE_ROLL_PITCH_YAWRATE_ALTITUDE: u8 = 2;

    pub const IGNORE_NONE: u8 = 0;
    pub const IGNORE_X: u8 = 1;
    pub const IGNORE_Y: u8 = 2;
    pub const IGNORE_Z: u8 = 4;
    pub const IGNORE_F: u8 = 8;
}

// Error.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Error {
    pub header: Header,
    pub error_code: u8,
    pub error_message: String,
    pub on_error: bool,
    pub rearm: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct PwmOutput {
    pub header: Header,
    pub values: [u16; 14], // Correct type for the simulator!
}


#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct RCRaw {
    pub header: Header,
    // Use a fixed-size array to match the ROS definition
    pub values: [u16; 8],
}

// Status.msg
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct Status {
    pub header: Header,
    pub armed: bool,
    pub failsafe: bool,
    pub rc_override: bool,
    pub offboard: bool,
    pub control_mode: u8,
    pub num_errors: u8,
    pub loop_time_us: u16,
}

impl Status {
    pub const CONTROL_MODE_ANGLE: u8 = 0;
    pub const CONTROL_MODE_RATE: u8 = 1;
    pub const CONTROL_MODE_PASS_THROUGH: u8 = 2;
}

// ============ ROSflight Services ============

// ParamFile.srv
pub mod param_file {
    use super::*;

    #[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
    pub struct Request {
        pub filename: String,
    }

    #[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
    pub struct Response {
        pub success: bool,
    }
}

// ParamGet.srv
pub mod param_get {
    use super::*;

    #[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
    pub struct Request {
        pub name: String,
    }

    #[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
    pub struct Response {
        pub exists: bool,
        pub value: f64,
    }
}

// ParamSet.srv
pub mod param_set {
    use super::*;

    #[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
    pub struct Request {
        pub name: String,
        pub value: f64,
    }

    #[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
    pub struct Response {
        pub exists: bool,
    }
}
