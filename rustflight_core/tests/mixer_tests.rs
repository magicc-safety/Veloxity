// /**
// ******************************************************************************
// * File     : mixer_tests.rs
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

use rustflight_core::mixer::Mixer;
use rustflight_core::mixer::quad_mixer::{QuadMixer, VehicleType};
use rustflight_core::controller::quad_controller::MixerInput;
use micro_algebra::stack::{
    vector::Vector,
};

// A helper function to check if two floats are approximately equal.
// This is important for avoiding issues with floating-point precision in tests.
fn assert_approx_eq(a: f64, b: f64, tolerance: f64) {
    assert!((a - b).abs() < tolerance, "Assertion failed: {} is not approx equal to {}", a, b);
}

#[test]
fn test_mixer_initialization() {
    let mixer = QuadMixer::new(VehicleType::QuadX);
    // This is a basic sanity check to ensure the mixer is set up for 4 motors as expected.
    assert_eq!(mixer.num_motors, 4);
}

#[test]
fn test_pure_thrust() {
    let mut mixer = QuadMixer::new(VehicleType::QuadX);
    let commands = MixerInput {
        thrust: 0.5, // 50% throttle
        torques: Vector::from_array([0.0, 0.0, 0.0]),
    };

    let motor_outputs = mixer.mix(&commands);

    // For pure thrust, all motors should receive the same command.
    assert_approx_eq(motor_outputs[0], 0.5, 1e-6);
    assert_approx_eq(motor_outputs[1], 0.5, 1e-6);
    assert_approx_eq(motor_outputs[2], 0.5, 1e-6);
    assert_approx_eq(motor_outputs[3], 0.5, 1e-6);
}

#[test]
fn test_pure_roll() {
    let mut mixer = QuadMixer::new(VehicleType::QuadX);
    let commands = MixerInput {
        thrust: 0.5,
        torques: Vector::from_array([0.1, 0.0, 0.0]), // Command a right roll
    };

    let motor_outputs = mixer.mix(&commands);

    // To roll right, left motors (2, 3) must speed up, right motors (0, 1) must slow down.
    assert!(motor_outputs[2] > 0.5, "Motor 2 should increase for roll");
    assert!(motor_outputs[3] > 0.5, "Motor 3 should increase for roll");
    assert!(motor_outputs[0] < 0.5, "Motor 0 should decrease for roll");
    assert!(motor_outputs[1] < 0.5, "Motor 1 should decrease for roll");
}

#[test]
fn test_pure_pitch() {
    let mut mixer = QuadMixer::new(VehicleType::QuadX);
    let commands = MixerInput {
        thrust: 0.5,
        torques: Vector::from_array([0.0, 0.1, 0.0]), // Command a forward pitch
    };

    let motor_outputs = mixer.mix(&commands);

    // To pitch forward, rear motors (1, 2) must speed up, front motors (0, 3) must slow down.
    assert!(motor_outputs[1] > 0.5, "Motor 1 should increase for pitch");
    assert!(motor_outputs[2] > 0.5, "Motor 2 should increase for pitch");
    assert!(motor_outputs[0] < 0.5, "Motor 0 should decrease for pitch");
    assert!(motor_outputs[3] < 0.5, "Motor 3 should decrease for pitch");
}

#[test]
fn test_pure_yaw() {
    let mut mixer = QuadMixer::new(VehicleType::QuadX);
    let commands = MixerInput {
        thrust: 0.5,
        torques: Vector::from_array([0.0, 0.0, 0.1]), // Command a clockwise yaw
    };

    let motor_outputs = mixer.mix(&commands);
    
    // To yaw right (CW), CCW motors (1, 3) speed up, CW motors (0, 2) slow down.
    assert!(motor_outputs[1] > 0.5, "Motor 1 should increase for yaw");
    assert!(motor_outputs[3] > 0.5, "Motor 3 should increase for yaw");
    assert!(motor_outputs[0] < 0.5, "Motor 0 should decrease for yaw");
    assert!(motor_outputs[2] < 0.5, "Motor 2 should decrease for yaw");
}

#[test]
fn test_saturation_logic() {
    let mut mixer = QuadMixer::new(VehicleType::QuadX);
    // This is an aggressive command that will push at least one motor over 1.0
    let commands = MixerInput {
        thrust: 0.8,
        torques: Vector::from_array([0.3, 0.3, 0.0]),
    };

    let motor_outputs = mixer.mix(&commands);

    // Find the maximum output value after mixing.
    let mut max_val = 0.0;
    for i in 0..4 {
        if motor_outputs[i] > max_val {
            max_val = motor_outputs[i];
        }
    }
    
    // The core test: ensure that the mixer scaled the outputs correctly,
    // so the maximum output is exactly 1.0.
    assert_approx_eq(max_val, 1.0, 1e-6);
    
    // Also check that no motor is below 0.0
    for i in 0..4 {
        assert!(motor_outputs[i] >= 0.0, "Motor output should not be negative");
    }
}
