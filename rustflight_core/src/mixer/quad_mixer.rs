
// /**
// ******************************************************************************
// * File     : quad_mixer.rs
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

use crate::controller::quad_controller::MixerInput;
use micro_algebra::stack::vector::Vector;
use micro_algebra::stack::quaternion::Quaternion;
use micro_algebra::stack::matrix::Matrix;
use crate::mixer::Mixer;

// An enum to select the airframe type, similar to the C++ version
#[derive(Debug, Clone, Copy)]
pub enum VehicleType {
    QuadX,
    // Other types like QuadPlus, HexX, etc., could be added here
}

// The main mixer struct
pub struct QuadMixer {
    // A 4x4 matrix: 4 motors (rows), 4 control inputs (cols)
    pub mixing_matrix: Matrix<f64, 4, 16>,
    pub num_motors: usize,
}

impl QuadMixer {
    /// Creates a new mixer for a specific vehicle type.
    pub fn new(vehicle_type: VehicleType) -> Self {
        let (mixing_matrix, num_motors) = match vehicle_type {
            VehicleType::QuadX => {
                // This is a standard Quad-X mixing matrix. It defines how thrust, roll,
                // pitch, and yaw commands are distributed to the four motors.
                // The motor numbering assumes: 0:Front-Right, 1:Rear-Right,
                // 2:Rear-Left, 3:Front-Left
                let data: [f64; 16] = [
                // Thrust,   Roll,   Pitch,    Yaw
                   1.0,    -1.0,    -1.0,     -1.0,  // Motor 0 (FR, CW)
                   1.0,    -1.0,     1.0,      1.0,  // Motor 1 (RR, CCW)
                   1.0,     1.0,     1.0,     -1.0,  // Motor 2 (RL, CW)
                   1.0,     1.0,    -1.0,      1.0,  // Motor 3 (FL, CCW)
                ];
                (Matrix::from_array(data), 4)
            }
        };
        
        Self { mixing_matrix, num_motors }
    }
}

impl Mixer for QuadMixer {
    type ControlOutput = MixerInput;
    type ActuatorCommands = Vector<f64, 4>; // Output for 4 motors

    fn mix(&mut self, controls: &Self::ControlOutput) -> Self::ActuatorCommands {
        // 1. Assemble the command vector from the controller's outputs.
        // The order must match the columns in the mixing matrix.
        let command_vector = Vector::from_array([
            controls.thrust,
            controls.torques[0], // Roll
            controls.torques[1], // Pitch
            controls.torques[2], // Yaw
        ]);

        // 2. Perform matrix-vector multiplication to get the raw motor commands.
        // motor_outputs = MixingMatrix * command_vector
        let mut motor_outputs = self.mixing_matrix.vmul(&command_vector);
        
        // 3. Handle saturation to maintain control authority, as seen in the C++ code.
        let mut max_output = 0.0;
        for i in 0..self.num_motors {
            if motor_outputs[i].abs() > max_output {
                max_output = motor_outputs[i].abs();
            }
        }
        
        // If any motor is commanded above 100%, scale all motor commands down proportionally.
        if max_output > 1.0 {
            for i in 0..self.num_motors {
                motor_outputs[i] /= max_output;
            }
        }
        
        // 4. Final clamping to ensure outputs are valid throttle values (e.g., 0.0 to 1.0).
        // This is also where a minimum "armed" throttle could be enforced.
        for i in 0..self.num_motors {
            motor_outputs[i] = motor_outputs[i].clamp(0.0, 1.0);
        }

        motor_outputs
    }
}

