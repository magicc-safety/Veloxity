// /**
// ******************************************************************************
// * File     : rc.rs
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
// * list of conditions and the following disclaimer. *
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

use crate::packets::RcPacket; // Use the correct RcPacket from your packets module
use crate::params2::{Params, ParamId, ParamValue};
use crate::state_machine::{Event, ErrorFlag, StateManager};

// --- Enums and Config Structs ---
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Stick { X = 0, Y = 1, Z = 2, F = 3 }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Switch { Arm = 0, AttOverride = 1, ThrottleOverride = 2, AttType = 3 }

const STICKS_COUNT: usize = 4;
const SWITCHES_COUNT: usize = 4;

#[derive(Clone, Copy, Default)]
struct StickConfig {
    channel: u8,
}

#[derive(Clone, Copy, Default)]
struct SwitchConfig {
    channel: u8,
    direction: i8,
    mapped: bool,
}

/// The main RC struct. It is now completely decoupled from MAVLink and the board.
#[derive(Default)]
pub struct Rc {
    new_command: bool,
    time_sticks_in_arming_position: u32,
    stick_values: [f32; STICKS_COUNT],
    switch_values: [bool; SWITCHES_COUNT],
    sticks: [StickConfig; STICKS_COUNT],
    switches: [SwitchConfig; SWITCHES_COUNT],
}

impl Rc {
    pub fn new() -> Self {
        Default::default()
    }

    pub fn init(&mut self, params: &Params) {
        self.init_sticks(params);
        self.init_switches(params);
    }

    pub fn run(&mut self, now_ms: u32, params: &Params, state_manager: &mut StateManager) {
        self.look_for_arm_disarm_signal(now_ms, params, state_manager);
    }

    /// Processes a new, processed `RcPacket` from the sensor pipeline.
    pub fn receive(&mut self, packet: &RcPacket, state_manager: &mut StateManager, params: &Params) {
        // 1. Check for hardware-level failsafe from the packet's `lol` flag.
        if packet.lol { // Assuming `lol` means "Loss of Link" or failsafe
            if !state_manager.is_in_failsafe() {
                state_manager.update(Event::ERROR_OCCURRED(ErrorFlag::RC_LOST), params);
            }
        } else {
            state_manager.update(Event::ERROR_CLEARED(ErrorFlag::RC_LOST), params);
        }

        //print!("\x1B[2J\x1B[H");

        // 2. Copy the pre-scaled stick values.
        for (i, stick_config) in self.sticks.iter().enumerate() {
            if stick_config.channel < packet.n_chan as u8 {
                self.stick_values[i] = packet.chan[stick_config.channel as usize];
                println!("\tStick Value {}: {}", i, self.stick_values[i])
            }
        }
        //println!();

        // 3. Process the switch values from the pre-scaled channels.
        for (i, switch_config) in self.switches.iter().enumerate() {
            if switch_config.mapped && switch_config.channel < packet.n_chan as u8 {
                // A switch is "on" if its scaled value is > 0.0 (i.e., above the 1500us center point)
                let is_on = packet.chan[switch_config.channel as usize] > 0.0;
                self.switch_values[i] = if switch_config.direction > 0 { is_on } else { !is_on };
            }
        }

        self.new_command = true;
    }

    // --- Public Accessor Functions ---
    pub fn new_command(&mut self) -> bool {
        let new = self.new_command;
        self.new_command = false;
        new
    }
    
    pub fn stick(&self, stick: Stick) -> f32 {
        self.stick_values[stick as usize]
    }

    pub fn switch_on(&self, switch: Switch) -> bool {
        self.switch_values[switch as usize]
    }

    pub fn switch_mapped(&self, switch: Switch) -> bool {
        self.switches[switch as usize].mapped
    }
    
    // --- Private Helper Functions ---
    fn init_sticks(&mut self, params: &Params) {
        let get_param = |id: ParamId, default: i32| -> i32 {
            if let ParamValue::Int(val) = params.get_by_id(id) { val } else { default }
        };

        self.sticks[Stick::X as usize].channel = get_param(ParamId::PARAM_RC_X_CHANNEL, 0) as u8;
        self.sticks[Stick::Y as usize].channel = get_param(ParamId::PARAM_RC_Y_CHANNEL, 1) as u8;
        self.sticks[Stick::Z as usize].channel = get_param(ParamId::PARAM_RC_Z_CHANNEL, 2) as u8;
        self.sticks[Stick::F as usize].channel = get_param(ParamId::PARAM_RC_F_CHANNEL, 3) as u8;
    }
    
    fn init_switches(&mut self, params: &Params) {
        // Helper to safely get an integer parameter or fall back to a default.
        let get_param = |id: ParamId, default: i32| -> i32 {
            if let ParamValue::Int(val) = params.get_by_id(id) { val } else { default }
        };

        // Configure the ARM switch
        let channel = get_param(ParamId::PARAM_RC_ARM_CHANNEL, 0) as u8;
        self.switches[Switch::Arm as usize] = SwitchConfig {
            channel,
            direction: get_param(ParamId::PARAM_RC_SWITCH_5_DIRECTION, 1) as i8, // Assuming ARM is on switch 5
            mapped: channel != 0,
        };

        // Configure the ATTITUDE OVERRIDE switch
        let channel = get_param(ParamId::PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL, 0) as u8;
        self.switches[Switch::AttOverride as usize] = SwitchConfig {
            channel,
            direction: get_param(ParamId::PARAM_RC_SWITCH_6_DIRECTION, 1) as i8, // Assuming ATT_OVERRIDE is on switch 6
            mapped: channel != 0,
        };

        // Configure the THROTTLE OVERRIDE switch
        let channel = get_param(ParamId::PARAM_RC_THROTTLE_OVERRIDE_CHANNEL, 0) as u8;
        self.switches[Switch::ThrottleOverride as usize] = SwitchConfig {
            channel,
            direction: get_param(ParamId::PARAM_RC_SWITCH_7_DIRECTION, 1) as i8, // Assuming THROTTLE_OVERRIDE is on switch 7
            mapped: channel != 0,
        };
    
        // Configure the ATTITUDE TYPE switch
        let channel = get_param(ParamId::PARAM_RC_ATT_CONTROL_TYPE_CHANNEL, 0) as u8;
        self.switches[Switch::AttType as usize] = SwitchConfig {
            channel,
            direction: get_param(ParamId::PARAM_RC_SWITCH_8_DIRECTION, 1) as i8, // Assuming ATT_TYPE is on switch 8
            mapped: channel != 0,
        };
    }

    
    fn look_for_arm_disarm_signal(&mut self, now_ms: u32, params: &Params, state_manager: &mut StateManager) {
        let arm_stick_threshold = if let ParamValue::Float(val) = params.get_by_id(ParamId::PARAM_ARM_THRESHOLD) {
            val
        } else {
            0.9 // Default to 90% stick deflection
        };

        let throttle = self.stick(Stick::F);
        let yaw = self.stick(Stick::Z);

        // ARM_TIME parameter is not in the new list, so we'll use a reasonable default.
        // In a full implementation, you might add this parameter back.
        let arm_time_ms = 500; 

        // Check for arming gesture: throttle low, yaw right
        if throttle < -0.9 && yaw > arm_stick_threshold {
            if self.time_sticks_in_arming_position == 0 {
                self.time_sticks_in_arming_position = now_ms;
            }
        
            if now_ms > self.time_sticks_in_arming_position + arm_time_ms {
                state_manager.update(Event::REQUEST_ARM, params);
                self.time_sticks_in_arming_position = 0; // Reset timer to prevent re-triggering
            }
        } 
        // Check for disarming gesture: throttle low, yaw left
        else if throttle < -0.9 && yaw < -arm_stick_threshold {
            if self.time_sticks_in_arming_position == 0 {
                self.time_sticks_in_arming_position = now_ms;
            }

            if now_ms > self.time_sticks_in_arming_position + arm_time_ms {
                state_manager.update(Event::REQUEST_DISARM, params);
                self.time_sticks_in_arming_position = 0; // Reset timer
            }
        } 
        // If sticks are not in a gesture position, reset the timer.
        else {
            self.time_sticks_in_arming_position = 0;
        }
    }
}