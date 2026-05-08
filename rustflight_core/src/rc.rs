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
use crate::packets::RcPacket;
use crate::params2::{ParamId, ParamValue, Params};
use crate::state_machine::{ErrorFlag, Event, StateManager};
use crate::{log_info, log_warn};

// --- Constants ---
pub const STICKS_COUNT: usize = 4;
pub const SWITCHES_COUNT: usize = 4;
pub const RC_STRUCT_CHANNELS: usize = 16; // A common max channel count

const RC_TIMEOUT_US: u64 = 500_000;

// --- Enums ---

#[repr(usize)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Stick {
    X = 0,
    Y = 1,
    Z = 2,
    F = 3,
}

#[repr(usize)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Switch {
    Arm = 0,
    AttOverride = 1,
    ThrottleOverride = 2,
    AttType = 3,
}

// --- RC Data Structs ---

/// Header for raw RC data
#[derive(Default, Debug, Copy, Clone)]
pub struct RcHeader {
    pub timestamp: u64, // Microseconds
    pub status: u16,    // Bitfield, 0 = OK
}

#[derive(Default, Copy, Clone)]
struct StickConfig {
    channel: i32,
    one_sided: bool,
}

#[derive(Default, Copy, Clone)]
struct SwitchConfig {
    channel: i32,
    mapped: bool,
    direction: i32,
}

pub struct RcStruct {
    pub header: RcHeader,
    pub chan: [f32; RC_STRUCT_CHANNELS],
    pub num_channels: usize,
    pub frame_lost: bool,
    pub failsafe_activated: bool,
}

impl Default for RcStruct {
    fn default() -> Self {
        Self {
            header: RcHeader::default(),
            chan: [0.0; RC_STRUCT_CHANNELS],
            num_channels: 0,
            frame_lost: false,
            failsafe_activated: false,
        }
    }
}

// --- Public RC Struct ---
pub struct Rc {
    rc: RcStruct, // Raw channel data
    new_command: bool,

    sticks: [StickConfig; STICKS_COUNT],
    switches: [SwitchConfig; SWITCHES_COUNT],

    stick_values: [f32; STICKS_COUNT],
    switch_values: [bool; SWITCHES_COUNT],

    prev_time_ms: u32,
    time_sticks_have_been_in_arming_position_ms: u32,
}

impl Rc {
    /// Creates a new, uninitialized RC handler
    pub fn new() -> Self {
        Self {
            rc: RcStruct::default(),
            new_command: false,
            sticks: [StickConfig::default(); STICKS_COUNT],
            switches: [SwitchConfig::default(); SWITCHES_COUNT],
            stick_values: [0.0; STICKS_COUNT],
            switch_values: [false; SWITCHES_COUNT],
            prev_time_ms: 0,
            time_sticks_have_been_in_arming_position_ms: 0,
        }
    }

    /// Initializes RC hardware and internal mappings
    pub fn init<B>(&mut self, board: &mut B, params: &Params) {
        self.init_rc(board, params);
        self.new_command = false;
    }

    fn init_rc<B>(&mut self, _board: &mut B, params: &Params) {
        self.init_sticks(params);
        self.update_switch_mappings(params);
    }

    // Maps stick parameters to internal config
    fn init_sticks(&mut self, params: &Params) {
        // --- REFACTORED: Stick::X ---
        self.sticks[Stick::X as usize] = StickConfig {
            channel: match params.get_by_id(ParamId::PARAM_RC_X_CHANNEL) {
                ParamValue::Int(val) => {
                    // println!("Init RC channel {} for stick X", val);
                    val
                }
                other => {
                    // println!("Error: PARAM_RC_X_CHANNEL is not an Int, but {:?}! Defaulting to 0.", other);
                    0 // Default C++ value
                }
            },
            one_sided: false,
        };

        // --- REFACTORED: Stick::Y ---
        self.sticks[Stick::Y as usize] = StickConfig {
            channel: match params.get_by_id(ParamId::PARAM_RC_Y_CHANNEL) {
                ParamValue::Int(val) => {
                    // println!("Init RC channel {} for stick Y", val);
                    val
                }
                other => {
                    // println!("Error: PARAM_RC_Y_CHANNEL is not an Int, but {:?}! Defaulting to 1.", other);
                    1 // Default C++ value
                }
            },
            one_sided: false,
        };

        // --- REFACTORED: Stick::Z ---
        self.sticks[Stick::Z as usize] = StickConfig {
            channel: match params.get_by_id(ParamId::PARAM_RC_Z_CHANNEL) {
                ParamValue::Int(val) => {
                    // println!("Init RC channel {} for stick Z", val);
                    val
                }
                other => {
                    // println!("Error: PARAM_RC_Z_CHANNEL is not an Int, but {:?}! Defaulting to 3.", other);
                    3 // Default C++ value
                }
            },
            one_sided: false,
        };

        // --- REFACTORED: Stick::F ---
        self.sticks[Stick::F as usize] = StickConfig {
            channel: match params.get_by_id(ParamId::PARAM_RC_F_CHANNEL) {
                ParamValue::Int(val) => {
                    // println!("Init RC channel {} for stick F", val);
                    val
                }
                other => {
                    // println!("Error: PARAM_RC_F_CHANNEL is not an Int, but {:?}! Defaulting to 2.", other);
                    2 // Default C++ value
                }
            },
            one_sided: true,
        };
    }

    fn update_switch_mappings(&mut self, params: &Params) {
        // --- REFACTORED: PARAM_RC_NUM_CHANNELS ---
        let rc_num_channels = match params.get_by_id(ParamId::PARAM_RC_NUM_CHANNELS) {
            ParamValue::Int(val) => val,
            other => {
                // println!("Error: PARAM_RC_NUM_CHANNELS is not an Int, but {:?}! Defaulting to 6.", other);
                6 // Default C++ value
            }
        };

        // must loop over the 4 logical functions we have defined
        for i in 0..SWITCHES_COUNT {
            // Using Option<ParamId> to handle the "INVALID" case safely
            let (channel_name, channel_param_id) = match i {
                i if i == Switch::Arm as usize => ("ARM", Some(ParamId::PARAM_RC_ARM_CHANNEL)),
                i if i == Switch::AttOverride as usize => (
                    "ATTITUDE OVERRIDE",
                    Some(ParamId::PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL),
                ),
                i if i == Switch::ThrottleOverride as usize => (
                    "THROTTLE OVERRIDE",
                    Some(ParamId::PARAM_RC_THROTTLE_OVERRIDE_CHANNEL),
                ),
                i if i == Switch::AttType as usize => (
                    "ATTITUDE TYPE",
                    Some(ParamId::PARAM_RC_ATT_CONTROL_TYPE_CHANNEL),
                ),
                _ => ("INVALID", None),
            };

            // --- REFACTORED: channel_num retrieval ---
            let channel_num = if let Some(id) = channel_param_id {
                match params.get_by_id(id) {
                    ParamValue::Int(val) => val,
                    other => {
                        // println!("Error: Param {:?} is not an Int, but {:?}! Defaulting to 255.", id, other);
                        255 // Default for "INVALID"
                    }
                }
            } else {
                255 // C++ default for "INVALID"
            };

            self.switches[i].channel = channel_num;
            self.switches[i].mapped = channel_num > 3 && channel_num < rc_num_channels;

            // // debugging code to see if we mapped it or not...
            // if self.switches[i].mapped {
            //     defmt::println!("Switch \"{}\" is mapped to channel {}", channel_name, channel_num);
            // } else {
            //     defmt::println!("Switch \"{}\" will not be mapped.", channel_name);
            // }

            let direction_param_id = match channel_num {
                4 => Some(ParamId::PARAM_RC_SWITCH_5_DIRECTION),
                5 => Some(ParamId::PARAM_RC_SWITCH_6_DIRECTION),
                6 => Some(ParamId::PARAM_RC_SWITCH_7_DIRECTION),
                7 => Some(ParamId::PARAM_RC_SWITCH_8_DIRECTION),
                _ => None, // No param
            };

            // --- REFACTORED: direction retrieval ---
            self.switches[i].direction = if let Some(id) = direction_param_id {
                match params.get_by_id(id) {
                    ParamValue::Int(val) => val,
                    other => {
                        // println!("Error: Param {:?} is not an Int, but {:?}! Defaulting to 1.", id, other);
                        1 // C++ default
                    }
                }
            } else {
                1 // C++ default
            };
        }
    }

    fn log_switch_mappings(&self) {
        for i in 0..SWITCHES_COUNT {
            let (channel_name, _) = match i {
                i if i == Switch::Arm as usize => ("ARM", Some(ParamId::PARAM_RC_ARM_CHANNEL)),
                i if i == Switch::AttOverride as usize => (
                    "ATTITUDE OVERRIDE",
                    Some(ParamId::PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL),
                ),
                i if i == Switch::ThrottleOverride as usize => (
                    "THROTTLE OVERRIDE",
                    Some(ParamId::PARAM_RC_THROTTLE_OVERRIDE_CHANNEL),
                ),
                i if i == Switch::AttType as usize => (
                    "ATTITUDE TYPE",
                    Some(ParamId::PARAM_RC_ATT_CONTROL_TYPE_CHANNEL),
                ),
                _ => ("INVALID", None),
            };

            if channel_name == "INVALID" {
                continue;
            }

            if self.switches[i].mapped {
                // TODO: Re-enable this when your CommManager has a log method
                // comm_manager.log(
                //     LogSeverity::LOG_INFO,
                //     &format!("{} switch mapped to RC channel {}", channel_name, self.switches[i].channel)
                // );
                log_info!(
                    "{} switch mapped to RC Channel {}",
                    channel_name,
                    self.switches[i].channel
                );
            } else {
                // comm_manager.log(
                //     LogSeverity::LOG_INFO,
                //     &format!("{} switch not mapped", channel_name)
                // );
                log_info!("{} switch not mapped", channel_name);
            }
        }
    }

    pub fn param_change_callback(&mut self, param_id: ParamId, params: &Params) {
        match param_id {
            // ... (PARAM_RC_TYPE case is removed)
            ParamId::PARAM_RC_X_CHANNEL
            | ParamId::PARAM_RC_Y_CHANNEL
            | ParamId::PARAM_RC_Z_CHANNEL
            | ParamId::PARAM_RC_F_CHANNEL => {
                self.init_sticks(params);
            }
            ParamId::PARAM_RC_ATTITUDE_OVERRIDE_CHANNEL
            | ParamId::PARAM_RC_THROTTLE_OVERRIDE_CHANNEL
            | ParamId::PARAM_RC_ATT_CONTROL_TYPE_CHANNEL
            | ParamId::PARAM_RC_ARM_CHANNEL
            | ParamId::PARAM_RC_SWITCH_5_DIRECTION
            | ParamId::PARAM_RC_SWITCH_6_DIRECTION
            | ParamId::PARAM_RC_SWITCH_7_DIRECTION
            | ParamId::PARAM_RC_SWITCH_8_DIRECTION => {
                self.update_switch_mappings(params); // <-- 1. Update mappings
                self.log_switch_mappings(); // <-- 2. Log the changes
            }
            _ => {
                // do nothing
            }
        }
    }

    pub fn receive(
        &mut self,
        packet: &RcPacket, // <-- Takes the packet
        params: &Params,
        state_manager: &mut StateManager,
    ) {
        // 1. Copy data from the packet into the internal rc_struct: We assume it has been normalized before this point
        // Get the number of channels
        let len = (packet.n_chan as usize).min(self.rc.chan.len());
        self.rc.chan[..len].copy_from_slice(&packet.chan[..len]);

        self.rc.header.timestamp = packet.header.timestamp;
        self.rc.header.status = packet.header.status;
        self.rc.num_channels = len;

        let status = packet.header.status;
        self.rc.frame_lost = (status & 1) != 0;
        self.rc.failsafe_activated = (status & 2) != 0;

        self.process_sticks_and_switches();
        self.new_command = true;
    }

    fn process_sticks_and_switches(&mut self) {
        // TODO add back in check for rc lost... no need to process switches if rc lost...

        // STICKS
        for channel in 0..STICKS_COUNT {
            let config = &self.sticks[channel];
            if config.channel < 0 || (config.channel as usize) >= self.rc.num_channels {
                continue;
            }
            let pwm = self.rc.chan[config.channel as usize]; // pwm is 0.0 to 1.0

            if config.one_sided {
                // generally only F
                self.stick_values[channel] = pwm;
            } else {
                // Converts [0.0, 1.0] to [-1.0, 1.0]
                self.stick_values[channel] = 2.0 * (pwm - 0.5);
            }
            // defmt::info!("Stick {}: {}",channel, self.stick_values[channel]);
        }

        // SWITCHES

        for channel in 0..SWITCHES_COUNT {
            let config = &self.switches[channel];
            if config.mapped {
                if config.channel < 0 || (config.channel as usize) >= self.rc.num_channels {
                    self.switch_values[channel] = false;
                    continue;
                }

                let pwm = self.rc.chan[config.channel as usize]; // pwm is 0.0 to 1.0

                if config.direction < 0 {
                    self.switch_values[channel] = pwm < 0.2; // C++ logic
                } else {
                    self.switch_values[channel] = pwm >= 0.8; // C++ logic
                }
            } else {
                self.switch_values[channel] = false;
            }

            // defmt::info!("Switch {}: {}",channel, self.switch_values[channel]);
        }
    }

    pub fn check_rc_health(&self, now_us: u64, params: &Params) -> bool {
        if now_us > self.rc.header.timestamp + RC_TIMEOUT_US {
            return false;
        }

        if self.rc.frame_lost || self.rc.failsafe_activated {
            return false;
        }

        let num_channels = match params.get_by_id(ParamId::PARAM_RC_NUM_CHANNELS) {
            ParamValue::Int(val) => val as usize,
            _ => 6,
        };
        let channels_to_check = num_channels.min(self.rc.num_channels);

        for i in 0..channels_to_check {
            let val = self.rc.chan[i];
            if val < -0.25 || val > 1.25 {
                return false;
            }
        }
        return true;
    }

    pub fn run(
        &mut self,
        now_ms: u32,
        params: &Params,
        state_manager: &mut StateManager, // Use the concrete StateManager
    ) {
        let now_us = (now_ms as u64) * 1000;

        if self.check_rc_health(now_us, params) {
            state_manager.update(Event::ERROR_CLEARED(ErrorFlag::RC_LOST), params);
            //println!("RC is Healthy!");

            // only run arming logic if rc is healthy
            self.look_for_arm_disarm_signal(now_ms, params, state_manager);
        } else {
            //println!("RC is Lost!");
            state_manager.update(Event::ERROR_OCCURRED(ErrorFlag::RC_LOST), params);
        }
    }

    /// Checks for stick or switch arming/disarming
    fn look_for_arm_disarm_signal(
        &mut self,
        now_ms: u32,
        params: &Params,
        state_manager: &mut StateManager, // Use the concrete StateManager
    ) {
        let dt = now_ms.saturating_sub(self.prev_time_ms);
        self.prev_time_ms = now_ms;

        let arm_threshold = match params.get_by_id(ParamId::PARAM_ARM_THRESHOLD) {
            ParamValue::Float(val) => val,
            other => {
                // println!("Error: PARAM_ARM_THRESHOLD is not a Float, but {:?}! Defaulting to 0.15.", other);
                0.15 // Default value from C++ param definitions
            }
        };

        //println!("Arm Threshold is: {}", arm_threshold);

        // Use the correct public method from StateManager
        let is_armed = state_manager.is_armed();

        if !self.switch_mapped(Switch::Arm) {
            //println!("The arm switch is not mapped!");
            // Stick arming
            let f_stick = self.stick(Stick::F);
            let z_stick = self.stick(Stick::Z);

            if !is_armed {
                // DISARMED
                // if left stick is down and to the right
                if f_stick < arm_threshold && z_stick > (1.0 - arm_threshold) {
                    self.time_sticks_have_been_in_arming_position_ms = self
                        .time_sticks_have_been_in_arming_position_ms
                        .saturating_add(dt);
                    // println!("Starting Arming Process");
                } else {
                    self.time_sticks_have_been_in_arming_position_ms = 0;
                }

                if self.time_sticks_have_been_in_arming_position_ms > 1000 {
                    // defmt::info!("Requesting Arm!");
                    // Use update() with params
                    state_manager.update(Event::REQUEST_ARM, params);
                }
            } else {
                // ARMED
                // if left stick is down and to the left
                if f_stick < arm_threshold && z_stick < -(1.0 - arm_threshold) {
                    self.time_sticks_have_been_in_arming_position_ms = self
                        .time_sticks_have_been_in_arming_position_ms
                        .saturating_add(dt);
                    // println!("Starting Disarm Process");
                } else {
                    self.time_sticks_have_been_in_arming_position_ms = 0;
                }

                if self.time_sticks_have_been_in_arming_position_ms > 1000 {
                    // Use update() with params
                    state_manager.update(Event::REQUEST_DISARM, params);
                    self.time_sticks_have_been_in_arming_position_ms = 0;
                    // defmt::info!("Requesting Disarm!")
                }
            }
        } else {
            // Switch arming
            //println!("The arm switch is mapped!!!");
            let f_stick = self.stick(Stick::F);
            if self.switch_on(Switch::Arm) {
                // defmt::info!("is_armed: {} f_stick: {} arm_threshold: {}", is_armed, f_stick, arm_threshold);
                if !is_armed && (f_stick < arm_threshold) {
                    // Use update() with params
                    state_manager.update(Event::REQUEST_ARM, params);
                }
            } else {
                if is_armed {
                    // Use update() with params
                    state_manager.update(Event::REQUEST_DISARM, params);
                }
            }
        }
    }

    // -----------------------------------------------------------------
    // Public Getters
    // -----------------------------------------------------------------

    /// Returns true if a new command has been processed
    pub fn new_command(&mut self) -> bool {
        if self.new_command {
            self.new_command = false;
            true
        } else {
            false
        }
    }

    /// Gets the processed stick value (-1.0 to 1.0, or 0.0 to 1.0 for F)
    pub fn stick(&self, channel: Stick) -> f32 {
        self.stick_values[channel as usize]
    }

    /// Gets the processed switch value (true or false)
    pub fn switch_on(&self, channel: Switch) -> bool {
        self.switch_values[channel as usize]
    }

    /// Checks if a switch is mapped to a channel
    pub fn switch_mapped(&self, channel: Switch) -> bool {
        self.switches[channel as usize].mapped
    }

    /// Gets a reference to the raw, unprocessed RC data
    pub fn get_rc_struct(&self) -> &RcStruct {
        &self.rc
    }

    /// Reads a raw, normalized (0.0 to 1.0) channel value
    pub fn read_raw_chan(&self, chan_idx: usize) -> f32 {
        if chan_idx < self.rc.num_channels {
            self.rc.chan[chan_idx]
        } else {
            0.0 // Safer than C++ OOB read
        }
    }
}
