// /**
// ******************************************************************************
// * File     : command_manager.rs
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

use crate::comm_manager::comm_link_trait::CommInterface;
use crate::comm_manager::CommManager;
use crate::mavlink::dialects::rosflight::enums::offboard_control_ignore;
use crate::rc::{Rc, Stick, Switch};
use crate::params2::{Params, ParamId, ParamValue};
use crate::state_machine::{StateManager, Event, ErrorFlag};
use crate::board::BoardTrait;
use crate::comm_messages::{messages::OffboardControlMsg, enums::{OffboardControlIgnore, OffboardControlMode::{self, *}}};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ControlType {
    Rate,        // Channel is is in rate mode (rad/s)
    Angle,       // Channel command is in angle mode (rad)
    Throttle,    // Channel is controlling throttle setting
    Passthrough, // Channel directly passes PWM input to the mixer
}

// simpler than the enum representation in c++ command_manager.h
const ATTITUDE_RATE_MODE: i32  = 0;
const ATTITUDE_ANGLE_MODE: i32 = 1;

#[derive(Clone, Copy, Debug)]
pub struct ControlChannel {
    pub active: bool,
    pub control_type: ControlType,
    pub value: f64,
}

impl Default for ControlChannel {
    fn default() -> Self {
        Self {
            active: false,
            control_type: ControlType::Rate,
            value: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CombinedControl {
    pub stamp_ms: u32,
    pub qx: ControlChannel,
    pub qy: ControlChannel,
    pub qz: ControlChannel,
    pub fx: ControlChannel,
    pub fy: ControlChannel,
    pub fz: ControlChannel,
}

#[derive(Default)]
pub struct CommandManager {
    // Command structs
    rc_command: CombinedControl,
    offboard_command: CombinedControl,
    combined_command: CombinedControl,
    multirotor_failsafe_command: CombinedControl,
    fixedwing_failsafe_command: CombinedControl,
    last_offboard_command_us: u64,
    last_stick_override_time: [u32; 3], // for x, y, and z sticks

    // State flags
    rc_throttle_override: bool,
    rc_attitude_override: bool,
}

impl CommandManager {
    /// Creates a new CommandManager with default initial values.
    pub fn new() -> Self {
        Self {
            multirotor_failsafe_command: CombinedControl {
                qx: ControlChannel { active: true, control_type: ControlType::Rate, value: 0.0 },
                qy: ControlChannel { active: true, control_type: ControlType::Rate, value: 0.0 },
                qz: ControlChannel { active: true, control_type: ControlType::Rate, value: 0.0 },
                fz: ControlChannel { active: true, control_type: ControlType::Passthrough, value: 0.0 },
                ..Default::default()
            },
            fixedwing_failsafe_command: CombinedControl {
                qx: ControlChannel { active: true, control_type: ControlType::Passthrough, value: 0.0 },
                qy: ControlChannel { active: true, control_type: ControlType::Passthrough, value: 0.0 },
                qz: ControlChannel { active: true, control_type: ControlType::Passthrough, value: 0.0 },
                fx: ControlChannel { active: true, control_type: ControlType::Passthrough, value: 0.0 },
                ..Default::default()
            },
            ..Default::default()
        }
    }

    pub fn init(&mut self, params: &Params, state_manager: &mut StateManager) {
        self.update_failsafe_config(params, state_manager);
    }

    pub fn update_failsafe_config(&mut self, params: &Params, state_manager: &mut StateManager) {
    
        let mut failsafe_throttle = match params.get_by_id(ParamId::PARAM_FAILSAFE_THROTTLE) {
            ParamValue::Float(val) => val,
            other => {
                // println!("Error: PARAM_FAILSAFE_THROTTLE is not a Float, but {:?}! Defaulting to 0.0.", other);
                0.0f32
            }
        };

        let is_fixed_wing = match params.get_by_id(ParamId::PARAM_FIXED_WING) {
            ParamValue::Bool(val) => val,
            other => {
                // println!("Error: PARAM_FIXED_WING is not a Bool, but {:?}! Defaulting to false.", other);
                false
            }
        };

        if !is_fixed_wing && (failsafe_throttle < 0.0 || failsafe_throttle > 1.0) {
            state_manager.update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_FAILSAFE), params);
            failsafe_throttle = 0.0f32;
        } else {
            state_manager.update(Event::ERROR_CLEARED(ErrorFlag::INVALID_FAILSAFE), params);
        }

        match params.get_by_id(ParamId::PARAM_RC_F_AXIS) {
            // The "happy path": it's an Int, as expected
            ParamValue::Int(axis) => {
                match axis { // Note: `axis` is an i32, not `&i32`, so no deref `*` needed
                    0 => self.multirotor_failsafe_command.fx.value = failsafe_throttle as f64,
                    1 => self.multirotor_failsafe_command.fy.value = failsafe_throttle as f64,
                    _ => self.multirotor_failsafe_command.fz.value = failsafe_throttle as f64,
                }
            },
            // Error case: it's the wrong type.
            // We log the error and apply a safe default (Fz).
            other_type => {
                // println!("Error: PARAM_RC_F_AXIS is not an Int, but {:?}! Defaulting to Fz.", other_type);
                self.multirotor_failsafe_command.fz.value = failsafe_throttle as f64;
            }
        }
        
        // This line is fine as-is
        self.fixedwing_failsafe_command.fx.value = failsafe_throttle as f64;
    }

    pub fn run(
        &mut self,
        now_ms: u32,
        params: &Params,
        rc: &mut Rc,
        state_manager: &mut StateManager, // <-- Must be mutable
    ) -> bool {
        
        let now_us = now_ms as u64 * 1000;

        // --- 1. Failsafe Action (C++ lines 240-243) ---
        // This is the highest priority. If in failsafe, override all commands.
        if state_manager.is_in_failsafe() {
            // println!("Command_Manager: We're in failsafe!!!");

            let is_fixed_wing = match params.get_by_id(ParamId::PARAM_FIXED_WING) {
                ParamValue::Bool(val) => val,
                other => {
                    // println!("Error: PARAM_FIXED_WING is not a Bool, but {:?}! Defaulting to false (multirotor).", other);
                    false 
                }
            };
            self.combined_command = if is_fixed_wing {
                // println!("Command_Manager: failsafe is fixedwing failsafe");
                self.fixedwing_failsafe_command
            } else {
                self.multirotor_failsafe_command
            };
            return true; // Exit early
        }

        // --- 2. RC Update (C++ line 244) ---
        // If not in failsafe, check for new RC commands and run muxing
        if rc.new_command() {
            // Port of interpret_rc()
            self.interpret_rc(rc, params); 

            // --- 3. Offboard Timeout "Fail-over" (C++ lines 246-252) ---
            let timeout_ms = match params.get_by_id(ParamId::PARAM_OFFBOARD_TIMEOUT) {
                ParamValue::Int(val) => val as u32,
                other => {
                    // println!("Error: PARAM_OFFBOARD_TIMEOUT is not an Int, but {:?}! Defaulting to 100ms.", other);
                    100 // Use the C++ default as a safe fallback
                }
            };

            // Use the microsecond timer for more precision
            if now_us > self.last_offboard_command_us + (timeout_ms as u64 * 1000) {
                // Timeout occurred! Deactivate offboard control.
                // This will cause the muxer to "fail-over" to RC.
                self.offboard_command.qx.active = false;
                self.offboard_command.qy.active = false;
                self.offboard_command.qz.active = false;
                self.offboard_command.fx.active = false;
                self.offboard_command.fy.active = false;
                self.offboard_command.fz.active = false;
            }
            
            // --- 4. Muxing (C++ lines 253-256) ---
            self.do_muxing(params, rc, now_ms);
        }
        
        true
    }

    /// Receives a new offboard control command.
    /// This should be called from CommManager::act_on_messages
    pub fn set_new_offboard_command(&mut self, now_us: u64, msg: &OffboardControlMsg, params: &Params) {
        // We got a new command, so update the timestamp
        self.last_offboard_command_us = now_us;
        self.offboard_command.stamp_ms = (now_us / 1000) as u32;

        self.offboard_command.qx.value = msg.qx as f64;
        self.offboard_command.qy.value = msg.qy as f64;
        self.offboard_command.qz.value = msg.qz as f64;
        self.offboard_command.fx.value = msg.fx as f64;
        self.offboard_command.fy.value = msg.fy as f64;
        self.offboard_command.fz.value = msg.fz as f64;

        self.offboard_command.qx.active = !msg.ignore.contains(OffboardControlIgnore::IGNORE_QX);
        self.offboard_command.qy.active = !msg.ignore.contains(OffboardControlIgnore::IGNORE_QY);
        self.offboard_command.qz.active = !msg.ignore.contains(OffboardControlIgnore::IGNORE_QZ);
        self.offboard_command.fx.active = !msg.ignore.contains(OffboardControlIgnore::IGNORE_FX);
        self.offboard_command.fy.active = !msg.ignore.contains(OffboardControlIgnore::IGNORE_FY);
        self.offboard_command.fz.active = !msg.ignore.contains(OffboardControlIgnore::IGNORE_FZ);

        // Logic moved from old `run` function
        match msg.mode {
            OffboardControlMode::ModePassThrough => {
                self.offboard_command.qx.control_type = ControlType::Passthrough;
                self.offboard_command.qy.control_type = ControlType::Passthrough;
                self.offboard_command.qz.control_type = ControlType::Passthrough;
                self.offboard_command.fx.control_type = ControlType::Passthrough;
                self.offboard_command.fy.control_type = ControlType::Passthrough;
                self.offboard_command.fz.control_type = ControlType::Passthrough;
            },
            OffboardControlMode::ModeRollratePitchrateYawrateThrottle => {
                self.offboard_command.qx.control_type = ControlType::Rate;
                self.offboard_command.qy.control_type = ControlType::Rate;
                self.offboard_command.qz.control_type = ControlType::Rate;
                self.offboard_command.fx.control_type = ControlType::Throttle;
                self.offboard_command.fy.control_type = ControlType::Throttle;
                self.offboard_command.fz.control_type = ControlType::Throttle;
            }
            OffboardControlMode::ModeRollPitchYawrateThrottle => {
                self.offboard_command.qx.control_type = ControlType::Angle;
                self.offboard_command.qy.control_type = ControlType::Angle;
                self.offboard_command.qz.control_type = ControlType::Rate;
                self.offboard_command.fx.control_type = ControlType::Throttle;
                self.offboard_command.fy.control_type = ControlType::Throttle;
                self.offboard_command.fz.control_type = ControlType::Throttle;
            },
        }
    }

    /// Port of C++ `interpret_rc` (command_manager.cpp:101)
    fn interpret_rc(&mut self, rc: &Rc, params: &Params) {
        // Read all stick values from the RC unit
        self.rc_command.qx.value = rc.stick(Stick::X) as f64;
        self.rc_command.qy.value = rc.stick(Stick::Y) as f64;
        self.rc_command.qz.value = rc.stick(Stick::Z) as f64;
        let f_stick_value = rc.stick(Stick::F) as f64;

        // C++: line 109, logic for F-axis (type-safe)
        match params.get_by_id(ParamId::PARAM_RC_F_AXIS) {
            // "Happy path": it's an Int as expected
            ParamValue::Int(axis) => {
                match axis {
                    0 => { // X_AXIS
                        self.rc_command.fx.value = f_stick_value;
                        self.rc_command.fy.value = 0.0;
                        self.rc_command.fz.value = 0.0;
                    },
                    1 => { // Y_AXIS
                        self.rc_command.fx.value = 0.0;
                        self.rc_command.fy.value = f_stick_value;
                        self.rc_command.fz.value = 0.0;
                    },
                    _ => { // Z_AXIS (default)
                        self.rc_command.fx.value = 0.0;
                        self.rc_command.fy.value = 0.0;
                        self.rc_command.fz.value = f_stick_value;
                    }
                }
            },
            // Error case: param is wrong type!
            other_type => {
                // println!("Error: PARAM_RC_F_AXIS is not an Int, but {:?}! Defaulting to Fz.", other_type);
                // Default to Z_AXIS for safety
                self.rc_command.fx.value = 0.0;
                self.rc_command.fy.value = 0.0;
                self.rc_command.fz.value = f_stick_value;
            }
        }

        // All RC channels are always active... active for rc doesn't really mean anything. Active on offboard command means override rc...
        self.rc_command.qx.active = true;
        self.rc_command.qy.active = true;
        self.rc_command.qz.active = true;
        self.rc_command.fx.active = true;
        self.rc_command.fy.active = true;
        self.rc_command.fz.active = true;

        // C++: lines 130-153
        let is_fixed_wing = match params.get_by_id(ParamId::PARAM_FIXED_WING) {
            ParamValue::Bool(val) => val,
            other => {
                // This is a param definition error. Default to the safer
                // (non-fixed-wing) case.
                // println!("Error: PARAM_FIXED_WING is not a Bool, but {:?}! Defaulting to false (multirotor).", other);
                false
            }
        };
        if is_fixed_wing {
            self.rc_command.qx.control_type = ControlType::Passthrough;
            self.rc_command.qy.control_type = ControlType::Passthrough;
            self.rc_command.qz.control_type = ControlType::Passthrough;
            self.rc_command.fx.control_type = ControlType::Passthrough;
            self.rc_command.fy.control_type = ControlType::Passthrough;
            self.rc_command.fz.control_type = ControlType::Passthrough;
        } else {
            
            // check if we've mapped the AttType channel... 
            let mut roll_pitch_type = ControlType::Rate;
            if rc.switch_mapped(Switch::AttType) {
                // if we have, probe it to know if we should use rate or angle for qx and qy
                roll_pitch_type = if rc.switch_on(Switch::AttType) { ControlType::Angle } else { ControlType::Rate };
            } else {
                // if not, fall back to the parameter Attitude mode
                let att_mode = match params.get_by_id(ParamId::PARAM_RC_ATTITUDE_MODE) {
                    ParamValue::Int(val) => val,
                    other => {
                        // println!("Error: PARAM_OVERRIDE_LAG_TIME is not an Int, but {:?}! Defaulting to 200ms.", other);
                        200 // Default value from C++ params
                    }
                }; 
                roll_pitch_type = match att_mode {
                    ATTITUDE_RATE_MODE => {
                        ControlType::Rate
                    },
                    _ => {
                        // if we're not in rate mode, we're in pitch mode...
                        ControlType::Angle
                    }
                }
            }

            self.rc_command.qx.control_type = roll_pitch_type;
            self.rc_command.qy.control_type = roll_pitch_type;

            match roll_pitch_type {
                ControlType::Rate => {
                    let max_rollrate = match params.get_by_id(ParamId::PARAM_RC_MAX_ROLLRATE) {
                        ParamValue::Float(val) => val as f64,
                        other => {
                            1.0
                        }
                    }; 
                    let max_pitchrate = match params.get_by_id(ParamId::PARAM_RC_MAX_PITCHRATE) {
                        ParamValue::Float(val) => val as f64,
                        other => {
                            // println!("Error: PARAM_OVERRIDE_LAG_TIME is not an Int, but {:?}! Defaulting to 200ms.", other);
                            1.0
                        }
                    }; 
                    self.rc_command.qx.value *= max_rollrate;
                    self.rc_command.qx.value *= max_pitchrate;
                },
                ControlType::Angle => {
                    let max_roll = match params.get_by_id(ParamId::PARAM_RC_MAX_ROLL) {
                        ParamValue::Float(val) => val as f64,
                        other => {
                            1.0
                        }
                    }; 
                    let max_pitch = match params.get_by_id(ParamId::PARAM_RC_MAX_PITCH) {
                        ParamValue::Float(val) => val as f64,
                        other => {
                            // println!("Error: PARAM_OVERRIDE_LAG_TIME is not an Int, but {:?}! Defaulting to 200ms.", other);
                            1.0
                        }
                    }; 
                    self.rc_command.qx.value *= max_roll;
                    self.rc_command.qx.value *= max_pitch;
                },
                _ => {
                }
            }

            self.rc_command.qz.control_type = ControlType::Rate;
            let max_yawrate = match params.get_by_id(ParamId::PARAM_RC_MAX_YAWRATE) {
                ParamValue::Float(val) => val as f64,
                other => {
                    // println!("Error: PARAM_OVERRIDE_LAG_TIME is not an Int, but {:?}! Defaulting to 200ms.", other);
                    1.0
                }
            }; 
            self.rc_command.qz.value *= max_yawrate;

            self.rc_command.fx.control_type = ControlType::Throttle;
            self.rc_command.fy.control_type = ControlType::Throttle;
            self.rc_command.fz.control_type = ControlType::Throttle;
        }
    }

    /// This is the C++ `run` function's muxing logic
    fn do_muxing(&mut self, params: &Params, rc: &Rc, now_ms: u32) {
        // C++: command_manager.cpp (lines 253-256)
        self.rc_attitude_override = self.do_attitude_muxing(params, rc, now_ms);
        self.rc_throttle_override = self.do_throttle_muxing(params, rc);
    }

    fn do_attitude_muxing(&mut self, params: &Params, rc: &Rc, now_ms: u32) -> bool {
       let deviation_param = match params.get_by_id(ParamId::PARAM_RC_OVERRIDE_DEVIATION) {
            ParamValue::Float(val) => val as f64, // Convert f32 to f64
            other => {
                // println!("Error: PARAM_RC_OVERRIDE_DEVIATION is not a Float, but {:?}! Defaulting to 0.1.", other);
                0.1 // Default value from C++ params
            }
        };

        // --- REFACTORED: PARAM_OVERRIDE_LAG_TIME ---
        let lag_time_ms = match params.get_by_id(ParamId::PARAM_OVERRIDE_LAG_TIME) {
            ParamValue::Int(val) => val as u32,
            other => {
                // println!("Error: PARAM_OVERRIDE_LAG_TIME is not an Int, but {:?}! Defaulting to 200ms.", other);
                200 // Default value from C++ params
            }
        }; 
    
        let switch_override = rc.switch_mapped(Switch::AttOverride) && rc.switch_on(Switch::AttOverride);

        let axes = [
            (Stick::X, &self.rc_command.qx, &self.offboard_command.qx, &mut self.combined_command.qx),
            (Stick::Y, &self.rc_command.qy, &self.offboard_command.qy, &mut self.combined_command.qy),
            (Stick::Z, &self.rc_command.qz, &self.offboard_command.qz, &mut self.combined_command.qz),
        ];

        let mut any_axis_overridden = false;

        for (stick, rc_command, offboard_channel, combined_channel) in axes {
            let mut stick_is_deviated = false;

            if (rc.stick(stick) as f64).abs() > deviation_param {
                self.last_stick_override_time[stick as usize] = now_ms;
                stick_is_deviated = true;
            } else if now_ms < self.last_stick_override_time[stick as usize].saturating_add(lag_time_ms) {
                stick_is_deviated = true;
            }

            let override_this_axis = switch_override || stick_is_deviated || !offboard_channel.active;

            if override_this_axis {
                //*combined_channel = rc.stick(stick) as f64;
                //*combined_channel = self.rc_command[stick as usize];
                //(*combined_channel).value = rc_command.value;
                *combined_channel = *rc_command;
                any_axis_overridden = true;
            } else{
                //(*combined_channel).value = offboard_channel.value;
                *combined_channel = *offboard_channel;
            }
        }

        true
    }

    fn do_throttle_muxing(&mut self, params: &Params, rc: &Rc) -> bool {
        let throttle_axis_idx = match params.get_by_id(ParamId::PARAM_RC_F_AXIS) {
            ParamValue::Int(val) => val as u32,
            other => {
                // println!("Error: PARAM_OVERRIDE_LAG_TIME is not an Int, but {:?}! Defaulting to 200ms.", other);
                200 // Default value from C++ params
            }
        }; 

        let (rc_throttle_value, offboard_throttle_channel) = match throttle_axis_idx {
            0 => (self.rc_command.fx.value, &self.offboard_command.fx),
            1 => (self.rc_command.fy.value, &self.offboard_command.fy),
            _ => (self.rc_command.fz.value, &self.offboard_command.fz),
        };

        let mut override_active = false;

        if rc.switch_mapped(Switch::ThrottleOverride) && rc.switch_on(Switch::ThrottleOverride) {
            override_active = true;
        } else {
            if offboard_throttle_channel.active {
                let take_min = match params.get_by_id(ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE) {
                    ParamValue::Bool(val) => val,
                        other => {
                            // println!("Error: PARAM_OVERRIDE_LAG_TIME is not an Int, but {:?}! Defaulting to 200ms.", other);
                            true
                    }
                }; 

                if take_min {
                    override_active = rc_throttle_value < offboard_throttle_channel.value;
                } else {
                    override_active = false;
                }
            } else {
                override_active = true;
            }
        }

        if override_active {
            self.combined_command.fx = self.rc_command.fx;
            self.combined_command.fy = self.rc_command.fy;
            self.combined_command.fz = self.rc_command.fz;
        } else {
            self.combined_command.fx = self.offboard_command.fx;
            self.combined_command.fy = self.offboard_command.fy;
            self.combined_command.fz = self.offboard_command.fz;
        }

        override_active
    }

    pub fn combined_control(&self) -> &CombinedControl {
        &self.combined_command
    }

    pub fn get_control_mode(&self) -> ControlType {
        // We assume the qx (roll) channel represents the
        // primary attitude control mode.
        self.combined_command.qx.control_type
    }

    pub fn rc_override_active(&self) -> bool {
        self.rc_attitude_override || self.rc_throttle_override
    }

    /// Port of C++ `offboard_control_active` (line 220)
    pub fn is_offboard_active(&self) -> bool {
        // This is true if *any* offboard channel is active
        self.offboard_command.qx.active 
            || self.offboard_command.qy.active 
            || self.offboard_command.qz.active 
            || self.offboard_command.fx.active
            || self.offboard_command.fy.active
            || self.offboard_command.fz.active
    }
}

impl From<ControlType> for OffboardControlMode {
    fn from(val: ControlType) -> Self {
        match val {
            ControlType::Rate => OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
            ControlType::Passthrough => OffboardControlMode::ModePassThrough,
            ControlType::Angle => OffboardControlMode::ModeRollPitchYawrateThrottle,
            _ => OffboardControlMode::ModeRollPitchYawrateThrottle, // qx cannot ever be anything but rate, passthrough, or angle... so to satisfy match statement, set to angle... TODO clean up logic here
        }
    }
}
