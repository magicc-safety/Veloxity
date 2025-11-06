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
use crate::rc::{Rc, Stick, Switch};
use crate::params2::{Params, ParamId, ParamValue};
use crate::state_machine::{StateManager, Event, ErrorFlag};
use crate::board::BoardTrait;
use crate::comm_messages::{messages::OffboardControlMsg, enums::{OffboardControlIgnore, OffboardControlMode::{self, *}}};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ControlType {
    Rate,        // Channel is is in rate mode (rad/s)
    //Angle,       // Channel command is in angle mode (rad)
    //Throttle,    // Channel is controlling throttle setting
    Passthrough, // Channel directly passes PWM input to the mixer
}

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
                fz: ControlChannel { active: true, control_type: ControlType::Passthrough, value: 0.3 },
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
        let failsafe_throttle = params.get_param_float(ParamId::PARAM_FAILSAFE_THROTTLE);
        let is_fixed_wing = params.get_param_bool(ParamId::PARAM_FIXED_WING);

        // C++ logic from lines 74-79
        if !is_fixed_wing && (failsafe_throttle < 0.0 || failsafe_throttle > 1.0) {
            // Failsafe throttle is invalid, set an error
            state_manager.update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_FAILSAFE), params);
        } else {
            // Failsafe throttle is valid
            state_manager.update(Event::ERROR_CLEARED(ErrorFlag::INVALID_FAILSAFE), params);
        }

        // C++ logic from lines 81-93
        // Update the internal failsafe command value based on F-axis
        match params.get_param_int(ParamId::PARAM_RC_F_AXIS) {
            0 => self.multirotor_failsafe_command.fx.value = failsafe_throttle as f64,
            1 => self.multirotor_failsafe_command.fy.value = failsafe_throttle as f64,
            _ => self.multirotor_failsafe_command.fz.value = failsafe_throttle as f64,
        }
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
            let is_fixed_wing = params.get_param_int(ParamId::PARAM_FIXED_WING) > 0;
            self.combined_command = if is_fixed_wing {
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
            if self.is_offboard_active() {
                let timeout_ms = params.get_param_int(ParamId::PARAM_OFFBOARD_TIMEOUT) as u32;

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
            }
            
            // --- 4. Muxing (C++ lines 253-256) ---
            self.do_muxing(params, rc, now_ms);
        }
        
        true
    }

    /// Receives a new offboard control command.
    /// This should be called from CommManager::act_on_messages
    pub fn set_new_offboard_command(&mut self, now_us: u64, msg: &OffboardControlMsg) {
        // We got a new command, so update the timestamp
        self.last_offboard_command_us = now_us;
        self.offboard_command.stamp_ms = (now_us / 1000) as u32;

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
                self.offboard_command.fz.control_type = ControlType::Passthrough; // This is throttle
                
                // Set others to inactive by default in this mode
                self.offboard_command.fx.active = false;
                self.offboard_command.fy.active = false;
            }
            // ... (Add other modes as you need them) ...
        }

        // Apply values and ignore flags
        self.offboard_command.qx.value = msg.qx as f64; // <-- Cast to f64
        self.offboard_command.qx.active = !msg.ignore.is_ignoring_qx();

        self.offboard_command.qy.value = msg.qy as f64; // <-- Cast to f64
        self.offboard_command.qy.active = !msg.ignore.is_ignoring_qy();

        self.offboard_command.qz.value = msg.qz as f64; // <-- Cast to f64
        self.offboard_command.qz.active = !msg.ignore.is_ignoring_qz();

        self.offboard_command.fx.value = msg.fx as f64; // <-- Cast to f64
        self.offboard_command.fx.active = !msg.ignore.is_ignoring_fx();

        self.offboard_command.fy.value = msg.fy as f64; // <-- Cast to f64
        self.offboard_command.fy.active = !msg.ignore.is_ignoring_fy();
        
        self.offboard_command.fz.value = msg.fz as f64; // <-- Cast to f64
        self.offboard_command.fz.active = !msg.ignore.is_ignoring_fz();
    }

    /// Port of C++ `interpret_rc` (command_manager.cpp:101)
    fn interpret_rc(&mut self, rc: &Rc, params: &Params) {
        // Read all stick values from the RC unit
        self.rc_command.qx.value = rc.stick(Stick::X) as f64;
        self.rc_command.qy.value = rc.stick(Stick::Y) as f64;
        self.rc_command.qz.value = rc.stick(Stick::Z) as f64;
        
        // C++: line 109, logic for F-axis
        match params.get_param_int(ParamId::PARAM_RC_F_AXIS) {
            0 => { // X_AXIS
                self.rc_command.fx.value = rc.stick(Stick::F) as f64;
                self.rc_command.fy.value = 0.0;
                self.rc_command.fz.value = 0.0;
            },
            1 => { // Y_AXIS
                self.rc_command.fx.value = 0.0;
                self.rc_command.fy.value = rc.stick(Stick::F) as f64;
                self.rc_command.fz.value = 0.0;
            },
            _ => { // Z_AXIS (default)
                self.rc_command.fx.value = 0.0;
                self.rc_command.fy.value = 0.0;
                self.rc_command.fz.value = rc.stick(Stick::F) as f64;
            }
        }

        // All RC channels are always active
        self.rc_command.qx.active = true;
        self.rc_command.qy.active = true;
        self.rc_command.qz.active = true;
        self.rc_command.fx.active = true;
        self.rc_command.fy.active = true;
        self.rc_command.fz.active = true;

        // C++: lines 130-153
        let is_fixed_wing = params.get_param_int(ParamId::PARAM_FIXED_WING) > 0;
        if is_fixed_wing {
            self.rc_command.qx.control_type = ControlType::Passthrough;
            self.rc_command.qy.control_type = ControlType::Passthrough;
            self.rc_command.qz.control_type = ControlType::Passthrough;
            self.rc_command.fx.control_type = ControlType::Passthrough;
            self.rc_command.fy.control_type = ControlType::Passthrough;
            self.rc_command.fz.control_type = ControlType::Passthrough;
        } else {
            // (Note: C++ `interpret_rc` (line 136) has complex logic for roll/pitch type.
            // Your old `interpret_rc` defaulted to Rate. We'll keep that.)
            self.rc_command.qx.control_type = ControlType::Rate;
            self.rc_command.qy.control_type = ControlType::Rate;
            self.rc_command.qz.control_type = ControlType::Rate;
            
            // Throttle is always Passthrough from RC
            self.rc_command.fx.control_type = ControlType::Passthrough;
            self.rc_command.fy.control_type = ControlType::Passthrough;
            self.rc_command.fz.control_type = ControlType::Passthrough;
        }
    }

    /// This is the C++ `run` function's muxing logic
    fn do_muxing(&mut self, params: &Params, rc: &Rc, now_ms: u32) {
        // C++: command_manager.cpp (lines 253-256)
        self.rc_attitude_override = self.do_attitude_muxing(params, rc, now_ms);
        self.rc_throttle_override = self.do_throttle_muxing(params, rc);

        // Update combined command based on muxing results
        if self.rc_attitude_override {
            self.combined_command.qx = self.rc_command.qx;
            self.combined_command.qy = self.rc_command.qy;
            self.combined_command.qz = self.rc_command.qz;
        } else {
            self.combined_command.qx = self.offboard_command.qx;
            self.combined_command.qy = self.offboard_command.qy;
            self.combined_command.qz = self.offboard_command.qz;
        }

        if self.rc_throttle_override {
            self.combined_command.fx = self.rc_command.fx;
            self.combined_command.fy = self.rc_command.fy;
            self.combined_command.fz = self.rc_command.fz;
        } else {
            self.combined_command.fx = self.offboard_command.fx;
            self.combined_command.fy = self.offboard_command.fy;
            self.combined_command.fz = self.offboard_command.fz;
        }
    }

    /// Port of C++ `do_roll_pitch_yaw_muxing` and `stick_deviated`
    fn do_attitude_muxing(&mut self, params: &Params, rc: &Rc, now_ms: u32) -> bool {
        // C++: line 186
        if rc.switch_mapped(Switch::AttOverride) && rc.switch_on(Switch::AttOverride) {
            return true;
        }
        
        // C++: `stick_deviated` logic (lines 160-179)
        let deviation_param = params.get_param_float(ParamId::PARAM_RC_OVERRIDE_DEVIATION) as f64;
        let lag_time_ms = params.get_param_int(ParamId::PARAM_OVERRIDE_LAG_TIME) as u32;

        // Check sticks [X, Y, Z]
        let sticks = [Stick::X, Stick::Y, Stick::Z];
        let mut stick_deviated = false;
        for (i, &stick) in sticks.iter().enumerate() {
            if (rc.stick(stick) as f64).abs() > deviation_param {
                self.last_stick_override_time[i] = now_ms; // Update last time
                stick_deviated = true;
            } else if now_ms < self.last_stick_override_time[i].saturating_add(lag_time_ms) {
                // If we are still in the lag time, it's still "deviated"
                stick_deviated = true;
            }
        }
        if stick_deviated {
            return true;
        }
        
        // C++: line 196
        // If offboard is inactive, RC takes over
        if !self.offboard_command.qx.active
            && !self.offboard_command.qy.active
            && !self.offboard_command.qz.active
        {
            return true;
        }

        false // Offboard has control
    }
    
    /// Port of C++ `do_throttle_muxing` (lines 203-236)
    fn do_throttle_muxing(&self, params: &Params, rc: &Rc) -> bool {
        // 1. Check for override switch (C++ line 206)
        // *** This is a FIX: Your code was checking AttOverride ***
        if rc.switch_mapped(Switch::ThrottleOverride) && rc.switch_on(Switch::ThrottleOverride) {
            return true;
        }

        // C++: line 214
        let offboard_force_active = self.offboard_command.fx.active
            || self.offboard_command.fy.active
            || self.offboard_command.fz.active;

        if offboard_force_active {
            // 2. Check "take minimum throttle" parameter (C++ line 217)
            let take_min_throttle = params.get_param_int(ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE) > 0;
            
            if take_min_throttle {
                // *** This is a FIX: We must compare the correct F-axis ***
                // (C++ lines 220-228)
                let rc_throttle = match params.get_param_int(ParamId::PARAM_RC_F_AXIS) {
                    0 => self.rc_command.fx.value,
                    1 => self.rc_command.fy.value,
                    _ => self.rc_command.fz.value,
                };
                let offboard_throttle = match params.get_param_int(ParamId::PARAM_RC_F_AXIS) {
                    0 => self.offboard_command.fx.value,
                    1 => self.offboard_command.fy.value,
                    _ => self.offboard_command.fz.value,
                };
                
                return rc_throttle < offboard_throttle;
            } else {
                // If not taking min, offboard has control
                return false;
            }
        } else {
            // 3. If offboard is inactive, RC takes over
            return true;
        }
    }

    /// C++: line 301
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
        }
    }
}







//     pub fn run<B: BoardTrait, CI: CommInterface<B>>(
//         &mut self,
//         now_ms: u32,
//         comm_manager: &CommManager<B, CI>,
//         params: &Params,
//         rc: &mut Rc,
//         state_manager: &StateManager,
//     ) -> bool
//     where
//         B: BoardTrait,
//     {

//         // 1. Check CommManager for a new offboard command before any other logic.
//         if let Some(msg) = &comm_manager.msgs.offboard_control {
//             // 1. Create a mutable new command to build into.
//             let mut new_cmd = Control {
//                 stamp_ms: now_ms,
//                 ..Default::default()
//             };

//             // 2. Determine the control types for each channel based on the mode.
//             match msg.mode {
//                 OffboardControlMode::ModePassThrough => {
//                     new_cmd.qx.control_type = ControlType::Passthrough;
//                     new_cmd.qy.control_type = ControlType::Passthrough;
//                     new_cmd.qz.control_type = ControlType::Passthrough;
//                     new_cmd.fx.control_type = ControlType::Passthrough;
//                     new_cmd.fy.control_type = ControlType::Passthrough;
//                     new_cmd.fz.control_type = ControlType::Passthrough;
//                 },
//                 OffboardControlMode::ModeRollratePitchrateYawrateThrottle => {
//                     new_cmd.qx.control_type = ControlType::Rate;
//                     new_cmd.qy.control_type = ControlType::Rate;
//                     new_cmd.qz.control_type = ControlType::Rate;
//                     new_cmd.fz.control_type = ControlType::Passthrough;
//                 }
//             }

//             // 3. Apply the values and ignore flags to all channels, using our robust helpers.
//             new_cmd.qx.value = msg.qx;
//             new_cmd.qx.active = !msg.ignore.is_ignoring_qx();

//             new_cmd.qy.value = msg.qy;
//             new_cmd.qy.active = !msg.ignore.is_ignoring_qy();

//             new_cmd.qz.value = msg.qz;
//             new_cmd.qz.active = !msg.ignore.is_ignoring_qz();

//             new_cmd.fx.value = msg.fx;
//             new_cmd.fx.active = !msg.ignore.is_ignoring_fx();

//             new_cmd.fy.value = msg.fy;
//             new_cmd.fy.active = !msg.ignore.is_ignoring_fy();
    
//             new_cmd.fz.value = msg.fz;
//             new_cmd.fz.active = !msg.ignore.is_ignoring_fz();
    
//             // 4. Finally, update the command manager's internal state.
//             self.offboard_command = new_cmd;
//         }

//         // 2. Failsafe Priority...
//         let is_fixed_wing = if let ParamValue::Bool(is_fixed_wing) = params.get_by_id(ParamId::PARAM_FIXED_WING) {
//             is_fixed_wing
//         } else {
//             false
//         };

//         if state_manager.is_in_failsafe() {
//             self.combined_command = if is_fixed_wing {
//                 self.fixedwing_failsafe_command
//             } else {
//                 self.multirotor_failsafe_command
//             };
//             return true
//         }

//         // 3. Check for new RC command to trigger muxing logic
//         if rc.new_command() {
//             self.interpret_rc(rc, params);

//             // 4. Check for offboard control timeout!
//             let offboard_timeout_ms = if let ParamValue::Uint(val) = params.get_by_id(ParamId::PARAM_OFFBOARD_TIMEOUT) {
//                 val
//             } else {
//                 // Default to a safe timeout (e.g., 100ms) if the parameter is not found or is the wrong type
//                 100 
//             };
            
//             if now_ms > self.offboard_command.stamp_ms + offboard_timeout_ms {
//                 // If it has been too long since the last offboard command, disable all channels.
//                 // This prevents the drone from executing stale commands and allows the muxer
//                 // to fallback to RC control.
//                 self.offboard_command.qx.active = false;
//                 self.offboard_command.qy.active = false;
//                 self.offboard_command.qz.active = false;
//                 self.offboard_command.fx.active = false;
//                 self.offboard_command.fy.active = false;
//                 self.offboard_command.fz.active = false;
//             }
        
//             // 5. Perform Muxing
//             self.rc_attitude_override = self.do_attitude_muxing(params, rc);
//             self.rc_throttle_override = self.do_throttle_muxing(params, rc);

//             // 6. Update combined command based on muxing results
//             if self.rc_attitude_override {
//                 self.combined_command.qx = self.rc_command.qx;
//                 self.combined_command.qy = self.rc_command.qy;
//                 self.combined_command.qz = self.rc_command.qz;
//             } else {
//                 self.combined_command.qx = self.offboard_command.qx;
//                 self.combined_command.qy = self.offboard_command.qy;
//                 self.combined_command.qz = self.offboard_command.qz;
//             }

//             if self.rc_throttle_override {
//                 // Note: RC only commands Fz (throttle), so Fx and Fy are set to inactive from the RC command.
//                 self.combined_command.fx = self.rc_command.fx;
//                 self.combined_command.fy = self.rc_command.fy;
//                 self.combined_command.fz = self.rc_command.fz;
//             } else {
//                 self.combined_command.fx = self.offboard_command.fx;
//                 self.combined_command.fy = self.offboard_command.fy;
//                 self.combined_command.fz = self.offboard_command.fz;
//             }
//         } 

//         true
//     }

//     fn interpret_rc(&mut self, rc: &Rc, params: &Params) {
//         // Read all relevant stick values from the RC unit
//         self.rc_command.qx.value = rc.stick(Stick::X); // Corresponds to STICK_X
//         self.rc_command.qy.value = rc.stick(Stick::Y); // Corresponds to STICK_Y
//         self.rc_command.qz.value = rc.stick(Stick::Z); // Corresponds to STICK_Z
//         self.rc_command.fz.value = rc.stick(Stick::F); // Corresponds to STICK_F

//         // Set the control types according to the new default.
//         // In a full implementation, this logic would be more complex, likely
//         // checking a parameter to see if the pilot prefers angle or rate mode.
//         self.rc_command.qx.control_type = ControlType::Rate;
//         self.rc_command.qy.control_type = ControlType::Rate;
//         self.rc_command.qz.control_type = ControlType::Rate;
//         self.rc_command.fz.control_type = ControlType::Passthrough;
//     }

//     fn do_attitude_muxing(&self, params: &Params, rc: &Rc) -> bool {
//         // 1. Check if any of the attitude sticks have deviated from center.
//         let deviation_param = if let ParamValue::Float(val) = params.get_by_id(ParamId::PARAM_RC_OVERRIDE_DEVIATION) {
//             val
//         } else {
//             0.15 // A safe default deviation (15%)
//         };

//         let roll_stick = rc.stick(Stick::X);  // Corresponds to STICK_X
//         let pitch_stick = rc.stick(Stick::Y); // Corresponds to STICK_Y
//         let yaw_stick = rc.stick(Stick::Z);   // Corresponds to STICK_Z

//         if roll_stick.abs() > deviation_param
//             || pitch_stick.abs() > deviation_param
//             || yaw_stick.abs() > deviation_param
//         {
//             return true;
//         }

//         // 3. If the offboard command is inactive, RC should take over by default.
//         if !self.offboard_command.qx.active
//             && !self.offboard_command.qy.active
//             && !self.offboard_command.qz.active
//         {
//             return true;
//         }

//         false
//     }

//     fn do_throttle_muxing(&self, params: &Params, rc: &Rc) -> bool {
//         // 1. Check for a dedicated override switch on the transmitter.
//         // Assumes switch 1 corresponds to RC::SWITCH_THROTTLE_OVERRIDE
//         if rc.switch_mapped(Switch::AttOverride) && rc.switch_on(Switch::AttOverride) {
//             return true;
//         }

//         // Check if any offboard force commands are active.
//         let offboard_force_active = self.offboard_command.fx.active
//             || self.offboard_command.fy.active
//             || self.offboard_command.fz.active;

//         if offboard_force_active {
//             // If offboard is active, check the "take minimum throttle" parameter.
//             let take_min_throttle = if let ParamValue::Bool(val) = params.get_by_id(ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE) {
//                 val
//             } else {
//                 false // Default to not using this feature
//             };
        
//             if take_min_throttle {
//                 // RC overrides if its throttle is less than the offboard throttle.
//                 // We only compare against Fz, as it's the primary throttle axis.
//                 return self.rc_command.fz.value < self.offboard_command.fz.value;
//             } else {
//                 // If not taking min, offboard has control.
//                 return false;
//             }
//         } else {
//             // 3. If the offboard command is inactive, RC should take over by default.
//             return true;
//         }
//     }

//     pub fn combined_control(&self) -> &Control {
//         &self.combined_command
//     }

//     pub fn get_control_mode(&self) -> ControlType {
//         // We assume the qx (roll) channel represents the
//         // primary attitude control mode.
//         self.combined_command.qx.control_type
//     }

//     pub fn rc_override_active(&self) -> bool {
//         self.rc_attitude_override || self.rc_throttle_override
//     }

//     pub fn is_offboard_active(&self) -> bool {
//         // "Offboard" means we are *not* in RC override AND
//         // at least one offboard channel is active.
//         !self.rc_override_active() &&
//         (self.offboard_command.qx.active ||
//          self.offboard_command.qy.active ||
//          self.offboard_command.qz.active ||
//          self.offboard_command.fx.active ||
//          self.offboard_command.fy.active ||
//          self.offboard_command.fz.active)
//     }
// }

// impl From<ControlType> for OffboardControlMode {
//     fn from(val: ControlType) -> Self {
//         match val {
//             ControlType::Rate => OffboardControlMode::ModeRollratePitchrateYawrateThrottle,
//             ControlType::Passthrough => OffboardControlMode::ModePassThrough,
//             // Add other mappings if you create more ControlTypes
//         }
//     }
// }