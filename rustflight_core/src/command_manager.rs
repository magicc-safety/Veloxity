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
use crate::state_machine::StateManager;
use crate::comm_manager::CommManager;
use crate::params2::{Params::{self, ParamId}, ParamValue};
use crate::board::BoardTrait;
use crate::comm_messages::{messages::OffboardControlMsg, enums::{OffboardControlIgnore::{self, *}, OffboardControlMode::{self, *}}};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ControlType {
    Rate,        // Channel is is in rate mode (rad/s)
    Angle,       // Channel command is in angle mode (rad)
    Throttle,    // Channel is controlling throttle setting
    Passthrough, // Channel directly passes PWM input to the mixer
}

#[derive(Clone, Copy, Debug)]
pub struct ControlChannel {
    pub active: bool,
    pub control_type: ControlType,
    pub value: f32,
}

impl Default for ControlChannel {
    fn default() -> Self {
        Self {
            active: false,
            control_type: ControlType::Angle,
            value: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Control {
    pub stamp_ms: u32,
    pub qx: ControlChannel,
    pub qy: ControlChannel,
    pub qz: ControlChannel,
    pub fx: ControlChannel,
    pub fy: ControlChannel,
    pub fz: ControlChannel,
}

mod mock_rc {
    pub struct Rc;
    impl Rc {
        pub fn new_command(&self) -> bool { true }
        pub fn stick(&self, _stick: u8) -> f32 { 0.0 }
        pub fn switch_mapped(&self, _switch: u8) -> bool { false }
        pub fn switch_on(&self, _switch: u8) -> bool { false }
    }
}

#[derive(Default)]
pub struct CommandManager {
    // Command structs
    rc_command: Control,
    offboard_command: Control,
    combined_command: Control,
    multirotor_failsafe_command: Control,
    fixedwing_failsafe_command: Control,

    // State flags
    rc_throttle_override: bool,
    rc_attitude_override: bool,
}

impl CommandManager {
    /// Creates a new CommandManager with default initial values.
    pub fn new() -> Self {
        Self {
            multirotor_failsafe_command: Control {
                qx: ControlChannel { active: true, control_type: ControlType::Angle, value: 0.0 },
                qy: ControlChannel { active: true, control_type: ControlType::Angle, value: 0.0 },
                qz: ControlChannel { active: true, control_type: ControlType::Rate, value: 0.0 },
                fz: ControlChannel { active: true, control_type: ControlType::Throttle, value: 0.3 },
                ..Default::default()
            },
            fixedwing_failsafe_command: Control {
                qx: ControlChannel { active: true, control_type: ControlType::Passthrough, value: 0.0 },
                qy: ControlChannel { active: true, control_type: ControlType::Passthrough, value: 0.0 },
                qz: ControlChannel { active: true, control_type: ControlType::Passthrough, value: 0.0 },
                fx: ControlChannel { active: true, control_type: ControlType::Passthrough, value: 0.0 },
                ..Default::default()
            },
            ..Default::default()
        }
    }

    pub fn run<B: BoardTrait, CI: CommInterface<B>>(
        &mut self,
        comm_manager: &CommManager<B, CI>,
        board: &B,
        params: &Params,
        rc: &mock_rc::Rc,
        state_manager: &StateManager,
    ) -> bool
    where
        B: BoardTrait,
    {

        // 1. Check CommManager for a new offboard command before any other logic.
        if let Some(msg) = &comm_manager.msgs.offboard_control {
            // 1. Create a mutable new command to build into.
            let mut new_cmd = Control {
                stamp_ms: board.clock_millis(),
                ..Default::default()
            };

            // 2. Determine the control types for each channel based on the mode.
            match msg.mode {
                OffboardControlMode::ModePassThrough => {
                    new_cmd.qx.control_type = ControlType::Passthrough;
                    new_cmd.qy.control_type = ControlType::Passthrough;
                    new_cmd.qz.control_type = ControlType::Passthrough;
                    new_cmd.fx.control_type = ControlType::Passthrough;
                    new_cmd.fy.control_type = ControlType::Passthrough;
                    new_cmd.fz.control_type = ControlType::Passthrough;
                },
                OffboardControlMode::ModeRollratePitchrateYawrateThrottle => {
                    new_cmd.qx.control_type = ControlType::Rate;
                    new_cmd.qy.control_type = ControlType::Rate;
                    new_cmd.qz.control_type = ControlType::Rate;
                    new_cmd.fz.control_type = ControlType::Throttle;
                },
                OffboardControlMode::ModeRollPitchYawrateThrottle => {
                    new_cmd.qx.control_type = ControlType::Angle;
                    new_cmd.qy.control_type = ControlType::Angle;
                    new_cmd.qz.control_type = ControlType::Rate;
                    new_cmd.fz.control_type = ControlType::Throttle;
                },
                OffboardControlMode::ModeRollPitchYawrateAltitude => {
                    new_cmd.qx.control_type = ControlType::Angle;
                    new_cmd.qy.control_type = ControlType::Angle;
                    new_cmd.qz.control_type = ControlType::Rate;
                    new_cmd.fz.control_type = ControlType::Altitude; // Command is altitude
                },
                OffboardControlMode::ModeXvelYvelYawrateAltitude => {
                    // Here, fx and fy now represent body-fixed velocities
                    new_cmd.fx.control_type = ControlType::Velocity; 
                    new_cmd.fy.control_type = ControlType::Velocity;
                    new_cmd.qz.control_type = ControlType::Rate;
                    new_cmd.fz.control_type = ControlType::Altitude;
                }
                OffboardControlMode::ModeXposYposYawAltitude => {
                    // Here, fx and fy now represent inertial positions
                    new_cmd.fx.control_type = ControlType::Position; 
                    new_cmd.fy.control_type = ControlType::Position;
                    new_cmd.qz.control_type = ControlType::Angle;    // Yaw is an angle
                    new_cmd.fz.control_type = ControlType::Altitude;
                }
            }

            // 3. Apply the values and ignore flags to all channels, using our robust helpers.
            new_cmd.qx.value = msg.qx;
            new_cmd.qx.active = !msg.ignore.is_ignoring_qx();

            new_cmd.qy.value = msg.qy;
            new_cmd.qy.active = !msg.ignore.is_ignoring_qy();

            new_cmd.qz.value = msg.qz;
            new_cmd.qz.active = !msg.ignore.is_ignoring_qz();

            new_cmd.fx.value = msg.fx;
            new_cmd.fx.active = !msg.ignore.is_ignoring_fx();

            new_cmd.fy.value = msg.fy;
            new_cmd.fy.active = !msg.ignore.is_ignoring_fy();
    
            new_cmd.fz.value = msg.fz;
            new_cmd.fz.active = !msg.ignore.is_ignoring_fz();
    
            // 4. Finally, update the command manager's internal state.
            self.offboard_command = new_cmd;
        }

        // 2. Failsafe Priority...
        let is_fixed_wing = if let ParamValue::Bool(is_fixed_wing) = params.get_by_id(ParamId::PARAM_FIXED_WING) {
            is_fixed_wing
        } else {
            false
        };

        if state_manager.is_in_failsafe() {
            self.combined_command = if is_fixed_wing {
                self.fixedwing_failsafe_command
            } else {
                self.multirotor_failsafe_command
            };
            return true
        }

        // 3. Check for new RC command to trigger muxing logic
        if rc.new_command() {
            self.interpret_rc(rc);

            // 4. Check for offboard control timeout!
            let offboard_timeout_ms = if let ParamValue(val) = params.get_by_id(ParamId::PARAM_OFFBOARD_TIMEOUT) as u32 {
                val
            } else {
                1u32
            };
            
            if board.clock_millis() > self.offboard_command.stamp_ms + offboard_timeout_ms {
                // If it has been too long since the last offboard command, disable all channels.
                // This prevents the drone from executing stale commands and allows the muxer
                // to fall back to RC control.
                self.offboard_command.qx.active = false;
                self.offboard_command.qy.active = false;
                self.offboard_command.qz.active = false;
                self.offboard_command.fx.active = false;
                self.offboard_command.fy.active = false;
                self.offboard_command.fz.active = false;
            };
        
            // 5. Perform Muxing
            self.rc_attitude_override = self.do_attitude_muxing(params, rc);
            self.rc_throttle_override = self.do_throttle_muxing(rc);

            true
        } else {
            true
        }

    }

    fn interpret_rc(&mut self, rc: &mock_rc::Rc) {
        self.rc_command.qx.value = rc.stick(0);
    }

    

}