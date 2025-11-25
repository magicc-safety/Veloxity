#![allow(non_camel_case_types)]

// /**
// ******************************************************************************
// * File     : mod.rs
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

#[cfg(test)]
mod tests;

use bitflags::bitflags;
use crate::{board::BoardTrait, comm_manager::CommManager, params2::{ParamValue, Params, ParamId}};
use core::{error, mem::take};
// use std::default;

// Events that trigger state transitions
#[derive(Debug, Clone, Copy)]
pub enum Event {
    INITIALIZED,
    REQUEST_ARM,
    REQUEST_DISARM,
    CALIBRATION_COMPLETE,
    CALIBRATION_FAILED,
    ERROR_OCCURRED(ErrorFlag),
    ERROR_CLEARED(ErrorFlag),
}

// Bitflags for tracking specific error conditions
bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
    pub struct ErrorFlag: u16 {
        const INVALID_MIXER = 1;
        const IMU_NOT_RESPONDING = 1 << 1;
        const RC_LOST = 1 << 2;
        const UNHEALTHY_ESTIMATOR = 1 << 3;
        const TIME_GOING_BACKWARDS = 1 << 4;
        const UNCALIBRATED_IMU = 1 << 5;
        const BUFFER_OVERRUN = 1 << 6;
        const INVALID_FAILSAFE = 1 << 7;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct State<S> {
    state: S,
    error_flags: ErrorFlag,
}

// Holds FSM as its type changes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)] 
pub enum StateMachine {
    Init(State<Init>),
    Preflight(State<Preflight>),
    Calibrating(State<Calibrating>),
    Armed(State<Armed>),
    Failsafe(State<Failsafe>),
    ErrorPresent(State<ErrorPresent>),
}

impl StateMachine {
    // New state machine starts in the Preflight state.
    pub fn new() -> Self {
        // let sm = State { state: Init, error_flags: ErrorFlag::empty() };
        // sm.state.on_event(sm, Event::INITIALIZED)

        StateMachine::Init(State::<Init>::default())
    }

    // gets a mutable reference to the error flags
    fn error_flags_mut(&mut self) -> &mut ErrorFlag {
        match self {
            StateMachine::Init(sm) => &mut sm.error_flags,
            StateMachine::Preflight(sm) => &mut sm.error_flags,
            StateMachine::Calibrating(sm) => &mut sm.error_flags,
            StateMachine::Armed(sm) => &mut sm.error_flags,
            StateMachine::Failsafe(sm) => &mut sm.error_flags,
            StateMachine::ErrorPresent(sm) => &mut sm.error_flags,
        }
    }

    // Transitions machine based on Event
    pub fn update(&mut self, event: Event, params: &Params) {
        // Handle errors
        match event {
            Event::ERROR_OCCURRED(flag) => self.error_flags_mut().insert(flag),
            Event::ERROR_CLEARED(flag) => self.error_flags_mut().remove(flag),
            _ => (),
        }

        // Consume old state, replace with new one
        let machine = take(self);
        *self = machine.transition(event, params);
    }

    fn transition(self, event: Event, params: &Params) -> Self {
        match self {
            StateMachine::Init(sm) => sm.state.on_event(sm, event),
            StateMachine::Preflight(sm) => sm.state.on_event(sm, event, params),
            StateMachine::Calibrating(sm) => sm.state.on_event(sm, event, params),
            StateMachine::Armed(sm) => sm.state.on_event(sm, event, params),
            StateMachine::Failsafe(sm) => sm.state.on_event(sm, event, params),
            StateMachine::ErrorPresent(sm) => sm.state.on_event(sm, event, params),
        }
    }
    
    pub fn get_errors(&self) -> ErrorFlag {
        match self {
            StateMachine::Init(sm) => sm.error_flags,
            StateMachine::Preflight(sm) => sm.error_flags,
            StateMachine::Calibrating(sm) => sm.error_flags,
            StateMachine::Armed(sm) => sm.error_flags,
            StateMachine::Failsafe(sm) => sm.error_flags,
            StateMachine::ErrorPresent(sm) => sm.error_flags,
        }
    }
    
    pub fn is_armed(&self) -> bool {
        matches!(self, StateMachine::Armed(_) | StateMachine::Failsafe(_))
    }

    pub fn is_in_failsafe(&self) -> bool {
        matches!(self, StateMachine::Failsafe(_))
    }

    pub fn is_in_error_state(&self) -> bool {
        matches!(self, StateMachine::ErrorPresent(_))
    }
}

impl Default for StateMachine {
    fn default() -> Self {
        Self::new()
    }
}

// State structs
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)] 
pub struct Init;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)] 
pub struct Preflight;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)] 
pub struct Calibrating;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)] 
pub struct Armed;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)] 
pub struct Failsafe;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)] 
pub struct ErrorPresent;

// State transition logic
impl Init {
    fn on_event(self, sm: State<Self>, event: Event) -> StateMachine {
        match event {
            Event::INITIALIZED => StateMachine::Preflight(State {
                state: Preflight,
                error_flags: sm.error_flags,
            }),
            _ => StateMachine::Init(sm),
        }
    }
}

impl Preflight {
    fn on_event(self, sm: State<Self>, event: Event, params: &Params) -> StateMachine {
        match event {
            Event::REQUEST_ARM => {
                //if let ParamValue::Bool(true) = Params::get_calibrate_gyro_on_arm(params) {
                if let ParamValue::Bool(true) = params.get_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM) {
                    let mut error_flags = sm.error_flags;
                    error_flags.remove(ErrorFlag::UNCALIBRATED_IMU);
                    StateMachine::Calibrating(State { state: Calibrating, error_flags: error_flags })
                } else {
                    let is_calibrated = {
                        let x_calibrated = match params.get_by_id(ParamId::PARAM_GYRO_X_BIAS) {
                            ParamValue::Float(val) => val != 0.0f32,
                            other => {
                                // defmt::info!("Error: PARAM_GYRO_X_BIAS is not a Float, but {:?}! Assuming uncalibrated.", other);
                                false
                            }
                        };
                        let y_calibrated = match params.get_by_id(ParamId::PARAM_GYRO_Y_BIAS) {
                            ParamValue::Float(val) => val != 0.0f32,
                            other => {
                                // defmt::info!("Error: PARAM_GYRO_Y_BIAS is not a Float, but {:?}! Assuming uncalibrated.", other);
                                false
                            }
                        };
                        let z_calibrated = match params.get_by_id(ParamId::PARAM_GYRO_Z_BIAS) {
                            ParamValue::Float(val) => val != 0.0f32,
                            other => {
                                // defmt::info!("Error: PARAM_GYRO_Z_BIAS is not a Float, but {:?}! Assuming uncalibrated.", other);
                                false
                            }
                        };
                        
                        x_calibrated || y_calibrated || z_calibrated
                    };

                    if is_calibrated {
                        // We are calibrated! Go to Armed.
                        StateMachine::Armed(State { state: Armed, error_flags: sm.error_flags })
                    } else {
                        // We are not calibrated, and we are not set to calibrate.
                        // This is an error. Set the flag. The `state_manager.run()`
                        // function will see this flag and move to ErrorPresent.
                        let mut error_flags = sm.error_flags;
                        error_flags.insert(ErrorFlag::UNCALIBRATED_IMU);
                        StateMachine::Preflight(State { state: Preflight, error_flags: error_flags })
                    }
                    
                }
            },
            Event::ERROR_OCCURRED(_) => StateMachine::ErrorPresent(State { state: ErrorPresent, error_flags: sm.error_flags }),
            _ => StateMachine::Preflight(sm),
        }
    }
}

impl Calibrating {
    fn on_event(self, sm: State<Self>, event: Event, _params: &Params) -> StateMachine {
        match event {
            Event::CALIBRATION_COMPLETE => {
                let mut error_flags = sm.error_flags;
                error_flags.remove(ErrorFlag::UNCALIBRATED_IMU);
                StateMachine::Armed(State { state: Armed, error_flags: error_flags })
            },
            Event::CALIBRATION_FAILED => StateMachine::Preflight(State { state: Preflight, error_flags: sm.error_flags }),
            Event::ERROR_OCCURRED(_) => StateMachine::ErrorPresent(State { state: ErrorPresent, error_flags: sm.error_flags }),
            _ => StateMachine::Calibrating(sm),
        }
    }
}

impl Armed {
    fn on_event(self, sm: State<Self>, event: Event, _params: &Params) -> StateMachine {
        match event {
            Event::REQUEST_DISARM => {
                if !sm.error_flags.is_empty() {
                    StateMachine::ErrorPresent(State { state: ErrorPresent, error_flags: sm.error_flags })
                } else {
                    StateMachine::Preflight(State { state: Preflight, error_flags: sm.error_flags })
                }
            },
            Event::ERROR_OCCURRED(ErrorFlag::RC_LOST) => StateMachine::Failsafe(State { state: Failsafe, error_flags: sm.error_flags }),
            _ => StateMachine::Armed(sm),
        }
    }
}

impl Failsafe {
    fn on_event(self, sm: State<Self>, event: Event, _params: &Params) -> StateMachine {
        match event {
            Event::ERROR_CLEARED(ErrorFlag::RC_LOST) => StateMachine::Armed(State { state: Armed, error_flags: sm.error_flags }),
            Event::REQUEST_DISARM => StateMachine::ErrorPresent(State { state: ErrorPresent, error_flags: sm.error_flags }),
            _ => StateMachine::Failsafe(sm),
        }
    }
}

impl ErrorPresent {
    fn on_event(self, sm: State<Self>, event: Event, _params: &Params) -> StateMachine {
        match event {
            Event::ERROR_CLEARED(_) => {
                if sm.error_flags.is_empty() {
                    StateMachine::Preflight(State { state: Preflight, error_flags: sm.error_flags })
                } else {
                    StateMachine::ErrorPresent(sm)
                }
            },
            _ => StateMachine::ErrorPresent(sm)
        }
    }
}

// Struct for state management
pub struct StateManager {
    machine: StateMachine,
}

impl StateManager {
    pub fn new() -> Self {
        StateManager { machine: StateMachine::new(), }
    }

    pub fn is_calibrating(&self) -> bool {
        matches!(self.machine, StateMachine::Calibrating(_))
    }

    // The main update loop. Takes an event and applies it to the internal state machine.
    pub fn update(&mut self, event: Event, params: &Params) {
        let start_state = self.machine;
        self.machine.update(event, params);
        if start_state != self.machine {
            // defmt::info!("Update: Armed {} | Failsafe {} | ErrorState {} | Errors {:b}", self.is_armed(), self.is_in_failsafe(), self.is_in_error_state(), self.get_errors().bits());
        }
    }

    pub fn run(&mut self, params: &Params) {
        // process errors
        if self.get_errors().is_empty() {
            // Move out of error state if cleared
            self.update(Event::ERROR_CLEARED(ErrorFlag::default()), params);
        } else {
            // Retry entering error state if errors still present
            self.update(Event::ERROR_OCCURRED(self.machine.get_errors()), params);
        }
        self.update_leds();
    }

    pub fn is_armed(&self) -> bool {
        self.machine.is_armed()
    }

    pub fn is_in_failsafe(&self) -> bool {
        self.machine.is_in_failsafe()
    }

    pub fn is_in_error_state(&self) -> bool {
        self.machine.is_in_error_state()
    }

    pub fn get_errors(&self) -> ErrorFlag {
        self.machine.get_errors()
    }

    fn update_leds(&self) {
        // TODO: Add board ref for LED updates
        // blink fast if in failsafe
        // blink slowly if in error
        // off if disarmed, on if armed
    }

    // left out backup data functionality
}
