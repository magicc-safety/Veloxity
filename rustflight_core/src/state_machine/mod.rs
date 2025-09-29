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
// Submodules
#[cfg(test)]
mod tests;

use core::clone::Clone;
use core::cmp::{Eq, PartialEq};
use core::default::Default;
use core::fmt::Debug;
use core::marker::{Copy, PhantomData};
use core::result::Result;
use core::error::Error;
use bitflags::{bitflags, Flags};
use crate::params::{ParamValue, Params};

/*
All possible states of the state manager
*/

// State enum
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Init(Init),
    Preflight(Preflight),
    Calibrating(Calibrating),
    Armed(Armed),
    Failsafe(Failsafe),
    ErrorPresent(ErrorPresent),
}

// States
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Init;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Preflight;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Calibrating;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Armed;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Failsafe;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ErrorPresent;

impl Default for State {
    fn default() -> Self {
        State::Init(Init)
    }
}

// Bitflags for errors
bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
    pub(crate) struct ErrorFlag: u16 {
        const INVALID_MIXER = 1;
        const IMU_NOT_RESPONDING = 1 << 1;
        const RC_LOST = 1 << 2;
        const UNHEALTHY_ESTIMATOR = 1 << 3;
        const TIME_GOING_BACKWARDS = 1 << 4;
        const UNCALIBRATED_IMU = 1 << 5;
        const INVALID_FAILSAFE = 1 << 6;
    }
}

// Events
#[derive(Debug)]
pub(crate) enum Event {
    INITIALIZED,
    REQUEST_ARM,
    REQUEST_DISARM,
    CALIBRATION_COMPLETE,
    CALIBRATION_FAILED,
    ERROR_OCCURRED(ErrorFlag),
    ERROR_CLEARED(ErrorFlag),
}

// Implementations for each state
impl Init {
    fn on_event(self, event: Event) -> State {
        match event {
            Event::INITIALIZED => State::Preflight(Preflight),
            _ => State::Init(self), // Ignore other events
        }
    }
}

impl Preflight {
    fn on_event(self, event: Event, params: &Params) -> State {
        match event {
            Event::REQUEST_ARM => {
                if let ParamValue::Bool(true) = Params::get_calibrate_gyro_on_arm(params) {
                    State::Calibrating(Calibrating)
                } else {
                    State::Armed(Armed)
                }
            },
            Event::ERROR_OCCURRED(_) => State::ErrorPresent(ErrorPresent),
            _ => State::Preflight(self),
        }
    }
}

impl Calibrating {
    fn on_event(self, event: Event) -> State {
        match event {
            Event::CALIBRATION_COMPLETE => State::Armed(Armed),
            Event::CALIBRATION_FAILED => State::Preflight(Preflight),
            Event::ERROR_OCCURRED(_) => State::ErrorPresent(ErrorPresent),
            _ => State::Calibrating(self),
        }
    }
}

impl Armed {
    fn on_event(self, event: Event, errors_present: bool) -> State {
        match event {
            Event::REQUEST_DISARM => {
                if errors_present { // disarm request goes to error if errors are present
                    State::ErrorPresent(ErrorPresent)
                } else {
                    State::Preflight(Preflight)
                }
            },
            Event::ERROR_OCCURRED(ErrorFlag::RC_LOST)=> State::Failsafe(Failsafe),
            // remains armed even if errors occur
            Event::ERROR_OCCURRED(_) => State::Armed(Self),
            _ => State::Armed(self),
        }
    }
}

impl Failsafe {
    fn on_event(self, event: Event) -> State {
        match event {
            Event::ERROR_CLEARED(ErrorFlag::RC_LOST) => State::Armed(Armed),
            // Disarming from failsafe means errors are still present
            Event::REQUEST_DISARM => State::ErrorPresent(ErrorPresent),
            // While in failsafe ignore errors
            Event::ERROR_OCCURRED(_) => State::Failsafe(self),
            _ => State::Failsafe(self),
        }
    }
}

impl ErrorPresent {
    fn on_event(self, event: Event, errors_cleared: bool) -> State {
        match event {
            Event::ERROR_CLEARED(_) => {
                if errors_cleared {
                    State::Preflight(Preflight)
                } else {
                    State::ErrorPresent(self)
                }
            },
            _ => State::ErrorPresent(self)
        }
    }
}

#[derive(Debug, Default)]
pub struct StateMachine {
    state: State,
    error_flags: ErrorFlag,
}

impl StateMachine {
    // Creates a new StateMachine and immediately transitions it to Preflight state.
    pub fn new(params: &Params) -> Self {
        let mut sm = Self {
            state: State::Init(Init),
            error_flags: ErrorFlag::empty(),
        };
        sm.update(Event::INITIALIZED, params);
        sm
    }

    // delegates transition logic to individual states
    pub fn update(&mut self, event: Event, params: &Params) {
        // handle side effects
        match event {
            Event::ERROR_CLEARED(flag) => self.error_flags.remove(flag),
            Event::ERROR_OCCURRED(flag) => self.error_flags.insert(flag),
            _ => (),
        };

        // transitions
        self.state = match self.state {
            State::Init(s) => s.on_event(event),
            State::Preflight(s) => s.on_event(event, params),
            State::Calibrating(s) => s.on_event(event),
            State::Armed(s) => s.on_event(event, !self.error_flags.is_empty()),
            State::Failsafe(s) => s.on_event(event),
            State::ErrorPresent(s) => s.on_event(event, self.error_flags.is_empty()),
        };
    }

    pub fn run(&mut self, params: &Params) {
        self.process_errors(params);
        self.update_leds();
    }

    fn process_errors(&mut self, params: &Params) {
        if self.get_errors().is_empty() {
            // Move out of error state if cleared
            self.update(Event::ERROR_CLEARED(ErrorFlag::default()), params);
        } else {
            // Retry entering error state if errors still present
            self.update(Event::ERROR_OCCURRED(self.error_flags), params);
        }
    }
    
    pub fn get_state(&self) -> &State {
        &self.state
    }

    pub fn get_errors(&self) -> ErrorFlag {
        self.error_flags
    }
    
    // Returns true if the state machine is currently in the Armed or Failsafe state.
    pub fn is_armed(&self) -> bool {
        matches!(self.state, State::Armed(_)) || matches!(self.state, State::Failsafe(_))
    }
    
    // Returns true if the state machine is currently in the Failsafe state.
    pub fn is_in_failsafe(&self) -> bool {
        matches!(self.state, State::Failsafe(_))
    }

    fn update_leds(&self) {
        // need to add board ref for LED updating
        // blink fast if in failsafe
        // blink slowly if in error
        // off if disarmed, on if armed
    }

    // left out backup data functionality

    // update with ROSflight status message
}