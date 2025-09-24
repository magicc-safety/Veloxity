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

// Imports
use core::clone::Clone;
use core::cmp::{Eq, PartialEq};
use core::default::Default;
use core::fmt::Debug;
use core::marker::{Copy, PhantomData};
use core::result::Result;
use core::error::Error;
use bitflags::{bitflags, Flags};

/*
All possible states of the state manager
*/

// #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
// pub(crate) struct State<State_Value> {
//     _state: PhantomData<State_Value>,
// }

// // TODO: Implement methods / members for these structs
// #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
// pub(crate) struct Init;
// #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
// pub(crate) struct Preflight;
// #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
// pub(crate) struct Calibrating;
// #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
// pub(crate) struct Armed;
// #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
// pub(crate) struct Failsafe;
// #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
// pub(crate) struct ErrorPresent;

// #[derive(Debug, Clone, Copy, PartialEq, Eq)]
// pub(crate) enum StateEnum {
//     INIT(State<Init>),
//     PREFLIGHT(State<Preflight>),
//     CALIBRATING(State<Calibrating>),
//     ARMED(State<Armed>),
//     FAILSAFE(State<Failsafe>),
//     ERROR_PRESENT(State<ErrorPresent>),
// }

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
    REQUEST_ARM_AND_CALIBRATE,
    REQUEST_DISARM,
    REQUEST_DISARM_AND_ERROR,
    RC_LOST, 
    RC_FOUND,
    CALIBRATION_COMPLETE,
    CALIBRATION_FAILED,
    ERRORS_CLEARED,
    ERROR_OCCURRED(ErrorFlag),
    ERROR_CLEARED(ErrorFlag),
}

// Event handling
trait StateLogic: Debug + Copy {
    fn on_event(self, event: Event) -> State;
}

// Implementations for each state
impl StateLogic for Init {
    fn on_event(self, event: Event) -> State {
        match event {
            Event::INITIALIZED => State::Preflight(Preflight),
            _ => State::Init(self), // Ignore other events
        }
    }
}

impl StateLogic for Preflight {
    fn on_event(self, event: Event) -> State {
        match event {
            Event::REQUEST_ARM => {
                println!("I got here!");
                State::Armed(Armed)
            },
            Event::REQUEST_ARM_AND_CALIBRATE => State::Calibrating(Calibrating),
            Event::ERROR_OCCURRED(_) => State::ErrorPresent(ErrorPresent),
            _ => State::Preflight(self),
        }
    }
}

impl StateLogic for Calibrating {
    fn on_event(self, event: Event) -> State {
        match event {
            Event::CALIBRATION_COMPLETE => State::Armed(Armed),
            Event::CALIBRATION_FAILED => State::Preflight(Preflight),
            Event::ERROR_OCCURRED(_) => State::ErrorPresent(ErrorPresent),
            _ => State::Calibrating(self),
        }
    }
}

impl StateLogic for Armed {
    fn on_event(self, event: Event) -> State {
        match event {
            Event::REQUEST_DISARM => State::Preflight(Preflight),
            // disarm request goes to error if errors are present
            Event::REQUEST_DISARM_AND_ERROR => State::ErrorPresent(ErrorPresent),
            Event::RC_LOST=> State::Failsafe(Failsafe),
            // remains armed even if errors occur
            Event::ERROR_OCCURRED(_) => State::Armed(Self),
            _ => State::Armed(self),
        }
    }
}

impl StateLogic for Failsafe {
    fn on_event(self, event: Event) -> State {
        match event {
            Event::RC_FOUND => State::Armed(Armed),
            // Disarming from failsafe means errors are still present
            Event::REQUEST_DISARM => State::ErrorPresent(ErrorPresent),
            // While in failsafe ignore errors
            Event::ERROR_OCCURRED(_) => State::Failsafe(self),
            _ => State::Failsafe(self),
        }
    }
}

impl StateLogic for ErrorPresent {
    fn on_event(self, event: Event) -> State {
        match event {
            Event::ERRORS_CLEARED => State::Preflight(Preflight),
            _ => State::ErrorPresent(self)
        }
    }
}

#[derive(Debug, Default)]
pub struct StateMachine {
    state: State,
    error_flags: ErrorFlag,
    // need to add board ref for LED updating
}

impl StateMachine {
    // Creates a new StateMachine and immediately transitions it to Preflight state.
    pub fn new() -> Self {
        let mut sm = Self {
            state: State::Init(Init),
            error_flags: ErrorFlag::empty(),
        };
        sm.update(Event::INITIALIZED);
        sm
    }

    // delegates transition logic to individual states
    pub fn update(&mut self, event: Event) {
        // handle side effects
        match event {
            Event::ERROR_CLEARED(flag) => self.error_flags.remove(flag),
            Event::ERROR_OCCURRED(flag) => self.error_flags.insert(flag),
            Event::RC_LOST => self.error_flags.insert(ErrorFlag::RC_LOST),
            Event::RC_FOUND => self.error_flags.remove(ErrorFlag::RC_LOST),
            _ => (),
        }

        // transitions
        self.state = match self.state {
            State::Init(s) => s.on_event(event),
            State::Preflight(s) => s.on_event(event),
            State::Calibrating(s) => s.on_event(event),
            State::Armed(s) => s.on_event(event),
            State::Failsafe(s) => s.on_event(event),
            State::ErrorPresent(s) => s.on_event(event),
        };
    }

    pub fn run(&mut self) {
        self.process_errors();
        self.update_leds();
    }

    fn process_errors(&mut self) {
        if self.get_errors().is_empty() {
            // Move out of error state if cleared
            self.update(Event::ERROR_CLEARED(ErrorFlag::default()));
        } else {
            // Retry entering error state if errors still present
            self.update(Event::ERROR_OCCURRED(self.error_flags));
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
        // blink fast if in failsafe
        // blink slowly if in error
        // off if disarmed, on if armed
    }
}

/*
State struct implementations

Each struct should have methods that are unique to that state.

*/

// impl State<Init> {
//     fn new() -> StateEnum {
//         /*
//         NOTE: This is the only way a state can be created directly.

//         All other states must be created from previous states. They
//         do not have direct constructors.
//          */
//         StateEnum::INIT(State {
//             _state: PhantomData,
//         })
//     }

//     fn initialize(self) -> StateEnum {
//         StateEnum::PREFLIGHT(State {
//             _state: PhantomData,
//         })
//     }
// }

// impl State<Preflight> {
//     fn request_arm(self) -> StateEnum {
//         StateEnum::ARMED(State {
//             _state: PhantomData,
//         })
//     }

//     fn request_arm_and_calibrate(self) -> StateEnum {
//         StateEnum::CALIBRATING(State {
//             _state: PhantomData,
//         })
//     }
// }

// impl State<Calibrating> {
//     fn calibration_complete(self) -> StateEnum {
//         StateEnum::ARMED(State {
//             _state: PhantomData,
//         })
//     }

//     fn calibration_failed(self) -> StateEnum {
//         StateEnum::PREFLIGHT(State {
//             _state: PhantomData,
//         })
//     }
// }

// impl State<Armed> {
//     fn request_disarm(self) -> StateEnum {
//         StateEnum::PREFLIGHT(State {
//             _state: PhantomData,
//         })
//     }

//     // do we need this if we deal with errors immediately?
//     fn request_disarm_and_error(self) -> StateEnum {
//         StateEnum::ERROR_PRESENT(State {
//             _state: PhantomData,
//         })
//     }

//     fn rc_lost(self) -> StateEnum {
//         StateEnum::FAILSAFE(State {
//             _state: PhantomData,
//         })
//     }
// }

// impl State<Failsafe> {
//     fn rc_found(self) -> StateEnum {
//         StateEnum::ARMED(State {
//             _state: PhantomData,
//         })
//     }

//     fn request_disarm(self) -> StateEnum {
//         StateEnum::ERROR_PRESENT(State {
//             _state: PhantomData,
//         })
//     }

//     // Failsafe does not transition to error state unless there is a disarm request
//     fn failsafe_error(self) -> StateEnum {
//         StateEnum::FAILSAFE(State {
//             _state: PhantomData,
//         })
//     }
// }

// impl State<ErrorPresent> {
//     fn clear_errors(self) -> StateEnum {
//         StateEnum::PREFLIGHT(State {
//             _state: PhantomData,
//         })
//     }
// }

// impl<State_Value> State<State_Value> {
//     fn error(self) -> StateEnum {
//         // TODO: Pass in an error bitflag?
//         StateEnum::ERROR_PRESENT(State {
//             _state: PhantomData,
//         })
//     }
// }

// #[derive(Debug, Default)]
// pub(crate) struct StateMachine {
//     state: StateEnum,
//     error_flags: ErrorFlag,
//     armed: bool,
//     failsafe: bool,
// }

// impl StateMachine {
//     pub fn new() -> Self {
//         let mut sm = StateMachine { 
//             state: State::new(), 
//             error_flags: ErrorFlag::empty(), 
//             armed: false, 
//             failsafe: false 
//         };
//         sm.update(Ok(Event::INITIALIZED));
//         sm
//     }

//     pub fn run(&self) {
//         // not sure how this will work
//         self.process_errors();
//     }

//     pub fn process_errors(&self) {
//         // TODO: possibly iterate over current errors to see if error state transition is necessary?
//     }

//     pub fn get_state(&self) -> &StateEnum {
//         &self.state
//     }

//     pub fn get_errors(&self) -> ErrorFlag {
//         self.error_flags
//     }

//     pub fn set_error(&mut self, error_flag: ErrorFlag) {
//         self.error_flags |= error_flag;
//     }

//     pub fn unset_error(&mut self, error_flag: ErrorFlag) {
//         self.error_flags &= !error_flag;
//     }

//     pub fn is_armed(&self) -> bool {
//         self.armed
//     }

//     pub fn is_in_failsafe(&self) -> bool {
//         self.failsafe
//     }

//     pub fn update(&mut self, event: Result<Event, ErrorFlag>) {
//         // TODO: Have this update function return a Result to indicate success or failure?

//         /*

//         TODO: Change the results of the match statement so that we construct the next state from the
//         previous one using the typestate pattern.

//         For example, going from init -> preflight would be State<Init>::to_preflight() -> State<Preflight>
//         */
//         self.state = match event {
//             Ok(_event) => match (self.state, _event) {
//                 (StateEnum::INIT(state), Event::INITIALIZED) => {
//                     state.initialize()
//                 },
//                 (StateEnum::PREFLIGHT(state), Event::REQUEST_ARM) => {
//                     // Require low throttle to arm
//                     // require either min throttle or override switch
//                     // check to ensure calibration
//                     // move these checks to sensors
//                     self.armed = true;
//                     state.request_arm()
//                 },
//                 (StateEnum::PREFLIGHT(state), Event::REQUEST_ARM_AND_CALIBRATE) => {
//                     // start gyro calibration
//                     self.armed = true;
//                     state.request_arm_and_calibrate()
//                 },
//                 // (StateEnum::PREFLIGHT(state), Event::RC_FOUND) => {
//                 //     self.unset_error(ErrorFlag::RC_LOST);
//                 //     // Stay in the same state
//                 // },
//                 (StateEnum::CALIBRATING(state), Event::CALIBRATION_COMPLETE) => {
//                     self.armed = true;
//                     state.calibration_complete()
//                 },
//                 (StateEnum::CALIBRATING(state), Event::CALIBRATION_FAILED) => {
//                     state.calibration_failed()
//                 },
//                 (StateEnum::ARMED(state), Event::REQUEST_DISARM) => {
//                     self.armed = false;
//                     state.request_disarm()
//                 },
//                 (StateEnum::ARMED(state), Event::REQUEST_DISARM_AND_ERROR) => {
//                     state.request_disarm_and_error()
//                 },
//                 (StateEnum::ARMED(state), Event::RC_LOST) => {
//                     self.failsafe = true;
//                     // update status
//                     state.rc_lost()
//                 },
//                 (StateEnum::FAILSAFE(state), Event::RC_FOUND) => {
//                     self.failsafe = false;
//                     self.unset_error(ErrorFlag::RC_LOST);
//                     // clear error
//                     state.rc_found()
//                 },
//                 (StateEnum::FAILSAFE(state), Event::REQUEST_DISARM) => {
//                     self.armed = false;
//                     state.request_disarm()
//                 }
//                 (state, _) => state,
//             },
//             Err(_error) => {
//                 self.set_error(_error);
//                 match (self.state) {
//                     StateEnum::INIT(state) => state.error(),
//                     StateEnum::PREFLIGHT(state) => state.error(),
//                     StateEnum::CALIBRATING(state) => state.error(),
//                     StateEnum::ARMED(state) => state.error(), // need to not go to error state
//                     StateEnum::FAILSAFE(state) => state.failsafe_error(), // does not transition to error state
//                     StateEnum::ERROR_PRESENT(state) => state.error(),
//                 }
//             }
//         };
//     }
// }

// fn update_leds() {
//     // TODO
// }