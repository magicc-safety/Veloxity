#![allow(non_camel_case_types)]

// Submodules
#[cfg(test)]
mod tests;

// Imports
use core::clone::Clone;
use core::cmp::{Eq, PartialEq};
use core::default::Default;
use core::fmt::Debug;
use core::marker::{Copy, PhantomData};

use bitflags::bitflags;

/*
All possible states of the state manager
 */

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct State<State_Value> {
    _state: PhantomData<State_Value>,
}

// TODO: Implement methods / members for these structs
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct Init;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct Preflight;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct Calibrating;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct Armed;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct Failsafe;
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct ErrorPresent;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StateEnum {
    INIT(State<Init>),
    PREFLIGHT(State<Preflight>),
    CALIBRATING(State<Calibrating>),
    ARMED(State<Armed>),
    FAILSAFE(State<Failsafe>),
    ERROR_PRESENT(State<ErrorPresent>),
}

impl Default for StateEnum {
    fn default() -> Self {
        State::<Init>::new()
    }
}

/*
Bitflags for errors
*/
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
        const INVALID_STATE_MACHINE_TRANSITION = 1 << 7;
    }
}

/*
State Machine Events.

Instead of having specific error events, it may be more idiomatic to have a
Result<Event, ErrorFlag> enum that we match on.

 */
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
    // ERROR,
    // NO_ERROR,
}

/*
State struct implementations

Each struct should have methods that are unique to that state.

*/

impl State<Init> {
    fn new() -> StateEnum {
        /*
        NOTE: This is the only way a state can be created directly.

        All other states must be created from previous states. They
        do not have direct constructors.
         */
        StateEnum::INIT(State { _state: PhantomData })
    }

    fn initialize(self) -> StateEnum {
        StateEnum::PREFLIGHT(State { _state: PhantomData })
    }
}

impl State<Preflight> {
    fn request_arm(self) -> StateEnum {
        StateEnum::ARMED(State { _state: PhantomData })
    }

    fn request_arm_and_calibrate(self) -> StateEnum {
        StateEnum::CALIBRATING(State { _state: PhantomData })
    }
}

impl State<Calibrating> {
    fn calibration_complete(self) -> StateEnum {
        StateEnum::ARMED(State { _state: PhantomData })
    }

    fn calibration_failed(self) -> StateEnum {
        StateEnum::PREFLIGHT(State { _state: PhantomData })
    }
}

impl State<Armed> {
    fn request_disarm(self) -> StateEnum {
        StateEnum::PREFLIGHT(State { _state: PhantomData })
    }

    fn request_disarm_and_error(self) -> StateEnum {
        StateEnum::ERROR_PRESENT(State { _state: PhantomData })
    }

    fn rc_lost(self) -> StateEnum {
        StateEnum::FAILSAFE(State { _state: PhantomData })
    }
}

impl State<Failsafe> {
    fn rc_found(self) -> StateEnum {
        StateEnum::ARMED(State { _state: PhantomData })
    }

    fn request_disarm(self) -> StateEnum {
        StateEnum::ERROR_PRESENT(State { _state: PhantomData })
    }
}

impl State<ErrorPresent> {
    fn clear_errors(self) -> StateEnum {
        StateEnum::PREFLIGHT(State { _state: PhantomData })
    }
}

impl<State_Value> State<State_Value> {
    fn error(self) -> StateEnum {
        // TODO: Pass in an error bitflag?
        StateEnum::ERROR_PRESENT(State { _state: PhantomData })
    }
}

#[derive(Debug, Default)]
pub(crate) struct StateMachine {
    state: StateEnum,
    error_flags: ErrorFlag,
}

impl StateMachine {
    pub fn get_state(&self) -> &StateEnum {
        &self.state
    }

    pub fn get_errors(&self) -> ErrorFlag {
        self.error_flags
    }

    pub fn set_error(&mut self, error_flag: ErrorFlag) {
        self.error_flags |= error_flag;
    }

    pub fn unset_error(&mut self, error_flag: ErrorFlag) {
        // TODO: Check this
        self.error_flags &= !error_flag;
    }

    pub fn update(&mut self, event: Result<Event, ErrorFlag>) {
        // TODO: Have this update function return a Result to indicate success or failure?

        /*

        TODO: Change the results of the match statement so that we construct the next state from the
        previous one using the typestate pattern.

        For example, going from init -> preflight would be State<Init>::to_preflight() -> State<Preflight>
         */
        self.state = match event {
            Ok(_event) => match (self.state, _event) {
                (StateEnum::INIT(state), Event::INITIALIZED) => state.initialize(),
                (StateEnum::PREFLIGHT(state), Event::REQUEST_ARM) => state.request_arm(),
                (StateEnum::PREFLIGHT(state), Event::REQUEST_ARM_AND_CALIBRATE) => state.request_arm_and_calibrate(),
                (StateEnum::CALIBRATING(state), Event::CALIBRATION_COMPLETE) => state.calibration_complete(),
                (StateEnum::CALIBRATING(state), Event::CALIBRATION_FAILED) => state.calibration_failed(),
                (StateEnum::ARMED(state), Event::REQUEST_DISARM) => state.request_disarm(),
                (StateEnum::ARMED(state), Event::REQUEST_DISARM_AND_ERROR) => state.request_disarm_and_error(),
                (StateEnum::ARMED(state), Event::RC_LOST) => state.rc_lost(),
                (StateEnum::FAILSAFE(state), Event::RC_FOUND) => state.rc_found(),
                (StateEnum::FAILSAFE(state), Event::REQUEST_DISARM) => state.request_disarm(),
                (state, _) => state,
            },
            Err(_error) => {
                self.set_error(_error);
                match (self.state) {
                    StateEnum::INIT(state) => state.error(),
                    StateEnum::PREFLIGHT(state) => state.error(),
                    StateEnum::CALIBRATING(state) => state.error(),
                    StateEnum::ARMED(state) => state.error(),
                    StateEnum::FAILSAFE(state) => state.error(),
                    StateEnum::ERROR_PRESENT(state) => state.error(),
                }
            }
        };
    }
}
