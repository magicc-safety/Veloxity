#![allow(non_camel_case_types)]
#[cfg(test)]
mod tests;

use crate::params::{ParamId, ParamValue, Params};
use bitflags::bitflags;
use core::mem::take;

// Events that trigger state transitions
#[derive(Debug, Clone, Copy)]
pub enum Event {
    INITIALIZED,
    REQUEST_ARM,
    REQUEST_DISARM,
    CALIBRATION_COMPLETE,
    CALIBRATION_FAILED,
    HARDFAULT_REARM_REQUESTED,
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
pub(crate) struct State<S> {
    state: S,
    error_flags: ErrorFlag,
}

// Holds FSM as its type changes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum StateMachine {
    Init(State<Init>),
    Preflight(State<Preflight>),
    Calibrating(State<Calibrating>),
    Armed(State<Armed>),
    Failsafe(State<Failsafe>),
    ErrorPresent(State<ErrorPresent>),
    ErrorFailsafe(State<ErrorFailsafe>),
}

impl StateMachine {
    // New state machine starts in the Preflight state.
    pub fn new() -> Self {
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
            StateMachine::ErrorFailsafe(sm) => &mut sm.error_flags,
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
            StateMachine::ErrorFailsafe(sm) => sm.state.on_event(sm, event, params),
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
            StateMachine::ErrorFailsafe(sm) => sm.error_flags,
        }
    }

    pub fn is_armed(&self) -> bool {
        matches!(self, StateMachine::Armed(_) | StateMachine::Failsafe(_))
    }

    pub fn is_in_failsafe(&self) -> bool {
        matches!(
            self,
            StateMachine::Failsafe(_) | StateMachine::ErrorFailsafe(_)
        )
    }

    pub fn is_in_error_state(&self) -> bool {
        matches!(
            self,
            StateMachine::ErrorPresent(_) | StateMachine::ErrorFailsafe(_)
        )
    }
}

impl Default for StateMachine {
    fn default() -> Self {
        Self::new()
    }
}

// State structs
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
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ErrorFailsafe;

// State transition logic
impl Init {
    fn on_event(self, sm: State<Self>, event: Event) -> StateMachine {
        match event {
            Event::INITIALIZED => StateMachine::Preflight(State {
                state: Preflight,
                error_flags: sm.error_flags,
            }),
            Event::HARDFAULT_REARM_REQUESTED => StateMachine::Init(sm),
            _ => StateMachine::Init(sm),
        }
    }
}

impl Preflight {
    fn on_event(self, sm: State<Self>, event: Event, params: &Params) -> StateMachine {
        match event {
            Event::REQUEST_ARM => {
                if matches!(
                    params.get_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM),
                    ParamValue::Int(value) if value != 0
                ) {
                    let mut error_flags = sm.error_flags;
                    error_flags.remove(ErrorFlag::UNCALIBRATED_IMU);
                    StateMachine::Calibrating(State {
                        state: Calibrating,
                        error_flags: error_flags,
                    })
                } else {
                    StateMachine::Armed(State {
                        state: Armed,
                        error_flags: sm.error_flags,
                    })
                }
            }
            Event::HARDFAULT_REARM_REQUESTED => StateMachine::Armed(State {
                state: Armed,
                error_flags: sm.error_flags,
            }),
            Event::ERROR_OCCURRED(_) => StateMachine::ErrorPresent(State {
                state: ErrorPresent,
                error_flags: sm.error_flags,
            }),
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
                StateMachine::Armed(State {
                    state: Armed,
                    error_flags: error_flags,
                })
            }
            Event::CALIBRATION_FAILED => StateMachine::Preflight(State {
                state: Preflight,
                error_flags: sm.error_flags,
            }),
            Event::ERROR_OCCURRED(_) => StateMachine::ErrorPresent(State {
                state: ErrorPresent,
                error_flags: sm.error_flags,
            }),
            Event::HARDFAULT_REARM_REQUESTED => StateMachine::Armed(State {
                state: Armed,
                error_flags: sm.error_flags,
            }),
            _ => StateMachine::Calibrating(sm),
        }
    }
}

impl Armed {
    fn on_event(self, sm: State<Self>, event: Event, _params: &Params) -> StateMachine {
        match event {
            Event::REQUEST_DISARM => {
                if !sm.error_flags.is_empty() {
                    StateMachine::ErrorPresent(State {
                        state: ErrorPresent,
                        error_flags: sm.error_flags,
                    })
                } else {
                    StateMachine::Preflight(State {
                        state: Preflight,
                        error_flags: sm.error_flags,
                    })
                }
            }
            Event::ERROR_OCCURRED(ErrorFlag::RC_LOST) => StateMachine::Failsafe(State {
                state: Failsafe,
                error_flags: sm.error_flags,
            }),
            Event::HARDFAULT_REARM_REQUESTED => StateMachine::Armed(sm),
            _ => StateMachine::Armed(sm),
        }
    }
}

impl Failsafe {
    fn on_event(self, sm: State<Self>, event: Event, _params: &Params) -> StateMachine {
        match event {
            Event::ERROR_CLEARED(ErrorFlag::RC_LOST) => StateMachine::Armed(State {
                state: Armed,
                error_flags: sm.error_flags,
            }),
            Event::REQUEST_DISARM => StateMachine::ErrorFailsafe(State {
                state: ErrorFailsafe,
                error_flags: sm.error_flags,
            }),
            Event::HARDFAULT_REARM_REQUESTED => StateMachine::Armed(State {
                state: Armed,
                error_flags: sm.error_flags,
            }),
            _ => StateMachine::Failsafe(sm),
        }
    }
}

impl ErrorFailsafe {
    fn on_event(self, sm: State<Self>, event: Event, _params: &Params) -> StateMachine {
        match event {
            Event::REQUEST_ARM => {
                log_arming_errors(sm.error_flags);
                StateMachine::ErrorFailsafe(sm)
            }
            Event::HARDFAULT_REARM_REQUESTED => StateMachine::Armed(State {
                state: Armed,
                error_flags: sm.error_flags,
            }),
            Event::ERROR_CLEARED(ErrorFlag::RC_LOST) => {
                if sm.error_flags.is_empty() {
                    StateMachine::Preflight(State {
                        state: Preflight,
                        error_flags: sm.error_flags,
                    })
                } else {
                    StateMachine::ErrorPresent(State {
                        state: ErrorPresent,
                        error_flags: sm.error_flags,
                    })
                }
            }
            Event::ERROR_CLEARED(_) => {
                if sm.error_flags.is_empty() {
                    StateMachine::Preflight(State {
                        state: Preflight,
                        error_flags: sm.error_flags,
                    })
                } else {
                    StateMachine::ErrorFailsafe(sm)
                }
            }
            _ => StateMachine::ErrorFailsafe(sm),
        }
    }
}

impl ErrorPresent {
    fn on_event(self, sm: State<Self>, event: Event, _params: &Params) -> StateMachine {
        match event {
            Event::REQUEST_ARM => {
                log_arming_errors(sm.error_flags);
                StateMachine::ErrorPresent(sm)
            }
            Event::HARDFAULT_REARM_REQUESTED => StateMachine::Armed(State {
                state: Armed,
                error_flags: sm.error_flags,
            }),
            Event::ERROR_CLEARED(_) => {
                if sm.error_flags.is_empty() {
                    StateMachine::Preflight(State {
                        state: Preflight,
                        error_flags: sm.error_flags,
                    })
                } else {
                    StateMachine::ErrorPresent(sm)
                }
            }
            _ => StateMachine::ErrorPresent(sm),
        }
    }
}

fn log_arming_errors(error_flags: ErrorFlag) {
    if error_flags.contains(ErrorFlag::INVALID_MIXER) {
        crate::log_error!("Unable to arm: Invalid mixer");
    }
    if error_flags.contains(ErrorFlag::IMU_NOT_RESPONDING) {
        crate::log_error!("Unable to arm: IMU not responding");
    }
    if error_flags.contains(ErrorFlag::RC_LOST) {
        crate::log_error!("Unable to arm: RC signal lost");
    }
    if error_flags.contains(ErrorFlag::UNHEALTHY_ESTIMATOR) {
        crate::log_error!("Unable to arm: Unhealthy estimator");
    }
    if error_flags.contains(ErrorFlag::TIME_GOING_BACKWARDS) {
        crate::log_error!("Unable to arm: Time going backwards");
    }
    if error_flags.contains(ErrorFlag::UNCALIBRATED_IMU) {
        crate::log_error!("Unable to arm: IMU not calibrated");
    }
    if error_flags.contains(ErrorFlag::INVALID_FAILSAFE) {
        crate::log_error!("Unable to arm: Invalid failsafe setting");
    }
}

// Struct for state management
pub struct StateManager {
    machine: StateMachine,
    arming_safety: ArmingSafety,
}

#[derive(Debug, Clone, Copy, Default)]
struct ArmingSafety {
    rc_throttle_low: bool,
    rc_throttle_override_switch_on: bool,
}

impl StateManager {
    pub fn new() -> Self {
        StateManager {
            machine: StateMachine::new(),
            arming_safety: ArmingSafety::default(),
        }
    }

    pub fn is_calibrating(&self) -> bool {
        matches!(self.machine, StateMachine::Calibrating(_))
    }

    pub fn update_arming_safety(
        &mut self,
        rc_throttle_low: bool,
        rc_throttle_override_switch_on: bool,
    ) {
        self.arming_safety = ArmingSafety {
            rc_throttle_low,
            rc_throttle_override_switch_on,
        };
    }

    fn arming_safety_allows_arm(&self, params: &Params) -> bool {
        if !self.arming_safety.rc_throttle_low {
            crate::log_error!("Cannot arm with RC throttle high");
            return false;
        }

        let take_min_throttle = matches!(
            params.get_by_id(ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE),
            ParamValue::Int(value) if value != 0
        );

        if !take_min_throttle && !self.arming_safety.rc_throttle_override_switch_on {
            crate::log_error!("RC throttle override must be active to arm");
            return false;
        }

        true
    }

    // The main update loop. Takes an event and applies it to the internal state machine.
    pub fn update(&mut self, event: Event, params: &Params) {
        if matches!(event, Event::REQUEST_ARM)
            && matches!(self.machine, StateMachine::Preflight(_))
            && !self.arming_safety_allows_arm(params)
        {
            return;
        }

        self.machine.update(event, params);
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
        // LED hardware output is synchronized by the world board stage.
    }
}
