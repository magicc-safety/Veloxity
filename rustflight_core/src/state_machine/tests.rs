// /**
// ******************************************************************************
// * File     : tests.rs
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
/*
Unit tests
*/
use crate::state_machine::{ErrorFlag, ErrorPresent, Event, Init, State, StateMachine};
use core::marker::PhantomData;
use core::result::Result;

#[test]
fn test_bitflags_default_value() {
    // Tests that the default value is zero.
    assert_eq!(ErrorFlag::default().bits(), 0u16);
    // You can also access the 0 value by calling .empty()
    assert_eq!(ErrorFlag::empty(), ErrorFlag::default())
}

#[test]
fn test_bitflags() {
    assert_eq!(ErrorFlag::INVALID_FAILSAFE.bits(), 64u16)
}

#[test]
#[allow(non_snake_case)]
fn test_bitflags_OR() {
    assert_eq!(
        (ErrorFlag::RC_LOST | ErrorFlag::TIME_GOING_BACKWARDS).bits(),
        4u16 + 16u16
    );
}

#[test]
fn test_sm_no_clone() {
    let sm = StateMachine::default();

    // Implicit copies are made
    let temp_state = sm.get_state();
    let temp_state_2 = sm.get_state();

    assert_eq!(
        *sm.get_state(),
        State::Init(Init)
    );
    assert_eq!(temp_state, temp_state_2)
}

#[test]
fn test_state_transitions_init_error() {
    /*
     Note on the assert statements:

         if let StateEnum::PREFLIGHT(state) = *sm.get_state() {} else { assert!(false); }

    This tests if `*sm.get_state()` is equal to the PREFLIGHT state. If so, the empty {} block simply
    causes the test to continue. If not, there is an error and we return `assert!(false)` to fail
    the test.
     */
    let mut sm = StateMachine::default();
    if let State::Init(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::IMU_NOT_RESPONDING)); // Now in ERROR_PRESENT

    // Check that the IMU_NOT_RESPONDING error got recorded in sm.error_flags
    assert_eq!(sm.error_flags.bits(), 2u16)
}

#[test]
fn test_state_transitions_init_preflight_error() {
    let mut sm = StateMachine::default();
    if let State::Init(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::INITIALIZED); // Now in PREFLIGHT
    if let State::Preflight(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::UNHEALTHY_ESTIMATOR)); // Now in ERROR_PRESENT
    if let State::ErrorPresent(_) = *sm.get_state() {
    } else {
        assert!(false);
    }

    // Check that the UNHEALTHY_ESTIMATOR error got recorded in sm.error_flags
    assert_eq!(sm.error_flags.bits(), 8u16)
}

#[test]
fn test_state_transitions_init_preflight_calibrating_error() {
    let mut sm = StateMachine::default();
    if let State::Init(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::INITIALIZED); // Now in PREFLIGHT
    if let State::Preflight(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::REQUEST_ARM_AND_CALIBRATE); // Now in CALIBRATING
    if let State::Calibrating(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_MIXER)); // Now in ERROR_PRESENT
    if let State::ErrorPresent(_) = *sm.get_state() {
    } else {
        assert!(false);
    }

    // Check that the INVALID_MIXER error got recorded in sm.error_flags
    assert_eq!(sm.error_flags.bits(), 1u16)
}

#[test]
fn test_state_transitions_init_preflight_armed_failsafe_armed_error() {
    let mut sm = StateMachine::default();
    if let State::Init(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::INITIALIZED); // Now in PREFLIGHT
    if let State::Preflight(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::REQUEST_ARM); // Now in ARMED
    if let State::Armed(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::RC_LOST); // Now in FAILSAFE
    if let State::Failsafe(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::RC_FOUND); // Now in ARMED
    if let State::Armed(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::REQUEST_DISARM_AND_ERROR); // Now in ERROR_PRESENT
    if let State::ErrorPresent(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
}

#[test]
fn test_state_transitions_init_preflight_calibrating_armed_failsafe_error() {
    let mut sm = StateMachine::default();
    if let State::Init(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::INITIALIZED); // Now in PREFLIGHT
    if let State::Preflight(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::REQUEST_ARM_AND_CALIBRATE); // Now in CALIBRATING
    if let State::Calibrating(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::CALIBRATION_COMPLETE); // Now in ARMED
    if let State::Armed(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::RC_LOST); // Now in FAILSAFE
    if let State::Failsafe(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
    sm.update(Event::REQUEST_DISARM); // Now in ERROR_PRESENT
    if let State::ErrorPresent(_) = *sm.get_state() {
    } else {
        assert!(false);
    }
}

/// Assert the state machine is in the given StateEnum variant.
/// Usage: assert_state!(sm, StateEnum::INIT);
macro_rules! assert_state {
    ($sm:expr, $variant:path) => {
        match *$sm.get_state() {
            $variant(_) => {}
            other => panic!(
                "expected state `{}`, got `{:?}`",
                stringify!($variant),
                other
            ),
        }
    };
}

macro_rules! assert_not_state {
    ($sm:expr, $variant:path) => {
        match *$sm.get_state() {
            $variant(_) => panic!("expected NOT state `{}`, but it was", stringify!($variant)),
            _ => {}
        }
    };
}

#[test]
fn test_init() {
    let mut sm = StateMachine::new();
    // Should be in PREFLIGHT MODE
    assert_not_state!(sm, State::Init);
    assert_state!(sm, State::Preflight);
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    assert!(sm.get_errors() == ErrorFlag::empty());
}

#[test]
fn test_set_and_clear_all_errors() {
    let mut sm = StateMachine::default();
    const ALL_ERRORS: [ErrorFlag; 7] = [
        ErrorFlag::INVALID_MIXER,
        ErrorFlag::IMU_NOT_RESPONDING,
        ErrorFlag::RC_LOST,
        ErrorFlag::UNHEALTHY_ESTIMATOR,
        ErrorFlag::TIME_GOING_BACKWARDS,
        ErrorFlag::UNCALIBRATED_IMU,
        ErrorFlag::INVALID_FAILSAFE,
    ];
    for error in ALL_ERRORS {
        // set error
        sm.update(Event::ERROR_OCCURRED(error));
        assert!(sm.is_armed() == false);
        assert!(sm.is_in_failsafe() == false);
        assert!(sm.get_errors() == error);

        // clear error
        sm.update(Event::ERROR_CLEARED(error));
        assert!(sm.is_armed() == false);
        assert!(sm.is_in_failsafe() == false);
        assert!(sm.get_errors() == ErrorFlag::empty());
    };
}

#[test]
fn test_set_and_clear_multiple_errors() {
    let mut sm = StateMachine::default();
    let error =  ErrorFlag::IMU_NOT_RESPONDING | 
                            ErrorFlag::UNHEALTHY_ESTIMATOR | 
                            ErrorFlag::TIME_GOING_BACKWARDS;
    // set multiple errors                        
    sm.update(Event::ERROR_OCCURRED(error));
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    assert!(sm.get_errors() == error);

    // clear all errors
    sm.update(Event::ERROR_CLEARED(error));
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    assert!(sm.get_errors() == ErrorFlag::empty());
}

#[test]
fn test_add_error_after_previous_error() {
    let mut sm = StateMachine::default();
    let error =  ErrorFlag::IMU_NOT_RESPONDING | 
                            ErrorFlag::UNHEALTHY_ESTIMATOR | 
                            ErrorFlag::TIME_GOING_BACKWARDS;
    // set multiple errors                        
    sm.update(Event::ERROR_OCCURRED(error));
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_MIXER));
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    let combined_error = error | ErrorFlag::INVALID_MIXER;
    assert!(sm.error_flags == combined_error);
}

#[test]
fn test_clear_one_of_many_errors() {
    let mut sm = StateMachine::default();
    let error =  ErrorFlag::IMU_NOT_RESPONDING | 
                            ErrorFlag::UNHEALTHY_ESTIMATOR | 
                            ErrorFlag::TIME_GOING_BACKWARDS;                     
    sm.update(Event::ERROR_OCCURRED(error));
    sm.update(Event::ERROR_CLEARED(ErrorFlag::TIME_GOING_BACKWARDS));
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    let remaining_errors = ErrorFlag::IMU_NOT_RESPONDING | ErrorFlag::UNHEALTHY_ESTIMATOR;
    assert!(sm.error_flags == remaining_errors);
}

#[test]
fn test_do_not_arm_if_error() {
    let mut sm = StateMachine::new();
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_MIXER));
    sm.update(Event::REQUEST_ARM);
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    assert!(sm.get_errors() == ErrorFlag::INVALID_MIXER);
    assert_not_state!(sm, State::Armed);
}

#[test]
fn test_arm_if_no_error() {
    let mut sm = StateMachine::new();
    assert!(sm.get_errors() == ErrorFlag::empty());
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    sm.update(Event::REQUEST_ARM);
    assert!(sm.is_armed() == true);
    assert!(sm.is_in_failsafe() == false);
    assert_state!(sm, State::Armed);
}

#[test]
fn test_clear_multiple_errors_at_once() {
    let mut sm = StateMachine::new();
    let errors = ErrorFlag::IMU_NOT_RESPONDING | ErrorFlag::TIME_GOING_BACKWARDS | ErrorFlag::UNCALIBRATED_IMU;
    sm.update(Event::ERROR_OCCURRED(errors));

    let to_clear = ErrorFlag::IMU_NOT_RESPONDING | ErrorFlag::TIME_GOING_BACKWARDS;
    sm.update(Event::ERROR_CLEARED(to_clear));

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::UNCALIBRATED_IMU);
}

#[test]
fn test_clear_all_errors() {
    let mut sm = StateMachine::new();
    let errors = ErrorFlag::IMU_NOT_RESPONDING | ErrorFlag::TIME_GOING_BACKWARDS | ErrorFlag::UNCALIBRATED_IMU;
    sm.update(Event::ERROR_OCCURRED(errors));
    sm.update(Event::ERROR_CLEARED(errors));

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
}

#[test]
fn test_arm_and_disarm() {
    let mut sm = StateMachine::new();
    assert!(sm.get_errors() == ErrorFlag::empty());
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    sm.update(Event::REQUEST_ARM);
    assert!(sm.is_armed() == true);
    assert!(sm.is_in_failsafe() == false);
    assert_state!(sm, State::Armed);
    sm.update(Event::REQUEST_DISARM);
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    assert_state!(sm, State::Preflight);
}

#[test]
fn test_wait_for_calibration_to_arm() {
    let mut sm = StateMachine::new();
    // set PARAM_CALIBRATE_GYRO_ON_ARM
    // Requesting arm should move to a calibrating state, not armed
    sm.update(Event::REQUEST_ARM);
    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, State::Calibrating); // Assumed state

    // Once calibration completes, the state machine should arm automatically
    sm.update(Event::CALIBRATION_COMPLETE);
    assert!(sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, State::Armed);
}

#[test]
fn test_calibration_failed_dont_arm() {
    let mut sm = StateMachine::new();
    // set PARAM_CALIBRATE_GYRO_ON_ARM

    sm.update(Event::REQUEST_ARM);
    sm.update(Event::CALIBRATION_FAILED);

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, State::Preflight);
}

#[test]
fn test_error_during_calibration_dont_arm() {
    let mut sm = StateMachine::new();
    // set PARAM_CALIBRATE_GYRO_ON_ARM

    sm.update(Event::REQUEST_ARM);
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_MIXER));

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::INVALID_MIXER);
    assert_state!(sm, State::ErrorPresent);
}

#[test]
fn test_rc_lost_during_calibration_dont_arm() {
    let mut sm = StateMachine::new();
    // set PARAM_CALIBRATE_GYRO_ON_ARM

    sm.update(Event::REQUEST_ARM);
    sm.update(Event::RC_LOST);

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::RC_LOST);
    assert_state!(sm, State::ErrorPresent);
}

#[test]
fn test_clear_error_stay_disarmed() {
    let mut sm = StateMachine::new();
    // set PARAM_CALIBRATE_GYRO_ON_ARM

    sm.update(Event::REQUEST_ARM);
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_MIXER));
    assert!(!sm.is_armed()); // Should be in an error state

    // Calibration finishing and clearing the error should not lead to arming
    sm.update(Event::CALIBRATION_COMPLETE);
    sm.update(Event::ERROR_CLEARED(ErrorFlag::INVALID_MIXER));

    assert!(!sm.is_armed());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, State::Preflight);
}

#[test]
fn test_recover_rc_stay_disarmed() {
    let mut sm = StateMachine::new();
    // set PARAM_CALIBRATE_GYRO_ON_ARM

    sm.update(Event::REQUEST_ARM);
    sm.update(Event::RC_LOST);
    assert!(!sm.is_armed()); // Should be in an error state

    // Calibration finishing and finding RC should not lead to arming
    sm.update(Event::CALIBRATION_COMPLETE);
    sm.update(Event::RC_FOUND);

    assert!(!sm.is_armed());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, State::Preflight);
}

#[test]
fn test_set_errors_while_armed() {
    let mut sm = StateMachine::new();
    sm.update(Event::REQUEST_ARM);
    assert!(sm.is_armed());

    sm.update(Event::ERROR_OCCURRED(ErrorFlag::TIME_GOING_BACKWARDS));

    assert!(sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::TIME_GOING_BACKWARDS);
}

#[test]
fn test_errors_persist_when_disarmed() {
    let mut sm = StateMachine::new();
    sm.update(Event::REQUEST_ARM);
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::TIME_GOING_BACKWARDS));
    sm.update(Event::REQUEST_DISARM);

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::TIME_GOING_BACKWARDS);
}

#[test]
fn test_unable_to_arm_with_persistent_errors() {
    let mut sm = StateMachine::new();
    sm.update(Event::REQUEST_ARM);
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::TIME_GOING_BACKWARDS));
    sm.update(Event::REQUEST_DISARM);

    // Attempt to arm again with the error still present
    sm.update(Event::REQUEST_ARM);

    assert!(!sm.is_armed());
    assert_eq!(sm.get_errors(), ErrorFlag::TIME_GOING_BACKWARDS);
}

// ignored throttle tests, should be implemented in RC logic

#[test]
fn test_lost_rc_when_disarmed_no_failsafe() {
    let mut sm = StateMachine::new();
    sm.update(Event::RC_LOST);

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::RC_LOST);
}

#[test]
fn test_unable_to_arm_without_rc() {
    let mut sm = StateMachine::new();
    sm.update(Event::RC_LOST);
    sm.update(Event::REQUEST_ARM);
    assert!(!sm.is_armed());
}

#[test]
fn test_able_to_arm_after_rc_recovery() {
    let mut sm = StateMachine::new();
    sm.update(Event::RC_LOST);
    sm.update(Event::REQUEST_ARM);
    assert!(!sm.is_armed()); // Should fail

    sm.update(Event::RC_FOUND);
    assert_eq!(sm.get_errors(), ErrorFlag::empty());

    sm.update(Event::REQUEST_ARM);
    assert!(sm.is_armed()); // Should succeed
}

#[test]
fn test_rc_lost_while_armed_enter_failsafe() {
    let mut sm = StateMachine::new();
    sm.update(Event::REQUEST_ARM);
    sm.update(Event::RC_LOST);

    assert!(sm.is_armed());
    assert!(sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::RC_LOST);
    assert_state!(sm, State::Failsafe);
}

#[test]
fn test_disarm_while_in_failsafe() {
    let mut sm = StateMachine::new();
    sm.update(Event::REQUEST_ARM);
    sm.update(Event::RC_LOST); // Enters failsafe
    sm.update(Event::REQUEST_DISARM);

    assert!(!sm.is_armed());
    assert!(sm.is_in_failsafe()); // Remains in a failsafe state
    assert_eq!(sm.get_errors(), ErrorFlag::RC_LOST);
}

#[test]
fn test_regain_rc_after_failsafe() {
    let mut sm = StateMachine::new();
    sm.update(Event::REQUEST_ARM);
    sm.update(Event::RC_LOST); // Enters failsafe
    assert!(sm.is_in_failsafe());

    sm.update(Event::RC_FOUND); // Exits failsafe

    assert!(sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, State::Armed);
}

#[test]
fn test_normal_boot_initial_state() {
    let sm = StateMachine::new();
    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, State::Preflight);
}

// Crash recovery tests skipped