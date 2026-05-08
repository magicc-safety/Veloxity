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
use crate::params::{ParamId, ParamValue, Params};
use crate::state_machine::{ErrorFlag, Event, StateMachine, StateManager};

fn setup_sm() -> (StateMachine, Params) {
    let params = Params::new();
    let mut sm = StateMachine::new();
    sm.update(Event::INITIALIZED, &params);
    (sm, params)
}

fn setup_state_manager() -> (StateManager, Params) {
    let params = Params::new();
    let mut sm = StateManager::new();
    sm.update(Event::INITIALIZED, &params);
    (sm, params)
}

// --------------------- Error Tests ---------------------

#[test]
fn test_bitflags_default_value() {
    assert_eq!(ErrorFlag::default().bits(), 0u16);
    assert_eq!(ErrorFlag::empty(), ErrorFlag::default())
}

#[test]
fn test_bitflags() {
    assert_eq!(ErrorFlag::INVALID_FAILSAFE.bits(), 128u16)
}

#[test]
#[allow(non_snake_case)]
fn test_bitflags_OR() {
    assert_eq!(
        (ErrorFlag::RC_LOST | ErrorFlag::TIME_GOING_BACKWARDS).bits(),
        4u16 + 16u16
    );
}

// Asserts the state machine is in the given State variant.
// Usage: `assert_state!(sm, StateMachine::Preflight);`
macro_rules! assert_state {
    ($sm:expr, $variant:path) => {
        match $sm {
            $variant(_) => {}
            other => panic!(
                "expected state `{}`, got `{:?}`",
                stringify!($variant),
                other
            ),
        }
    };
}

#[test]
fn test_set_and_clear_all_errors() {
    let (mut sm, mut params) = setup_sm();
    // manually put it in an "init" state for this test
    let mut sm = StateMachine::Init(crate::state_machine::State {
        state: crate::state_machine::Init,
        error_flags: ErrorFlag::empty(),
    });

    const ALL_ERRORS: [ErrorFlag; 8] = [
        ErrorFlag::INVALID_MIXER,
        ErrorFlag::IMU_NOT_RESPONDING,
        ErrorFlag::RC_LOST,
        ErrorFlag::UNHEALTHY_ESTIMATOR,
        ErrorFlag::TIME_GOING_BACKWARDS,
        ErrorFlag::UNCALIBRATED_IMU,
        ErrorFlag::BUFFER_OVERRUN,
        ErrorFlag::INVALID_FAILSAFE,
    ];
    for error in ALL_ERRORS {
        sm.update(Event::ERROR_OCCURRED(error), &params);
        assert!(sm.is_armed() == false);
        assert!(sm.is_in_failsafe() == false);
        assert!(sm.get_errors() == error);

        sm.update(Event::ERROR_CLEARED(error), &params);
        assert!(sm.is_armed() == false);
        assert!(sm.is_in_failsafe() == false);
        assert!(sm.get_errors() == ErrorFlag::empty());
    }
}

#[test]
fn test_set_and_clear_multiple_errors() {
    let (mut sm, params) = setup_sm();
    let error = ErrorFlag::IMU_NOT_RESPONDING
        | ErrorFlag::UNHEALTHY_ESTIMATOR
        | ErrorFlag::TIME_GOING_BACKWARDS;
    sm.update(Event::ERROR_OCCURRED(error), &params);
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    assert!(sm.get_errors() == error);
    assert_state!(sm, StateMachine::ErrorPresent);

    sm.update(Event::ERROR_CLEARED(error), &params);
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    assert!(sm.get_errors() == ErrorFlag::empty());
    assert_state!(sm, StateMachine::Preflight);
}

#[test]
fn test_add_error_after_previous_error() {
    let (mut sm, params) = setup_sm();
    let error = ErrorFlag::IMU_NOT_RESPONDING
        | ErrorFlag::UNHEALTHY_ESTIMATOR
        | ErrorFlag::TIME_GOING_BACKWARDS;
    sm.update(Event::ERROR_OCCURRED(error), &params);
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_MIXER), &params);
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    let combined_error = error | ErrorFlag::INVALID_MIXER;
    assert!(sm.get_errors() == combined_error);
    assert_state!(sm, StateMachine::ErrorPresent);
}

#[test]
fn test_clear_one_of_many_errors() {
    let (mut sm, params) = setup_sm();
    let error = ErrorFlag::IMU_NOT_RESPONDING
        | ErrorFlag::UNHEALTHY_ESTIMATOR
        | ErrorFlag::TIME_GOING_BACKWARDS;
    sm.update(Event::ERROR_OCCURRED(error), &params);
    sm.update(
        Event::ERROR_CLEARED(ErrorFlag::TIME_GOING_BACKWARDS),
        &params,
    );
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    let remaining_errors = ErrorFlag::IMU_NOT_RESPONDING | ErrorFlag::UNHEALTHY_ESTIMATOR;
    assert!(sm.get_errors() == remaining_errors);
    assert_state!(sm, StateMachine::ErrorPresent);
}

#[test]
fn test_do_not_arm_if_error() {
    let (mut sm, params) = setup_sm();
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_MIXER), &params);
    sm.update(Event::REQUEST_ARM, &params);
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    assert!(sm.get_errors() == ErrorFlag::INVALID_MIXER);
    assert_state!(sm, StateMachine::ErrorPresent);
}

#[test]
fn test_arm_if_no_error() {
    let (mut sm, params) = setup_sm();
    assert!(sm.get_errors() == ErrorFlag::empty());
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    sm.update(Event::REQUEST_ARM, &params);
    assert!(sm.is_armed() == true);
    assert!(sm.is_in_failsafe() == false);
    assert_state!(sm, StateMachine::Armed);
}

#[test]
fn test_clear_multiple_errors_at_once() {
    let (mut sm, params) = setup_sm();
    let errors = ErrorFlag::IMU_NOT_RESPONDING
        | ErrorFlag::TIME_GOING_BACKWARDS
        | ErrorFlag::UNCALIBRATED_IMU;
    sm.update(Event::ERROR_OCCURRED(errors), &params);
    assert_state!(sm, StateMachine::ErrorPresent);

    let to_clear = ErrorFlag::IMU_NOT_RESPONDING | ErrorFlag::TIME_GOING_BACKWARDS;
    sm.update(Event::ERROR_CLEARED(to_clear), &params);

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::UNCALIBRATED_IMU);
    assert_state!(sm, StateMachine::ErrorPresent);
}

#[test]
fn test_clear_all_errors() {
    let (mut sm, params) = setup_sm();
    let errors = ErrorFlag::IMU_NOT_RESPONDING
        | ErrorFlag::TIME_GOING_BACKWARDS
        | ErrorFlag::UNCALIBRATED_IMU;
    sm.update(Event::ERROR_OCCURRED(errors), &params);
    sm.update(Event::ERROR_CLEARED(errors), &params);

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, StateMachine::Preflight);
}

// --------------------- State Transition Tests ---------------------

#[test]
fn test_init() {
    let (sm, _params) = setup_sm();
    assert_state!(sm, StateMachine::Preflight);
    assert!(sm.is_armed() == false);
    assert!(sm.is_in_failsafe() == false);
    assert!(sm.get_errors() == ErrorFlag::empty());
}

#[test]
fn test_arm_and_disarm() {
    let (mut sm, params) = setup_sm();
    assert!(sm.get_errors() == ErrorFlag::empty());
    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    sm.update(Event::REQUEST_ARM, &params);
    assert!(sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_state!(sm, StateMachine::Armed);
    sm.update(Event::REQUEST_DISARM, &params);
    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_state!(sm, StateMachine::Preflight);
}

#[test]
fn test_wait_for_calibration_to_arm() {
    let (mut sm, mut params) = setup_sm();
    //params.set_calibrate_gyro_on_arm(ParamValue::Bool(true));
    params.set_by_id(
        ParamId::PARAM_CALIBRATE_GYRO_ON_ARM,
        ParamValue::Bool((true)),
    );
    sm.update(Event::REQUEST_ARM, &params);
    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, StateMachine::Calibrating);

    sm.update(Event::CALIBRATION_COMPLETE, &params);
    assert!(sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, StateMachine::Armed);
}

#[test]
fn test_calibration_failed_dont_arm() {
    let (mut sm, mut params) = setup_sm();
    //params.set_calibrate_gyro_on_arm(ParamValue::Bool(true));
    params.set_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM, ParamValue::Bool(true));

    sm.update(Event::REQUEST_ARM, &params);
    sm.update(Event::CALIBRATION_FAILED, &params);

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, StateMachine::Preflight);
}

#[test]
fn test_error_during_calibration_dont_arm() {
    let (mut sm, mut params) = setup_sm();
    // params.set_calibrate_gyro_on_arm(ParamValue::Bool(true));
    params.set_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM, ParamValue::Bool(true));

    sm.update(Event::REQUEST_ARM, &params);
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_MIXER), &params);

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::INVALID_MIXER);
    assert_state!(sm, StateMachine::ErrorPresent);
}

#[test]
fn test_rc_lost_during_calibration_dont_arm() {
    let (mut sm, mut params) = setup_sm();
    // params.set_calibrate_gyro_on_arm(ParamValue::Bool(true));
    params.set_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM, ParamValue::Bool(true));

    sm.update(Event::REQUEST_ARM, &params);
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::RC_LOST), &params);

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::RC_LOST);
    assert_state!(sm, StateMachine::ErrorPresent);
}

#[test]
fn test_clear_error_stay_disarmed() {
    let (mut sm, mut params) = setup_sm();
    // params.set_calibrate_gyro_on_arm(ParamValue::Bool(true));
    params.set_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM, ParamValue::Bool(true));

    sm.update(Event::REQUEST_ARM, &params);
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_MIXER), &params);
    assert!(!sm.is_armed());
    assert_state!(sm, StateMachine::ErrorPresent);

    sm.update(Event::CALIBRATION_COMPLETE, &params);
    sm.update(Event::ERROR_CLEARED(ErrorFlag::INVALID_MIXER), &params);

    assert!(!sm.is_armed());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, StateMachine::Preflight);
}

#[test]
fn test_recover_rc_stay_disarmed() {
    let (mut sm, mut params) = setup_sm();
    // params.set_calibrate_gyro_on_arm(ParamValue::Bool(true));
    params.set_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM, ParamValue::Bool(true));

    sm.update(Event::REQUEST_ARM, &params);
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::RC_LOST), &params);
    assert!(!sm.is_armed());
    assert_state!(sm, StateMachine::ErrorPresent);

    sm.update(Event::CALIBRATION_COMPLETE, &params);
    sm.update(Event::ERROR_CLEARED(ErrorFlag::RC_LOST), &params);

    assert!(!sm.is_armed());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, StateMachine::Preflight);
}

#[test]
fn test_set_errors_while_armed() {
    let (mut sm, params) = setup_sm();
    sm.update(Event::REQUEST_ARM, &params);
    assert!(sm.is_armed());

    sm.update(
        Event::ERROR_OCCURRED(ErrorFlag::TIME_GOING_BACKWARDS),
        &params,
    );

    assert!(sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::TIME_GOING_BACKWARDS);
    assert_state!(sm, StateMachine::Armed); // Errors other than RC_LOST shouldn't change the state from Armed
}

#[test]
fn test_errors_persist_when_disarmed() {
    let (mut sm, params) = setup_sm();
    sm.update(Event::REQUEST_ARM, &params);
    sm.update(
        Event::ERROR_OCCURRED(ErrorFlag::TIME_GOING_BACKWARDS),
        &params,
    );
    sm.update(Event::REQUEST_DISARM, &params);

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::TIME_GOING_BACKWARDS);
    assert_state!(sm, StateMachine::ErrorPresent);
}

#[test]
fn test_unable_to_arm_with_persistent_errors() {
    let (mut sm, params) = setup_sm();
    sm.update(Event::REQUEST_ARM, &params);
    sm.update(
        Event::ERROR_OCCURRED(ErrorFlag::TIME_GOING_BACKWARDS),
        &params,
    );
    sm.update(Event::REQUEST_DISARM, &params);

    sm.update(Event::REQUEST_ARM, &params);
    assert!(!sm.is_armed());
    assert_eq!(sm.get_errors(), ErrorFlag::TIME_GOING_BACKWARDS);
    assert_state!(sm, StateMachine::ErrorPresent);
}

#[test]
fn test_lost_rc_when_disarmed_no_failsafe() {
    let (mut sm, params) = setup_sm();
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::RC_LOST), &params);

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::RC_LOST);
    assert_state!(sm, StateMachine::ErrorPresent);
}

#[test]
fn test_unable_to_arm_without_rc() {
    let (mut sm, params) = setup_sm();
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::RC_LOST), &params);
    sm.update(Event::REQUEST_ARM, &params);
    assert!(!sm.is_armed());
    assert_state!(sm, StateMachine::ErrorPresent);
}

#[test]
fn test_able_to_arm_after_rc_recovery() {
    let (mut sm, params) = setup_sm();
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::RC_LOST), &params);
    sm.update(Event::REQUEST_ARM, &params);
    assert!(!sm.is_armed());
    assert_state!(sm, StateMachine::ErrorPresent);

    sm.update(Event::ERROR_CLEARED(ErrorFlag::RC_LOST), &params);
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, StateMachine::Preflight);

    sm.update(Event::REQUEST_ARM, &params);
    assert!(sm.is_armed());
    assert_state!(sm, StateMachine::Armed);
}

#[test]
fn test_rc_lost_while_armed_enter_failsafe() {
    let (mut sm, params) = setup_sm();
    sm.update(Event::REQUEST_ARM, &params);
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::RC_LOST), &params);

    assert!(sm.is_armed());
    assert!(sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::RC_LOST);
    assert_state!(sm, StateMachine::Failsafe);
}

#[test]
fn test_disarm_while_in_failsafe() {
    let (mut sm, params) = setup_sm();
    sm.update(Event::REQUEST_ARM, &params);
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::RC_LOST), &params);
    sm.update(Event::REQUEST_DISARM, &params);

    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_state!(sm, StateMachine::ErrorPresent);
    assert_eq!(sm.get_errors(), ErrorFlag::RC_LOST);
}

#[test]
fn test_regain_rc_after_failsafe() {
    let (mut sm, params) = setup_sm();
    sm.update(Event::REQUEST_ARM, &params);
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::RC_LOST), &params);
    assert!(sm.is_in_failsafe());

    sm.update(Event::ERROR_CLEARED(ErrorFlag::RC_LOST), &params);
    assert!(sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, StateMachine::Armed);
}

#[test]
fn test_normal_boot_initial_state() {
    let (sm, _params) = setup_sm();
    assert!(!sm.is_armed());
    assert!(!sm.is_in_failsafe());
    assert_eq!(sm.get_errors(), ErrorFlag::empty());
    assert_state!(sm, StateMachine::Preflight);
}

#[test]
fn test_manager_controller_arm_and_disarm() {
    let (mut manager, params) = setup_state_manager();

    assert_eq!(manager.get_errors(), ErrorFlag::empty());
    assert!(!manager.is_armed());
    assert!(!manager.is_in_failsafe());
    assert_state!(manager.machine, StateMachine::Preflight);

    manager.update(Event::REQUEST_ARM, &params);

    assert!(manager.is_armed());
    assert!(!manager.is_in_failsafe());
    assert_state!(manager.machine, StateMachine::Armed);

    manager.update(Event::REQUEST_DISARM, &params);

    assert!(!manager.is_armed());
    assert!(!manager.is_in_failsafe());
    assert_state!(manager.machine, StateMachine::Preflight);
}

// --------------------- Sim Loop Tests ---------------------

#[test]
fn test_step_failsafe_recovery() {
    let (mut state_manager, params) = setup_state_manager();

    let script = [
        (10, Event::REQUEST_ARM),
        (50, Event::ERROR_OCCURRED(ErrorFlag::RC_LOST)), // Failsafe should trigger
        (100, Event::ERROR_CLEARED(ErrorFlag::RC_LOST)), // RC signal recovered
    ];

    let mut script_iter = script.iter().peekable();

    // The main simulation loop
    for tick in 0..150 {
        state_manager.run(&params);

        if let Some((event_tick, event)) = script_iter.peek() {
            if tick >= *event_tick {
                state_manager.update(*event, &params);
                script_iter.next();
            }
        }

        if tick == 11 {
            assert!(state_manager.is_armed());
            assert!(!state_manager.is_in_failsafe());
        }
        if tick == 51 {
            assert!(state_manager.is_in_failsafe());
        }
        if tick == 101 {
            assert!(state_manager.is_armed());
            assert!(!state_manager.is_in_failsafe());
        }
    }
}

#[test]
fn test_step_errors_during_failsafe_persist() {
    let (mut state_manager, params) = setup_state_manager();

    let script = [
        (10, Event::REQUEST_ARM),
        (25, Event::ERROR_OCCURRED(ErrorFlag::RC_LOST)), // Failsafe should trigger
        (50, Event::ERROR_OCCURRED(ErrorFlag::UNHEALTHY_ESTIMATOR)), // another error triggers
        (75, Event::ERROR_CLEARED(ErrorFlag::RC_LOST)),  // RC signal recovered
        (100, Event::REQUEST_DISARM),                    // Should transition to error state
    ];

    let mut script_iter = script.iter().peekable();

    // The main simulation loop
    for tick in 0..150 {
        state_manager.run(&params);

        if let Some((event_tick, event)) = script_iter.peek() {
            if tick >= *event_tick {
                state_manager.update(*event, &params);
                script_iter.next();
            }
        }

        // Arm request
        if tick == 11 {
            assert!(state_manager.is_armed());
            assert!(!state_manager.is_in_failsafe());
        }
        // Lost RC
        if tick == 26 {
            assert!(state_manager.is_armed());
            assert!(state_manager.is_in_failsafe());
        }
        // Second error occurs
        if tick == 51 {
            assert!(state_manager.is_armed());
            assert!(state_manager.is_in_failsafe());
            assert!(
                state_manager.get_errors() == ErrorFlag::RC_LOST | ErrorFlag::UNHEALTHY_ESTIMATOR
            );
        }
        // RC found
        if tick == 76 {
            assert!(state_manager.is_armed());
            assert!(!state_manager.is_in_failsafe());
            assert!(state_manager.get_errors() == ErrorFlag::UNHEALTHY_ESTIMATOR);
        }
        // Disarm request
        if tick == 101 {
            assert!(!state_manager.is_armed());
            assert!(!state_manager.is_in_failsafe());
            assert_state!(state_manager.machine, StateMachine::ErrorPresent);
            assert!(state_manager.get_errors() == ErrorFlag::UNHEALTHY_ESTIMATOR);
        }
    }
}
