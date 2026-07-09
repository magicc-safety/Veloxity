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
    sm.update_arming_safety(true, true);
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
    let (_, params) = setup_sm();
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
    //params.set_calibrate_gyro_on_arm(ParamValue::Int(1));
    params.set_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM, ParamValue::Int(1));
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
    //params.set_calibrate_gyro_on_arm(ParamValue::Int(1));
    params.set_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM, ParamValue::Int(1));

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
    // params.set_calibrate_gyro_on_arm(ParamValue::Int(1));
    params.set_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM, ParamValue::Int(1));

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
    // params.set_calibrate_gyro_on_arm(ParamValue::Int(1));
    params.set_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM, ParamValue::Int(1));

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
    // params.set_calibrate_gyro_on_arm(ParamValue::Int(1));
    params.set_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM, ParamValue::Int(1));

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
    // params.set_calibrate_gyro_on_arm(ParamValue::Int(1));
    params.set_by_id(ParamId::PARAM_CALIBRATE_GYRO_ON_ARM, ParamValue::Int(1));

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
fn test_unable_to_arm_with_uncalibrated_imu_error() {
    let (mut sm, params) = setup_sm();
    sm.update(Event::ERROR_OCCURRED(ErrorFlag::UNCALIBRATED_IMU), &params);
    sm.update(Event::REQUEST_ARM, &params);
    assert!(!sm.is_armed());
    assert_eq!(sm.get_errors(), ErrorFlag::UNCALIBRATED_IMU);
    assert_state!(sm, StateMachine::ErrorPresent);
}

#[test]
fn test_bypass_unhealthy_estimator_default_allows_arm_with_only_estimator_error() {
    let (mut manager, params) = setup_state_manager();
    manager.set_error_flag(ErrorFlag::UNHEALTHY_ESTIMATOR, true, &params);

    manager.update(Event::REQUEST_ARM, &params);

    assert!(manager.is_armed());
    assert_eq!(manager.get_errors(), ErrorFlag::UNHEALTHY_ESTIMATOR);
    assert_state!(manager.machine, StateMachine::Armed);
}

#[test]
fn test_bypass_unhealthy_estimator_param_off_blocks_arm_with_estimator_error() {
    let (mut manager, mut params) = setup_state_manager();
    params.set_by_id(ParamId::PARAM_ALLOW_UNHEALTHY_ESTIMATOR, ParamValue::Int(0));
    manager.set_error_flag(ErrorFlag::UNHEALTHY_ESTIMATOR, true, &params);

    manager.update(Event::REQUEST_ARM, &params);

    assert!(!manager.is_armed());
    assert_eq!(manager.get_errors(), ErrorFlag::UNHEALTHY_ESTIMATOR);
    assert_state!(manager.machine, StateMachine::ErrorPresent);
}

#[test]
fn test_bypass_unhealthy_estimator_keeps_other_errors_blocking_arm() {
    let (mut manager, params) = setup_state_manager();
    manager.set_error_flag(
        ErrorFlag::UNHEALTHY_ESTIMATOR | ErrorFlag::RC_LOST,
        true,
        &params,
    );

    manager.update(Event::REQUEST_ARM, &params);

    assert!(!manager.is_armed());
    assert_eq!(
        manager.get_errors(),
        ErrorFlag::UNHEALTHY_ESTIMATOR | ErrorFlag::RC_LOST
    );
    assert_state!(manager.machine, StateMachine::ErrorPresent);
}

#[test]
fn test_bypass_unhealthy_estimator_still_enforces_arming_safety() {
    let params = Params::new();
    let mut manager = StateManager::new();
    manager.update(Event::INITIALIZED, &params);
    manager.update_arming_safety(false, true);
    manager.set_error_flag(ErrorFlag::UNHEALTHY_ESTIMATOR, true, &params);

    manager.update(Event::REQUEST_ARM, &params);

    assert!(!manager.is_armed());
    assert_eq!(manager.get_errors(), ErrorFlag::UNHEALTHY_ESTIMATOR);
    assert_state!(manager.machine, StateMachine::ErrorPresent);
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
    assert!(sm.is_in_failsafe());
    assert_state!(sm, StateMachine::ErrorFailsafe);
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

#[test]
fn state_manager_rejects_arm_when_rc_throttle_high() {
    let mut params = Params::new();
    params.set_by_id(
        ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE,
        ParamValue::Int(1),
    );
    let mut manager = StateManager::new();
    manager.update(Event::INITIALIZED, &params);
    manager.update_arming_safety(false, true);

    manager.update(Event::REQUEST_ARM, &params);

    assert!(!manager.is_armed());
    assert_state!(manager.machine, StateMachine::Preflight);
}

#[test]
fn state_manager_rejects_arm_without_take_min_or_throttle_override() {
    let mut params = Params::new();
    params.set_by_id(
        ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE,
        ParamValue::Int(0),
    );
    let mut manager = StateManager::new();
    manager.update(Event::INITIALIZED, &params);
    manager.update_arming_safety(true, false);

    manager.update(Event::REQUEST_ARM, &params);

    assert!(!manager.is_armed());
    assert_state!(manager.machine, StateMachine::Preflight);
}

#[test]
fn state_manager_allows_arm_with_low_throttle_and_take_min() {
    let mut params = Params::new();
    params.set_by_id(
        ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE,
        ParamValue::Int(1),
    );
    let mut manager = StateManager::new();
    manager.update(Event::INITIALIZED, &params);
    manager.update_arming_safety(true, false);

    manager.update(Event::REQUEST_ARM, &params);

    assert!(manager.is_armed());
    assert_state!(manager.machine, StateMachine::Armed);
}

#[test]
fn state_manager_allows_arm_with_low_throttle_and_throttle_override() {
    let mut params = Params::new();
    params.set_by_id(
        ParamId::PARAM_RC_OVERRIDE_TAKE_MIN_THROTTLE,
        ParamValue::Int(0),
    );
    let mut manager = StateManager::new();
    manager.update(Event::INITIALIZED, &params);
    manager.update_arming_safety(true, true);

    manager.update(Event::REQUEST_ARM, &params);

    assert!(manager.is_armed());
    assert_state!(manager.machine, StateMachine::Armed);
}

#[test]
fn hardfault_rearm_is_an_explicit_state_transition() {
    let params = Params::new();
    let mut manager = StateManager::new();
    manager.update(Event::INITIALIZED, &params);
    manager.update_arming_safety(false, false);

    manager.update(Event::HARDFAULT_REARM_REQUESTED, &params);

    assert!(manager.is_armed());
    assert_state!(manager.machine, StateMachine::Armed);
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
