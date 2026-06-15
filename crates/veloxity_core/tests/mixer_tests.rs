use nalgebra::SVector as Vector;
use veloxity_core::controller::quad::ControllerOutput;
use veloxity_core::mixer::matrix::{MatrixMixer, sync_reflected_mixer_params};
use veloxity_core::mixer::{Mixer, MixerCtx, MixerStatus};
use veloxity_core::params::{ParamId, ParamValue, Params};
use veloxity_core::state_machine::{Event, StateManager};

fn test_params() -> Params {
    let mut params = Params::default();

    // --- REALISTIC PHYSICS CONSTANTS ---
    // k_t approx 1e-5 (Typical for 10-inch prop)
    // k_q approx 1e-6
    // This ensures that 20N of thrust requires ~700 rad/s (reasonable RPM),
    // which is ~70% throttle, well above the 5% idle.
    params.set_by_id(ParamId::PARAM_PROP_CT, ParamValue::Float(1.0e-5));
    params.set_by_id(ParamId::PARAM_PROP_CQ, ParamValue::Float(1.0e-6));

    params.set_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE, ParamValue::Float(0.05));
    params.set_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED, ParamValue::Int(1));

    // Default KV (900) + 12.6V -> Max RPM ~ 1000 rad/s.
    // This aligns with the physics above.

    params
}

/// Helper to create a Mixer with deterministic parameters for testing
fn create_test_mixer() -> MatrixMixer<f64> {
    let params = test_params();
    MatrixMixer::new(&params)
}

fn output_types(mixer: &MatrixMixer<f64>) -> &[veloxity_core::mixer::MixerOutputType] {
    <MatrixMixer<f64> as Mixer<f64>>::output_types(mixer)
}

fn default_pwm_rates(mixer: &MatrixMixer<f64>) -> &[f64] {
    <MatrixMixer<f64> as Mixer<f64>>::default_pwm_rates(mixer)
}

fn armed_state() -> StateManager {
    let mut params = Params::default();
    params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));

    let mut state = StateManager::new();
    state.update(Event::INITIALIZED, &params);
    state.update(Event::REQUEST_ARM, &params);
    state
}

fn controller_output(thrust: f64, torques: Vector<f64, 3>) -> ControllerOutput<f64> {
    ControllerOutput::from_forces_torques_and_passthrough(
        Vector::from([0.0, 0.0, -thrust]),
        torques,
        [0.0; 4],
    )
}

fn mixer_ctx<'a>(state: &'a StateManager, params: &'a Params) -> MixerCtx<'a, f64> {
    MixerCtx {
        state,
        params,
        rc_override: 0,
        air_density: 1.225,
        battery_voltage: Some(12.6),
    }
}

#[test]
fn test_hover_condition() {
    let mut mixer = create_test_mixer();

    // Command: 10N Thrust (approx 1kg hover)
    let input = controller_output(10.0, Vector::from([0.0, 0.0, 0.0]));
    let params = test_params();
    let state = armed_state();

    let outputs = mixer.mix(&input, mixer_ctx(&state, &params)).commands;

    println!("Hover Outputs: {:?}", outputs);

    // 1. Symmetry check: All motors should be equal
    assert!((outputs[0] - outputs[1]).abs() < 1e-6);
    assert!((outputs[1] - outputs[2]).abs() < 1e-6);
    assert!((outputs[2] - outputs[3]).abs() < 1e-6);

    // 2. Magnitude check: Should be significantly above idle
    assert!(outputs[0] > 0.1);
}

#[test]
fn test_quad_emits_rosflight_ten_output_shape() {
    let mut mixer = create_test_mixer();
    let input = ControllerOutput::from_forces_torques_and_passthrough(
        Vector::from([0.0, 0.0, -0.4]),
        Vector::from([0.0, 0.0, 0.0]),
        [0.6, 0.7, 0.8, 0.9],
    );
    let params = test_params();
    let state = armed_state();

    let outputs = mixer.mix(&input, mixer_ctx(&state, &params)).commands;

    assert_eq!(outputs.as_ref().len(), 10);
    assert_eq!(output_types(&mixer).len(), 10);
    assert!(
        output_types(&mixer)[0..4]
            .iter()
            .all(|kind| *kind == veloxity_core::mixer::MixerOutputType::Motor)
    );
    assert!(
        output_types(&mixer)[4..10]
            .iter()
            .all(|kind| *kind == veloxity_core::mixer::MixerOutputType::Aux)
    );
    assert_eq!(&outputs[4..10], &[0.0; 6]);
}

#[test]
fn test_quad_x_canned_mixer_uses_rosflight_runtime_inversion() {
    let mut params = test_params();
    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(2));
    let mut mixer = MatrixMixer::new(&params);
    let input = controller_output(0.4, Vector::from([0.0, 0.0, 0.0]));
    let state = armed_state();

    let run = mixer.mix(&input, mixer_ctx(&state, &params));

    assert_eq!(run.status, MixerStatus::Healthy);
    for output in run.commands.iter().take(4) {
        assert!((output - 0.4).abs() < 1e-6);
    }
}

#[test]
fn test_esc_calibration_mixer_uses_rosflight_rank_deficient_pseudoinverse() {
    let mut params = test_params();
    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(0));
    params.set_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE, ParamValue::Float(0.0));
    let mut mixer = MatrixMixer::new(&params);
    let input = ControllerOutput::from_forces_torques_and_passthrough(
        Vector::from([0.0, 0.0, 0.4]),
        Vector::from([0.0, 0.0, 0.0]),
        [0.0; 4],
    );
    let state = armed_state();

    let run = mixer.mix(&input, mixer_ctx(&state, &params));

    assert_eq!(run.status, MixerStatus::Healthy);
    for output in run.commands.iter().take(10) {
        assert!((output - 0.4).abs() < 1e-6);
    }
}

#[test]
fn test_custom_mixer_loads_rosflight_parameter_matrix_and_output_types() {
    let mut params = test_params();
    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(11));
    params.set_by_name("PRI_MIXER_OUT_0", ParamValue::Int(2));
    params.set_by_name("PRI_MIXER_OUT_1", ParamValue::Int(0));
    params.set_by_name("PRI_MIXER_PWM_0", ParamValue::Float(490.0));
    params.set_by_name("PRI_MIXER_2_0", ParamValue::Float(-0.5));

    let mut mixer = MatrixMixer::new(&params);
    let input = controller_output(0.4, Vector::from([0.0, 0.0, 0.0]));
    let state = armed_state();

    let outputs = mixer.mix(&input, mixer_ctx(&state, &params)).commands;

    assert_eq!(
        output_types(&mixer)[0],
        veloxity_core::mixer::MixerOutputType::Motor
    );
    assert_eq!(
        output_types(&mixer)[1],
        veloxity_core::mixer::MixerOutputType::Aux
    );
    assert_eq!(default_pwm_rates(&mixer)[0], 490.0);
    assert!((outputs[0] - 0.2).abs() < 1e-6);
    assert_eq!(outputs[1], 0.0);
}

#[test]
fn test_invalid_primary_mixer_reports_status_without_mutating_state() {
    let mut params = test_params();
    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(255));
    let state = armed_state();
    let mut mixer = MatrixMixer::new(&params);
    let input = controller_output(0.4, Vector::from([0.0, 0.0, 0.0]));

    let run = mixer.mix(&input, mixer_ctx(&state, &params));

    assert_eq!(run.status, MixerStatus::InvalidMixer);
    assert!(
        !state
            .get_errors()
            .contains(veloxity_core::state_machine::ErrorFlag::INVALID_MIXER)
    );
}

#[test]
fn mixer_config_refreshes_from_param_change_hook_not_mix_polling() {
    let mut params = test_params();
    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(2));
    let state = armed_state();
    let mut mixer = MatrixMixer::<f64>::new(&params);
    let input = controller_output(0.4, Vector::from([0.0, 0.0, 0.0]));

    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(255));
    let run_before_event = mixer.mix(&input, mixer_ctx(&state, &params));
    assert_eq!(run_before_event.status, MixerStatus::Healthy);

    let status = mixer.on_param_changed(&params, ParamId::PARAM_PRIMARY_MIXER);
    assert_eq!(status, Some(MixerStatus::InvalidMixer));

    let run_after_event = mixer.mix(&input, mixer_ctx(&state, &params));
    assert_eq!(run_after_event.status, MixerStatus::InvalidMixer);
}

#[test]
fn test_canned_hex_x_selection_uses_rosflight_output_ownership() {
    let mut params = test_params();
    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(4));
    let mut mixer = MatrixMixer::new(&params);
    let input = controller_output(0.6, Vector::from([0.0, 0.0, 0.0]));
    let state = armed_state();

    let run = mixer.mix(&input, mixer_ctx(&state, &params));

    assert_eq!(run.status, MixerStatus::Healthy);
    assert!(
        output_types(&mixer)[0..6]
            .iter()
            .all(|kind| *kind == veloxity_core::mixer::MixerOutputType::Motor)
    );
    assert!(
        output_types(&mixer)[6..10]
            .iter()
            .all(|kind| *kind == veloxity_core::mixer::MixerOutputType::Aux)
    );
    assert_eq!(default_pwm_rates(&mixer)[0], 490.0);
    assert_eq!(default_pwm_rates(&mixer)[8], 50.0);
    assert!(run.commands[0] > 0.0);
    assert_eq!(run.commands[6], 0.0);
}

#[test]
fn test_fixedwing_mixer_applies_reversal_params_before_mixing() {
    let mut params = test_params();
    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(9));
    params.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Int(1));
    params.set_by_id(ParamId::PARAM_AILERON_REVERSE, ParamValue::Int(1));
    let mut mixer = MatrixMixer::new(&params);
    let input = ControllerOutput::from_forces_torques_and_passthrough(
        Vector::from([0.2, 0.0, 0.0]),
        Vector::from([0.4, 0.5, 0.6]),
        [0.0; 4],
    );
    let state = armed_state();

    let outputs = mixer.mix(&input, mixer_ctx(&state, &params)).commands;

    assert_eq!(
        output_types(&mixer)[0],
        veloxity_core::mixer::MixerOutputType::Servo
    );
    assert_eq!(
        output_types(&mixer)[4],
        veloxity_core::mixer::MixerOutputType::Motor
    );
    assert!((outputs[0] + 0.4).abs() < 1e-6);
    assert!((outputs[1] - 0.5).abs() < 1e-6);
    assert!((outputs[2] - 0.6).abs() < 1e-6);
    assert!((outputs[4] - 0.2).abs() < 1e-6);
}

#[test]
fn fixedwing_canned_mixer_updates_rosflight_reflection_params() {
    let mut params = test_params();
    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(10));

    sync_reflected_mixer_params(&mut params, ParamId::PARAM_PRIMARY_MIXER);

    assert_eq!(
        params.get_by_name("PRI_MIXER_OUT_0"),
        Some(ParamValue::Int(1))
    );
    assert_eq!(
        params.get_by_name("PRI_MIXER_OUT_3"),
        Some(ParamValue::Int(0))
    );
    assert_eq!(
        params.get_by_name("PRI_MIXER_OUT_4"),
        Some(ParamValue::Int(2))
    );
    assert_eq!(
        params.get_by_name("PRI_MIXER_3_0"),
        Some(ParamValue::Float(1.0))
    );
    assert_eq!(
        params.get_by_name("PRI_MIXER_4_1"),
        Some(ParamValue::Float(-0.5))
    );
    assert_eq!(
        params.get_by_name("PRI_MIXER_5_2"),
        Some(ParamValue::Float(0.5))
    );
    assert_eq!(
        params.get_by_name("PRI_MIXER_0_4"),
        Some(ParamValue::Float(1.0))
    );
}

#[test]
fn test_secondary_inverted_vtail_matches_rosflight_pseudoinverse_branch() {
    let mut params = test_params();
    params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(9));
    params.set_by_id(ParamId::PARAM_SECONDARY_MIXER, ParamValue::Int(10));
    params.set_by_id(ParamId::PARAM_FIXED_WING, ParamValue::Int(1));
    let mut mixer = MatrixMixer::new(&params);
    let input = ControllerOutput::from_forces_torques_and_passthrough(
        Vector::from([0.25, 0.0, 0.0]),
        Vector::from([0.3, 0.4, 0.2]),
        [0.0; 4],
    );
    let state = armed_state();

    let outputs = mixer.mix(&input, mixer_ctx(&state, &params)).commands;

    assert!((outputs[0] - 0.3).abs() < 1e-6);
    assert!((outputs[1] - (-0.2)).abs() < 1e-6);
    assert!((outputs[2] - 0.6).abs() < 1e-6);
    assert!((outputs[4] - 0.25).abs() < 1e-6);
}

#[test]
fn test_pure_roll_right() {
    let mut mixer = create_test_mixer();

    // Command: Hover (10N) + Roll Torque (0.5 Nm)
    // 0.5 Nm is a reasonable control effort for a small quad
    let input = controller_output(10.0, Vector::from([0.5, 0.0, 0.0]));
    let params = test_params();
    let state = armed_state();

    let outputs = mixer.mix(&input, mixer_ctx(&state, &params)).commands;

    println!("Roll Right Outputs: {:?}", outputs);

    let right_motors_avg = (outputs[0] + outputs[1]) / 2.0;
    let left_motors_avg = (outputs[2] + outputs[3]) / 2.0;

    // Verify Left > Right for Positive Roll (Standard X Config)
    assert!(
        left_motors_avg > right_motors_avg,
        "Left motors should spin faster than right motors for positive roll torque"
    );
}

#[test]
fn test_pure_pitch_down() {
    let mut mixer = create_test_mixer();

    // Command: Hover + Pitch Torque
    let input = controller_output(10.0, Vector::from([0.0, 0.5, 0.0]));
    let params = test_params();
    let state = armed_state();

    let outputs = mixer.mix(&input, mixer_ctx(&state, &params)).commands;

    let front_motors_avg = (outputs[0] + outputs[3]) / 2.0;
    let rear_motors_avg = (outputs[1] + outputs[2]) / 2.0;

    println!(
        "Pitch Outputs (Front: {}, Rear: {})",
        front_motors_avg, rear_motors_avg
    );

    // Current MatrixMixer convention: positive pitch input increases the front pair.
    assert!(
        front_motors_avg > rear_motors_avg,
        "Front motors should spin faster for positive pitch input"
    );
}

#[test]
fn test_pure_yaw_clockwise() {
    let mut mixer = create_test_mixer();

    // Command: Hover + Yaw Torque (0.1 Nm - yaw is usually weaker)
    let input = controller_output(10.0, Vector::from([0.0, 0.0, 0.1]));
    let params = test_params();
    let state = armed_state();

    let outputs = mixer.mix(&input, mixer_ctx(&state, &params)).commands;

    let cw_motors_avg = (outputs[0] + outputs[2]) / 2.0;
    let ccw_motors_avg = (outputs[1] + outputs[3]) / 2.0;

    println!(
        "Yaw Outputs (CW: {}, CCW: {})",
        cw_motors_avg, ccw_motors_avg
    );

    // ROSflight quad-X convention: positive yaw increases the CW motor pair.
    assert!(
        cw_motors_avg > ccw_motors_avg,
        "CW motors should spin faster for positive yaw input"
    );
}

#[test]
fn test_low_throttle_suppresses_yaw() {
    let mut mixer = create_test_mixer();

    let input = controller_output(0.01, Vector::from([0.0, 0.0, 0.8]));
    let params = test_params();
    let state = armed_state();

    let outputs = mixer.mix(&input, mixer_ctx(&state, &params)).commands;

    assert!((outputs[0] - outputs[1]).abs() < 1e-6);
    assert!((outputs[1] - outputs[2]).abs() < 1e-6);
    assert!((outputs[2] - outputs[3]).abs() < 1e-6);
}

#[test]
fn test_saturation_scaling() {
    let mut mixer = create_test_mixer();

    // Command: Massive Thrust that definitely exceeds 40N (approx physical limit)
    let input = controller_output(1000.0, Vector::from([0.0, 0.0, 0.0]));
    let params = test_params();
    let state = armed_state();

    let outputs = mixer.mix(&input, mixer_ctx(&state, &params)).commands;

    println!("Saturation Outputs: {:?}", outputs);

    // Max output should be exactly 1.0
    let max_val = outputs[0].max(outputs[1]).max(outputs[2]).max(outputs[3]);

    assert!(
        (max_val - 1.0).abs() < 1e-6,
        "Mixer did not clamp output to 1.0"
    );

    // All motors should be equal (pure thrust)
    assert!((outputs[0] - outputs[3]).abs() < 1e-6);
}

#[test]
fn test_saturation_preserves_ratio() {
    let mut mixer = create_test_mixer();

    // Command: High Thrust + High Roll
    // Both inputs are large enough to saturate the mixer individually.
    let input = controller_output(1000.0, Vector::from([500.0, 0.0, 0.0]));
    let params = test_params();
    let state = armed_state();

    let outputs = mixer.mix(&input, mixer_ctx(&state, &params)).commands;

    // Check that we are still saturated at 1.0
    let max_val = outputs[0].max(outputs[1]).max(outputs[2]).max(outputs[3]);
    assert!((max_val - 1.0).abs() < 1e-6);

    // Check that we still have differential thrust (Roll is active)
    let right_motors_avg = (outputs[0] + outputs[1]) / 2.0;
    let left_motors_avg = (outputs[2] + outputs[3]) / 2.0;

    assert!(
        left_motors_avg > right_motors_avg,
        "Differential thrust lost during saturation"
    );
}
