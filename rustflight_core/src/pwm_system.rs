use crate::{
    board::BoardIo,
    comm_messages::{
        enums::RosflightAuxCmdType,
        messages::RosflightAuxCmdMsg,
    },
    mixer::MixerOutputType,
    params2::{ParamId, ParamValue, Params},
    pwm::{PwmDriver, PwmError},
    state_machine::StateManager,
};

pub const PWM_OUTPUT_CHANNELS: usize = 14;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PwmOutputState {
    enabled: bool,
}

impl PwmOutputState {
    pub fn new(enabled: bool) -> Self {
        Self { enabled }
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}

pub fn sync_pwm_output_state<B, P>(
    board: &mut B,
    pwm: &mut P,
    output: &mut PwmOutputState,
    state: &StateManager,
) -> Result<bool, PwmError>
where
    B: BoardIo,
    P: PwmDriver,
{
    let desired_enabled = state.is_armed();
    if desired_enabled == output.enabled {
        return Ok(false);
    }

    if desired_enabled {
        pwm.enable_all()?;
    } else {
        pwm.disable_all();
        pwm.flush(board);
    }

    output.enabled = desired_enabled;
    Ok(true)
}

pub fn write_pwm_commands<B, P>(
    board: &mut B,
    pwm: &mut P,
    output: &PwmOutputState,
    commands: &[f64],
) -> bool
where
    B: BoardIo,
    P: PwmDriver,
{
    if !output.is_enabled() {
        return false;
    }

    pwm.send_commands(board, commands);
    true
}

pub fn compose_pwm_outputs(
    primary_commands: &[f64],
    primary_output_types: &[MixerOutputType],
    aux_command: Option<&RosflightAuxCmdMsg>,
    state: &StateManager,
    params: &Params,
) -> [f64; PWM_OUTPUT_CHANNELS] {
    let idle_throttle = match params.get_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE) {
        ParamValue::Float(value) => value as f64,
        _ => 0.0,
    };
    let spin_when_armed = match params.get_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED) {
        ParamValue::Bool(value) => value,
        _ => false,
    };

    let mut outputs = [0.0; PWM_OUTPUT_CHANNELS];

    for channel in 0..PWM_OUTPUT_CHANNELS {
        let primary_type = primary_output_types
            .get(channel)
            .copied()
            .unwrap_or(MixerOutputType::Aux);
        let primary_value = primary_commands.get(channel).copied().unwrap_or(0.0);
        let (output_type, value) = if primary_type == MixerOutputType::Aux {
            aux_output_for_channel(aux_command, channel)
        } else {
            (primary_type, primary_value)
        };

        outputs[channel] =
            raw_output_for_type(output_type, value, state, idle_throttle, spin_when_armed);
    }

    outputs
}

fn aux_output_for_channel(
    aux_command: Option<&RosflightAuxCmdMsg>,
    channel: usize,
) -> (MixerOutputType, f64) {
    let Some(aux_command) = aux_command else {
        return (MixerOutputType::Aux, 0.0);
    };

    match aux_command.type_array[channel] {
        RosflightAuxCmdType::Disabled => (MixerOutputType::Aux, 0.0),
        RosflightAuxCmdType::Servo => (
            MixerOutputType::Servo,
            aux_command.aux_cmd_array[channel] as f64,
        ),
        RosflightAuxCmdType::Motor => (
            MixerOutputType::Motor,
            aux_command.aux_cmd_array[channel] as f64,
        ),
    }
}

fn raw_output_for_type(
    output_type: MixerOutputType,
    value: f64,
    state: &StateManager,
    idle_throttle: f64,
    spin_when_armed: bool,
) -> f64 {
    match output_type {
        MixerOutputType::Aux => 0.0,
        MixerOutputType::Servo => value.clamp(-1.0, 1.0) * 0.5 + 0.5,
        MixerOutputType::Motor => {
            if !state.is_armed() {
                0.0
            } else if value > 1.0 {
                1.0
            } else if value < idle_throttle && spin_when_armed {
                idle_throttle
            } else if value < 0.0 {
                0.0
            } else {
                value
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        errors,
        params2::{ParamId, ParamValue, Params},
        state_machine::Event,
    };

    struct TestBoard {
        now_us: u64,
    }

    impl BoardIo for TestBoard {
        fn serial_rx_read(&mut self, _buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
            None
        }

        fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
            Some(Ok(bytes.len()))
        }

        fn clock_millis(&self) -> u32 {
            (self.now_us / 1000) as u32
        }

        fn clock_micros(&self) -> u64 {
            self.now_us
        }
    }

    struct TestPwm {
        enabled: bool,
        enable_all_count: usize,
        disable_all_count: usize,
        flush_count: usize,
        send_count: usize,
    }

    impl TestPwm {
        fn new(enabled: bool) -> Self {
            Self {
                enabled,
                enable_all_count: 0,
                disable_all_count: 0,
                flush_count: 0,
                send_count: 0,
            }
        }
    }

    impl PwmDriver for TestPwm {
        fn len(&self) -> usize {
            4
        }

        fn is_enabled(&self) -> bool {
            self.enabled
        }

        fn enable(&mut self, _channel: usize) -> Result<(), PwmError> {
            self.enabled = true;
            Ok(())
        }

        fn disable(&mut self, _channel: usize) -> Result<(), PwmError> {
            self.enabled = false;
            Ok(())
        }

        fn enable_all(&mut self) -> Result<(), PwmError> {
            self.enabled = true;
            self.enable_all_count += 1;
            Ok(())
        }

        fn disable_all(&mut self) {
            self.enabled = false;
            self.disable_all_count += 1;
        }

        fn set_duty_cycle(&mut self, _channel: usize, _duty: u16) -> Result<(), PwmError> {
            Ok(())
        }

        fn flush<Board: BoardIo>(&mut self, _board: &mut Board) {
            self.flush_count += 1;
        }

        fn send_commands<Board: BoardIo>(&mut self, _board: &mut Board, _commands: &[f64]) {
            self.send_count += 1;
        }
    }

    #[test]
    fn pwm_output_state_enables_and_disables_only_on_state_transitions() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        let mut board = TestBoard { now_us: 0 };
        let mut pwm = TestPwm::new(false);
        let mut output = PwmOutputState::new(pwm.is_enabled());

        assert!(!sync_pwm_output_state(&mut board, &mut pwm, &mut output, &state).unwrap());
        assert_eq!(pwm.enable_all_count, 0);
        assert_eq!(pwm.disable_all_count, 0);

        state.update(Event::REQUEST_ARM, &params);

        assert!(sync_pwm_output_state(&mut board, &mut pwm, &mut output, &state).unwrap());
        assert!(output.is_enabled());
        assert_eq!(pwm.enable_all_count, 1);

        assert!(!sync_pwm_output_state(&mut board, &mut pwm, &mut output, &state).unwrap());
        assert_eq!(pwm.enable_all_count, 1);

        state.update(Event::REQUEST_DISARM, &params);

        assert!(sync_pwm_output_state(&mut board, &mut pwm, &mut output, &state).unwrap());
        assert!(!output.is_enabled());
        assert_eq!(pwm.disable_all_count, 1);
        assert_eq!(pwm.flush_count, 1);
    }

    #[test]
    fn write_pwm_commands_only_writes_when_output_enabled() {
        let mut board = TestBoard { now_us: 0 };
        let mut pwm = TestPwm::new(false);
        let disabled = PwmOutputState::new(false);
        let enabled = PwmOutputState::new(true);

        assert!(!write_pwm_commands(
            &mut board,
            &mut pwm,
            &disabled,
            &[0.1, 0.2]
        ));
        assert_eq!(pwm.send_count, 0);

        assert!(write_pwm_commands(
            &mut board,
            &mut pwm,
            &enabled,
            &[0.1, 0.2]
        ));
        assert_eq!(pwm.send_count, 1);
    }

    #[test]
    fn compose_pwm_outputs_preserves_primary_and_applies_aux_to_unused_channels() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_MOTOR_IDLE_THROTTLE, ParamValue::Float(0.2));
        params.set_by_id(ParamId::PARAM_SPIN_MOTORS_WHEN_ARMED, ParamValue::Bool(true));
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state.update(Event::REQUEST_ARM, &params);
        let mut aux = RosflightAuxCmdMsg {
            type_array: [RosflightAuxCmdType::Disabled; PWM_OUTPUT_CHANNELS],
            aux_cmd_array: [0.0; PWM_OUTPUT_CHANNELS],
        };
        aux.type_array[4] = RosflightAuxCmdType::Servo;
        aux.aux_cmd_array[4] = -0.5;
        aux.type_array[5] = RosflightAuxCmdType::Motor;
        aux.aux_cmd_array[5] = 0.1;

        let output_types = [
            MixerOutputType::Motor,
            MixerOutputType::Motor,
            MixerOutputType::Motor,
            MixerOutputType::Motor,
        ];
        let outputs = compose_pwm_outputs(
            &[0.1, 0.2, 0.3, 0.4],
            &output_types,
            Some(&aux),
            &state,
            &params,
        );

        assert!((outputs[0] - 0.2).abs() < 1e-6);
        assert!((outputs[1] - 0.2).abs() < 1e-6);
        assert_eq!(outputs[2], 0.3);
        assert_eq!(outputs[3], 0.4);
        assert_eq!(outputs[4], 0.25);
        assert!((outputs[5] - 0.2).abs() < 1e-6);
        assert_eq!(outputs[6], 0.0);
    }

    #[test]
    fn compose_pwm_outputs_forces_aux_motors_low_when_disarmed() {
        let params = Params::new();
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        let mut aux = RosflightAuxCmdMsg {
            type_array: [RosflightAuxCmdType::Disabled; PWM_OUTPUT_CHANNELS],
            aux_cmd_array: [0.0; PWM_OUTPUT_CHANNELS],
        };
        aux.type_array[4] = RosflightAuxCmdType::Motor;
        aux.aux_cmd_array[4] = 0.8;

        let output_types = [
            MixerOutputType::Motor,
            MixerOutputType::Motor,
            MixerOutputType::Motor,
            MixerOutputType::Motor,
        ];
        let outputs = compose_pwm_outputs(
            &[0.1, 0.2, 0.3, 0.4],
            &output_types,
            Some(&aux),
            &state,
            &params,
        );

        assert_eq!(outputs[4], 0.0);
    }

    #[test]
    fn compose_pwm_outputs_uses_aux_inside_primary_range_only_for_aux_owned_slots() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(0.1));
        let mut state = StateManager::new();
        state.update(Event::INITIALIZED, &params);
        state.update(Event::REQUEST_ARM, &params);
        let mut aux = RosflightAuxCmdMsg {
            type_array: [RosflightAuxCmdType::Disabled; PWM_OUTPUT_CHANNELS],
            aux_cmd_array: [0.0; PWM_OUTPUT_CHANNELS],
        };
        aux.type_array[1] = RosflightAuxCmdType::Servo;
        aux.aux_cmd_array[1] = 1.0;
        aux.type_array[2] = RosflightAuxCmdType::Servo;
        aux.aux_cmd_array[2] = -1.0;
        let output_types = [
            MixerOutputType::Motor,
            MixerOutputType::Aux,
            MixerOutputType::Motor,
            MixerOutputType::Motor,
        ];

        let outputs = compose_pwm_outputs(
            &[0.1, 0.2, 0.3, 0.4],
            &output_types,
            Some(&aux),
            &state,
            &params,
        );

        assert!((outputs[0] - 0.1).abs() < 1e-6);
        assert_eq!(outputs[1], 1.0);
        assert_eq!(outputs[2], 0.3);
        assert_eq!(outputs[3], 0.4);
    }
}
