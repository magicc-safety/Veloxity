use crate::{
    board::BoardIo,
    pwm::{PwmDriver, PwmError},
    state_machine::StateManager,
};

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
}
