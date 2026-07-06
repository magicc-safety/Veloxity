use stm_32::peripherals::pwm::PixRacerProServoMonstrosity;
use veloxity_core::board::BoardIo;
use veloxity_core::mixer::MixerOutputType;
use veloxity_core::pwm::{PwmDriver, PwmError, PwmOutputProtocol};

const NUM_HW_CHANNELS: usize = 0;

pub struct BoardPwmDriver<'a> {
    _servos: &'a mut PixRacerProServoMonstrosity,
}

impl<'a> BoardPwmDriver<'a> {
    pub fn new(servos: &'a mut PixRacerProServoMonstrosity) -> Self {
        Self { _servos: servos }
    }
}

impl<'a> PwmDriver<f64> for BoardPwmDriver<'a> {
    fn len(&self) -> usize {
        NUM_HW_CHANNELS
    }

    fn is_enabled(&self) -> bool {
        true
    }

    fn enable(&mut self, channel: usize) -> Result<(), PwmError> {
        let _ = channel;
        Err(PwmError::ChannelOutOfRange)
    }

    fn disable(&mut self, channel: usize) -> Result<(), PwmError> {
        let _ = channel;
        Err(PwmError::ChannelOutOfRange)
    }

    fn enable_all(&mut self) -> Result<(), PwmError> {
        for i in 0..NUM_HW_CHANNELS {
            self.enable(i)?;
        }
        Ok(())
    }

    fn disable_all(&mut self) {
        for i in 0..NUM_HW_CHANNELS {
            let _ = self.disable(i);
        }
    }

    fn set_duty_cycle(&mut self, channel: usize, duty: u16) -> Result<(), PwmError> {
        let _ = (channel, duty);
        Err(PwmError::ChannelOutOfRange)
    }

    fn configure_output_rates(&mut self, rates_hz: &[f64]) -> Result<(), PwmError> {
        let _ = rates_hz;
        Ok(())
    }

    fn output_protocol(&self, channel: usize) -> Result<PwmOutputProtocol, PwmError> {
        let _ = channel;
        Err(PwmError::ChannelOutOfRange)
    }

    fn flush<B: BoardIo>(&mut self, _board: &mut B) {
        // Hardware state is already applied in set_duty_cycle.
    }

    fn send_commands<B: BoardIo>(
        &mut self,
        board: &mut B,
        commands_slice: &[f64],
    ) -> Result<(), PwmError> {
        let _ = commands_slice;
        self.flush(board);
        Ok(())
    }

    fn send_disarmed_commands<B: BoardIo>(
        &mut self,
        board: &mut B,
        output_types: &[MixerOutputType],
    ) -> Result<(), PwmError> {
        let _ = output_types;
        self.flush(board);
        Ok(())
    }
}
