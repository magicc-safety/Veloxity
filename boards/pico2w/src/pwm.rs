use crate::config::MAX_PWM_OUTPUTS;
use voloxide_core::{
    board::BoardIo,
    pwm::{PwmDriver, PwmError},
};

pub struct PioPwmDriver {
    enabled_mask: u16,
    values: [u16; MAX_PWM_OUTPUTS],
    rates_hz: [f64; MAX_PWM_OUTPUTS],
}

impl PioPwmDriver {
    pub fn new() -> Self {
        Self {
            enabled_mask: 0,
            values: [0; MAX_PWM_OUTPUTS],
            rates_hz: [0.0; MAX_PWM_OUTPUTS],
        }
    }
}

impl Default for PioPwmDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl PwmDriver for PioPwmDriver {
    fn len(&self) -> usize {
        MAX_PWM_OUTPUTS
    }

    fn is_enabled(&self) -> bool {
        self.enabled_mask == ((1 << MAX_PWM_OUTPUTS) - 1)
    }

    fn enable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= self.len() {
            return Err(PwmError::ChannelOutOfRange);
        }
        self.enabled_mask |= 1 << channel;
        Ok(())
    }

    fn disable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= self.len() {
            return Err(PwmError::ChannelOutOfRange);
        }
        self.enabled_mask &= !(1 << channel);
        Ok(())
    }

    fn enable_all(&mut self) -> Result<(), PwmError> {
        for channel in 0..self.len() {
            self.enable(channel)?;
        }
        Ok(())
    }

    fn disable_all(&mut self) {
        self.enabled_mask = 0;
    }

    fn set_duty_cycle(&mut self, channel: usize, duty: u16) -> Result<(), PwmError> {
        if channel >= self.len() {
            return Err(PwmError::ChannelOutOfRange);
        }
        self.values[channel] = duty;
        Ok(())
    }

    fn flush<B: BoardIo>(&mut self, _board: &mut B) {}

    fn configure_output_rates(&mut self, rates_hz: &[f64]) -> Result<(), PwmError> {
        for (slot, rate) in self.rates_hz.iter_mut().zip(rates_hz.iter().copied()) {
            *slot = rate;
        }
        Ok(())
    }

    fn send_commands<B: BoardIo>(
        &mut self,
        board: &mut B,
        commands: &[f64],
    ) -> Result<(), PwmError> {
        for (channel, command) in commands.iter().copied().enumerate().take(self.len()) {
            let duty = (command.clamp(0.0, 1.0) * u16::MAX as f64) as u16;
            self.set_duty_cycle(channel, duty)?;
        }
        self.flush(board);
        Ok(())
    }
}
