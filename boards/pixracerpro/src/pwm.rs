// ******************************************************************************
// * File     : boards/pixracerpro/src/pwm.rs
// * Date     : June 28, 2026
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

use stm_32::peripherals::pwm::{PixRacerProServoMonstrosity, TimerError};
use veloxity_core::board::BoardIo;
use veloxity_core::mixer::MixerOutputType;
use veloxity_core::pwm::{PwmDriver, PwmError, PwmOutputProtocol};

const NUM_HW_CHANNELS: usize = 4;

pub struct BoardPwmDriver<'a> {
    servos: &'a mut PixRacerProServoMonstrosity,
    current_values: [f32; NUM_HW_CHANNELS],
    enabled_chan_mask: u16,
    max_duty_counts: [u16; NUM_HW_CHANNELS],
}

impl<'a> BoardPwmDriver<'a> {
    pub fn new(servos: &'a mut PixRacerProServoMonstrosity) -> Self {
        let mut max_duty_counts = [0u16; NUM_HW_CHANNELS];
        for (channel, max_duty) in max_duty_counts.iter_mut().enumerate() {
            *max_duty = servos.max_duty_cycle(channel);
        }

        Self {
            servos,
            current_values: [1000.0; NUM_HW_CHANNELS],
            enabled_chan_mask: 0,
            max_duty_counts,
        }
    }

    fn duty_u16_to_pwm_us(duty: u16) -> f32 {
        let normalized = duty as f32 / u16::MAX as f32;
        (normalized.clamp(0.0, 1.0) * 1000.0) + 1000.0
    }
}

impl<'a> PwmDriver<f64> for BoardPwmDriver<'a> {
    fn len(&self) -> usize {
        NUM_HW_CHANNELS
    }

    fn is_enabled(&self) -> bool {
        self.enabled_chan_mask == ((1 << NUM_HW_CHANNELS) - 1)
    }

    fn enable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= NUM_HW_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        self.servos
            .enable(channel)
            .map_err(|_| PwmError::GenericError)?;

        self.enabled_chan_mask |= 1 << channel;

        Ok(())
    }

    fn disable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= NUM_HW_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        self.servos
            .disable(channel)
            .map_err(|_| PwmError::GenericError)?;

        self.enabled_chan_mask &= !(1 << channel);

        Ok(())
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
        if channel >= NUM_HW_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        let pwm_us = Self::duty_u16_to_pwm_us(duty);
        self.current_values[channel] = pwm_us;

        let max_duty = self.max_duty_counts[channel] as f32;
        let raw_pwm = pwm_us / 2500.0 * max_duty;

        self.servos
            .set_duty_cycle(channel, raw_pwm as u16)
            .map_err(|_| PwmError::GenericError)
    }

    fn configure_output_rates(&mut self, rates_hz: &[f64]) -> Result<(), PwmError> {
        self.servos
            .configure_output_rates(&rates_hz[..rates_hz.len().min(NUM_HW_CHANNELS)])
            .map_err(timer_error_to_pwm_error)?;

        for (channel, max_duty) in self.max_duty_counts.iter_mut().enumerate() {
            *max_duty = self.servos.max_duty_cycle(channel);
        }

        Ok(())
    }

    fn output_protocol(&self, channel: usize) -> Result<PwmOutputProtocol, PwmError> {
        if channel >= NUM_HW_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        self.servos
            .output_protocol(channel)
            .map_err(timer_error_to_pwm_error)
    }

    fn flush<B: BoardIo>(&mut self, _board: &mut B) {
        // Hardware state is already applied in set_duty_cycle.
    }

    fn send_commands<B: BoardIo>(
        &mut self,
        board: &mut B,
        commands_slice: &[f64],
    ) -> Result<(), PwmError> {
        self.servos
            .send_normalized_commands(&commands_slice[..commands_slice.len().min(NUM_HW_CHANNELS)])
            .map_err(timer_error_to_pwm_error)?;
        self.flush(board);
        Ok(())
    }

    fn send_disarmed_commands<B: BoardIo>(
        &mut self,
        board: &mut B,
        output_types: &[MixerOutputType],
    ) -> Result<(), PwmError> {
        self.servos
            .send_disarmed_commands(&output_types[..output_types.len().min(NUM_HW_CHANNELS)])
            .map_err(timer_error_to_pwm_error)?;
        self.flush(board);
        Ok(())
    }
}

fn timer_error_to_pwm_error(error: TimerError) -> PwmError {
    match error {
        TimerError::ChanNotSupported => PwmError::ChannelOutOfRange,
        TimerError::InvalidRate => PwmError::InvalidRate,
        TimerError::UnsupportedProtocol => PwmError::UnsupportedProtocol,
        TimerError::TimerNotSupported => PwmError::GenericError,
    }
}
