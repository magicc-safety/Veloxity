#![no_std]
#![no_main]
// /**
// ******************************************************************************
// * File     : pwm.rs
// * Date     : November 3, 2025
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
use rustflight_core::board::BoardTrait;
use rustflight_core::pwm::{self, PwmDriver, PwmError};
//use stm_32::defmt;
// use crate::ros_messages::{OutputRaw, Header, Time};
use stm_32::peripherals::pwm::PixRacerProServoMonstrosity;

const NUM_HW_CHANNELS: usize = 7;

pub struct BoardPwmDriver<'a> {
    servos: &'a mut PixRacerProServoMonstrosity,
    current_values: [f32; NUM_HW_CHANNELS],
    enabled_chan_mask: u16,
    max_duty_counts: [u16; NUM_HW_CHANNELS],
}

impl<'a> BoardPwmDriver<'a> {
    pub fn new(servos: &'a mut PixRacerProServoMonstrosity) -> Self {
        let mut max_duty_counts = [0u16; NUM_HW_CHANNELS];
        for i in 0..NUM_HW_CHANNELS {
            max_duty_counts[i] = servos.max_duty_cycle(i);
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

impl<'a> PwmDriver for BoardPwmDriver<'a> {
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
        for i in 0..self.servos.len() {
            self.servos.enable(i).map_err(|_| PwmError::GenericError)?;
        }
        Ok(())
    }

    fn disable_all(&mut self) {
        for i in 0..self.servos.len() {
            self.servos.disable(i);
        }
    }

    fn set_duty_cycle(&mut self, channel: usize, duty: u16) -> Result<(), PwmError> {
        if channel >= NUM_HW_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        let pwm_us = Self::duty_u16_to_pwm_us(duty);
        // defmt::info!("Channel: {} Duty: {} PWM_US: {}, Raw PWM: {}", channel, duty, pwm_us, raw_pwm);
        //self.current_values[channel] = pwm_us;

        let max_duty = self.max_duty_counts[channel] as f32;
        let raw_pwm = pwm_us / 2500.0 * max_duty;
        //let raw_pwm = (pwm_us - 1000.0) / 1000.0 * max_duty;
        //let raw_pwm = ((pwm_us - 1000.0) / 1000.0 * max_duty) * 1000.0 / 2500.0 + 1000.0;
        //defmt::info!("Channel: {} Duty: {} MAX_DUTY: {}, PWM_OUT: {}", channel, duty, max_duty, raw_pwm);

        self.servos
            .set_duty_cycle(channel, raw_pwm as u16)
            .map_err(|_| PwmError::GenericError)
    }

    fn flush<B: BoardTrait>(&mut self, board: &mut B) {
        // Hardware state is already applied in set_duty_cycle.
        // Telemetry publishing:
        let now_us = board.clock_micros();
        // let msg = OutputRaw {
        //     header: Header {
        //         stamp: Time {
        //             sec: (now_us / 1_000_000) as i32,
        //             nanosec: ((now_us % 1_000_000) * 1000) as u32,
        //         },
        //         frame_id: String::new(),
        //     },
        //     values: self.current_values,
        // };
        // TODO: Send via telemetry channel (similar to sim driver)
    }

    fn send_commands<B: BoardTrait>(&mut self, board: &mut B, commands_slice: &[f64]) {
        let count = commands_slice.len().min(NUM_HW_CHANNELS);
        for i in 0..count {
            let duty_u16 = (commands_slice[i].clamp(0.0, 1.0) * (u16::MAX as f64)) as u16;
            let _ = self.set_duty_cycle(i, duty_u16);
            // defmt::info!("PWM Channel {} set to duty {}", i, duty_u16);
        }
        // defmt::info!("\x1B[2J\x1B[1;1H"); // Clear terminal
        // defmt::info!("PWM commands sent: {:?}", &commands_slice[0..count]);
        self.flush(board);
    }
}
