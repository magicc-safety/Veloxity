// /**
// ******************************************************************************
// * File     : pwm.rs
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
// THIS CODE HAS BEEN MADE SAFE BUT SAFETY HAS NOT BEEN TESTED
//use defmt::trace;
use embassy_futures::block_on;
use embassy_stm32::peripherals::{
    DMA2_CH0, DMA2_CH1, TIM1, TIM2, TIM3, TIM4, TIM5, TIM8, TIM12, TIM13, TIM14, TIM15, TIM16,
    TIM17,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel as EmbassyTimerChannel;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use voloxide_core::pwm::{
    DshotCommand, PwmOutputProtocol, effective_output_rate_hz, output_protocol_for_rate,
};

const DSHOT_FRAME_WORDS: usize = DshotCommand::FRAME_BITS + 1;

#[derive(Clone, Copy)]
pub enum PwmTimerBlockKind {
    StandardOnly,
    DshotCapable,
}

impl PwmTimerBlockKind {
    fn supports(self, protocol: PwmOutputProtocol) -> bool {
        match (self, protocol) {
            (_, PwmOutputProtocol::StandardPwm) => true,
            (PwmTimerBlockKind::DshotCapable, PwmOutputProtocol::Dshot) => true,
            (PwmTimerBlockKind::StandardOnly, PwmOutputProtocol::Dshot) => false,
        }
    }
}

pub struct ServoMonstrosity {
    pub timers: [TimerEnum; 4],
    pub chan_list: [(usize, TimerChannel); 12],
    timer_kinds: [PwmTimerBlockKind; 4],
    timer_protocols: [PwmOutputProtocol; 4],
    output_protocols: [PwmOutputProtocol; 12],
    output_rates_hz: [f64; 12],
    dshot_frames: [[u16; DSHOT_FRAME_WORDS]; 12],
}

impl ServoMonstrosity {
    pub fn new(timers: [TimerEnum; 4], chan_list: [(usize, TimerChannel); 12]) -> Self {
        Self::with_timer_kinds(timers, chan_list, [PwmTimerBlockKind::StandardOnly; 4])
    }

    pub fn with_timer_kinds(
        timers: [TimerEnum; 4],
        chan_list: [(usize, TimerChannel); 12],
        timer_kinds: [PwmTimerBlockKind; 4],
    ) -> Self {
        Self {
            timers,
            chan_list,
            timer_kinds,
            timer_protocols: [PwmOutputProtocol::StandardPwm; 4],
            output_protocols: [PwmOutputProtocol::StandardPwm; 12],
            output_rates_hz: [50.0; 12],
            dshot_frames: [[0; DSHOT_FRAME_WORDS]; 12],
        }
    }

    pub fn len(&mut self) -> usize {
        self.chan_list.len()
    }
    pub fn enable(&mut self, ch: usize) -> Result<(), TimerError> {
        let (ix, chan) = self.chan_list[ch];
        self.timers[ix].enable(chan)
    }
    pub fn disable(&mut self, ch: usize) -> Result<(), TimerError> {
        let (ix, chan) = self.chan_list[ch];
        //trace!(
        //    "PWM: Accessing index {}, array len: {}",
        //    ix,
        //    self.chan_list.len()
        //);
        self.timers[ix].disable(chan)
    }
    pub fn set_duty_cycle(&mut self, ch: usize, duty: u16) -> Result<(), TimerError> {
        let (ix, chan) = self.chan_list[ch];
        self.timers[ix].set_duty_cycle(chan, duty)
    }

    pub fn configure_output_rates(&mut self, rates_hz: &[f64]) -> Result<(), TimerError> {
        let mut timer_configs = [None; 4];
        for (output, rate) in rates_hz.iter().take(self.chan_list.len()).enumerate() {
            let (timer_index, _) = self.chan_list[output];
            let protocol = output_protocol_for_rate(*rate).map_err(|_| TimerError::InvalidRate)?;
            let effective_rate =
                effective_output_rate_hz(*rate).map_err(|_| TimerError::InvalidRate)?;
            if !self.timer_kinds[timer_index].supports(protocol) {
                return Err(TimerError::UnsupportedProtocol);
            }
            self.output_rates_hz[output] = effective_rate;
            self.output_protocols[output] = protocol;
            timer_configs[timer_index] = Some((protocol, rate_to_hz(effective_rate)?));
        }

        for (timer_index, (timer, config)) in self.timers.iter_mut().zip(timer_configs).enumerate()
        {
            if let Some((protocol, rate_hz)) = config {
                self.timer_protocols[timer_index] = protocol;
                timer.set_frequency_hz(rate_hz);
            }
        }

        Ok(())
    }

    pub fn output_protocol(&self, ch: usize) -> Result<PwmOutputProtocol, TimerError> {
        self.output_protocols
            .get(ch)
            .copied()
            .ok_or(TimerError::ChanNotSupported)
    }

    pub fn send_normalized_commands(&mut self, commands: &[f64]) -> Result<(), TimerError> {
        let count = commands.len().min(self.chan_list.len());
        for output in 0..count {
            match self.output_protocols[output] {
                PwmOutputProtocol::StandardPwm => {
                    let duty = self.standard_pwm_duty(output, commands[output])?;
                    self.set_duty_cycle(output, duty)?;
                }
                PwmOutputProtocol::Dshot => {
                    self.prepare_dshot_frame(output, commands[output])?;
                    return Err(TimerError::UnsupportedProtocol);
                }
            }
        }
        Ok(())
    }

    fn prepare_dshot_frame(&mut self, output: usize, command: f64) -> Result<(), TimerError> {
        let (timer_index, _) = self.chan_list[output];
        let max_duty = self.timers[timer_index].max_duty_cycle();
        self.dshot_frames[output] =
            dshot_waveform(DshotCommand::from_normalized(command), max_duty);
        Ok(())
    }

    fn standard_pwm_duty(&self, output: usize, command: f64) -> Result<u16, TimerError> {
        let (timer_index, _) = self.chan_list[output];
        Ok(standard_pwm_duty(
            command,
            self.output_rates_hz[output],
            self.timers[timer_index].max_duty_cycle(),
        )?)
    }
}

pub struct PixRacerProServoMonstrosity {
    pub timers: [TimerEnum; 3],
    pub chan_list: [(usize, TimerChannel); 7],
    timer_kinds: [PwmTimerBlockKind; 3],
    timer_dmas: [Option<DshotDma>; 3],
    timer_protocols: [PwmOutputProtocol; 3],
    output_protocols: [PwmOutputProtocol; 7],
    output_rates_hz: [f64; 7],
    dshot_frames: [[u16; DSHOT_FRAME_WORDS]; 7],
}

impl PixRacerProServoMonstrosity {
    pub fn new(timers: [TimerEnum; 3], chan_list: [(usize, TimerChannel); 7]) -> Self {
        Self::with_timer_kinds(timers, chan_list, [PwmTimerBlockKind::StandardOnly; 3])
    }

    pub fn with_timer_kinds(
        timers: [TimerEnum; 3],
        chan_list: [(usize, TimerChannel); 7],
        timer_kinds: [PwmTimerBlockKind; 3],
    ) -> Self {
        Self::with_timer_kinds_and_dma(timers, chan_list, timer_kinds, [const { None }; 3])
    }

    pub fn with_timer_kinds_and_dma(
        timers: [TimerEnum; 3],
        chan_list: [(usize, TimerChannel); 7],
        timer_kinds: [PwmTimerBlockKind; 3],
        timer_dmas: [Option<DshotDma>; 3],
    ) -> Self {
        Self {
            timers,
            chan_list,
            timer_kinds,
            timer_dmas,
            timer_protocols: [PwmOutputProtocol::StandardPwm; 3],
            output_protocols: [PwmOutputProtocol::StandardPwm; 7],
            output_rates_hz: [50.0; 7],
            dshot_frames: [[0; DSHOT_FRAME_WORDS]; 7],
        }
    }

    pub fn len(&mut self) -> usize {
        self.chan_list.len()
    }
    pub fn enable(&mut self, ch: usize) -> Result<(), TimerError> {
        let (ix, chan) = self.chan_list[ch];
        self.timers[ix].enable(chan)
    }
    pub fn disable(&mut self, ch: usize) -> Result<(), TimerError> {
        let (ix, chan) = self.chan_list[ch];
        //trace!(
        //    "PWM: Accessing index {}, array len: {}",
        //    ix,
        //    self.chan_list.len()
        //);
        self.timers[ix].disable(chan)
    }
    pub fn set_duty_cycle(&mut self, ch: usize, duty: u16) -> Result<(), TimerError> {
        let (ix, chan) = self.chan_list[ch];
        self.timers[ix].set_duty_cycle(chan, duty)
    }

    pub fn configure_output_rates(&mut self, rates_hz: &[f64]) -> Result<(), TimerError> {
        let mut timer_configs = [None; 3];
        for (output, rate) in rates_hz.iter().take(self.chan_list.len()).enumerate() {
            let (timer_index, _) = self.chan_list[output];
            let protocol = output_protocol_for_rate(*rate).map_err(|_| TimerError::InvalidRate)?;
            let effective_rate =
                effective_output_rate_hz(*rate).map_err(|_| TimerError::InvalidRate)?;
            if !self.timer_kinds[timer_index].supports(protocol) {
                return Err(TimerError::UnsupportedProtocol);
            }
            self.output_rates_hz[output] = effective_rate;
            self.output_protocols[output] = protocol;
            timer_configs[timer_index] = Some((protocol, rate_to_hz(effective_rate)?));
        }

        for (timer_index, (timer, config)) in self.timers.iter_mut().zip(timer_configs).enumerate()
        {
            if let Some((protocol, rate_hz)) = config {
                self.timer_protocols[timer_index] = protocol;
                timer.set_frequency_hz(rate_hz);
            }
        }

        Ok(())
    }

    pub fn output_protocol(&self, ch: usize) -> Result<PwmOutputProtocol, TimerError> {
        self.output_protocols
            .get(ch)
            .copied()
            .ok_or(TimerError::ChanNotSupported)
    }

    pub fn send_normalized_commands(&mut self, commands: &[f64]) -> Result<(), TimerError> {
        let count = commands.len().min(self.chan_list.len());
        for output in 0..count {
            match self.output_protocols[output] {
                PwmOutputProtocol::StandardPwm => {
                    let duty = self.standard_pwm_duty(output, commands[output])?;
                    self.set_duty_cycle(output, duty)?;
                }
                PwmOutputProtocol::Dshot => {
                    self.prepare_dshot_frame(output, commands[output])?;
                    self.emit_dshot_frame(output)?;
                }
            }
        }
        Ok(())
    }

    pub fn max_duty_cycle(&self, ch: usize) -> u16 {
        let (ix, _chan) = self.chan_list[ch];
        self.timers[ix].max_duty_cycle()
    }

    fn prepare_dshot_frame(&mut self, output: usize, command: f64) -> Result<(), TimerError> {
        let (timer_index, _) = self.chan_list[output];
        let max_duty = self.timers[timer_index].max_duty_cycle();
        self.dshot_frames[output] =
            dshot_waveform(DshotCommand::from_normalized(command), max_duty);
        Ok(())
    }

    fn standard_pwm_duty(&self, output: usize, command: f64) -> Result<u16, TimerError> {
        let (timer_index, _) = self.chan_list[output];
        Ok(standard_pwm_duty(
            command,
            self.output_rates_hz[output],
            self.timers[timer_index].max_duty_cycle(),
        )?)
    }

    fn emit_dshot_frame(&mut self, output: usize) -> Result<(), TimerError> {
        let (timer_index, channel) = self.chan_list[output];
        let Some(dma) = self.timer_dmas[timer_index].as_mut() else {
            return Err(TimerError::UnsupportedProtocol);
        };

        block_on(dma.emit(
            &mut self.timers[timer_index],
            channel,
            &self.dshot_frames[output],
        ))
    }
}

pub enum DshotDma {
    Dma2Ch0(DMA2_CH0),
    Dma2Ch1(DMA2_CH1),
}

impl DshotDma {
    async fn emit(
        &mut self,
        timer: &mut TimerEnum,
        channel: TimerChannel,
        waveform: &[u16],
    ) -> Result<(), TimerError> {
        let channel = channel.into_embassy();
        match (timer, self) {
            (TimerEnum::TIM1(timer), DshotDma::Dma2Ch0(dma)) => {
                timer.waveform_up(dma, channel, waveform).await;
                Ok(())
            }
            (TimerEnum::TIM1(timer), DshotDma::Dma2Ch1(dma)) => {
                timer.waveform_up(dma, channel, waveform).await;
                Ok(())
            }
            (TimerEnum::TIM2(timer), DshotDma::Dma2Ch0(dma)) => {
                timer.waveform_up(dma, channel, waveform).await;
                Ok(())
            }
            (TimerEnum::TIM2(timer), DshotDma::Dma2Ch1(dma)) => {
                timer.waveform_up(dma, channel, waveform).await;
                Ok(())
            }
            (TimerEnum::TIM4(timer), DshotDma::Dma2Ch0(dma)) => {
                timer.waveform_up(dma, channel, waveform).await;
                Ok(())
            }
            (TimerEnum::TIM4(timer), DshotDma::Dma2Ch1(dma)) => {
                timer.waveform_up(dma, channel, waveform).await;
                Ok(())
            }
            _ => Err(TimerError::UnsupportedProtocol),
        }
    }
}

pub enum TimerEnum {
    TIM1(SimplePwm<'static, TIM1>),
    TIM2(SimplePwm<'static, TIM2>),
    TIM3(SimplePwm<'static, TIM3>),
    TIM4(SimplePwm<'static, TIM4>),
    TIM5(SimplePwm<'static, TIM5>),
    // 6 and 7 are not PWM capable
    TIM8(SimplePwm<'static, TIM8>),
    TIM12(SimplePwm<'static, TIM12>),
    TIM13(SimplePwm<'static, TIM13>),
    TIM14(SimplePwm<'static, TIM14>),
    TIM15(SimplePwm<'static, TIM15>),
    TIM16(SimplePwm<'static, TIM16>),
    TIM17(SimplePwm<'static, TIM17>),
}

#[derive(Clone, Copy)]
pub enum TimerChannel {
    Ch1,
    Ch2,
    Ch3,
    Ch4,
}

impl TimerChannel {
    fn into_embassy(self) -> EmbassyTimerChannel {
        match self {
            TimerChannel::Ch1 => EmbassyTimerChannel::Ch1,
            TimerChannel::Ch2 => EmbassyTimerChannel::Ch2,
            TimerChannel::Ch3 => EmbassyTimerChannel::Ch3,
            TimerChannel::Ch4 => EmbassyTimerChannel::Ch4,
        }
    }
}

pub enum TimerError {
    ChanNotSupported,
    TimerNotSupported,
    InvalidRate,
    UnsupportedProtocol,
}

fn rate_to_hz(rate: f64) -> Result<u32, TimerError> {
    if !rate.is_finite() || rate <= 0.0 || rate > u32::MAX as f64 {
        return Err(TimerError::TimerNotSupported);
    }

    Ok((rate + 0.5) as u32)
}

fn dshot_waveform(command: DshotCommand, max_duty: u16) -> [u16; DSHOT_FRAME_WORDS] {
    let high = ((max_duty as u32 * 3) / 4) as u16;
    let low = ((max_duty as u32 * 3) / 8) as u16;
    let frame = command.frame();
    let mut waveform = [0u16; DSHOT_FRAME_WORDS];

    for (bit, slot) in waveform
        .iter_mut()
        .take(DshotCommand::FRAME_BITS)
        .enumerate()
    {
        *slot = if DshotCommand::bit_is_high(frame, bit) {
            high
        } else {
            low
        };
    }

    waveform[DshotCommand::FRAME_BITS] = 0;
    waveform
}

fn standard_pwm_duty(command: f64, rate_hz: f64, max_duty: u16) -> Result<u16, TimerError> {
    if !rate_hz.is_finite() || rate_hz <= 0.0 {
        return Err(TimerError::InvalidRate);
    }

    let pulse_us = command.clamp(0.0, 1.0) * 1000.0 + 1000.0;
    let period_us = 1_000_000.0 / rate_hz;
    let raw = pulse_us / period_us * max_duty as f64;
    Ok(raw.clamp(0.0, max_duty as f64) as u16)
}

impl TimerEnum {
    pub fn set_frequency_hz(&mut self, rate_hz: u32) {
        match self {
            TimerEnum::TIM1(timer) => timer.set_frequency(Hertz::hz(rate_hz)),
            TimerEnum::TIM2(timer) => timer.set_frequency(Hertz::hz(rate_hz)),
            TimerEnum::TIM3(timer) => timer.set_frequency(Hertz::hz(rate_hz)),
            TimerEnum::TIM4(timer) => timer.set_frequency(Hertz::hz(rate_hz)),
            TimerEnum::TIM5(timer) => timer.set_frequency(Hertz::hz(rate_hz)),
            TimerEnum::TIM8(timer) => timer.set_frequency(Hertz::hz(rate_hz)),
            TimerEnum::TIM12(timer) => timer.set_frequency(Hertz::hz(rate_hz)),
            TimerEnum::TIM13(timer) => timer.set_frequency(Hertz::hz(rate_hz)),
            TimerEnum::TIM14(timer) => timer.set_frequency(Hertz::hz(rate_hz)),
            TimerEnum::TIM15(timer) => timer.set_frequency(Hertz::hz(rate_hz)),
            TimerEnum::TIM16(timer) => timer.set_frequency(Hertz::hz(rate_hz)),
            TimerEnum::TIM17(timer) => timer.set_frequency(Hertz::hz(rate_hz)),
        }
    }

    pub fn enable(&mut self, channel: TimerChannel) -> Result<(), TimerError> {
        match self {
            TimerEnum::TIM1(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().enable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().enable();
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().enable();
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().enable();
                    Ok(())
                }
            },
            TimerEnum::TIM2(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().enable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().enable();
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().enable();
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().enable();
                    Ok(())
                }
            },
            TimerEnum::TIM3(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().enable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().enable();
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().enable();
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().enable();
                    Ok(())
                }
            },
            TimerEnum::TIM4(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().enable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().enable();
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().enable();
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().enable();
                    Ok(())
                }
            },
            TimerEnum::TIM5(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().enable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().enable();
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().enable();
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().enable();
                    Ok(())
                }
            },
            TimerEnum::TIM8(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().enable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().enable();
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().enable();
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().enable();
                    Ok(())
                }
            },
            TimerEnum::TIM12(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().enable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().enable();
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM13(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().enable();
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM14(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().enable();
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM15(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().enable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().enable();
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM16(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().enable();
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM17(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().enable();
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
        }
    }
    pub fn disable(&mut self, channel: TimerChannel) -> Result<(), TimerError> {
        match self {
            TimerEnum::TIM1(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().disable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().disable();
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().disable();
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().disable();
                    Ok(())
                }
            },
            TimerEnum::TIM2(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().disable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().disable();
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().disable();
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().disable();
                    Ok(())
                }
            },
            TimerEnum::TIM3(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().disable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().disable();
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().disable();
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().disable();
                    Ok(())
                }
            },
            TimerEnum::TIM4(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().disable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().disable();
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().disable();
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().disable();
                    Ok(())
                }
            },
            TimerEnum::TIM5(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().disable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().disable();
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().disable();
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().disable();
                    Ok(())
                }
            },
            TimerEnum::TIM8(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().disable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().disable();
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().disable();
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().disable();
                    Ok(())
                }
            },
            TimerEnum::TIM12(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().disable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().disable();
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM13(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().disable();
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM14(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().disable();
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM15(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().disable();
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().disable();
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM16(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().disable();
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM17(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().disable();
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
        }
    }

    pub fn max_duty_cycle(&self) -> u16 {
        match self {
            TimerEnum::TIM1(timer) => timer.max_duty_cycle(),
            TimerEnum::TIM2(timer) => timer.max_duty_cycle(),
            TimerEnum::TIM3(timer) => timer.max_duty_cycle(),
            TimerEnum::TIM4(timer) => timer.max_duty_cycle(),
            TimerEnum::TIM5(timer) => timer.max_duty_cycle(),
            TimerEnum::TIM8(timer) => timer.max_duty_cycle(),
            TimerEnum::TIM12(timer) => timer.max_duty_cycle(),
            TimerEnum::TIM13(timer) => timer.max_duty_cycle(),
            TimerEnum::TIM14(timer) => timer.max_duty_cycle(),
            TimerEnum::TIM15(timer) => timer.max_duty_cycle(),
            TimerEnum::TIM16(timer) => timer.max_duty_cycle(),
            TimerEnum::TIM17(timer) => timer.max_duty_cycle(),
        }
    }

    pub fn set_duty_cycle(&mut self, channel: TimerChannel, duty: u16) -> Result<(), TimerError> {
        match self {
            TimerEnum::TIM1(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().set_duty_cycle(duty);
                    Ok(())
                }
            },
            TimerEnum::TIM2(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().set_duty_cycle(duty);
                    Ok(())
                }
            },
            TimerEnum::TIM3(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().set_duty_cycle(duty);
                    Ok(())
                }
            },
            TimerEnum::TIM4(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().set_duty_cycle(duty);
                    Ok(())
                }
            },
            TimerEnum::TIM5(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().set_duty_cycle(duty);
                    Ok(())
                }
            },
            TimerEnum::TIM8(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch3 => {
                    timer.ch3().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch4 => {
                    timer.ch4().set_duty_cycle(duty);
                    Ok(())
                }
            },
            TimerEnum::TIM12(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().set_duty_cycle(duty);
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM13(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().set_duty_cycle(duty);
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM14(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().set_duty_cycle(duty);
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM15(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().set_duty_cycle(duty);
                    Ok(())
                }
                TimerChannel::Ch2 => {
                    timer.ch2().set_duty_cycle(duty);
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM16(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().set_duty_cycle(duty);
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
            TimerEnum::TIM17(timer) => match channel {
                TimerChannel::Ch1 => {
                    timer.ch1().set_duty_cycle(duty);
                    Ok(())
                }
                _ => Err(TimerError::ChanNotSupported),
            },
        }
    }
}
