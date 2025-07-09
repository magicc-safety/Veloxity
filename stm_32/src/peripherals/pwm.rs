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
use embassy_stm32::peripherals::{
    TIM1, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17, TIM2, TIM3, TIM4, TIM5, TIM8,
};
use embassy_stm32::timer::simple_pwm::SimplePwm;

pub struct ServoMonstrosity {
    pub timers: [TimerEnum; 4],
    pub chan_list: [(usize, TimerChannel); 12],
}

impl ServoMonstrosity {
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

pub enum TimerError {
    ChanNotSupported,
    TimerNotSupported,
}

impl TimerEnum {
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
