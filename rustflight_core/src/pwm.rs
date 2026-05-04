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
// ********************************************************

use crate::board::BoardIo;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PwmError {
    ChannelOutOfRange,
    GenericError,
}

pub trait PwmDriver {
    fn len(&self) -> usize;
    fn is_enabled(&self) -> bool;

    fn enable(&mut self, channel: usize) -> Result<(), PwmError>;
    fn disable(&mut self, channel: usize) -> Result<(), PwmError>;

    fn enable_all(&mut self) -> Result<(), PwmError>;
    fn disable_all(&mut self);

    /// Sets the duty cycle for a specific channel.
    ///
    /// # Arguments
    /// * `channel` - The output channel index (0-based).
    /// * `duty`    - The desired duty cycle, typically represented as a u16 value.
    ///             The exact interpretation (e.g., 0-ARR, 0-u16::MAX) depends
    ///             on the implementation. For simulation, we'll map 0-u16::MAX
    ///             to the simulator's expected range (e.g., 1000-2000us).
    fn set_duty_cycle(&mut self, channel: usize, duty: u16) -> Result<(), PwmError>;

    /// Sends the current state of all PWM channels to the output/simulator.
    /// This should be called once per control loop after all individual
    /// `set_duty_cycle` calls for that loop iteration are complete.
    ///
    /// # Arguments
    /// * `now_us` - The current flight controller time in microseconds for timestamping.
    fn flush<B: BoardIo>(&mut self, board: &mut B);

    // actually loops over the channels (up to self.len()) and sends pwm commands via set_duty_cycle
    fn send_commands<B: BoardIo>(&mut self, board: &mut B, commands: &[f64]);
}
