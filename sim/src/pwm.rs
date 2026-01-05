// /*
// ******************************************************************************
// * File     : lib.rs
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

use std::time::Instant;

use crate::ros_messages::{self, Header, Time, PwmOutput}; use rustflight_core::board::BoardTrait;
// Ensure OutputRaw is imported
use rustflight_core::errors; // Assuming errors is in core
use rustflight_core::packets; // Assuming packets is in core
use rustflight_core::pwm::{PwmDriver, PwmError}; // Import updated trait and error
// Use the re-exported path from rustflight_core if needed by other parts of the file
// use rustflight_core::micro_algebra::stack::vector::Vector;


use cdr::{CdrLe, Infinite};
use tokio::sync::mpsc;
use tokio::net::UdpSocket;
use tokio::io::ErrorKind;

use zenoh::bytes::{Encoding, ZBytes};
use zenoh::handlers::FifoChannelHandler;
use zenoh::pubsub::{Publisher, Subscriber};
use zenoh::sample::Sample;
use zenoh::session::Session;
// use zenoh::session::Session; // Already imported via prelude

const NUM_SIM_CHANNELS: usize = 14; // Match OutputRaw array size

/// Simulator implementation of the PwmDriver trait.
#[derive(Clone)]
pub struct SimPwmDriver {
    sender: mpsc::Sender<ros_messages::PwmOutput>,
    // Internal state to hold the current PWM value (1000-2000us) for each channel
    current_values: [u16; NUM_SIM_CHANNELS],
    // Optional: Track enabled state if needed, otherwise enable/disable just sets value
    // enabled_mask: u16, // Example using a bitmask for 14 channels
}

impl SimPwmDriver {
    /// Create a new driver, taking the Zenoh session.
    pub async fn new(session: &Session) -> Self {
        let publisher = session
            .declare_publisher("sim/pwm_output")
            .encoding(Encoding::APPLICATION_OCTET_STREAM)
            .await
            .unwrap();

        let (sender, mut receiver) = mpsc::channel::<ros_messages::PwmOutput>(10);

        // Spawn a dedicated tokio task to handle publishing.
        tokio::spawn(async move {
            while let Some(msg) = receiver.recv().await {
                match cdr::serialize::<_, _, CdrLe>(&msg, Infinite) {
                    Ok(bytes) => {
                        let zb = ZBytes::from(bytes);
                        if publisher.put(zb).await.is_err() {
                            println!("Error sending PWM zbytes");
                        }
                    }
                    Err(e) => {
                        println!("Error serializing PWM message: {:?}", e);
                    }
                }
            }
            println!("PWM publishing task finished."); // Helpful for debugging shutdown
        });

        Self {
             sender,
             // Initialize all channels to the minimum value (disarmed/disabled state)
             current_values: [1000u16; NUM_SIM_CHANNELS],
             // enabled_mask: 0, // Initialize if using mask
        }
    }

    /// Helper to convert u16 duty (0-u16::MAX) to simulator PWM (1000-2000 us).
    fn duty_u16_to_normalized(duty: u16) -> f32 {
        // Map u16 range linearly to 1000-2000 us range
        let normalized_duty = duty as f32 / u16::MAX as f32;
        // Clamp normalized duty before scaling
        normalized_duty.clamp(0.0, 1.0)
    }
}

impl PwmDriver for SimPwmDriver {
    fn len(&self) -> usize {
        NUM_SIM_CHANNELS
    }

    fn enable_all(&mut self) -> Result<(), PwmError> {
        for i in 0..self.len() {
            self.enable(i)?;
        }
        
        Ok(())
    }

    fn disable_all(&mut self) {
        for i in 0..self.len() {
            self.disable(i);
        }
    }

    fn is_enabled(&self) -> bool {
        true
    }

    fn enable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= NUM_SIM_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        // Optional: Set to idle (1000us) on enable
        // self.current_values[channel] = 1000.0; 
        //println!("SimPwmDriver: Enabled channel {}", channel);
        Ok(())
    }

    fn disable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= NUM_SIM_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        // Set to 1000us (disarmed/min throttle)
        self.current_values[channel] = 1000;
        //println!("SimPwmDriver: Disabled channel {} (set to 1000us)", channel);
        Ok(())
    }

    fn set_duty_cycle(&mut self, channel: usize, duty: u16) -> Result<(), PwmError> {
        if channel >= NUM_SIM_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        
        // If something calls this with a u16 (0-65535), map it to 1000-2000us
        let normalized = (duty as f32) / (u16::MAX as f32);
        let pwm_us = 1000.0 + (normalized * 1000.0);
        
        self.current_values[channel] = pwm_us as u16;
        Ok(())
    }

    fn flush<B: BoardTrait>(&mut self, board: &mut B) {
        let now_us = board.clock_micros();
        let now_sec = (now_us / 1_000_000) as i32;
        let now_nanosec = ((now_us % 1_000_000) * 1000) as u32;

        let msg = ros_messages::PwmOutput {
            header: ros_messages::Header {
                stamp: ros_messages::Time { sec: now_sec, nanosec: now_nanosec },
                frame_id: String::from(""),
            },
            values: self.current_values,
        };
        
        let _ = self.sender.try_send(msg);
    }

    fn send_commands<B: BoardTrait>(&mut self, board: &mut B, commands_slice: &[f64]) {
        let num_channels_to_write = commands_slice.len().min(self.len());
        
        for i in 0..num_channels_to_write {
            // 1. Clamp 0.0-1.0
            let cmd_norm = commands_slice[i].clamp(0.0, 1.0);
            
            // 2. Scale to 1000-2000us
            let pwm_us = 1000.0 + (cmd_norm * 1000.0);
            
            // 3. Store as u16
            self.current_values[i] = pwm_us as u16;
        }

        self.flush(board);
    }
}