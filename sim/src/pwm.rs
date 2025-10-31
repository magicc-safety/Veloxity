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

use crate::ros_messages::{self, Header, Time, OutputRaw}; use rustflight_core::board::BoardTrait;
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
    sender: mpsc::Sender<ros_messages::OutputRaw>,
    // Internal state to hold the current PWM value (1000-2000us) for each channel
    current_values: [f32; NUM_SIM_CHANNELS],
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

        let (sender, mut receiver) = mpsc::channel::<ros_messages::OutputRaw>(10);

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
             current_values: [1000.0f32; NUM_SIM_CHANNELS],
             // enabled_mask: 0, // Initialize if using mask
        }
    }

    /// Helper to convert u16 duty (0-u16::MAX) to simulator PWM (1000-2000 us).
    fn duty_u16_to_pwm_us(duty: u16) -> f32 {
        // Map u16 range linearly to 1000-2000 us range
        let normalized_duty = duty as f32 / u16::MAX as f32;
        // Clamp normalized duty before scaling
        let clamped_normalized = normalized_duty.clamp(0.0, 1.0);
        (clamped_normalized * 1000.0) + 1000.0
    }
}

// Implement the updated PwmDriver trait
impl PwmDriver for SimPwmDriver {
    fn len(&self) -> usize {
        NUM_SIM_CHANNELS
    }

    fn enable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= NUM_SIM_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        // For the sim, 'enabling' might mean ensuring it's not stuck at a zero value
        // if the mixer could potentially output zero. Since we initialize to 1000us,
        // and disable sets to 1000us, this might be a no-op unless a specific
        // 'armed idle' value is desired upon enabling.
        println!("SimPwmDriver: Enabled channel {}", channel); // Placeholder log
        Ok(())
    }

    fn disable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= NUM_SIM_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        // Set the channel to its minimum value (1000 us) to simulate disabling it
        self.current_values[channel] = 1000.0;
        println!("SimPwmDriver: Disabled channel {} (set to 1000us)", channel);
        Ok(())
    }

    fn set_duty_cycle(&mut self, channel: usize, duty: u16) -> Result<(), PwmError> {
        if channel >= NUM_SIM_CHANNELS {
            println!("Error: PWM channel {} out of range (0-{})", channel, NUM_SIM_CHANNELS - 1);
            return Err(PwmError::ChannelOutOfRange);
        }
        // Convert u16 duty to 1000-2000 us range and store it internally
        self.current_values[channel] = Self::duty_u16_to_pwm_us(duty);
        // Optional debug print:
        // println!("Set channel {} duty {} -> stored {}us", channel, duty, self.current_values[channel]);
        Ok(())
    }

    fn flush<B: BoardTrait>(&mut self, board: &mut B) {
        let now_us = board.clock_micros();
        let now_sec = (now_us / 1_000_000) as i32;
        let now_nanosec = ((now_us % 1_000_000) * 1000) as u32;

        // Construct the message using the current internal state array
        let msg = ros_messages::OutputRaw {
            header: ros_messages::Header {
                stamp: ros_messages::Time { sec: now_sec, nanosec: now_nanosec },
                frame_id: String::new(), // Or an appropriate frame ID
            },
            // Assign the internal state directly, as it's already [f32; 14]
            values: self.current_values,
        };

        // Attempt to send the message through the MPSC channel to the async publishing task
        match self.sender.try_send(msg) {
            Ok(_) => {} // Success
            Err(mpsc::error::TrySendError::Full(_)) => {
                // Log if the channel is full (indicates sender is faster than receiver/publisher)
                println!("Warning: PWM output channel full during flush. Dropping message.");
            }
            Err(mpsc::error::TrySendError::Closed(_)) => {
                // Log if the channel is closed (indicates the publishing task stopped)
                println!("Error: PWM output channel closed during flush!");
            }
        }
    }

    fn send_commands<B: BoardTrait>(&mut self, board: &mut B, commands_slice: &[f64]) {
        let num_channels_to_write = commands_slice.len().min(self.len()); // Don't write past driver's capacity
        for i in 0..num_channels_to_write {
            // Convert mixer output (0.0 to 1.0) to u16 (0 to u16::MAX)
            let duty_u16 = (commands_slice[i].clamp(0.0, 1.0) * (u16::MAX as f64)) as u16;
            // Set duty cycle for the current channel
            if let Err(e) = self.set_duty_cycle(i, duty_u16) {
                // Handle potential error (e.g., channel out of range, though we checked)
                println!("Error setting duty cycle for channel {}: {:?}", i, e);
            }
        }

        // After setting all channels for this loop, flush/send the state
        self.flush(board);
    }
}

