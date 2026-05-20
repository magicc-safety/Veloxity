use crate::ros_messages;
use voloxide_core::board::BoardIo;
use voloxide_core::pwm::{
    PwmDriver, PwmError, PwmOutputProtocol, effective_output_rate_hz, output_protocol_for_rate,
};

use cdr::{CdrLe, Infinite};
use tokio::sync::mpsc;

use zenoh::bytes::{Encoding, ZBytes};
use zenoh::session::Session;

const NUM_SIM_CHANNELS: usize = 14; // Match OutputRaw array size

/// Simulator implementation of the PwmDriver trait.
#[derive(Clone)]
pub struct SimPwmDriver {
    sender: mpsc::Sender<ros_messages::PwmOutput>,
    // Internal state to hold the current PWM value (1000-2000us) for each channel
    current_values: [u16; NUM_SIM_CHANNELS],
    output_rates_hz: [f64; NUM_SIM_CHANNELS],
    output_protocols: [PwmOutputProtocol; NUM_SIM_CHANNELS],
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
            output_rates_hz: [50.0; NUM_SIM_CHANNELS],
            output_protocols: [PwmOutputProtocol::StandardPwm; NUM_SIM_CHANNELS],
            // enabled_mask: 0, // Initialize if using mask
        }
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
            let _ = self.disable(i);
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
        Ok(())
    }

    fn disable(&mut self, channel: usize) -> Result<(), PwmError> {
        if channel >= NUM_SIM_CHANNELS {
            return Err(PwmError::ChannelOutOfRange);
        }
        // Set to 1000us (disarmed/min throttle)
        self.current_values[channel] = 1000;
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

    fn flush<B: BoardIo>(&mut self, board: &mut B) {
        let now_us = board.clock_micros();
        let now_sec = (now_us / 1_000_000) as i32;
        let now_nanosec = ((now_us % 1_000_000) * 1000) as u32;

        let msg = ros_messages::PwmOutput {
            header: ros_messages::Header {
                stamp: ros_messages::Time {
                    sec: now_sec,
                    nanosec: now_nanosec,
                },
                frame_id: String::from(""),
            },
            values: self.current_values,
        };

        let _ = self.sender.try_send(msg);
    }

    fn configure_output_rates(&mut self, rates_hz: &[f64]) -> Result<(), PwmError> {
        for (index, rate) in rates_hz.iter().take(NUM_SIM_CHANNELS).enumerate() {
            self.output_protocols[index] = output_protocol_for_rate(*rate)?;
            self.output_rates_hz[index] = effective_output_rate_hz(*rate)?;
        }
        Ok(())
    }

    fn send_commands<B: BoardIo>(
        &mut self,
        board: &mut B,
        commands_slice: &[f64],
    ) -> Result<(), PwmError> {
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
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use voloxide_core::{board::BoardIo, errors};

    struct TestBoard {
        elapsed_us: u64,
    }

    impl BoardIo for TestBoard {
        fn serial_rx_read(&mut self, _buf: &mut [u8]) -> Option<Result<usize, errors::TelemError>> {
            None
        }

        fn serial_tx_write(&mut self, bytes: &[u8]) -> Option<Result<usize, errors::TelemError>> {
            Some(Ok(bytes.len()))
        }

        fn clock_millis(&self) -> u32 {
            Duration::from_micros(self.elapsed_us).as_millis() as u32
        }

        fn clock_micros(&self) -> u64 {
            self.elapsed_us
        }
    }

    #[test]
    fn send_commands_scales_clamps_and_publishes_pwm_output() {
        let (sender, mut receiver) = mpsc::channel(1);
        let mut driver = SimPwmDriver {
            sender,
            current_values: [1000u16; NUM_SIM_CHANNELS],
            output_rates_hz: [50.0; NUM_SIM_CHANNELS],
            output_protocols: [PwmOutputProtocol::StandardPwm; NUM_SIM_CHANNELS],
        };
        let mut board = TestBoard {
            elapsed_us: 1_234_567,
        };

        driver
            .send_commands(&mut board, &[-0.25, 0.0, 0.5, 1.0, 1.25])
            .unwrap();

        let msg = receiver.try_recv().expect("PWM output should be queued");
        assert_eq!(msg.header.stamp.sec, 1);
        assert_eq!(msg.header.stamp.nanosec, 234_567_000);
        assert_eq!(msg.values[0], 1000);
        assert_eq!(msg.values[1], 1000);
        assert_eq!(msg.values[2], 1500);
        assert_eq!(msg.values[3], 2000);
        assert_eq!(msg.values[4], 2000);
        assert!(msg.values[5..].iter().all(|value| *value == 1000));
    }

    #[test]
    fn disable_sets_channel_to_minimum_pwm() {
        let (sender, _receiver) = mpsc::channel(1);
        let mut driver = SimPwmDriver {
            sender,
            current_values: [1500u16; NUM_SIM_CHANNELS],
            output_rates_hz: [50.0; NUM_SIM_CHANNELS],
            output_protocols: [PwmOutputProtocol::StandardPwm; NUM_SIM_CHANNELS],
        };

        driver.disable(3).unwrap();

        assert_eq!(driver.current_values[3], 1000);
    }

    #[test]
    fn configure_output_rates_records_mixer_rates() {
        let (sender, _receiver) = mpsc::channel(1);
        let mut driver = SimPwmDriver {
            sender,
            current_values: [1000u16; NUM_SIM_CHANNELS],
            output_rates_hz: [50.0; NUM_SIM_CHANNELS],
            output_protocols: [PwmOutputProtocol::StandardPwm; NUM_SIM_CHANNELS],
        };

        driver
            .configure_output_rates(&[490.0, 490.0, 50.0])
            .unwrap();

        assert_eq!(driver.output_rates_hz[0], 490.0);
        assert_eq!(driver.output_rates_hz[1], 490.0);
        assert_eq!(driver.output_rates_hz[2], 50.0);
        assert_eq!(driver.output_rates_hz[3], 50.0);
    }

    #[test]
    fn configure_output_rates_classifies_dshot_rates() {
        let (sender, _receiver) = mpsc::channel(1);
        let mut driver = SimPwmDriver {
            sender,
            current_values: [1000u16; NUM_SIM_CHANNELS],
            output_rates_hz: [50.0; NUM_SIM_CHANNELS],
            output_protocols: [PwmOutputProtocol::StandardPwm; NUM_SIM_CHANNELS],
        };

        driver.configure_output_rates(&[300_000.0]).unwrap();

        assert_eq!(driver.output_protocols[0], PwmOutputProtocol::Dshot);
        assert_eq!(driver.output_protocols[1], PwmOutputProtocol::StandardPwm);
    }

    #[test]
    fn configure_output_rates_treats_zero_as_default_standard_pwm() {
        let (sender, _receiver) = mpsc::channel(1);
        let mut driver = SimPwmDriver {
            sender,
            current_values: [1000u16; NUM_SIM_CHANNELS],
            output_rates_hz: [490.0; NUM_SIM_CHANNELS],
            output_protocols: [PwmOutputProtocol::Dshot; NUM_SIM_CHANNELS],
        };

        driver.configure_output_rates(&[0.0]).unwrap();

        assert_eq!(driver.output_protocols[0], PwmOutputProtocol::StandardPwm);
        assert_eq!(driver.output_rates_hz[0], 50.0);
        assert_eq!(driver.output_rates_hz[1], 490.0);
    }
}
