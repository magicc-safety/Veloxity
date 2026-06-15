pub use embassy_stm32::mode::Async;
pub use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
pub use embassy_sync::signal::Signal;
use veloxity_core::{errors, packets};

// I2C Specific
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::i2c::I2c;
use embedded_hal_async::i2c::I2c as _;

// Polled Sensors
use crate::synch_at;
use embassy_time::Duration;
use embassy_time::Timer;

// Other

pub static PITOT_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::PitotPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::PitotPacket, errors::SensorError>>::new();

pub struct Ms4525Sensor {
    pub dev: I2cDevice<
        'static,
        CriticalSectionRawMutex,
        I2c<'static, Async, embassy_stm32::i2c::mode::Master>,
    >,
}

impl Ms4525Sensor {
    pub async fn run(&mut self) {
        const ADDRESS: u8 = 0x28;
        const NO_ERROR: u8 = 0x00;

        // Start a read
        let mut data = [0u8; 2];
        if self.dev.read(ADDRESS, &mut data).await.is_err() {
            PITOT_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "MS4525 Pitot failed: reading data",
            )));
            return;
        }

        Timer::after(Duration::from_micros(2000)).await;

        // Check if read OK.
        let mut data = [0u8; 2];
        if self.dev.read(ADDRESS, &mut data).await.is_err() {
            PITOT_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "MS4525 Pitot failed: reading data",
            )));
            return;
        }

        let status = (data[0] >> 6) & 0x03;
        if status != NO_ERROR {
            PITOT_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "MS4525 Pitot failed: bad status",
            )));
            return;
        }

        let sample_period_us = Duration::from_hz(100).as_micros();
        let loop_period = Duration::from_hz(400);
        const PMAX: f64 = 6894.76; // (=-pmin) Pa
        let mut sum_pressure: u32 = 0;
        let mut sum_temperature: u32 = 0;
        let mut sum_count: u32 = 0;

        loop {
            let timestamp = synch_at(loop_period);
            Timer::at(timestamp).await; // Wait for top of timer

            let mut data = [0u8; 4];
            if self.dev.read(ADDRESS, &mut data).await.is_err() {
                PITOT_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                    "MS4525 Pitot failed: reading data",
                )));
                continue;
            }

            let status = (data[0] >> 6) & 0x03;
            if status == NO_ERROR {
                let pressure_u32 = (u32::from(data[0]) & 0x3F) << 8 | u32::from(data[1]);
                let temperature_u32 =
                    (0xFFE0 & ((u32::from(data[2]) << 8) | u32::from(data[3]))) >> 5;

                sum_pressure = sum_pressure + pressure_u32;
                sum_temperature = sum_temperature + temperature_u32;
                sum_count = sum_count + 1;
            }

            let timestamp_us = timestamp.as_micros();

            if timestamp_us % sample_period_us == 0 {
                if sum_count > 0 {
                    let avg_pressure = ((f64::from(sum_pressure) / (sum_count as f64) - 1638.3f64)
                        / 6553.2f64
                        - 1.0f64)
                        * PMAX;
                    let avg_temperature: f64 =
                        ((200.0f64 * f64::from(sum_temperature) / (sum_count as f64)) / 2047f64)
                            - 50f64;

                    let header = packets::RosflightPacketHeader {
                        timestamp: timestamp_us,
                        status: status as u16,
                    };
                    let pitot_packet = packets::PitotPacket {
                        header,
                        differential_pressure: avg_pressure as f32,
                        temperature: avg_temperature as f32,
                        ..Default::default()
                    };
                    PITOT_SIGNAL.signal(Ok(pitot_packet)); // make data available for other tasks.
                } else {
                    PITOT_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                        "MS4525 Pitot failed: no valid data",
                    )));
                }
                // reset sum
                //previous_timestamp_us = timestamp_us;
                sum_count = 0u32;
                sum_pressure = 0u32;
                sum_temperature = 0u32;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut ms4525: Ms4525Sensor) {
    ms4525.run().await;
}
