// /**
// ******************************************************************************
// * File     : dlhrl20g.rs
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
//#![allow(unused)]
use crate::synch_at;
//use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Duration;
use embassy_time::Timer;
use embassy_time::with_timeout;
use embedded_hal_async::i2c::I2c as _;
use voloxide_core::{errors, packets};

pub static PITOT_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::PitotPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::PitotPacket, errors::SensorError>>::new();

pub struct DlhrL20GSensor {
    pub dev: I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
    pub drdy: ExtiInput<'static>,
}

impl DlhrL20GSensor {
    //pub async fn run( &mut self, mut i2c: I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>)
    pub async fn run(&mut self) {
        const ADDRESS: u8 = 0x29;
        const START: u8 = 0xAC;

        let sample_period = Duration::from_hz(100);

        loop {
            let timestamp = synch_at(sample_period);
            Timer::at(timestamp).await; // Wait for top of 100 Hz timer

            let write_res = self.dev.write(ADDRESS, &[START]).await;
            if let Err(_e) = write_res {
                PITOT_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                    "Pitot failed: write_register",
                )))
            }

            if let Ok(()) =
                with_timeout(Duration::from_millis(100), self.drdy.wait_for_rising_edge()).await
            {
                let mut data = [0u8; 7];
                self.dev
                    .read(ADDRESS, &mut data)
                    .await
                    .map_err(|e| match e {
                        _ => {
                            errors::SensorError::GenericSensorError("Pitot failed: reading problem")
                        }
                    });
                let status = data[0] as u16;
                let u32_pressure =
                    u32::from(data[1]) << 16 | u32::from(data[2]) << 8 | u32::from(data[3]);
                let u32_temperature =
                    u32::from(data[4]) << 16 | u32::from(data[5]) << 8 | u32::from(data[6]);

                let fs = 5000.0; // Pa, Full Scale pressure

                let pressure = 1.25 * fs * (f64::from(u32_pressure) / 16777216.0 - 0.1); // Pa
                let temperature = 125.0 * f64::from(u32_temperature) / 16777216.0 - 40.0; // C

                //if status == 0x0040
                {
                    let header = packets::RosflightPacketHeader {
                        timestamp: timestamp.as_micros(),
                        status,
                    };
                    let pitot_packet = packets::PitotPacket {
                        header,
                        differential_pressure: pressure as f32,
                        temperature: temperature as f32,
                        ..Default::default()
                    };
                    PITOT_SIGNAL.signal(Ok(pitot_packet)); // make data available for other tasks.
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut dlhr: DlhrL20GSensor) {
    dlhr.run().await;
}
