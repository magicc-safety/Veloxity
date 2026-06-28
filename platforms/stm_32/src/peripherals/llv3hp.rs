// ******************************************************************************
// * File     : platforms/stm_32/src/peripherals/llv3hp.rs
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

pub static RANGE_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::RangePacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::RangePacket, errors::SensorError>>::new();

pub struct Llv3hpSensor {
    pub dev: I2cDevice<
        'static,
        CriticalSectionRawMutex,
        I2c<'static, Async, embassy_stm32::i2c::mode::Master>,
    >,
}

// Control Register List - Address Definitions
const ACQ_COMMAND: u8 = 0x00; // Device command
const STATUS: u8 = 0x01; // System status
const SIG_COUNT_VAL: u8 = 0x02; // Maximum acquisition count
const ACQ_CONFIG_REG: u8 = 0x04; // Acquisition mode control
const DATA: u8 = 0x0F; // Distance measurement high byte
const REF_COUNT_VAL: u8 = 0x12; // Reference acquisition count
const THRESHOLD_BYPASS: u8 = 0x1C; // Peak detection threshold bypass
const HEALTH_STATUS: u8 = 0x48; // Used to diagnose major hardware issues at initialization

impl Llv3hpSensor {
    async fn write_read(
        &mut self,
        address: u8,
        register: &[u8],
        data: &mut [u8],
    ) -> Result<(), ()> {
        match self.dev.write(address, register).await {
            Err(_e) => return Err(()),
            Ok(_) => {}
        }

        Timer::after(Duration::from_micros(0)).await;

        // Read register
        match self.dev.read(address, data).await {
            Err(_e) => return Err(()),
            Ok(_) => {}
        }

        Ok(())
    }

    pub async fn run(&mut self) {
        const ADDRESS: u8 = 0x62;

        // Check System Status Register
        let mut status = [0u8; 1];
        if self
            .write_read(ADDRESS, &[STATUS], &mut status)
            .await
            .is_err()
        {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "LLV3HP Lidar failed: reading STATUS",
            )));
            return;
        }
        if self
            .write_read(ADDRESS, &[STATUS], &mut status)
            .await
            .is_err()
        {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "LLV3HP Lidar failed: reading STATUS",
            )));
            return;
        }

        if (status[0] & 0x30) != 0x30 {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "LLV3HP Lidar failed: bad STATUS",
            )));
            return;
        }

        // Check Health Status Register
        let mut health = [0u8; 1];
        if self
            .write_read(ADDRESS, &[HEALTH_STATUS], &mut health)
            .await
            .is_err()
        {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "LLV3HP Lidar failed: reading HEALTH_STATUS",
            )));
            return;
        }

        if (health[0] & 0x17) != 0x17 {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "LLV3HP Lidar failed: bad HEALTH_STATUS",
            )));
            return;
        }

        let sig_count_max: u8 = 0x80;
        let acq_config_reg: u8 = 0x08;
        let ref_count_max: u8 = 0x05;
        let threshold_bypass: u8 = 0x00;

        if self
            .dev
            .write(ADDRESS, &[SIG_COUNT_VAL, sig_count_max])
            .await
            .is_err()
        {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "LLV3HP Lidar failed: writing SIG_COUNT_VAL",
            )));
            return;
        }
        if self
            .dev
            .write(ADDRESS, &[ACQ_CONFIG_REG, acq_config_reg])
            .await
            .is_err()
        {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "LLV3HP Lidar failed: writing ACQ_CONFIG_REG",
            )));
            return;
        }
        if self
            .dev
            .write(ADDRESS, &[REF_COUNT_VAL, ref_count_max])
            .await
            .is_err()
        {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "LLV3HP Lidar failed: writing REF_COUNT_VAL",
            )));
            return;
        }
        if self
            .dev
            .write(ADDRESS, &[THRESHOLD_BYPASS, threshold_bypass])
            .await
            .is_err()
        {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "LLV3HP Lidar failed: writing THRESHOLD_BYPASS",
            )));
            return;
        }

        let loop_period = Duration::from_hz(100);
        loop {
            // Initiate another data read
            if self
                .dev
                .write(ADDRESS, &[ACQ_COMMAND, 0x04u8])
                .await
                .is_err()
            {
                RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                    "LLV3HP Lidar failed: writing ACQ_COMMAND",
                )));
            }

            let timestamp = synch_at(loop_period) + Duration::from_micros(5800);
            Timer::at(timestamp).await;

            // Read Data
            let mut data = [0u8; 2];
            if self.write_read(ADDRESS, &[DATA], &mut data).await.is_err() {
                RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                    "LLV3HP Lidar failed: reading DATA",
                )));
            } else {
                let urange = (u16::from(data[0]) << 8) | u16::from(data[1]); // cm
                let range = f32::from(urange) / 100f32;

                let timestamp_us = timestamp.as_micros();

                let header = packets::RosflightPacketHeader {
                    timestamp: timestamp_us,
                    status: status[0] as u16,
                };

                let range_packet = packets::RangePacket {
                    header,
                    range,
                    min_range: 0f32,
                    max_range: 40f32,
                    range_type: packets::RangeType::Lidar,
                };
                RANGE_SIGNAL.signal(Ok(range_packet)); // make data available for other tasks
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut llv3hp: Llv3hpSensor) {
    llv3hp.run().await;
}
