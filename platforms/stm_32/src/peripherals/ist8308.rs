// ******************************************************************************
// * File     : platforms/stm_32/src/peripherals/ist8308.rs
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

pub static MAG_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::MagPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::MagPacket, errors::SensorError>>::new();

// ROSflight C compatibility scales for the Pixracer Pro IST8308 driver. The
// sensor datasheet specifies a nominal 1.5e-7 T/LSB in the configured ±500 µT
// range, but C uses these axis-specific values. Preserve them here so raw and
// calibrated measurements, including persisted calibration parameters, remain
// interchangeable between the C and Rust firmware.
const ROSFLIGHT_C_SCALE_X_T_PER_LSB: f32 = 1.515e-7;
const ROSFLIGHT_C_SCALE_YZ_T_PER_LSB: f32 = 1.1515e-7;

pub struct Ist8308Sensor {
    pub dev: I2cDevice<
        'static,
        CriticalSectionRawMutex,
        I2c<'static, Async, embassy_stm32::i2c::mode::Master>,
    >,
}

impl Ist8308Sensor {
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
        const ADDRESS: u8 = 0x0C;

        // Check device ID

        const WAI_REG: u8 = 0x00;
        let mut device_id = [0u8; 1];
        if self
            .write_read(ADDRESS, &[WAI_REG], &mut device_id)
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: reading WAI_REG",
            )));
            return;
        }
        const DEVICE_ID: u8 = 0x08;
        if device_id[0] != DEVICE_ID {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: bad device ID",
            )));
            return;
        }

        // Reset

        const CNTL3_REG: u8 = 0x32;
        const CNTL3_VAL_SRST: u8 = 1;
        if self
            .dev
            .write(ADDRESS, &[CNTL3_REG, CNTL3_VAL_SRST])
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: writing CNTL3_REG",
            )));
            return;
        }
        Timer::after(Duration::from_millis(20)).await; // allow 20 ms to reset

        //  Check status
        let mut cntrl3 = [0u8; 1];
        if self
            .write_read(ADDRESS, &[CNTL3_REG], &mut cntrl3)
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: reading CNTL3_REG",
            )));
            return;
        }
        if (cntrl3[0] & 0x01) != 0 {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: bad status CNTL3_REG",
            )));
            return;
        }

        // Configure

        // Enable DRDY (None Connected)
        const CNTL3_VAL_DRDY_EN: u8 = 1 << 3;

        if self
            .dev
            .write(ADDRESS, &[CNTL3_REG, CNTL3_VAL_DRDY_EN])
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: writing CNTL4_REG",
            )));
            return;
        }

        const CNTL4_REG: u8 = 0x34;
        const CNTL4_VAL_DYNAMIC_RANGE_500: u8 = 0;

        if self
            .dev
            .write(ADDRESS, &[CNTL4_REG, CNTL4_VAL_DYNAMIC_RANGE_500])
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: writing CNTL4_REG",
            )));
            return;
        }

        const OSRCNTL_REG: u8 = 0x41;
        const OSRCNTL_VAL_Y_16: u8 = 4 << 3;
        const OSRCNTL_VAL_XZ_16: u8 = 4;

        if self
            .dev
            .write(
                ADDRESS,
                &[OSRCNTL_REG, OSRCNTL_VAL_Y_16 | OSRCNTL_VAL_XZ_16],
            )
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: writing OSRCNTL_REG",
            )));
            return;
        }

        // Set ODR
        const CNTL2_REG: u8 = 0x31;
        const CNTL2_VAL_CONT_ODR100_MODE: u8 = 0x08; //Continuous (100Hz) mode
        if self
            .dev
            .write(ADDRESS, &[CNTL2_REG, CNTL2_VAL_CONT_ODR100_MODE])
            .await
            .is_err()
        {
            MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                "IST8308 Mag failed: writing CNTL2_REG",
            )));
            return;
        }

        let loop_period = Duration::from_hz(100);
        loop {
            let timestamp = synch_at(loop_period) + Duration::from_micros(900);
            Timer::at(timestamp).await;

            // Read Data
            const STAT1_REG: u8 = 0x10;
            let mut data = [0u8; 7];
            if self
                .write_read(ADDRESS, &[STAT1_REG], &mut data)
                .await
                .is_err()
            {
                MAG_SIGNAL.signal(Err(errors::SensorError::GenericSensorError(
                    "IST8308 Mag failed: reading STAT1_REG",
                )));
                continue;
            }

            const STAT1_VAL_DRDY: u8 = 0x01;
            let status = data[0];
            let data_ready = (status & STAT1_VAL_DRDY) != 0;
            if data_ready {
                let flux = [
                    f32::from((((data[2] as u16) << 8) | (data[1] as u16)) as i16)
                        * ROSFLIGHT_C_SCALE_X_T_PER_LSB,
                    f32::from((((data[4] as u16) << 8) | (data[3] as u16)) as i16)
                        * ROSFLIGHT_C_SCALE_YZ_T_PER_LSB,
                    // Match the ROSflight C Pixracer driver coordinate convention.
                    -f32::from((((data[6] as u16) << 8) | (data[5] as u16)) as i16)
                        * ROSFLIGHT_C_SCALE_YZ_T_PER_LSB,
                ]; // Units of Tesla

                let timestamp_us = timestamp.as_micros();

                let header = packets::RosflightPacketHeader {
                    timestamp: timestamp_us,
                    status: status as u16,
                };

                let mag_packet = packets::MagPacket {
                    header,
                    flux,
                    temperature: 0.0f32,
                };
                MAG_SIGNAL.signal(Ok(mag_packet)); // make data available for other tasks
                // previous_timestamp_us = timestamp_us;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut ist: Ist8308Sensor) {
    ist.run().await;
}
