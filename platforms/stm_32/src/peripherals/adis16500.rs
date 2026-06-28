// ******************************************************************************
// * File     : platforms/stm_32/src/peripherals/adis16500.rs
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

use crate::peripherals::pwm::{self, TimerEnum};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Instant, Timer};
use embedded_hal_async::spi::SpiDevice as _;
use veloxity_core::errors;
use veloxity_core::packets::{ImuPacket, RosflightPacketHeader};

// Device dependent
const SPI_READ: u8 = 0x00;
const SPI_WRITE: u8 = 0x80;

// Registers

// Chip ID

pub static IMU_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<ImuPacket<f64>, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<ImuPacket<f64>, errors::SensorError>>::new();

#[repr(u16)]
#[derive(Clone, Copy)]
pub enum DecRate {
    Odr2000Hz = 0, //  2000/2000-1 = 0
    Odr1000Hz = 1, //  2000/1000-1 = 1
    Odr400Hz = 4,  //  2000/400-1 = 4
}

pub struct Adis16500Sensor {
    pub dev: SpiDevice<
        'static,
        CriticalSectionRawMutex,
        spi::Spi<'static, Async, spi::mode::Master>,
        Output<'static>,
    >,
    pub dec_rate: DecRate,
    pub drdy: ExtiInput<'static, Async>,
    pub reset: Output<'static>,
    pub timer: TimerEnum,
}

const ADIS_BUFFBYTES16: usize = 22;
const ADIS_BUFFBYTES32: usize = 34;
const BURST_READ: u8 = 0x68;

impl Adis16500Sensor {
    async fn read_register(&mut self, reg_addr: u8) -> Result<u16, errors::SensorError> {
        let tx = [reg_addr | SPI_READ, 0x00];
        self.dev.write(&tx).await.map_err(|e| match e {
            _ => errors::SensorError::GenericSensorError("SPI failed: write_register"),
        })?;
        Timer::after_micros(100).await; // Required 16us delay till you can read again
        let tx = [0u8; 2];
        let mut rx = [0u8; 2];
        self.dev.transfer(&mut rx, &tx).await.map_err(|e| match e {
            _ => errors::SensorError::GenericSensorError("SPI failed: read_register"),
        })?;
        Timer::after_micros(100).await; // Required 16us delay till you can read again
        Ok(rx[1] as u16 | ((rx[0] as u16) << 8))
    }

    async fn write_register(
        &mut self,
        reg_addr: u8,
        value: u16,
    ) -> Result<(), errors::SensorError> {
        let lo = (value & 0x00FF) as u8;
        let tx = [reg_addr | SPI_WRITE, lo];
        // Soft Reset
        self.dev.write(&tx).await.map_err(|e| match e {
            _ => errors::SensorError::GenericSensorError("SPI failed: write_register"),
        })?;
        Timer::after_micros(100).await; // (100) Required 16us delay till you can read again

        let hi = ((value >> 8) & 0x00FF) as u8; //
        let tx = [(reg_addr + 1) | SPI_WRITE, hi];
        // Soft Reset
        self.dev.write(&tx).await.map_err(|e| match e {
            _ => errors::SensorError::GenericSensorError("SPI failed: write_register"),
        })?;
        Timer::after_micros(100).await; // (100) Required 16us delay till you can read again
        Ok(())
    }

    async fn initialize_sensor(&mut self) -> Result<u16, errors::SensorError> {
        self.reset.set_low(); // Hold in reset

        let _ = self.timer.enable(pwm::TimerChannel::Ch1);
        let _ = self.timer.set_duty_cycle(pwm::TimerChannel::Ch1, 500); // 500 us

        Timer::after_micros(1000).await;

        self.reset.set_high();
        Timer::after_millis(300).await; // Data sheet specifies 255ms for power-on startup empirically 300 is required

        // Check the hardware ID
        const ADIS16500_PROD_ID_ADDR: u8 = 0x72;
        const ADIS16500_PROD_ID: u16 = 0x4074;
        let prod_id = self.read_register(ADIS16500_PROD_ID_ADDR).await?;
        if prod_id != ADIS16500_PROD_ID {
            return Err(errors::SensorError::GenericSensorError(
                "ADIS16500 ID mismatch",
            ));
        }

        const ADIS16500_FILT_CTRL: u8 = 0x5C; // shift so we can or the data into the first 16 bit packet
        // [15:3] not used
        // [2:0] 0 no digital filter default)
        self.write_register(ADIS16500_FILT_CTRL, 0).await?;

        const ADIS16500_DEC_RATE: u8 = 0x64; // decimation
        // [15:11] don't care
        // [10:0] decimation rate minus 1, e.g., use 5-1 = 4

        self.write_register(ADIS16500_DEC_RATE, self.dec_rate as u16)
            .await?;

        // Miscellaneous Control Register (MSC_CTRL)
        const ADIS16500_MSC_CTRL: u8 = 0x60;
        // [15:10] 0's unused
        // [9] 1 32-bit burst data (default = 0)
        // [8] 0 burst data has gyro and accel data (default = 0)

        // [7] 1 enable linear acceleration compensation for gyros (default  0)
        // [6] 0 point of percussion alignment
        // [5] 0 always zero
        // [4] 0 wide sensor bandwidth (default)

        // [3:2] 01 Direct Input Sync Mode
        // [1] 0 falling edge sync (default =0)
        // [0] 1 active high when data is valid (default is 0, low)
        // 0b0000 0010 1000 0101 = 0x0285 // external clock
        // 0b0000 0010 1000 0001 = 0x0281 // internal clock

        if (self.dec_rate as u16) == 0 {
            // 2000Hz, sample rate, use 16-bit data mode
            self.write_register(ADIS16500_MSC_CTRL, 0x0085).await?; // values 0b0000 0000 1000 0101 = 0x0085
        } else {
            self.write_register(ADIS16500_MSC_CTRL, 0x0285).await?; // values 0b0000 0010 1000 0101 = 0x0285
        }

        const ADIS16500_DIAG_STAT: u8 = 0x02;
        let diag_stat = self.read_register(ADIS16500_DIAG_STAT).await?;

        if diag_stat != 0 {
            return Err(errors::SensorError::GenericSensorError(
                "ADIS16500 diagnostic status error",
            ));
        }
        Ok(diag_stat)
    }

    async fn read_data_16(&mut self) -> Result<[u8; ADIS_BUFFBYTES16], errors::SensorError> {
        self.drdy.wait_for_rising_edge().await;
        let mut rx = [0u8; ADIS_BUFFBYTES16];
        let mut tx = [0u8; ADIS_BUFFBYTES16];
        tx[0] = BURST_READ | SPI_READ;
        self.dev.transfer(&mut rx, &tx).await.map_err(|e| match e {
            _ => errors::SensorError::GenericSensorError("SPI failed: read_burst_data_16"),
        })?;
        Ok(rx)
    }

    async fn read_data_32(&mut self) -> Result<[u8; ADIS_BUFFBYTES32], errors::SensorError> {
        self.drdy.wait_for_rising_edge().await;
        let mut rx = [0u8; ADIS_BUFFBYTES32];
        let mut tx = [0u8; ADIS_BUFFBYTES32];
        tx[0] = BURST_READ | SPI_READ;
        self.dev.transfer(&mut rx, &tx).await.map_err(|e| match e {
            _ => errors::SensorError::GenericSensorError("SPI failed: read_burst_data_32"),
        })?;
        Ok(rx)
    }

    fn validate_data_16(
        &self,
        rx: &[u8; ADIS_BUFFBYTES16],
        data: &[i16; ADIS_BUFFBYTES16 / 2],
    ) -> Result<(), errors::SensorError> {
        let rx_u16 = rx.map(|x| x as u16);
        let rx_u16_subarray = &rx_u16[2..ADIS_BUFFBYTES16 - 2];
        let checksum: u16 = rx_u16_subarray.iter().sum();

        if checksum != data[10] as u16 {
            return Err(errors::SensorError::GenericSensorError(
                "ADIS16500 checksum mismatch",
            ));
        }

        let status: u16 = data[1] as u16;
        if status != 0 {
            return Err(errors::SensorError::GenericSensorError(
                "ADIS16500 status error",
            ));
        }

        Ok(())
    }

    fn validate_data_32(
        &self,
        rx: &[u8; ADIS_BUFFBYTES32],
        data: &[u16; ADIS_BUFFBYTES32 / 2],
    ) -> Result<(), errors::SensorError> {
        let rx_u16 = rx.map(|x| x as u16);
        let rx_u16_subarray = &rx_u16[2..ADIS_BUFFBYTES32 - 2];
        let checksum: u16 = rx_u16_subarray.iter().sum();

        if checksum != data[16] as u16 {
            return Err(errors::SensorError::GenericSensorError(
                "ADIS16500 checksum mismatch",
            ));
        }

        let status: u16 = data[1] as u16;
        if status != 0 {
            return Err(errors::SensorError::GenericSensorError(
                "ADIS16500 status error",
            ));
        }

        Ok(())
    }

    fn process_data_16(
        &self,
        data: &[i16; ADIS_BUFFBYTES16 / 2],
        timestamp: embassy_time::Instant,
    ) -> ImuPacket<f64> {
        let gyro = [
            -f64::from(data[2]) * 0.001745329251994,
            -f64::from(data[3]) * 0.001745329251994,
            f64::from(data[4]) * 0.001745329251994,
        ];
        let accel = [
            -f64::from(data[5]) * 0.01225,
            -f64::from(data[6]) * 0.01225,
            f64::from(data[7]) * 0.01225,
        ];
        let temperature = f32::from(data[8]) * 0.1; // + 273.15
        let seq = data[9] as u32; // sequence counter    
        let status: u16 = data[1] as u16;
        let header = RosflightPacketHeader {
            timestamp: timestamp.as_micros(),
            status: status,
        };
        ImuPacket {
            header,
            accel,
            gyro,
            temperature,
            seq,
        }
    }

    fn process_data_32(
        &self,
        data: &[u16; ADIS_BUFFBYTES32 / 2],
        timestamp: embassy_time::Instant,
    ) -> ImuPacket<f64> {
        let gyros_sf: f64 = 0.001745329251994f64 / f64::from(1u32 << 16);
        let gyro = [
            -f64::from(((data[2] as u32) | ((data[3] as u32) << 16)) as i32) * gyros_sf,
            -f64::from(((data[4] as u32) | ((data[5] as u32) << 16)) as i32) * gyros_sf,
            f64::from(((data[6] as u32) | ((data[7] as u32) << 16)) as i32) * gyros_sf,
        ];
        let accel_sf: f64 = 0.012254f64 / f64::from(1u32 << 16);
        let accel = [
            -f64::from(((data[8] as u32) | ((data[9] as u32) << 16)) as i32) * accel_sf,
            -f64::from(((data[10] as u32) | ((data[11] as u32) << 16)) as i32) * accel_sf,
            f64::from(((data[12] as u32) | ((data[13] as u32) << 16)) as i32) * accel_sf,
        ];
        let temperature = f32::from(data[14] as i16) * 0.1; // + 273.15
        let sample_period_us = 500u32 * ((self.dec_rate as u32) + 1);
        let seq = (data[15] as u32) * sample_period_us; // sequence counter 
        let status: u16 = data[1] as u16;
        let header = RosflightPacketHeader {
            timestamp: timestamp.as_micros(),
            status: status,
        };
        ImuPacket {
            header,
            accel,
            gyro,
            temperature,
            seq,
        }
    }

    pub async fn run(&mut self) {
        let _status = match self.initialize_sensor().await {
            Ok(status) => status,
            Err(e) => {
                IMU_SIGNAL.signal(Err(e));
                return;
            }
        };

        loop {
            if (self.dec_rate as u16) == 0 {
                // 2000Hz, sample rate, use 16-bit data mode
                let timestamp = Instant::now();

                let rx = match self.read_data_16().await {
                    Ok(data) => data,
                    Err(e) => {
                        IMU_SIGNAL.signal(Err(e));
                        continue;
                    }
                };

                let mut data = [0i16; ADIS_BUFFBYTES16 / 2];
                for (i, x) in data.iter_mut().enumerate() {
                    *x = ((rx[2 * i] as i16) << 8) | ((rx[2 * i + 1] as i16) & 0x00FF);
                }

                if let Err(e) = self.validate_data_16(&rx, &data) {
                    IMU_SIGNAL.signal(Err(e));
                    continue;
                }

                let imu_packet = self.process_data_16(&data, timestamp);
                IMU_SIGNAL.signal(Ok(imu_packet));
            } else {
                let timestamp = Instant::now();

                let rx = match self.read_data_32().await {
                    Ok(data) => data,
                    Err(e) => {
                        IMU_SIGNAL.signal(Err(e));
                        continue;
                    }
                };

                let mut data = [0u16; ADIS_BUFFBYTES32 / 2];
                for (i, x) in data.iter_mut().enumerate() {
                    *x = ((rx[2 * i] as u16) << 8) | ((rx[2 * i + 1] as u16) & 0x00FF);
                }

                if let Err(e) = self.validate_data_32(&rx, &data) {
                    IMU_SIGNAL.signal(Err(e));
                    continue;
                }

                let imu_packet = self.process_data_32(&data, timestamp);
                IMU_SIGNAL.signal(Ok(imu_packet));
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut adis: Adis16500Sensor) {
    adis.run().await;
}
