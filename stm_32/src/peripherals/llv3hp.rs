// /**
// ******************************************************************************
// * File     : llv3hp.rs
// * Date     : Nov 19, 2025
// ******************************************************************************
// *
// * Copyright (c) 2025, AeroVironment, Inc.
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
//#![deny(unused)]

// Common to all Sensors
pub use embassy_sync::signal::Signal;
pub use embassy_stm32::mode::Async;
pub use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use rustflight_core::{errors, packets};

// I2C Specific
use embassy_stm32::i2c::I2c;
//use embassy_stm32::i2c::Master;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embedded_hal_async::i2c::I2c as _;
use embassy_embedded_hal::shared_bus::I2cDeviceError;

// Polled Sensors
use crate::synch_at;
use embassy_time::Duration;
use embassy_time::Timer;

// Other
//use core::f32;
//use embassy_time::Instant;
use defmt::info;

pub static RANGE_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::RangePacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::RangePacket, errors::SensorError>>::new();

pub struct Llv3hpSensor {
    pub dev: I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
}

// Control Register List - Address Definitions
const ACQ_COMMAND: u8         = 0x00; // Device command
const STATUS: u8              = 0x01; // System status
const SIG_COUNT_VAL: u8       = 0x02; // Maximum acquisition count
const ACQ_CONFIG_REG: u8      = 0x04; // Acquisition mode control
//const LEGACY_RESET_EN: u8     = 0x06; // Enables unit reset
//const SIGNAL_STRENGTH: u8     = 0x0E; // Received signal strength
const DATA: u8                  = 0x0F; // Distance measurement high byte
//const FULL_DELAY_HIGH: u8     = 0x0F; // Distance measurement high byte
//const FULL_DELAY_LOW: u8      = 0x10; // Distance measurement low byte
const REF_COUNT_VAL: u8       = 0x12; // Reference acquisition count
//const UNIT_ID_HIGH: u8        = 0x16; // Serial number high byte
//const UNIT_ID_LOW: u8         = 0x17; // Serial number low byte
//const I2C_ID_HIGH: u8         = 0x18; // Write serial number high byte for I2C address unlock
//const I2C_ID_LOW: u8          = 0x19; // Write serial number low byte for I2C address unlock
//const I2C_SEC_ADDR: u8        = 0x1A; // Write new I2C address after unlock
const THRESHOLD_BYPASS: u8    = 0x1C; // Peak detection threshold bypass
//const I2C_CONFIG: u8          = 0x1E; // Default address response control
//const PEAK_STACK_HIGH_BYTE: u8 = 0x26; // Registers read successive values from the peak stack register (high byte)
//const PEAK_STACK_LOW_BYTE: u8 = 0x27; // Registers read successive values from the peak stack register (low byte)
//const COMMAND: u8             = 0x40; // State command
const HEALTH_STATUS: u8       = 0x48; // Used to diagnose major hardware issues at initialization
//const CORR_DATA: u8           = 0x52; // Correlation record data low byte
//const CORR_DATA_SIGN: u8      = 0x53; // Correlation record data high byte
//const POWER_CONTROL: u8       = 0x65; // Power state control

impl Llv3hpSensor {

    async fn write_read(&mut self, address: u8, register: &[u8], data: &mut [u8]) -> Result<(),()> 
    {
        match self.dev.write(address, register).await {
            Err(e) => return Err(()),
            Ok(_) => {}
        }
        // Read register
        match self.dev.read(address, data).await {
            Err(e) => return Err(()),
            Ok(_) => {}           
        }
        Ok(())
    }

    pub async fn run(&mut self) {
        const ADDRESS: u8 = 0x62;

        // Check System Status Register
        let mut status = [0u8;1];
        if self.write_read(ADDRESS,&[STATUS],&mut status ).await.is_err() {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("LLV3HP Lidar failed: reading STATUS")));
            return;           
        }
        if self.write_read(ADDRESS,&[STATUS],&mut status ).await.is_err() {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("LLV3HP Lidar failed: reading STATUS")));
            return;           
        }

        if (status[0] & 0x30) != 0x30 {
           // defmt::error!("LLV3HP Lidar failed: bad STATUS {:02X}",status[0]);
           RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("LLV3HP Lidar failed: bad STATUS")));
            return;        
        }

        // Check Health Status Register
        let mut health = [0u8;1];
        if self.write_read(ADDRESS,&[HEALTH_STATUS],&mut health ).await.is_err() {
              RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("LLV3HP Lidar failed: reading HEALTH_STATUS")));
            return;           
        }
        if (health[0] & 0x17) != 0x17 {
            // defmt::error!("LLV3HP Lidar failed: bad HEALTH_STATUS {:02X}",health[0]);
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("LLV3HP Lidar failed: bad HEALTH_STATUS")));
            return;        
        }

        // This is just here for reference, use the default.
        // let configuration: u8 = 0;
        // let mut sig_count_max:u8 = 0x80;
        // let mut acq_config_reg:u8 = 0x08;
        // let mut ref_count_max:u8 = 0x05;
        // let mut threshold_bypass:u8 = 0x00;       // let (sig_count_max, acq_config_reg, ref_count_max, threshold_bypass) = match configuration {
        //     0 => (0x80, 0x08, 0x05, 0x00), // Default mode, balanced performance
        //     1 => (0x1d, 0x08, 0x03, 0x00), // Short range, high speed
        //     2 => (0x80, 0x00, 0x03, 0x00), // Default range, higher speed short range
        //     3 => (0xff, 0x08, 0x05, 0x00), // Maximum range
        //     4 => (0x80, 0x08, 0x05, 0x80), // High sensitivity detection, high erroneous measurements
        //     5 => (0x80, 0x08, 0x05, 0xb0), // Low sensitivity detection, low erroneous measurements
        //     6 => (0x04, 0x01, 0x03, 0x00), // Short range, high speed, higher error
        //     _ => (0x80, 0x08, 0x05, 0x00), // Default case (using default mode as fallback)
        // };

        let sig_count_max:u8 = 0x80;
        let acq_config_reg:u8 = 0x08;
        let ref_count_max:u8 = 0x05;
        let threshold_bypass:u8 = 0x00;

        if self.dev.write(ADDRESS, &[SIG_COUNT_VAL,sig_count_max]).await.is_err() {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("LLV3HP Lidar failed: writing SIG_COUNT_VAL")));
            return;           
        }
        if self.dev.write(ADDRESS, &[ACQ_CONFIG_REG,acq_config_reg]).await.is_err() {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("LLV3HP Lidar failed: writing ACQ_CONFIG_REG")));
            return;           
        }
        if self.dev.write(ADDRESS, &[REF_COUNT_VAL,ref_count_max]).await.is_err() {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("LLV3HP Lidar failed: writing REF_COUNT_VAL")));
            return;           
        }   
        if self.dev.write(ADDRESS, &[THRESHOLD_BYPASS,threshold_bypass]).await.is_err() {
            RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("LLV3HP Lidar failed: writing THRESHOLD_BYPASS")));
            return;           
        }

        let loop_period = Duration::from_hz(100);
        let mut last_timestamp_us = 0u64;
        loop {
            // Initiate another data read
            if self.dev.write(ADDRESS,&[ACQ_COMMAND, 0x04u8 ]).await.is_err() {
                RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("LLV3HP Lidar failed: writing ACQ_COMMAND")));          
            }

            let timestamp = synch_at(loop_period)+Duration::from_micros(5800);
            Timer::at(timestamp).await; 

             // Check System Status Register
            let mut status = [0u8;1];
            if self.write_read(ADDRESS,&[STATUS],&mut status ).await.is_err() {
                RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("LLV3HP Lidar failed: reading STATUS")));
                continue;           
            }
            if (status[0] & 0x30) != 0x30 {
                RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("LLV3HP Lidar failed: bad STATUS")));
                continue;        
            }           
  
            // Read Data
            let mut data = [0u8;2];
            if self.write_read(ADDRESS,&[DATA],&mut data ).await.is_err() {
                RANGE_SIGNAL.signal(Err(errors::SensorError::GenericSensorError("LLV3HP Lidar failed: reading DATA")));          
            } else {
                let urange = (u16::from(data[0])<<8) | u16::from(data[1]); // cm
                let range = f32::from(urange)/100f32;

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
                    range_type: packets::RangeType::Lidar
                };
                RANGE_SIGNAL.signal(Ok(range_packet)); // make data available for other tasks
                //defmt::info!("{:?} ", range_packet);
                //defmt::info!("{:?}", timestamp_us-last_timestamp_us);
                last_timestamp_us = timestamp_us;
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut llv3hp: Llv3hpSensor) {
    llv3hp.run().await;
}

