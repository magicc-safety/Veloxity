// /**
// ******************************************************************************
// * File     : board.rs
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
#[cfg(feature = "default")]
pub mod dummy;
#[cfg(feature = "nucleo")]
pub mod nucleo;
#[cfg(feature = "nucleo")]
pub mod nucleo_config;

use crate::{errors, packets, params::Params, sensors};

/*
TODO: Should encode the priority of the packet, with 0 being highest.
*/

pub trait Board {
    /*
    TODO:
        * Check which functions actually need `&mut self` vs just passing &self
        * Check input types. Can we encode anything in Enums?
        * Check return types. Can we encode anything in Enums? For example, change booleans to enums
     */

    // Setup
    // fn init_board(&mut self);
    // fn board_reset(&mut self, bootloader: bool);

    // Clock
    // fn clock_millis(&self) -> u32;
    // fn clock_micros(&self) -> u64;
    // fn clock_delay(&self, milliseconds: u32);

    // Sensors
    // fn sensors_init(&mut self);
    // fn num_sensor_errors(&self) -> u16;

    fn imu_read(&self) -> Option<Result<packets::ImuPacket, errors::SensorError>>;
    fn mag_read(&self) -> Option<Result<packets::MagPacket, errors::SensorError>>;
    fn baro_read(&self) -> Option<Result<packets::BaroPacket, errors::SensorError>>;
    fn diff_pressure_read(&self) -> Option<Result<packets::PitotPacket, errors::SensorError>>;
    fn sonar_read(&self) -> Option<Result<packets::RangePacket, errors::SensorError>>;
    // GPS <-- remember that at some point we're going to want to distinguish between gnss and
    //         gnss_full
    fn gnss_read(&self) -> Option<Result<packets::GNSSPacket, errors::SensorError>>;
    fn battery_read(&self) -> Option<Result<packets::BatteryPacket, errors::SensorError>>;
    // fn battery_voltage_set_multiplier(&mut self, multiplier: f64);
    // fn battery_current_set_multiplier(&mut self, multiplier: f64);
    fn rc_read(&self) -> Option<Result<packets::RcPacket, errors::SensorError>>;
    fn attitude_read(&self) -> Option<Result<packets::AttitudePacket, errors::SensorError>>;
    fn serial_rx_read(&self) -> Option<Result<packets::SerialRxPacket, errors::TelemError>>;
    fn serial_tx_write(
        &self,
        bytes: &[u8],
    ) -> Option<Result<packets::SerialTxPacket, errors::TelemError>>;

    // PWM
    // fn pwm_init(&mut self, refresh_rate: u32, idle_pwm: u16);
    // fn pwm_init_multi(&mut self, rate: &[f32], channels: u32);
    // fn pwm_disable(&mut self);
    // fn pwm_write(&mut self, channel: u8, value: f32);
    // fn pwm_write_multi(&mut self, value: &[f32], channels: u32);

    // Non-volatile memory
    // fn memory_init(&mut self);
    // fn memory_read(&self, dest: &mut Params) -> bool;
    // fn memory_write(&mut self, src: &Params) -> bool;
    // fn memory_read(&self) -> bool;

    // LEDs
    // fn led0_on(&mut self);
    // fn led0_off(&mut self);
    // fn led0_toggle(&mut self);

    // fn led1_on(&mut self);
    // fn led1_off(&mut self);
    // fn led1_toggle(&mut self);

    // Backup memory
    // fn backup_memory_init(&mut self);
    // fn backup_memory_read(&self, dest: &mut [u8]) -> bool;
    // fn backup_memory_write(&mut self, src: &[u8]);
    // fn backup_memory_clear(&mut self, len: usize);
}
