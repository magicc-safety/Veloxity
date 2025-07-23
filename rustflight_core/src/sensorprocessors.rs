// /**
// ******************************************************************************
// * File     : sensorprocessors.rs
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
// **

use crate::errors;
use crate::hlist::*;
use crate::packets::*;
use crate::params::Params;
use bitflags::bitflags;
//use defmt;

bitflags! {
    /// A bitflag for representing multiple, simultaneous calibration flags.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub struct CalibrationFlags: u16 {
        const IMU = 1 << 0; // The 1st bit (value 1)
        const BARO = 1 << 1; // The 2nd bit (value 2)
        // Add other sensors here
    }
}

// ------------------------------
// Battery Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughBatteryProcessor;
impl<'a> Func<&'a mut Option<Result<BatteryPacket, errors::SensorError>>>
    for PassthroughBatteryProcessor
{
    type Output = Option<BatteryPacket>;
    fn call(
        &self,
        arg: &'a mut Option<Result<BatteryPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Self::Output {
        if let Some(Ok(_packet)) = arg.take() {
            // do something with the parameters access...
            Some(_packet)
        } else {
            None
        }
    }
}

// ------------------------------
// IMU Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughImuProcessor;
impl<'a> Func<&'a mut Option<Result<ImuPacket, errors::SensorError>>> for PassthroughImuProcessor {
    type Output = Option<ImuPacket>;
    fn call(
        &self,
        arg: &'a mut Option<Result<ImuPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Self::Output {
        if let Some(Ok(_packet)) = arg.take() {
            // do something with the parameters access...
            //println!("Got IMU");
            //defmt::debug!("Got IMU");
            Some(_packet)
        } else {
            None
        }
    }
}

// ------------------------------
// Baro Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughBaroProcessor;
impl<'a> Func<&'a mut Option<Result<BaroPacket, errors::SensorError>>>
    for PassthroughBaroProcessor
{
    type Output = Option<BaroPacket>;
    fn call(
        &self,
        arg: &'a mut Option<Result<BaroPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Self::Output {
        if let Some(Ok(_packet)) = arg.take() {
            // do something with the parameters access...
            //println!("Got Baro");
            //defmt::debug!("Got Baro");
            Some(_packet)
        } else {
            None
        }
    }
}

// ------------------------------
// Pitot Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughPitotProcessor;
impl<'a> Func<&'a mut Option<Result<PitotPacket, errors::SensorError>>>
    for PassthroughPitotProcessor
{
    type Output = Option<PitotPacket>;
    fn call(
        &self,
        arg: &'a mut Option<Result<PitotPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Self::Output {
        if let Some(Ok(_packet)) = arg.take() {
            // do something with the parameters access...
            //println!("Got Pitot");
            //defmt::debug!("Got Pitot");
            Some(_packet)
        } else {
            None
        }
    }
}

// ------------------------------
// Mag Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughMagProcessor;
impl<'a> Func<&'a mut Option<Result<MagPacket, errors::SensorError>>> for PassthroughMagProcessor {
    type Output = Option<MagPacket>;
    fn call(
        &self,
        arg: &'a mut Option<Result<MagPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Self::Output {
        if let Some(Ok(_packet)) = arg.take() {
            // do something with the parameters access...
            //println!("Got Mag");
            //defmt::debug!("Got Mag");
            Some(_packet)
        } else {
            None
        }
    }
}

// ------------------------------
// Rc Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughRcProcessor;
impl<'a> Func<&'a mut Option<Result<RcPacket, errors::SensorError>>> for PassthroughRcProcessor {
    type Output = Option<RcPacket>;
    fn call(
        &self,
        arg: &'a mut Option<Result<RcPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Self::Output {
        if let Some(Ok(_packet)) = arg.take() {
            // do something with the parameters access...
            //println!("Got Rc");
            //defmt::debug!("Got Rc");
            Some(_packet)
        } else {
            None
        }
    }
}

// ------------------------------
// Range Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughRangeProcessor;
impl<'a> Func<&'a mut Option<Result<RangePacket, errors::SensorError>>>
    for PassthroughRangeProcessor
{
    type Output = Option<RangePacket>;
    fn call(
        &self,
        arg: &'a mut Option<Result<RangePacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Self::Output {
        if let Some(Ok(_packet)) = arg.take() {
            // do something with the parameters access...
            //println!("Got Range");
            Some(_packet)
        } else {
            None
        }
    }
}

// ------------------------------
// GNSS Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughGNSSProcessor;
impl<'a> Func<&'a mut Option<Result<GNSSPacket, errors::SensorError>>>
    for PassthroughGNSSProcessor
{
    type Output = Option<GNSSPacket>;
    fn call(
        &self,
        arg: &'a mut Option<Result<GNSSPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Self::Output {
        if let Some(Ok(_packet)) = arg.take() {
            // do something with the parameters access...
            //println!("Got GNSS");
            Some(_packet)
        } else {
            None
        }
    }
}

// ------------------------------
// PPS Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughPpsProcessor;
impl<'a> Func<&'a mut Option<Result<PpsPacket, errors::SensorError>>> for PassthroughPpsProcessor {
    type Output = Option<PpsPacket>;
    fn call(
        &self,
        arg: &'a mut Option<Result<PpsPacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Self::Output {
        if let Some(Ok(_packet)) = arg.take() {
            // do something with the parameters access...
            //println!("Got Pps");
            Some(_packet)
        } else {
            None
        }
    }
}

// ------------------------------
// Attitude Packet
// ------------------------------

#[derive(Default, Copy, Clone)]
pub struct PassthroughAttitudeProcessor;
impl<'a> Func<&'a mut Option<Result<AttitudePacket, errors::SensorError>>>
    for PassthroughAttitudeProcessor
{
    type Output = Option<AttitudePacket>;
    fn call(
        &self,
        arg: &'a mut Option<Result<AttitudePacket, errors::SensorError>>,
        flags: &mut CalibrationFlags,
        params: &mut Params,
    ) -> Self::Output {
        if let Some(Ok(_packet)) = arg.take() {
            // do something with the parameters access...
            //println!("Got Attitude");
            Some(_packet)
        } else {
            None
        }
    }
}
