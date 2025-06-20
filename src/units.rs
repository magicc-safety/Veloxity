// /**
// ******************************************************************************
// * File     : units.rs
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
/*

This module uses the Newtype pattern to prevent issues with incorrect units.

More info: https://www.lurklurk.org/effective-rust/newtype.html

Notably, the newtype pattern is stricter than typedefs in C. Incorrect usages of a type will be
caught by the compiler and the program will not compile.

In addition, the `From` trait can be used to perform automatic conversions where appropriate
(e.g. automatically divide by 10 to convert MM to CM).

Many of these units are units for quantities listed in the MAVLink specification:
https://mavlink.io/en/messages/common.html

TODO: Consider using `derive_alias` to get rid of a lot of the `derive` statements
(see https://docs.rs/macro_rules_attribute/0.1.3/macro_rules_attribute/macro.derive_alias.html)

TODO: Why are all of these units integer types in the original ROSFlight code?
In MAVLink many of these are listed as floats.
 */

// Length / Distance / Location
#[derive(Default, Debug)]
pub struct MM(pub i32);

#[derive(Default, Debug)]
pub struct UnsignedMM(pub i32);

#[derive(Default, Debug)]
pub struct CM(pub i32);

#[derive(Default, Debug)]
pub struct Meter(pub i32);

#[derive(Default, Debug)]
pub struct UnsignedCM(pub u32);

#[derive(Default, Debug)]
pub struct Longitude(pub i32);

#[derive(Default, Debug)]
pub struct Latitude(pub i32);

#[derive(Default, Debug)]
pub struct Height(pub i32);

#[derive(Default, Debug)]
pub struct HeightMSL(pub i32);

#[derive(Default, Debug)]
pub struct H_Acc(pub i32);

// Angles

#[derive(Default, Debug)]
pub struct Deg(pub f32);

#[derive(Default, Debug)]
pub struct DegENeg7(pub f32); // deg*10^-7

#[derive(Default, Debug)]
pub struct Radians(pub u32);

// Time
#[derive(Default, Debug)]
pub struct UnixTimeSeconds(pub i64); // Unix time, in seconds

#[derive(Default, Debug)]
pub struct FracTime(pub u64); // Fractional time

#[derive(Default, Debug)]
pub struct ROSFlightTimestamp(pub u64); // Microseconds; timestamp of last byte in message

#[derive(Default, Debug)]
pub struct TimeOfWeek(pub u64);

#[derive(Default, Debug)]
pub struct Year(pub u16);

#[derive(Default, Debug)]
pub struct Month(pub u8);

#[derive(Default, Debug)]
pub struct Day(pub u8);

#[derive(Default, Debug)]
pub struct Hour(pub u8);

#[derive(Default, Debug)]
pub struct Minute(pub u8);

#[derive(Default, Debug)]
pub struct Second(pub u8);

#[derive(Default, Debug)]
pub struct T_Acc(pub u32);

#[derive(Default, Debug)]
pub struct Nanosecond(pub i32);

// Velocities, Accelerations

#[derive(Default, Debug)]
pub struct MMPerSec(pub i32);

#[derive(Default, Debug)]
pub struct CMPerSec(pub i32);

#[derive(Default, Debug)]
pub struct UnsignedCMPerSec(pub u32);

#[derive(Default, Debug)]
pub struct MeterPerSec(pub i32);

// Other

#[derive(Default, Debug)]
pub struct Valid(pub u8);
