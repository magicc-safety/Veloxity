
/*

This module uses the Newtype pattern to prevent issues with incorrect units.

More info: https://www.lurklurk.org/effective-rust/newtype.html

 */


// Length
#[derive(Default)]
pub struct MM(pub i32);
#[derive(Default)]
pub struct MMPerSec(pub i32);

#[derive(Default)]
pub struct UnsignedMM(pub i32);

#[derive(Default)]
pub struct CM(pub i32);

#[derive(Default)]
pub struct CMPerSec(pub i32);

#[derive(Default)]
pub struct UnsignedCM(pub u32);

#[derive(Default)]
pub struct UnsignedCMPerSec(pub u32);


// Angles

#[derive(Default)]
pub struct Deg(pub f32);

#[derive(Default)]
pub struct DegENeg7(pub f32); // deg*10^-7


// Time
#[derive(Default)]
pub struct UnixTimeSeconds(pub i64); // Unix time, in seconds

#[derive(Default)]
pub struct FracTime(pub u64); // Fractional time