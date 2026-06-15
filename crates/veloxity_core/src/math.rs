use core::ops::{AddAssign, DivAssign, MulAssign, SubAssign};

use nalgebra::RealField;

pub trait FlightFloat:
    RealField
    + Copy
    + PartialOrd
    + Default
    + AddAssign
    + SubAssign
    + MulAssign
    + DivAssign
    + core::fmt::Debug
    + 'static
{
    fn from_f32(value: f32) -> Self;
    fn from_f64(value: f64) -> Self;
    fn from_usize(value: usize) -> Self;
    fn from_u64(value: u64) -> Self;
    fn from_i32(value: i32) -> Self;
    fn from_flight_float<T: FlightFloat>(value: T) -> Self;
    fn to_f32_lossy(self) -> f32;
    fn to_f64_lossy(self) -> f64;
    fn is_finite_value(self) -> bool;
    fn infinity() -> Self;
}

impl FlightFloat for f32 {
    fn from_f32(value: f32) -> Self {
        value
    }

    fn from_f64(value: f64) -> Self {
        value as f32
    }

    fn from_usize(value: usize) -> Self {
        value as f32
    }

    fn from_u64(value: u64) -> Self {
        value as f32
    }

    fn from_i32(value: i32) -> Self {
        value as f32
    }

    fn from_flight_float<T: FlightFloat>(value: T) -> Self {
        value.to_f32_lossy()
    }

    fn to_f32_lossy(self) -> f32 {
        self
    }

    fn to_f64_lossy(self) -> f64 {
        self as f64
    }

    fn is_finite_value(self) -> bool {
        self.is_finite()
    }

    fn infinity() -> Self {
        f32::INFINITY
    }
}

impl FlightFloat for f64 {
    fn from_f32(value: f32) -> Self {
        value as f64
    }

    fn from_f64(value: f64) -> Self {
        value
    }

    fn from_usize(value: usize) -> Self {
        value as f64
    }

    fn from_u64(value: u64) -> Self {
        value as f64
    }

    fn from_i32(value: i32) -> Self {
        value as f64
    }

    fn from_flight_float<T: FlightFloat>(value: T) -> Self {
        value.to_f64_lossy()
    }

    fn to_f32_lossy(self) -> f32 {
        self as f32
    }

    fn to_f64_lossy(self) -> f64 {
        self
    }

    fn is_finite_value(self) -> bool {
        self.is_finite()
    }

    fn infinity() -> Self {
        f64::INFINITY
    }
}

pub fn zero<R: FlightFloat>() -> R {
    <R as FlightFloat>::from_f32(0.0)
}

pub fn one<R: FlightFloat>() -> R {
    <R as FlightFloat>::from_f32(1.0)
}

pub fn pi<R: FlightFloat>() -> R {
    <R as FlightFloat>::from_f64(core::f64::consts::PI)
}

pub mod prelude {
    pub use super::{FlightFloat, one, pi, zero};
}
