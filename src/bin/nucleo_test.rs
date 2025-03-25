#![cfg(feature = "nucleo")]
#![cfg_attr(not(feature = "use_std"), no_std)] // if the feature "use_std" is not enabled, definitely turn off the entire std environment using the compiler directive "no_std"
#![no_main]

use crate::board::nucleo::Nucleo;
use rustflight_alpha::*;

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let mut b = Nucleo::new();
    let mut rosflight = rustflight::ROSFlight::init(1000, b);
    rosflight.run();

    // Never get here. 
    loop{}
}
