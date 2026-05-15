#![no_std]
use embassy_time::{Duration, Instant};

pub mod peripherals;

pub fn synch_at(slot_rate: Duration) -> Instant {
    let dt = slot_rate.as_micros();
    let now = Instant::now().as_micros();
    Instant::from_micros((now / dt + 1u64) * dt)
}

//------------ Re-Exports ------------
pub use cortex_m;
pub use cortex_m_rt;
// pub use defmt;
// pub use defmt_rtt;
pub use embassy_embedded_hal;
pub use embassy_executor;
pub use embassy_futures;
pub use embassy_stm32;
pub use embassy_sync;
pub use embassy_time;
pub use embedded_hal_async;
pub use embedded_hal_nb;
pub use embedded_io_async;
// pub use panic_probe;
pub use static_cell;
