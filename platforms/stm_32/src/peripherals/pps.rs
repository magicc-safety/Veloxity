use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;

use veloxity_core::packets;

pub static PPS_SIGNAL: Signal<CriticalSectionRawMutex, packets::PpsPacket> =
    Signal::<CriticalSectionRawMutex, packets::PpsPacket>::new();

pub struct PpsSensor {
    pub pps: ExtiInput<'static, Async>,
}

impl PpsSensor {
    pub async fn run(&mut self) {
        loop {
            self.pps.wait_for_rising_edge().await;
            let timestamp = Instant::now();
            let status = 1;
            let header = packets::RosflightPacketHeader {
                timestamp: timestamp.as_micros(),
                status,
            };
            let pps_packet = packets::PpsPacket { header };
            PPS_SIGNAL.signal(pps_packet);
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut pps: PpsSensor) {
    pps.run().await;
}
