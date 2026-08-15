#![no_std]
#![no_main]

use core::fmt::{self, Write};

use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use panic_halt as _;
use pico2w::gps::{UbxNavPvtParser, gps_stats, make_ubx_packet, record_gps_byte, record_nav_pvt};
use rp2350_platform::hal::{
    self as rp, bind_interrupts,
    clocks::ClockConfig,
    config::Config as HalConfig,
    peripherals::PIO0,
    pio::{InterruptHandler as PioInterruptHandler, Pio},
    pio_programs::uart::{PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram},
    uart::{Config as UartConfig, Uart},
};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

struct UartWriter<'a>(&'a mut Uart<'static, rp::uart::Blocking>);

impl Write for UartWriter<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.0
            .blocking_write(s.as_bytes())
            .map_err(|_| fmt::Error)?;
        self.0.blocking_flush().map_err(|_| fmt::Error)?;
        Ok(())
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let peripherals = rp::init(HalConfig::new(
        ClockConfig::system_freq(300_000_000).unwrap(),
    ));
    let mut debug_uart = Uart::new_blocking(
        peripherals.UART0,
        peripherals.PIN_0,
        peripherals.PIN_1,
        UartConfig::default(),
    );
    let mut writer = UartWriter(&mut debug_uart);

    let mut pio = Pio::new(peripherals.PIO0, Irqs);
    let rx_program = PioUartRxProgram::new(&mut pio.common);
    let tx_program = PioUartTxProgram::new(&mut pio.common);
    let mut gps_rx = PioUartRx::new(
        115_200,
        &mut pio.common,
        pio.sm0,
        peripherals.PIN_7,
        &rx_program,
    );
    let mut gps_tx = PioUartTx::new(
        115_200,
        &mut pio.common,
        pio.sm1,
        peripherals.PIN_6,
        &tx_program,
    );

    let _ = writeln!(writer, "veloxity pico2w gps pio nav probe");
    let _ = writeln!(writer, "gps rx=gp7 tx=gp6 baud=115200");

    configure_nav_pvt(&mut gps_tx).await;

    let mut parser = UbxNavPvtParser::new();
    let mut next_poll_us = Instant::now().as_micros();
    let mut next_report_us = Instant::now().as_micros().saturating_add(1_000_000);
    loop {
        let byte = gps_rx.read_u8().await;
        let now_us = Instant::now().as_micros();
        record_gps_byte(byte);
        if let Some(packet) = parser.feed_byte(byte, now_us) {
            record_nav_pvt();
            let _ = writeln!(
                writer,
                "navpvt fix={:?} sats={} lat={:.7} lon={:.7} h={:.1}",
                packet.fix_type, packet.num_sats, packet.lat, packet.lon, packet.height_msl
            );
        }
        if now_us >= next_poll_us {
            poll_nav_pvt(&mut gps_tx).await;
            next_poll_us = now_us.saturating_add(100_000);
        }
        if now_us >= next_report_us {
            let stats = gps_stats();
            let _ = writeln!(
                writer,
                "gps bytes={} sync={} frames={} last=0x{:08x} navpvt={}",
                stats.total_bytes,
                stats.ubx_sync,
                stats.ubx_frames,
                stats.last_frame,
                stats.nav_pvt
            );
            next_report_us = now_us.saturating_add(1_000_000);
        }
    }
}

async fn configure_nav_pvt(gps_tx: &mut PioUartTx<'static, PIO0, 1>) {
    let mut packet = [0_u8; 40];
    if let Some(len) = make_ubx_packet(0x06, 0x01, &[0x01, 0x07, 1], &mut packet) {
        write_packet(gps_tx, &packet[..len]).await;
    }
    Timer::after_millis(50).await;
    if let Some(len) = make_ubx_packet(0x06, 0x01, &[0x01, 0x07, 0, 1, 0, 0, 0, 0], &mut packet) {
        write_packet(gps_tx, &packet[..len]).await;
    }
    Timer::after_millis(50).await;
    let rate_payload = [
        100_u16.to_le_bytes()[0],
        100_u16.to_le_bytes()[1],
        1,
        0,
        0,
        0,
    ];
    if let Some(len) = make_ubx_packet(0x06, 0x08, &rate_payload, &mut packet) {
        write_packet(gps_tx, &packet[..len]).await;
    }
}

async fn poll_nav_pvt(gps_tx: &mut PioUartTx<'static, PIO0, 1>) {
    let mut packet = [0_u8; 8];
    if let Some(len) = make_ubx_packet(0x01, 0x07, &[], &mut packet) {
        write_packet(gps_tx, &packet[..len]).await;
    }
}

async fn write_packet(gps_tx: &mut PioUartTx<'static, PIO0, 1>, bytes: &[u8]) {
    for byte in bytes {
        gps_tx.write_u8(*byte).await;
    }
}
