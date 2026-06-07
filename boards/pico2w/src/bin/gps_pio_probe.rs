#![no_std]
#![no_main]

use core::fmt::{self, Write};

use embassy_executor::Spawner;
use panic_halt as _;
use rp2350_platform::hal::{
    self as rp, bind_interrupts,
    peripherals::PIO0,
    pio::{InterruptHandler as PioInterruptHandler, Pio},
    pio_programs::uart::{PioUartRx, PioUartRxProgram},
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

fn delay() {
    for _ in 0..30_000 {
        core::hint::spin_loop();
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let peripherals = rp::init(Default::default());

    let mut debug_uart = Uart::new_blocking(
        peripherals.UART0,
        peripherals.PIN_0,
        peripherals.PIN_1,
        UartConfig::default(),
    );
    let mut writer = UartWriter(&mut debug_uart);

    let mut pio = Pio::new(peripherals.PIO0, Irqs);
    let rx_program = PioUartRxProgram::new(&mut pio.common);
    let mut gps_rx = PioUartRx::new(
        115_200,
        &mut pio.common,
        pio.sm0,
        peripherals.PIN_7,
        &rx_program,
    );

    let _ = writeln!(writer, "voloxide pico2w gps pio rx probe");
    let _ = writeln!(writer, "pio0 sm0 gps_rx=gp7 baud=115200 expect m100 tx -> gp7");

    let mut last = 0_u8;
    let mut total = 0_u32;
    loop {
        let mut bytes = 0_u32;
        let mut ubx_sync = 0_u32;
        for _ in 0..64 {
            let byte = gps_rx.read_u8().await;
            total = total.wrapping_add(1);
            bytes = bytes.wrapping_add(1);
            if last == 0xb5 && byte == 0x62 {
                ubx_sync = ubx_sync.wrapping_add(1);
            }
            last = byte;
        }

        let _ = writeln!(writer, "gps pio bytes={} total={} ubx_sync={}", bytes, total, ubx_sync);
        delay();
    }
}
