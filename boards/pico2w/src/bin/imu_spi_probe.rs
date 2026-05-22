#![no_std]
#![no_main]

use core::fmt::{self, Write};

use cortex_m_rt::entry;
use panic_halt as _;
use pico2w::gy91::Gy91;
use rp2350_platform::hal::{
    self as rp,
    gpio::{Level, Output},
    spi::{Config as SpiConfig, Phase, Polarity, Spi},
    uart::{Config as UartConfig, Uart},
};

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

fn trace(uart: &mut Uart<'_, rp::uart::Blocking>, message: &[u8]) {
    let _ = uart.blocking_write(message);
    let _ = uart.blocking_flush();
}

fn trace_hex_byte(uart: &mut Uart<'_, rp::uart::Blocking>, value: u8) {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let bytes = [
        b'0',
        b'x',
        HEX[(value >> 4) as usize],
        HEX[(value & 0x0f) as usize],
        b'\r',
        b'\n',
    ];
    trace(uart, &bytes);
}

fn delay() {
    for _ in 0..100_000 {
        core::hint::spin_loop();
    }
}

#[entry]
fn main() -> ! {
    let peripherals = rp::init(Default::default());

    let mut uart = Uart::new_blocking(
        peripherals.UART0,
        peripherals.PIN_0,
        peripherals.PIN_1,
        UartConfig::default(),
    );

    trace(&mut uart, b"voloxide pico2w gy-91 spi probe\r\n");

    let mpu_cs = Output::new(peripherals.PIN_13, Level::High);
    let bmp_cs = Output::new(peripherals.PIN_14, Level::High);

    let mut spi_config = SpiConfig::default();
    spi_config.frequency = 1_000_000;
    spi_config.polarity = Polarity::IdleLow;
    spi_config.phase = Phase::CaptureOnFirstTransition;

    let spi = Spi::new_blocking(
        peripherals.SPI1,
        peripherals.PIN_10,
        peripherals.PIN_11,
        peripherals.PIN_12,
        spi_config,
    );

    let mut gy91 = Gy91::new(spi, mpu_cs, bmp_cs);
    match gy91.init() {
        Ok(ids) => {
            trace(&mut uart, b"mpu9250 whoami ");
            trace_hex_byte(&mut uart, ids.mpu);
            trace(&mut uart, b"bmp280 chipid ");
            trace_hex_byte(&mut uart, ids.bmp);
        }
        Err(err) => {
            let mut writer = UartWriter(&mut uart);
            let _ = writeln!(writer, "gy91 init failed: {:?}\r", err);
        }
    }

    let mut now_us = 0_u64;
    loop {
        now_us = now_us.wrapping_add(20_000);
        match gy91.sample_imu(now_us) {
            Ok(imu) => {
                let mut writer = UartWriter(&mut uart);
                let _ = writeln!(
                    writer,
                    "imu seq={} accel=({:.3},{:.3},{:.3}) gyro=({:.3},{:.3},{:.3}) temp={:.2}\r",
                    imu.seq,
                    imu.accel[0],
                    imu.accel[1],
                    imu.accel[2],
                    imu.gyro[0],
                    imu.gyro[1],
                    imu.gyro[2],
                    imu.temperature
                );
            }
            Err(err) => {
                let mut writer = UartWriter(&mut uart);
                let _ = writeln!(writer, "imu sample error: {:?}\r", err);
            }
        }

        match gy91.sample_baro(now_us) {
            Ok(Some(baro)) => {
                let mut writer = UartWriter(&mut uart);
                let _ = writeln!(
                    writer,
                    "baro pressure={:.1} temp={:.2}\r",
                    baro.pressure, baro.temperature
                );
            }
            Ok(None) => {}
            Err(err) => {
                let mut writer = UartWriter(&mut uart);
                let _ = writeln!(writer, "baro sample error: {:?}\r", err);
            }
        }

        for _ in 0..20 {
            delay();
        }
    }
}
