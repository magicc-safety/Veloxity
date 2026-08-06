//! Standalone Pixracer Pro BMI08x accelerometer identity probe.
//!
//! Run under `probe-rs run` so the semihosting result is printed. The result is
//! also retained in `PIXRACER_BMI_ACCEL_ID` for debugger memory inspection.

#![no_main]
#![no_std]

use core::{
    arch::asm,
    sync::atomic::{AtomicU32, Ordering},
};

use panic_halt as _;
use pixracerpro::board::clock_config;
use stm_32::{
    cortex_m,
    cortex_m_rt::entry,
    embassy_stm32::{
        gpio::{Level, Output, Speed},
        mode::Blocking,
        spi::{self, Spi, mode::Master},
    },
};

const BMI08X_ACCEL_CHIP_ID_REGISTER: u8 = 0x00;
const BMI088_ACCEL_CHIP_ID: u8 = 0x1e;
const BMI085_ACCEL_CHIP_ID: u8 = 0x1f;
const SPI_READ: u8 = 0x80;

#[unsafe(no_mangle)]
pub static PIXRACER_BMI_ACCEL_ID: AtomicU32 = AtomicU32::new(u32::MAX);

fn semihost_write0(message: &'static [u8]) {
    debug_assert_eq!(message.last(), Some(&0));
    unsafe {
        asm!(
            "bkpt 0xab",
            inout("r0") 0x04usize => _,
            in("r1") message.as_ptr(),
            options(nostack)
        );
    }
}

fn read_accel_chip_id(
    spi: &mut Spi<'static, Blocking, Master>,
    chip_select: &mut Output<'static>,
) -> Result<u8, spi::Error> {
    let mut frame = [BMI08X_ACCEL_CHIP_ID_REGISTER | SPI_READ, 0, 0];
    chip_select.set_low();
    cortex_m::asm::delay(1_000);
    let result = spi.blocking_transfer_in_place(&mut frame);
    chip_select.set_high();
    result.map(|()| frame[2])
}

#[entry]
fn main() -> ! {
    let peripherals = stm_32::embassy_stm32::init(clock_config(24));

    let mut spi_config = spi::Config::default();
    spi_config.frequency = stm_32::embassy_stm32::time::mhz(2);
    spi_config.mode = spi::MODE_3;
    spi_config.bit_order = spi::BitOrder::MsbFirst;
    spi_config.miso_pull = stm_32::embassy_stm32::gpio::Pull::Up;

    let mut spi = Spi::new_blocking(
        peripherals.SPI5,
        peripherals.PF7,
        peripherals.PF9,
        peripherals.PF8,
        spi_config,
    );
    let mut chip_select = Output::new(peripherals.PF6, Level::High, Speed::Low);

    // Allow sensor power to settle. The first ID transaction switches the
    // accelerometer interface into SPI mode; the second returns the stable ID.
    cortex_m::asm::delay(40_000_000);
    let _ = read_accel_chip_id(&mut spi, &mut chip_select);
    let chip_id = read_accel_chip_id(&mut spi, &mut chip_select);

    match chip_id {
        Ok(BMI088_ACCEL_CHIP_ID) => {
            PIXRACER_BMI_ACCEL_ID.store(BMI088_ACCEL_CHIP_ID.into(), Ordering::Release);
            semihost_write0(b"Pixracer BMI08x probe: ID 0x1E => BMI088\n\0");
        }
        Ok(BMI085_ACCEL_CHIP_ID) => {
            PIXRACER_BMI_ACCEL_ID.store(BMI085_ACCEL_CHIP_ID.into(), Ordering::Release);
            semihost_write0(b"Pixracer BMI08x probe: ID 0x1F => BMI085\n\0");
        }
        Ok(other) => {
            PIXRACER_BMI_ACCEL_ID.store(other.into(), Ordering::Release);
            semihost_write0(b"Pixracer BMI08x probe: unexpected accelerometer ID\n\0");
        }
        Err(_) => {
            PIXRACER_BMI_ACCEL_ID.store(0xffff_fffe, Ordering::Release);
            semihost_write0(b"Pixracer BMI08x probe: SPI transaction failed\n\0");
        }
    }

    loop {
        cortex_m::asm::wfi();
    }
}
