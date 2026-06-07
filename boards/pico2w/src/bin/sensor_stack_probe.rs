#![no_std]
#![no_main]

use core::{
    fmt::{self, Write},
    sync::atomic::{AtomicU32, AtomicU8, Ordering},
};

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_hal_nb::serial::Read as _;
use panic_halt as _;
use pico2w::rc_receiver::{CRSF_BAUDRATE, CrsfRcParser};
use rp2350_platform::hal::{
    self as rp, bind_interrupts,
    gpio::{Level, Output},
    peripherals::PIO0,
    pio::{InterruptHandler as PioInterruptHandler, Pio},
    pio_programs::uart::{PioUartRx, PioUartRxProgram},
    spi::{Config as SpiConfig, Phase, Polarity, Spi},
    uart::{Config as UartConfig, Uart},
};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

static GPS_TOTAL_BYTES: AtomicU32 = AtomicU32::new(0);
static GPS_UBX_SYNC: AtomicU32 = AtomicU32::new(0);
static GPS_LAST_BYTE: AtomicU8 = AtomicU8::new(0);

const ISM330DHCX_WHO_AM_I: u8 = 0x6b;
const BMP280_ID_REG: u8 = 0xd0;
const BMP280_CALIB_REG: u8 = 0x88;
const BMP280_CTRL_MEAS_REG: u8 = 0xf4;
const BMP280_CONFIG_REG: u8 = 0xf5;
const BMP280_PRESS_REG: u8 = 0xf7;

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

#[derive(Default)]
struct Calibration {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
}

fn spi_read_reg(
    spi: &mut Spi<'static, rp::peripherals::SPI1, rp::spi::Blocking>,
    cs: &mut Output<'static>,
    reg: u8,
) -> Result<u8, ()> {
    let mut bytes = [reg | 0x80, 0];
    cs.set_low();
    let result = spi.blocking_transfer_in_place(&mut bytes);
    cs.set_high();
    result.map_err(|_| ())?;
    Ok(bytes[1])
}

fn spi_read_regs(
    spi: &mut Spi<'static, rp::peripherals::SPI1, rp::spi::Blocking>,
    cs: &mut Output<'static>,
    reg: u8,
    out: &mut [u8],
) -> Result<(), ()> {
    let mut txrx = [0_u8; 32];
    if out.len() + 1 > txrx.len() {
        return Err(());
    }
    txrx[0] = reg | 0x80;
    cs.set_low();
    let result = spi.blocking_transfer_in_place(&mut txrx[..out.len() + 1]);
    cs.set_high();
    result.map_err(|_| ())?;
    out.copy_from_slice(&txrx[1..out.len() + 1]);
    Ok(())
}

fn spi_write_reg(
    spi: &mut Spi<'static, rp::peripherals::SPI1, rp::spi::Blocking>,
    cs: &mut Output<'static>,
    reg: u8,
    value: u8,
) -> Result<(), ()> {
    let mut bytes = [reg & 0x7f, value];
    cs.set_low();
    let result = spi.blocking_transfer_in_place(&mut bytes);
    cs.set_high();
    result.map_err(|_| ())
}

fn init_imu(
    spi: &mut Spi<'static, rp::peripherals::SPI1, rp::spi::Blocking>,
    imu_cs: &mut Output<'static>,
) -> Result<u8, ()> {
    let who = spi_read_reg(spi, imu_cs, 0x0f)?;
    if who != ISM330DHCX_WHO_AM_I {
        return Ok(who);
    }
    spi_write_reg(spi, imu_cs, 0x12, 0x44)?;
    spi_write_reg(spi, imu_cs, 0x10, 0xa4)?;
    spi_write_reg(spi, imu_cs, 0x11, 0xac)?;
    spi_write_reg(spi, imu_cs, 0x0b, 0x80)?;
    spi_write_reg(spi, imu_cs, 0x0d, 0x03)?;
    Ok(who)
}

fn read_imu(
    spi: &mut Spi<'static, rp::peripherals::SPI1, rp::spi::Blocking>,
    imu_cs: &mut Output<'static>,
) -> Result<([f32; 3], [f32; 3], f32), ()> {
    let mut raw = [0_u8; 14];
    spi_read_regs(spi, imu_cs, 0x20, &mut raw)?;

    let temperature_raw = i16::from_le_bytes([raw[0], raw[1]]);
    let gyro_raw = [
        i16::from_le_bytes([raw[2], raw[3]]),
        i16::from_le_bytes([raw[4], raw[5]]),
        i16::from_le_bytes([raw[6], raw[7]]),
    ];
    let accel_raw = [
        i16::from_le_bytes([raw[8], raw[9]]),
        i16::from_le_bytes([raw[10], raw[11]]),
        i16::from_le_bytes([raw[12], raw[13]]),
    ];

    const GYRO_2000DPS_TO_RAD_S: f32 = 0.07 * core::f32::consts::PI / 180.0;
    const ACCEL_16G_TO_M_S2: f32 = 0.000_488 * 9.80665;

    Ok((
        [
            accel_raw[0] as f32 * ACCEL_16G_TO_M_S2,
            accel_raw[1] as f32 * ACCEL_16G_TO_M_S2,
            accel_raw[2] as f32 * ACCEL_16G_TO_M_S2,
        ],
        [
            gyro_raw[0] as f32 * GYRO_2000DPS_TO_RAD_S,
            gyro_raw[1] as f32 * GYRO_2000DPS_TO_RAD_S,
            gyro_raw[2] as f32 * GYRO_2000DPS_TO_RAD_S,
        ],
        25.0 + temperature_raw as f32 / 256.0,
    ))
}

fn read_calibration(
    spi: &mut Spi<'static, rp::peripherals::SPI1, rp::spi::Blocking>,
    baro_cs: &mut Output<'static>,
) -> Result<Calibration, ()> {
    let mut raw = [0_u8; 24];
    spi_read_regs(spi, baro_cs, BMP280_CALIB_REG, &mut raw)?;
    Ok(Calibration {
        dig_t1: u16::from_le_bytes([raw[0], raw[1]]),
        dig_t2: i16::from_le_bytes([raw[2], raw[3]]),
        dig_t3: i16::from_le_bytes([raw[4], raw[5]]),
        dig_p1: u16::from_le_bytes([raw[6], raw[7]]),
        dig_p2: i16::from_le_bytes([raw[8], raw[9]]),
        dig_p3: i16::from_le_bytes([raw[10], raw[11]]),
        dig_p4: i16::from_le_bytes([raw[12], raw[13]]),
        dig_p5: i16::from_le_bytes([raw[14], raw[15]]),
        dig_p6: i16::from_le_bytes([raw[16], raw[17]]),
        dig_p7: i16::from_le_bytes([raw[18], raw[19]]),
        dig_p8: i16::from_le_bytes([raw[20], raw[21]]),
        dig_p9: i16::from_le_bytes([raw[22], raw[23]]),
    })
}

fn read_baro(
    spi: &mut Spi<'static, rp::peripherals::SPI1, rp::spi::Blocking>,
    baro_cs: &mut Output<'static>,
    cal: &Calibration,
) -> Result<(f32, f32), ()> {
    let mut raw = [0_u8; 6];
    spi_read_regs(spi, baro_cs, BMP280_PRESS_REG, &mut raw)?;
    let adc_p = ((raw[0] as i32) << 12) | ((raw[1] as i32) << 4) | ((raw[2] as i32) >> 4);
    let adc_t = ((raw[3] as i32) << 12) | ((raw[4] as i32) << 4) | ((raw[5] as i32) >> 4);
    Ok(compensate_bmp280(cal, adc_p, adc_t))
}

fn compensate_bmp280(cal: &Calibration, adc_p: i32, adc_t: i32) -> (f32, f32) {
    let var1 = (((adc_t >> 3) - ((cal.dig_t1 as i32) << 1)) * cal.dig_t2 as i32) >> 11;
    let var2 = (((((adc_t >> 4) - cal.dig_t1 as i32)
        * ((adc_t >> 4) - cal.dig_t1 as i32))
        >> 12)
        * cal.dig_t3 as i32)
        >> 14;
    let t_fine = var1 + var2;
    let temperature = ((t_fine * 5 + 128) >> 8) as f32 / 100.0;

    let mut p_var1 = t_fine as i64 - 128000;
    let mut p_var2 = p_var1 * p_var1 * cal.dig_p6 as i64;
    p_var2 += (p_var1 * cal.dig_p5 as i64) << 17;
    p_var2 += (cal.dig_p4 as i64) << 35;
    p_var1 =
        ((p_var1 * p_var1 * cal.dig_p3 as i64) >> 8) + ((p_var1 * cal.dig_p2 as i64) << 12);
    p_var1 = (((1_i64 << 47) + p_var1) * cal.dig_p1 as i64) >> 33;
    if p_var1 == 0 {
        return (0.0, temperature);
    }
    let mut pressure = 1048576 - adc_p as i64;
    pressure = (((pressure << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (cal.dig_p9 as i64 * (pressure >> 13) * (pressure >> 13)) >> 25;
    p_var2 = (cal.dig_p8 as i64 * pressure) >> 19;
    pressure = ((pressure + p_var1 + p_var2) >> 8) + ((cal.dig_p7 as i64) << 4);
    (pressure as f32 / 256.0, temperature)
}

#[embassy_executor::task]
async fn gps_pio_task(mut gps_rx: PioUartRx<'static, PIO0, 0>) -> ! {
    loop {
        let byte = gps_rx.read_u8().await;
        let last = GPS_LAST_BYTE.swap(byte, Ordering::Relaxed);
        GPS_TOTAL_BYTES.fetch_add(1, Ordering::Relaxed);
        if last == 0xb5 && byte == 0x62 {
            GPS_UBX_SYNC.fetch_add(1, Ordering::Relaxed);
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = rp::init(Default::default());

    let mut debug_uart = Uart::new_blocking(
        peripherals.UART0,
        peripherals.PIN_0,
        peripherals.PIN_1,
        UartConfig::default(),
    );
    let mut writer = UartWriter(&mut debug_uart);

    let mut crsf_config = UartConfig::default();
    crsf_config.baudrate = CRSF_BAUDRATE;
    let mut crsf_uart = Uart::new_blocking(
        peripherals.UART1,
        peripherals.PIN_8,
        peripherals.PIN_9,
        crsf_config,
    );

    let mut pio = Pio::new(peripherals.PIO0, Irqs);
    let gps_rx_program = PioUartRxProgram::new(&mut pio.common);
    let gps_rx = PioUartRx::new(
        115_200,
        &mut pio.common,
        pio.sm0,
        peripherals.PIN_7,
        &gps_rx_program,
    );
    let gps_task_spawned = if let Ok(token) = gps_pio_task(gps_rx) {
        spawner.spawn(token);
        true
    } else {
        false
    };

    let mut imu_cs = Output::new(peripherals.PIN_13, Level::High);
    let mut baro_cs = Output::new(peripherals.PIN_15, Level::High);

    let mut spi_config = SpiConfig::default();
    spi_config.frequency = 1_000_000;
    spi_config.polarity = Polarity::IdleLow;
    spi_config.phase = Phase::CaptureOnFirstTransition;
    let mut spi = Spi::new_blocking(
        peripherals.SPI1,
        peripherals.PIN_10,
        peripherals.PIN_11,
        peripherals.PIN_12,
        spi_config,
    );

    let _ = writeln!(writer, "voloxide pico2w sensor stack probe");
    let _ = writeln!(
        writer,
        "spi1 gp10/11/12 imu_cs=gp13 baro_cs=gp15 crsf uart1 gp8/gp9 gps pio_rx=gp7"
    );
    let _ = writeln!(writer, "gps pio task spawned={}", gps_task_spawned);

    let imu_who = init_imu(&mut spi, &mut imu_cs).unwrap_or(0);
    let _ = writeln!(writer, "imu whoami=0x{:02x}", imu_who);

    let bmp_id = spi_read_reg(&mut spi, &mut baro_cs, BMP280_ID_REG).unwrap_or(0);
    let _ = writeln!(writer, "bmp280 chipid=0x{:02x}", bmp_id);
    let baro_cal = read_calibration(&mut spi, &mut baro_cs).unwrap_or_default();
    let _ = spi_write_reg(&mut spi, &mut baro_cs, BMP280_CONFIG_REG, 0xa0);
    let _ = spi_write_reg(&mut spi, &mut baro_cs, BMP280_CTRL_MEAS_REG, 0x4f);

    let mut crsf_parser = CrsfRcParser::new();
    let mut crsf_frames = 0_u32;
    let mut last_gps_total = 0_u32;
    let mut last_gps_sync = 0_u32;
    let mut latest_rc = None;
    let mut seq = 0_u32;
    loop {
        seq = seq.wrapping_add(1);
        let mut crsf_bytes = 0_u32;
        for _ in 0..50_000 {
            match crsf_uart.read() {
                Ok(byte) => {
                    crsf_bytes += 1;
                    if let Some(packet) = crsf_parser.push_bytes(&[byte], seq as u64) {
                        crsf_frames = crsf_frames.wrapping_add(1);
                        latest_rc = Some(packet);
                    }
                }
                Err(nb::Error::WouldBlock) => {}
                Err(nb::Error::Other(_)) => {}
            }
            core::hint::spin_loop();
        }

        match read_imu(&mut spi, &mut imu_cs) {
            Ok((accel, gyro, temp)) => {
                let _ = writeln!(
                    writer,
                    "imu seq={} accel=({:.2},{:.2},{:.2}) gyro=({:.3},{:.3},{:.3}) temp={:.2}",
                    seq, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], temp
                );
            }
            Err(()) => {
                let _ = writeln!(writer, "imu read failed");
            }
        }

        match read_baro(&mut spi, &mut baro_cs, &baro_cal) {
            Ok((pressure, temp)) => {
                let _ = writeln!(writer, "baro pressure={:.1} temp={:.2}", pressure, temp);
            }
            Err(()) => {
                let _ = writeln!(writer, "baro read failed");
            }
        }

        let gps_total = GPS_TOTAL_BYTES.load(Ordering::Relaxed);
        let gps_sync = GPS_UBX_SYNC.load(Ordering::Relaxed);
        let gps_bytes = gps_total.wrapping_sub(last_gps_total);
        let gps_ubx = gps_sync.wrapping_sub(last_gps_sync);
        last_gps_total = gps_total;
        last_gps_sync = gps_sync;

        if let Some(rc) = latest_rc {
            let _ = writeln!(
                writer,
                "crsf bytes={} frames={} ch1={:.3} ch2={:.3} ch3={:.3} ch4={:.3}",
                crsf_bytes, crsf_frames, rc.chan[0], rc.chan[1], rc.chan[2], rc.chan[3]
            );
        } else {
            let _ = writeln!(writer, "crsf bytes={} frames={}", crsf_bytes, crsf_frames);
        }
        let _ = writeln!(
            writer,
            "gps pio bytes={} total={} ubx_sync={}",
            gps_bytes, gps_total, gps_ubx
        );
        Timer::after_millis(50).await;
    }
}
