#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal::spi::{ErrorKind, ErrorType as SpiErrorType};
use embedded_hal_async::spi::{Operation, SpiDevice};
use ism330dhcx_rs::asynchronous::{
    self as ism330dhcx, Ism330dhcx,
    prelude::{BdrGy, BdrXl, FifoMode, FifoTag, FsGy, FsXl, MainBank, OdrGy, OdrXl, PinInt1Route},
};
use panic_halt as _;
use rp2350_platform::hal::{
    self as rp, bind_interrupts,
    clocks::ClockConfig,
    config::Config as HalConfig,
    dma,
    executor::Executor,
    gpio::{Input, Level, Output, Pull},
    peripherals::{DMA_CH0, DMA_CH1, DMA_CH2, DMA_CH3, SPI1, UART0},
    spi::{Async as SpiAsync, Config as SpiConfig, Phase, Polarity, Spi},
    uart::{
        Async as UartAsync, Config as UartConfig, InterruptHandler as UartInterruptHandler, Uart,
        UartTx,
    },
};
use static_cell::StaticCell;

type ImuBus = st_mems_bus::asynchronous::spi::SpiBus<RpAsyncSpiDevice>;
type Imu = Ism330dhcx<ImuBus, Delay, MainBank>;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 =>
        dma::InterruptHandler<DMA_CH0>,
        dma::InterruptHandler<DMA_CH1>,
        dma::InterruptHandler<DMA_CH2>,
        dma::InterruptHandler<DMA_CH3>;
    UART0_IRQ => UartInterruptHandler<UART0>;
});

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

const SAMPLE_COUNT: usize = 64;
const WARMUP_SAMPLES: usize = 8;
const FIFO_WATERMARK_PAIRS: usize = 8;
const FIFO_WATERMARK_ENTRIES: u16 = (FIFO_WATERMARK_PAIRS * 2) as u16;
const SETTLE_MS: u64 = 10;
const REPORT_BAUD: u32 = 2_000_000;
const REPORT_SYSID: u8 = 1;
const REPORT_COMPID: u8 = 250;
const MAVLINK_V1_STX: u8 = 0xFE;
const MAVLINK_STATUSTEXT: u8 = 253;
const MAVLINK_STATUSTEXT_LEN: usize = 51;
const MAVLINK_STATUSTEXT_CRC_EXTRA: u8 = 83;
const MAV_SEVERITY_ERROR: u8 = 3;
const MAV_SEVERITY_INFO: u8 = 6;
const SELECTED_RATE: RateCase = selected_rate();
const SELECTED_RATE_CODE: u32 = selected_rate_code();

#[derive(Clone, Copy)]
struct RateCase {
    accel_odr: OdrXl,
    gyro_odr: OdrGy,
    accel_batch: BdrXl,
    gyro_batch: BdrGy,
    period_us: u64,
}

impl RateCase {
    const fn batch_expected_us(self) -> u64 {
        self.period_us * FIFO_WATERMARK_PAIRS as u64
    }

    const fn batch_timeout_us(self) -> u64 {
        let timeout = self.batch_expected_us() * 4;
        if timeout < 10_000 { 10_000 } else { timeout }
    }

    const fn watermark_poll_interval_us(self) -> u64 {
        let interval = self.period_us / 8;
        if interval < 50 { 50 } else { interval }
    }
}

const fn selected_rate() -> RateCase {
    match SELECTED_RATE_CODE {
        125 => RateCase {
            accel_odr: OdrXl::_12_5hz,
            gyro_odr: OdrGy::_12_5hz,
            accel_batch: BdrXl::_12_5hz,
            gyro_batch: BdrGy::_12_5hz,
            period_us: 80_000,
        },
        260 => RateCase {
            accel_odr: OdrXl::_26hz,
            gyro_odr: OdrGy::_26hz,
            accel_batch: BdrXl::_26hz,
            gyro_batch: BdrGy::_26hz,
            period_us: 38_462,
        },
        520 => RateCase {
            accel_odr: OdrXl::_52hz,
            gyro_odr: OdrGy::_52hz,
            accel_batch: BdrXl::_52hz,
            gyro_batch: BdrGy::_52hz,
            period_us: 19_231,
        },
        1040 => RateCase {
            accel_odr: OdrXl::_104hz,
            gyro_odr: OdrGy::_104hz,
            accel_batch: BdrXl::_104hz,
            gyro_batch: BdrGy::_104hz,
            period_us: 9_615,
        },
        2080 => RateCase {
            accel_odr: OdrXl::_208hz,
            gyro_odr: OdrGy::_208hz,
            accel_batch: BdrXl::_208hz,
            gyro_batch: BdrGy::_208hz,
            period_us: 4_808,
        },
        4160 => RateCase {
            accel_odr: OdrXl::_416hz,
            gyro_odr: OdrGy::_416hz,
            accel_batch: BdrXl::_417hz,
            gyro_batch: BdrGy::_417hz,
            period_us: 2_404,
        },
        8330 => RateCase {
            accel_odr: OdrXl::_833hz,
            gyro_odr: OdrGy::_833hz,
            accel_batch: BdrXl::_833hz,
            gyro_batch: BdrGy::_833hz,
            period_us: 1_200,
        },
        16660 => RateCase {
            accel_odr: OdrXl::_1666hz,
            gyro_odr: OdrGy::_1666hz,
            accel_batch: BdrXl::_1667hz,
            gyro_batch: BdrGy::_1667hz,
            period_us: 600,
        },
        33320 => RateCase {
            accel_odr: OdrXl::_3332hz,
            gyro_odr: OdrGy::_3332hz,
            accel_batch: BdrXl::_3333hz,
            gyro_batch: BdrGy::_3333hz,
            period_us: 300,
        },
        66670 => RateCase {
            accel_odr: OdrXl::_6667hz,
            gyro_odr: OdrGy::_6667hz,
            accel_batch: BdrXl::_6667hz,
            gyro_batch: BdrGy::_6667hz,
            period_us: 150,
        },
        _ => panic!("unsupported VELOXITY_IMU_TEST_ODR_HZ"),
    }
}

const fn selected_rate_code() -> u32 {
    let bytes = env!("VELOXITY_IMU_TEST_ODR_CODE").as_bytes();
    let mut index = 0;
    let mut value = 0_u32;
    while index < bytes.len() {
        value = value * 10 + (bytes[index] - b'0') as u32;
        index += 1;
    }
    value
}

#[derive(Debug)]
enum RpAsyncSpiDeviceError {
    Spi(rp2350_platform::hal::spi::Error),
}

impl embedded_hal::spi::Error for RpAsyncSpiDeviceError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

struct RpAsyncSpiDevice {
    spi: Spi<'static, SPI1, SpiAsync>,
    cs: Output<'static>,
}

impl RpAsyncSpiDevice {
    fn new(spi: Spi<'static, SPI1, SpiAsync>, mut cs: Output<'static>) -> Self {
        cs.set_high();
        Self { spi, cs }
    }
}

impl SpiErrorType for RpAsyncSpiDevice {
    type Error = RpAsyncSpiDeviceError;
}

impl SpiDevice<u8> for RpAsyncSpiDevice {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.cs.set_low();

        let result = async {
            for operation in operations {
                match operation {
                    Operation::Read(buf) => self.spi.read(buf).await,
                    Operation::Write(buf) => self.spi.write(buf).await,
                    Operation::Transfer(read, write) => self.spi.transfer(read, write).await,
                    Operation::TransferInPlace(buf) => self.spi.transfer_in_place(buf).await,
                    Operation::DelayNs(ns) => {
                        self.spi.flush().map_err(RpAsyncSpiDeviceError::Spi)?;
                        Timer::after_nanos(*ns as u64).await;
                        Ok(())
                    }
                }
                .map_err(RpAsyncSpiDeviceError::Spi)?;
            }
            self.spi.flush().map_err(RpAsyncSpiDeviceError::Spi)
        }
        .await;

        self.cs.set_high();
        result
    }
}

#[entry]
fn main() -> ! {
    let peripherals = rp::init(hal_config());

    let mut spi_config = SpiConfig::default();
    spi_config.frequency = 10_000_000;
    spi_config.polarity = Polarity::IdleLow;
    spi_config.phase = Phase::CaptureOnFirstTransition;

    let spi = Spi::new(
        peripherals.SPI1,
        peripherals.PIN_10,
        peripherals.PIN_11,
        peripherals.PIN_12,
        peripherals.DMA_CH0,
        peripherals.DMA_CH1,
        Irqs,
        spi_config,
    );
    let cs = Output::new(peripherals.PIN_13, Level::High);
    let watermark = Input::new(peripherals.PIN_14, Pull::Down);
    let scope = ScopePins::new(
        Output::new(peripherals.PIN_18, Level::Low),
        Output::new(peripherals.PIN_19, Level::Low),
        Output::new(peripherals.PIN_20, Level::Low),
    );
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = REPORT_BAUD;
    let uart = Uart::new(
        peripherals.UART0,
        peripherals.PIN_0,
        peripherals.PIN_1,
        Irqs,
        peripherals.DMA_CH2,
        peripherals.DMA_CH3,
        uart_config,
    );
    let (uart_tx, _) = uart.split();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        if let Ok(token) = imu_async_rate_test(spi, cs, watermark, uart_tx, scope) {
            spawner.spawn(token);
        }
    })
}

#[embassy_executor::task]
async fn imu_async_rate_test(
    spi: Spi<'static, SPI1, SpiAsync>,
    cs: Output<'static>,
    mut watermark: Input<'static>,
    uart_tx: UartTx<'static, UartAsync>,
    mut scope: ScopePins,
) -> ! {
    let mut report = ReportWriter::new(uart_tx);
    report.start().await;

    let dev = RpAsyncSpiDevice::new(spi, cs);
    let mut imu = Ism330dhcx::new_spi(dev, Delay);

    let result = async {
        initialize_imu(&mut imu).await?;
        validate_rate(&mut imu, &mut watermark, SELECTED_RATE, &mut scope).await
    }
    .await;

    let _ = imu.xl_data_rate_set(OdrXl::Off).await;
    let _ = imu.gy_data_rate_set(OdrGy::Off).await;

    match result {
        Ok(stats) => report.pass(&stats).await,
        Err(failure) => {
            report.fail(&failure).await;
            panic!("ism330dhcx async rate test failed");
        }
    }

    loop {
        Timer::after_secs(1).await;
    }
}

async fn initialize_imu(imu: &mut Imu) -> Result<(), TestFailure> {
    let who_am_i = imu
        .device_id_get()
        .await
        .map_err(|_| TestFailure::new("whoami-read"))?;
    if who_am_i != ism330dhcx::ISM330DHCX_ID {
        return Err(TestFailure::new("whoami-id"));
    }

    imu.reset_set(ism330dhcx::PROPERTY_ENABLE)
        .await
        .map_err(|_| TestFailure::new("reset-set"))?;
    loop {
        if imu
            .reset_get()
            .await
            .map_err(|_| TestFailure::new("reset-get"))?
            == ism330dhcx::PROPERTY_DISABLE
        {
            break;
        }
        Timer::after_millis(1).await;
    }

    imu.device_conf_set(ism330dhcx::PROPERTY_ENABLE)
        .await
        .map_err(|_| TestFailure::new("device-conf"))?;
    imu.block_data_update_set(ism330dhcx::PROPERTY_ENABLE)
        .await
        .map_err(|_| TestFailure::new("bdu"))?;
    imu.xl_full_scale_set(FsXl::_16g)
        .await
        .map_err(|_| TestFailure::new("accel-scale"))?;
    imu.gy_full_scale_set(FsGy::_2000dps)
        .await
        .map_err(|_| TestFailure::new("gyro-scale"))?;

    let mut route = PinInt1Route::default();
    route
        .int1_ctrl
        .set_int1_fifo_th(ism330dhcx::PROPERTY_ENABLE);
    imu.pin_int1_route_set(&mut route)
        .await
        .map_err(|_| TestFailure::new("fifo-int-route"))?;
    Ok(())
}

async fn validate_rate(
    imu: &mut Imu,
    watermark: &mut Input<'static>,
    rate: RateCase,
    scope: &mut ScopePins,
) -> Result<TestStats, TestFailure> {
    configure_fifo(imu, rate).await?;

    for _ in 0..WARMUP_SAMPLES {
        let _ = drain_fifo_batch(imu, watermark, rate, scope).await?;
    }

    let first_batch = drain_fifo_batch(imu, watermark, rate, scope).await?;
    let mut timed_batches = [TimedBatch::EMPTY; (SAMPLE_COUNT / FIFO_WATERMARK_PAIRS) - 1];
    let mut previous_batch_at = first_batch.at;
    let mut total_pairs = 0_u32;
    let mut total_accel = 0_u32;
    let mut total_gyro = 0_u32;
    let mut accel_changes = 0_u32;
    let mut gyro_changes = 0_u32;
    let mut previous_accel: Option<[i16; 3]> = None;
    let mut previous_gyro: Option<[i16; 3]> = None;
    let mut min_batch_pairs = u32::MAX;
    let mut max_batch_pairs = 0_u32;

    record_batch_stats(
        first_batch,
        &mut total_pairs,
        &mut total_accel,
        &mut total_gyro,
        &mut min_batch_pairs,
        &mut max_batch_pairs,
        &mut previous_accel,
        &mut previous_gyro,
        &mut accel_changes,
        &mut gyro_changes,
    );

    for timed_batch in timed_batches.iter_mut() {
        let batch = drain_fifo_batch(imu, watermark, rate, scope).await?;
        *timed_batch = TimedBatch {
            interval_us: batch.at.duration_since(previous_batch_at).as_micros(),
            pairs: batch.pairs,
        };
        previous_batch_at = batch.at;
        record_batch_stats(
            batch,
            &mut total_pairs,
            &mut total_accel,
            &mut total_gyro,
            &mut min_batch_pairs,
            &mut max_batch_pairs,
            &mut previous_accel,
            &mut previous_gyro,
            &mut accel_changes,
            &mut gyro_changes,
        );
    }

    if total_accel == 0 {
        return Err(TestFailure::new("fifo-accel-zero"));
    }
    if total_gyro == 0 {
        return Err(TestFailure::new("fifo-gyro-zero"));
    }
    if total_accel.abs_diff(total_gyro) > FIFO_WATERMARK_PAIRS as u32 {
        return Err(TestFailure::new("fifo-unbalanced"));
    }
    if total_pairs < SAMPLE_COUNT as u32 / 2 {
        return Err(TestFailure::new("fifo-low-pairs"));
    }
    if accel_changes == 0 {
        return Err(TestFailure::new("fifo-accel-static"));
    }
    if gyro_changes == 0 {
        return Err(TestFailure::new("fifo-gyro-static"));
    }

    validate_timed_batches(&timed_batches, rate.period_us).map(|stats| TestStats {
        accel_changes,
        gyro_changes,
        pairs: total_pairs,
        batches: timed_batches.len() as u32 + 1,
        min_batch_pairs,
        max_batch_pairs,
        ..stats
    })
}

async fn configure_fifo(imu: &mut Imu, rate: RateCase) -> Result<(), TestFailure> {
    imu.fifo_mode_set(FifoMode::BypassMode)
        .await
        .map_err(|_| TestFailure::new("fifo-bypass"))?;
    imu.fifo_xl_batch_set(BdrXl::NotBatched)
        .await
        .map_err(|_| TestFailure::new("fifo-xl-off"))?;
    imu.fifo_gy_batch_set(BdrGy::NotBatched)
        .await
        .map_err(|_| TestFailure::new("fifo-gy-off"))?;
    imu.xl_data_rate_set(OdrXl::Off)
        .await
        .map_err(|_| TestFailure::new("accel-stop"))?;
    imu.gy_data_rate_set(OdrGy::Off)
        .await
        .map_err(|_| TestFailure::new("gyro-stop"))?;
    Timer::after_millis(SETTLE_MS).await;

    imu.xl_data_rate_set(rate.accel_odr)
        .await
        .map_err(|_| TestFailure::new("accel-odr"))?;
    imu.gy_data_rate_set(rate.gyro_odr)
        .await
        .map_err(|_| TestFailure::new("gyro-odr"))?;
    imu.fifo_watermark_set(FIFO_WATERMARK_ENTRIES)
        .await
        .map_err(|_| TestFailure::new("fifo-watermark"))?;
    imu.fifo_xl_batch_set(rate.accel_batch)
        .await
        .map_err(|_| TestFailure::new("fifo-xl-batch"))?;
    imu.fifo_gy_batch_set(rate.gyro_batch)
        .await
        .map_err(|_| TestFailure::new("fifo-gy-batch"))?;
    imu.fifo_mode_set(FifoMode::StreamMode)
        .await
        .map_err(|_| TestFailure::new("fifo-stream"))?;
    Ok(())
}

async fn drain_fifo_batch(
    imu: &mut Imu,
    watermark: &mut Input<'static>,
    rate: RateCase,
    scope: &mut ScopePins,
) -> Result<FifoBatch, TestFailure> {
    wait_for_fifo_watermark(imu, watermark, rate, scope).await?;
    let at = Instant::now();

    scope.poll.set_high();
    let level = imu
        .fifo_data_level_get()
        .await
        .map_err(|_| TestFailure::new("fifo-level-read"))?;
    scope.poll.set_low();

    if imu
        .fifo_ovr_flag_get()
        .await
        .map_err(|_| TestFailure::new("fifo-ovr-read"))?
        == ism330dhcx::PROPERTY_ENABLE
    {
        return Err(TestFailure::new("fifo-overrun"));
    }

    let mut batch = FifoBatch {
        at,
        accel_count: 0,
        gyro_count: 0,
        pairs: 0,
        last_accel: None,
        last_gyro: None,
    };

    scope.read.set_high();
    for _ in 0..level {
        match imu
            .fifo_sensor_tag_get()
            .await
            .map_err(|_| TestFailure::new("fifo-tag-read"))?
        {
            FifoTag::XlNcTag | FifoTag::XlNcT1 | FifoTag::XlNcT2 => {
                let raw = imu
                    .fifo_out_raw_get()
                    .await
                    .map_err(|_| TestFailure::new("fifo-xl-read"))?;
                batch.accel_count += 1;
                batch.last_accel = Some(raw_to_i16x3(raw));
            }
            FifoTag::GyroNcTag | FifoTag::GyroNcT1 | FifoTag::GyroNcT2 => {
                let raw = imu
                    .fifo_out_raw_get()
                    .await
                    .map_err(|_| TestFailure::new("fifo-gy-read"))?;
                batch.gyro_count += 1;
                batch.last_gyro = Some(raw_to_i16x3(raw));
            }
            FifoTag::SensorhubNackTag => {
                let _ = imu
                    .fifo_out_raw_get()
                    .await
                    .map_err(|_| TestFailure::new("fifo-nack-read"))?;
            }
            _ => {
                let _ = imu
                    .fifo_out_raw_get()
                    .await
                    .map_err(|_| TestFailure::new("fifo-skip-read"))?;
            }
        }
    }
    scope.read.set_low();

    batch.pairs = batch.accel_count.min(batch.gyro_count);
    if batch.accel_count == 0 || batch.gyro_count == 0 {
        return Err(TestFailure::new("fifo-empty-sensor"));
    }
    scope.toggle_sample();
    Ok(batch)
}

async fn wait_for_fifo_watermark(
    imu: &mut Imu,
    watermark: &mut Input<'static>,
    rate: RateCase,
    scope: &mut ScopePins,
) -> Result<(), TestFailure> {
    let timeout_us = rate.batch_timeout_us();
    scope.poll.set_high();
    let result = with_timeout(Duration::from_micros(timeout_us), async {
        loop {
            if imu
                .fifo_wtm_flag_get()
                .await
                .map_err(|_| TestFailure::new("fifo-wtm-read"))?
                == ism330dhcx::PROPERTY_ENABLE
            {
                break;
            }
            let _ = with_timeout(
                Duration::from_micros(rate.watermark_poll_interval_us()),
                watermark.wait_for_rising_edge(),
            )
            .await;
        }
        Ok(())
    })
    .await
    .map_err(|_| TestFailure::new("fifo-wtm-timeout"));
    scope.poll.set_low();
    result?
}

fn record_batch_stats(
    batch: FifoBatch,
    total_pairs: &mut u32,
    total_accel: &mut u32,
    total_gyro: &mut u32,
    min_batch_pairs: &mut u32,
    max_batch_pairs: &mut u32,
    previous_accel: &mut Option<[i16; 3]>,
    previous_gyro: &mut Option<[i16; 3]>,
    accel_changes: &mut u32,
    gyro_changes: &mut u32,
) {
    *total_pairs += batch.pairs;
    *total_accel += batch.accel_count;
    *total_gyro += batch.gyro_count;
    *min_batch_pairs = (*min_batch_pairs).min(batch.pairs);
    *max_batch_pairs = (*max_batch_pairs).max(batch.pairs);

    if let Some(accel) = batch.last_accel {
        if previous_accel.is_some_and(|previous| previous != accel) {
            *accel_changes += 1;
        }
        *previous_accel = Some(accel);
    }
    if let Some(gyro) = batch.last_gyro {
        if previous_gyro.is_some_and(|previous| previous != gyro) {
            *gyro_changes += 1;
        }
        *previous_gyro = Some(gyro);
    }
}

fn validate_timed_batches(
    batches: &[TimedBatch],
    expected_us: u64,
) -> Result<TestStats, TestFailure> {
    let mut min = u64::MAX;
    let mut max = 0_u64;
    let mut interval_sum = 0_u64;
    let mut pair_sum = 0_u32;

    for batch in batches {
        if batch.pairs == 0 {
            return Err(TestFailure::new("timed-zero-pairs"));
        }
        let per_pair = batch.interval_us / batch.pairs as u64;
        min = min.min(per_pair);
        max = max.max(per_pair);
        interval_sum += batch.interval_us;
        pair_sum += batch.pairs;

        let expected_interval = expected_us * batch.pairs as u64;
        let max_missing_gap = expected_interval + expected_us + expected_interval / 2;
        if batch.interval_us > max_missing_gap {
            return Err(TestFailure::new("missed-gap"));
        }
    }

    let average = interval_sum / pair_sum as u64;
    let average_error = average.abs_diff(expected_us);
    let average_tolerance = (expected_us / 10).max(25);
    let jitter_tolerance = (expected_us / 5).max(50);

    let stats = TestStats::from_intervals(expected_us, min, max, average, 0, 0, 0, 0, 0, 0);
    if average_error > average_tolerance {
        return Err(TestFailure::with_stats("avg-rate", stats));
    }
    if max - min > jitter_tolerance {
        return Err(TestFailure::with_stats("jitter", stats));
    }

    Ok(stats)
}

fn hal_config() -> HalConfig {
    HalConfig::new(ClockConfig::system_freq(300_000_000).unwrap())
}

fn raw_to_i16x3(raw: [u8; 6]) -> [i16; 3] {
    [
        i16::from_le_bytes([raw[0], raw[1]]),
        i16::from_le_bytes([raw[2], raw[3]]),
        i16::from_le_bytes([raw[4], raw[5]]),
    ]
}

#[derive(Clone, Copy)]
struct FifoBatch {
    at: Instant,
    accel_count: u32,
    gyro_count: u32,
    pairs: u32,
    last_accel: Option<[i16; 3]>,
    last_gyro: Option<[i16; 3]>,
}

#[derive(Clone, Copy)]
struct TimedBatch {
    interval_us: u64,
    pairs: u32,
}

impl TimedBatch {
    const EMPTY: Self = Self {
        interval_us: 0,
        pairs: 0,
    };
}

struct ScopePins {
    poll: Output<'static>,
    read: Output<'static>,
    sample: Output<'static>,
    sample_high: bool,
}

impl ScopePins {
    fn new(poll: Output<'static>, read: Output<'static>, sample: Output<'static>) -> Self {
        Self {
            poll,
            read,
            sample,
            sample_high: false,
        }
    }

    fn toggle_sample(&mut self) {
        if self.sample_high {
            self.sample.set_low();
        } else {
            self.sample.set_high();
        }
        self.sample_high = !self.sample_high;
    }
}

#[derive(Clone, Copy)]
struct TestStats {
    expected_us: u64,
    min_us: u64,
    max_us: u64,
    avg_us: u64,
    accel_changes: u32,
    gyro_changes: u32,
    pairs: u32,
    batches: u32,
    min_batch_pairs: u32,
    max_batch_pairs: u32,
}

impl TestStats {
    const fn from_intervals(
        expected_us: u64,
        min_us: u64,
        max_us: u64,
        avg_us: u64,
        accel_changes: u32,
        gyro_changes: u32,
        pairs: u32,
        batches: u32,
        min_batch_pairs: u32,
        max_batch_pairs: u32,
    ) -> Self {
        Self {
            expected_us,
            min_us,
            max_us,
            avg_us,
            accel_changes,
            gyro_changes,
            pairs,
            batches,
            min_batch_pairs,
            max_batch_pairs,
        }
    }
}

struct TestFailure {
    reason: &'static str,
    stats: Option<TestStats>,
}

impl TestFailure {
    const fn new(reason: &'static str) -> Self {
        Self {
            reason,
            stats: None,
        }
    }

    const fn with_stats(reason: &'static str, stats: TestStats) -> Self {
        Self {
            reason,
            stats: Some(stats),
        }
    }
}

struct ReportWriter {
    uart: UartTx<'static, UartAsync>,
    seq: u8,
}

impl ReportWriter {
    fn new(uart: UartTx<'static, UartAsync>) -> Self {
        Self { uart, seq: 0 }
    }

    async fn start(&mut self) {
        let mut line = LineBuf::new();
        line.push_str("IMUTEST START o=");
        line.push_rate_code(SELECTED_RATE_CODE);
        line.push_str(" s=");
        line.push_u64(SAMPLE_COUNT as u64);
        self.statustext(MAV_SEVERITY_INFO, &line).await;
    }

    async fn pass(&mut self, stats: &TestStats) {
        let mut line = LineBuf::new();
        line.push_str("IMUTEST PASS o=");
        line.push_rate_code(SELECTED_RATE_CODE);
        line.push_str(" a=");
        line.push_u64(stats.avg_us);
        line.push_str(" n=");
        line.push_u64(stats.min_us);
        line.push_str(" x=");
        line.push_u64(stats.max_us);
        self.statustext(MAV_SEVERITY_INFO, &line).await;

        let mut detail = LineBuf::new();
        detail.push_str("IMUF e=");
        detail.push_u64(stats.expected_us);
        detail.push_str(" p=");
        detail.push_u64(stats.pairs as u64);
        detail.push_str(" b=");
        detail.push_u64(stats.batches as u64);
        detail.push_str(" q=");
        detail.push_u64(stats.min_batch_pairs as u64);
        detail.push_byte(b'-');
        detail.push_u64(stats.max_batch_pairs as u64);
        detail.push_str(" ac=");
        detail.push_u64(stats.accel_changes as u64);
        detail.push_str(" gy=");
        detail.push_u64(stats.gyro_changes as u64);
        self.statustext(MAV_SEVERITY_INFO, &detail).await;
    }

    async fn fail(&mut self, failure: &TestFailure) {
        let mut line = LineBuf::new();
        line.push_str("IMUTEST FAIL ");
        line.push_str(failure.reason);
        line.push_str(" o=");
        line.push_rate_code(SELECTED_RATE_CODE);
        self.statustext(MAV_SEVERITY_ERROR, &line).await;

        if let Some(stats) = failure.stats {
            let mut detail = LineBuf::new();
            detail.push_str("IMUTEST RATE a=");
            detail.push_u64(stats.avg_us);
            detail.push_str(" n=");
            detail.push_u64(stats.min_us);
            detail.push_str(" x=");
            detail.push_u64(stats.max_us);
            self.statustext(MAV_SEVERITY_ERROR, &detail).await;
        }
    }

    async fn statustext(&mut self, severity: u8, line: &LineBuf) {
        let mut payload = [0_u8; MAVLINK_STATUSTEXT_LEN];
        payload[0] = severity;
        payload[1..1 + line.len].copy_from_slice(line.as_bytes());

        let mut frame = [0_u8; MAVLINK_STATUSTEXT_LEN + 8];
        frame[0] = MAVLINK_V1_STX;
        frame[1] = MAVLINK_STATUSTEXT_LEN as u8;
        frame[2] = self.seq;
        frame[3] = REPORT_SYSID;
        frame[4] = REPORT_COMPID;
        frame[5] = MAVLINK_STATUSTEXT;
        frame[6..6 + MAVLINK_STATUSTEXT_LEN].copy_from_slice(&payload);

        let crc = mavlink_crc(&frame[1..6 + MAVLINK_STATUSTEXT_LEN]);
        frame[6 + MAVLINK_STATUSTEXT_LEN] = crc as u8;
        frame[7 + MAVLINK_STATUSTEXT_LEN] = (crc >> 8) as u8;
        self.seq = self.seq.wrapping_add(1);

        let _ = self.uart.write(&frame).await;
    }
}

struct LineBuf {
    bytes: [u8; 50],
    len: usize,
}

impl LineBuf {
    const fn new() -> Self {
        Self {
            bytes: [0; 50],
            len: 0,
        }
    }

    fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    fn push_str(&mut self, value: &str) {
        for byte in value.as_bytes() {
            self.push_byte(*byte);
        }
    }

    fn push_rate_code(&mut self, code: u32) {
        self.push_u64((code / 10) as u64);
        if code % 10 != 0 {
            self.push_byte(b'.');
            self.push_u64((code % 10) as u64);
        }
        self.push_str("Hz");
    }

    fn push_u64(&mut self, value: u64) {
        let mut digits = [0_u8; 20];
        let mut n = value;
        let mut count = 0;
        loop {
            digits[count] = b'0' + (n % 10) as u8;
            count += 1;
            n /= 10;
            if n == 0 {
                break;
            }
        }
        while count > 0 {
            count -= 1;
            self.push_byte(digits[count]);
        }
    }

    fn push_byte(&mut self, byte: u8) {
        if self.len < self.bytes.len() {
            self.bytes[self.len] = byte;
            self.len += 1;
        }
    }
}

fn mavlink_crc(bytes: &[u8]) -> u16 {
    let mut crc = 0xFFFF_u16;
    for byte in bytes {
        crc = crc_accumulate(*byte, crc);
    }
    crc_accumulate(MAVLINK_STATUSTEXT_CRC_EXTRA, crc)
}

fn crc_accumulate(byte: u8, crc: u16) -> u16 {
    let mut tmp = byte ^ (crc as u8);
    tmp ^= tmp << 4;
    (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4)
}
