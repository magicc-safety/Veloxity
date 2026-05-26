#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use cortex_m_rt::entry;
use embassy_executor::Executor;
use embassy_time::{Duration, Instant, Timer};
use panic_halt as _;
use pico2w::comms_core::{SHARED_MAVLINK_MAILBOX, SharedMavlinkMailbox};
#[cfg(any(
    feature = "synthetic-imu",
    all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu"))
))]
use pico2w::ism330dhcx::SHARED_ISM330DHCX_IMU_QUEUE;
use pico2w::rc_receiver::{CRSF_BAUDRATE, CrsfRcParser, SHARED_CRSF_RC_QUEUE};
use pico2w::{board, config::Pico2WConfig, pwm::PioPwmDriver};
use rp2350_platform::hal::clocks::ClockConfig;
use rp2350_platform::hal::dma;
#[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
use rp2350_platform::hal::peripherals::{PIN_10, PIN_11, PIN_12, PIN_13, SPI1};
use rp2350_platform::hal::{
    self as rp, Peri, bind_interrupts,
    config::Config as HalConfig,
    multicore::{Stack, spawn_core1},
    peripherals::{
        CORE1, DMA_CH0, DMA_CH1, DMA_CH2, DMA_CH3, PIN_0, PIN_1, PIN_8, PIN_9, UART0, UART1,
    },
    uart::{
        Async as UartAsync, Config as UartConfig, InterruptHandler as UartInterruptHandler, Uart,
        UartRx, UartTx,
    },
};
#[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
use rp2350_platform::hal::{
    gpio::{Level, Output},
    spi::{Blocking, Config as SpiConfig, Phase, Polarity, Spi},
};
use static_cell::StaticCell;
#[cfg(feature = "release-loop-bench")]
use voloxide_core::board::SerialTxPriority;
#[cfg(any(
    feature = "synthetic-imu",
    all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu"))
))]
use voloxide_core::packets::{ImuPacket, RosflightPacketHeader};
use voloxide_core::{
    board::{BoardIo, SerialRxPriority},
    comm::TelemetryRates,
    params::Params,
    state_machine::StateManager,
    vehicle::quadrotor,
    world::World,
};
use voloxide_mavlink::{MavlinkInterface, parser::MavlinkParser};

type PicoReal = f32;

type Pico2WWorld = World<
    board::Board,
    quadrotor::Estimator<PicoReal>,
    quadrotor::Controller<PicoReal>,
    quadrotor::Mixer<PicoReal>,
    MavlinkInterface,
    PioPwmDriver,
    PicoReal,
>;

static mut CORE1_STACK: Stack<65536> = Stack::new();
static CORE1_EXECUTOR: StaticCell<Executor> = StaticCell::new();

const UART_TX_BATCH_BYTES: usize = 256;
const UART_RX_CHUNK_BYTES: usize = 16;
const UART_IDLE_DELAY_US: u64 = 50;
const MAVLINK_UART_BAUDRATE: u32 = 2_000_000;
const CRSF_RX_CHUNK_BYTES: usize = 8;
#[cfg(feature = "synthetic-imu")]
const SYNTHETIC_IMU_PERIOD_US: u64 = synthetic_imu_period_us();
#[cfg(all(
    feature = "synthetic-imu-1khz",
    not(feature = "synthetic-imu-2khz"),
    not(feature = "synthetic-imu-4khz")
))]
const fn synthetic_imu_period_us() -> u64 {
    1_000
}
#[cfg(all(
    feature = "synthetic-imu-2khz",
    not(feature = "synthetic-imu-1khz"),
    not(feature = "synthetic-imu-4khz")
))]
const fn synthetic_imu_period_us() -> u64 {
    500
}
#[cfg(all(
    feature = "synthetic-imu-4khz",
    not(feature = "synthetic-imu-1khz"),
    not(feature = "synthetic-imu-2khz")
))]
const fn synthetic_imu_period_us() -> u64 {
    250
}
#[cfg(all(
    feature = "synthetic-imu",
    not(feature = "synthetic-imu-1khz"),
    not(feature = "synthetic-imu-2khz"),
    not(feature = "synthetic-imu-4khz")
))]
const fn synthetic_imu_period_us() -> u64 {
    125
}
#[cfg(any(
    all(feature = "synthetic-imu-1khz", feature = "synthetic-imu-2khz"),
    all(feature = "synthetic-imu-1khz", feature = "synthetic-imu-4khz"),
    all(feature = "synthetic-imu-2khz", feature = "synthetic-imu-4khz")
))]
compile_error!("select only one synthetic IMU rate feature");
#[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
const ISM330DHCX_IMU_PERIOD_US: u64 = 250;
#[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
const ISM330DHCX_SPI_HZ: u32 = 10_000_000;
#[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
const ISM330DHCX_WHO_AM_I: u8 = 0x6b;
#[cfg(feature = "release-loop-bench")]
const LOOP_BENCH_REPORT_US: u64 = 1_000_000;
#[cfg(feature = "release-loop-bench")]
const LOOP_BENCH_BUDGET_US: u32 = 250;
#[cfg(feature = "release-loop-bench")]
const LOOP_BENCH_BUCKET_US: u32 = 10;
#[cfg(feature = "release-loop-bench")]
const LOOP_BENCH_BUCKETS: usize = 128;
#[cfg(feature = "release-loop-bench")]
const MAVLINK_V1_STX: u8 = 0xfe;
#[cfg(feature = "release-loop-bench")]
const MAVLINK_STATUSTEXT_ID: u8 = 253;
#[cfg(feature = "release-loop-bench")]
const MAVLINK_STATUSTEXT_LEN: usize = 51;
#[cfg(feature = "release-loop-bench")]
const MAVLINK_STATUSTEXT_CRC_EXTRA: u8 = 83;
#[cfg(feature = "release-loop-bench")]
const MAVLINK_SYSTEM_ID: u8 = 1;
#[cfg(feature = "release-loop-bench")]
const MAVLINK_COMPONENT_ID: u8 = 250;

bind_interrupts!(struct Irqs {
    UART0_IRQ => UartInterruptHandler<UART0>;
    UART1_IRQ => UartInterruptHandler<UART1>;
    DMA_IRQ_0 =>
        dma::InterruptHandler<DMA_CH0>,
        dma::InterruptHandler<DMA_CH1>,
        dma::InterruptHandler<DMA_CH2>,
        dma::InterruptHandler<DMA_CH3>;
});

fn mavlink_uart_config() -> UartConfig {
    let mut config = UartConfig::default();
    config.baudrate = MAVLINK_UART_BAUDRATE;
    config
}

fn crsf_uart_config() -> UartConfig {
    let mut config = UartConfig::default();
    config.baudrate = CRSF_BAUDRATE;
    config
}

#[embassy_executor::task]
async fn core1_heartbeat_task(mailbox: SharedMavlinkMailbox) -> ! {
    loop {
        mailbox.record_core1_heartbeat();
        Timer::after_millis(500).await;
    }
}

#[cfg(feature = "synthetic-imu")]
#[embassy_executor::task]
async fn synthetic_imu_task() -> ! {
    let mut seq = 0_u32;
    loop {
        let now_us = Instant::now().as_micros();
        SHARED_ISM330DHCX_IMU_QUEUE.push_from_interrupt(ImuPacket {
            header: RosflightPacketHeader {
                timestamp: now_us,
                status: 0,
            },
            accel: [0.0, 0.0, -9.80665],
            gyro: [0.0, 0.0, 0.0],
            temperature: 25.0,
            seq,
        });
        seq = seq.wrapping_add(1);
        Timer::after(Duration::from_micros(SYNTHETIC_IMU_PERIOD_US)).await;
    }
}

#[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
#[embassy_executor::task]
async fn ism330dhcx_imu_task(mut spi: Spi<'static, SPI1, Blocking>, mut cs: Output<'static>) -> ! {
    let mut seq = 0_u32;
    loop {
        if ism330dhcx_init(&mut spi, &mut cs).is_ok() {
            loop {
                let now_us = Instant::now().as_micros();
                if let Ok(packet) = ism330dhcx_read_packet(&mut spi, &mut cs, now_us, seq) {
                    SHARED_ISM330DHCX_IMU_QUEUE.push_from_interrupt(packet);
                    seq = seq.wrapping_add(1);
                }
                Timer::after(Duration::from_micros(ISM330DHCX_IMU_PERIOD_US)).await;
            }
        }
        Timer::after_millis(1_000).await;
    }
}

#[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
fn ism330dhcx_init(
    spi: &mut Spi<'static, SPI1, Blocking>,
    cs: &mut Output<'static>,
) -> Result<(), ()> {
    if ism330dhcx_read_reg(spi, cs, 0x0f)? != ISM330DHCX_WHO_AM_I {
        return Err(());
    }
    ism330dhcx_write_reg(spi, cs, 0x12, 0x44)?;
    ism330dhcx_write_reg(spi, cs, 0x10, 0xa4)?;
    ism330dhcx_write_reg(spi, cs, 0x11, 0xac)?;
    Ok(())
}

#[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
fn ism330dhcx_read_packet(
    spi: &mut Spi<'static, SPI1, Blocking>,
    cs: &mut Output<'static>,
    now_us: u64,
    seq: u32,
) -> Result<ImuPacket<f32>, ()> {
    let mut bytes = [0_u8; 15];
    bytes[0] = 0x20 | 0x80;
    cs.set_low();
    let result = spi.blocking_transfer_in_place(&mut bytes);
    cs.set_high();
    result.map_err(|_| ())?;

    let temperature_raw = i16::from_le_bytes([bytes[1], bytes[2]]);
    let gyro_raw = [
        i16::from_le_bytes([bytes[3], bytes[4]]),
        i16::from_le_bytes([bytes[5], bytes[6]]),
        i16::from_le_bytes([bytes[7], bytes[8]]),
    ];
    let accel_raw = [
        i16::from_le_bytes([bytes[9], bytes[10]]),
        i16::from_le_bytes([bytes[11], bytes[12]]),
        i16::from_le_bytes([bytes[13], bytes[14]]),
    ];

    const GYRO_2000DPS_TO_RAD_S: f32 = 0.07 * core::f32::consts::PI / 180.0;
    const ACCEL_16G_TO_M_S2: f32 = 0.000_488 * 9.80665;

    Ok(ImuPacket {
        header: RosflightPacketHeader {
            timestamp: now_us,
            status: 0,
        },
        accel: [
            accel_raw[0] as f32 * ACCEL_16G_TO_M_S2,
            accel_raw[1] as f32 * ACCEL_16G_TO_M_S2,
            accel_raw[2] as f32 * ACCEL_16G_TO_M_S2,
        ],
        gyro: [
            gyro_raw[0] as f32 * GYRO_2000DPS_TO_RAD_S,
            gyro_raw[1] as f32 * GYRO_2000DPS_TO_RAD_S,
            gyro_raw[2] as f32 * GYRO_2000DPS_TO_RAD_S,
        ],
        temperature: 25.0 + temperature_raw as f32 / 256.0,
        seq,
    })
}

#[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
fn ism330dhcx_read_reg(
    spi: &mut Spi<'static, SPI1, Blocking>,
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

#[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
fn ism330dhcx_write_reg(
    spi: &mut Spi<'static, SPI1, Blocking>,
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

#[embassy_executor::task]
async fn crsf_rx_task(mut uart_rx: UartRx<'static, UartAsync>) -> ! {
    let mut parser = CrsfRcParser::new();
    let mut rx = [0_u8; CRSF_RX_CHUNK_BYTES];
    loop {
        if uart_rx.read(&mut rx).await.is_ok() {
            if let Some(packet) = parser.push_bytes(&rx, Instant::now().as_micros()) {
                SHARED_CRSF_RC_QUEUE.push_from_receiver_task(packet);
            }
        } else {
            Timer::after(Duration::from_micros(UART_IDLE_DELAY_US)).await;
        }
    }
}

#[embassy_executor::task]
async fn uart_tx_task(mut uart_tx: UartTx<'static, UartAsync>, mailbox: SharedMavlinkMailbox) -> ! {
    let mut tx = [0_u8; UART_TX_BATCH_BYTES];
    loop {
        let n = mailbox.drain_tx_batch_into(&mut tx);
        if n == 0 {
            Timer::after(Duration::from_micros(UART_IDLE_DELAY_US)).await;
            continue;
        }
        if uart_tx.write(&tx[..n]).await.is_ok() {
            mailbox.record_uart_tx_batch(n);
        } else {
            mailbox.record_uart_tx_error();
        }
    }
}

#[embassy_executor::task]
async fn uart_rx_task(mut uart_rx: UartRx<'static, UartAsync>, mailbox: SharedMavlinkMailbox) -> ! {
    let mut parser = MavlinkParser::new();
    let mut rx = [0_u8; UART_RX_CHUNK_BYTES];
    loop {
        if uart_rx.read(&mut rx).await.is_ok() {
            mailbox.record_uart_rx_chunk(rx.len());
            for byte in rx {
                if let Some(frame) = parser.feed_byte(byte) {
                    let priority = mavlink_rx_priority(frame.data[5]);
                    let _ = mailbox.push_rx_frame_priority(&frame.data[..frame.len], priority);
                }
            }
        } else {
            mailbox.record_uart_rx_error();
            Timer::after(Duration::from_micros(UART_IDLE_DELAY_US)).await;
        }
    }
}

fn mavlink_rx_priority(message_id: u8) -> SerialRxPriority {
    match message_id {
        0 | 23 | 111 | 188 => SerialRxPriority::CRITICAL,
        20 | 21 | 180 | 193 | 195 => SerialRxPriority::DEFAULT,
        _ => SerialRxPriority::REPLACEABLE_TELEMETRY,
    }
}

struct Core1Resources {
    core1: Peri<'static, CORE1>,
    uart0: Peri<'static, UART0>,
    uart1: Peri<'static, UART1>,
    #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
    spi1: Peri<'static, SPI1>,
    dma_ch0: Peri<'static, DMA_CH0>,
    dma_ch1: Peri<'static, DMA_CH1>,
    dma_ch2: Peri<'static, DMA_CH2>,
    dma_ch3: Peri<'static, DMA_CH3>,
    pin0: Peri<'static, PIN_0>,
    pin1: Peri<'static, PIN_1>,
    pin8: Peri<'static, PIN_8>,
    pin9: Peri<'static, PIN_9>,
    #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
    pin10: Peri<'static, PIN_10>,
    #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
    pin11: Peri<'static, PIN_11>,
    #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
    pin12: Peri<'static, PIN_12>,
    #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
    pin13: Peri<'static, PIN_13>,
}

fn spawn_core1_services(resources: Core1Resources, mailbox: SharedMavlinkMailbox) {
    spawn_core1(
        resources.core1,
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        move || {
            mailbox.set_comms_state(20);
            let mavlink_uart = Uart::new(
                resources.uart0,
                resources.pin0,
                resources.pin1,
                Irqs,
                resources.dma_ch0,
                resources.dma_ch1,
                mavlink_uart_config(),
            );
            let (uart_tx, uart_rx) = mavlink_uart.split();

            let crsf_uart = Uart::new(
                resources.uart1,
                resources.pin8,
                resources.pin9,
                Irqs,
                resources.dma_ch2,
                resources.dma_ch3,
                crsf_uart_config(),
            );
            let (_crsf_tx, crsf_rx) = crsf_uart.split();

            #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
            let imu_spi = {
                let mut spi_config = SpiConfig::default();
                spi_config.frequency = ISM330DHCX_SPI_HZ;
                spi_config.polarity = Polarity::IdleLow;
                spi_config.phase = Phase::CaptureOnFirstTransition;
                Spi::new_blocking(
                    resources.spi1,
                    resources.pin10,
                    resources.pin11,
                    resources.pin12,
                    spi_config,
                )
            };
            #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
            let imu_cs = Output::new(resources.pin13, Level::High);

            let executor = CORE1_EXECUTOR.init(Executor::new());
            mailbox.set_comms_state(21);
            executor.run(|spawner| {
                if let Ok(token) = core1_heartbeat_task(mailbox) {
                    spawner.spawn(token);
                }
                if let Ok(token) = uart_tx_task(uart_tx, mailbox) {
                    spawner.spawn(token);
                }
                if let Ok(token) = uart_rx_task(uart_rx, mailbox) {
                    spawner.spawn(token);
                }
                if let Ok(token) = crsf_rx_task(crsf_rx) {
                    spawner.spawn(token);
                }
                #[cfg(feature = "synthetic-imu")]
                if let Ok(token) = synthetic_imu_task() {
                    spawner.spawn(token);
                }
                #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
                if let Ok(token) = ism330dhcx_imu_task(imu_spi, imu_cs) {
                    spawner.spawn(token);
                }
                mailbox.set_comms_state(22);
            })
        },
    );
}

fn init_world(board: board::Board, params: Params, pwm_driver: PioPwmDriver) -> Pico2WWorld {
    let mixer = quadrotor::mixer(&params);
    let mut world = Pico2WWorld::init(
        board,
        params,
        MavlinkInterface::new(),
        StateManager::new(),
        quadrotor::Estimator::<PicoReal>::default(),
        quadrotor::Controller::<PicoReal>::default(),
        mixer,
        pwm_driver,
    );
    #[cfg(all(feature = "synthetic-imu", feature = "release-loop-bench"))]
    world.set_telemetry_rates(TelemetryRates {
        imu_hz: 0,
        attitude_hz: 50,
        output_raw_hz: 50,
        diff_pressure_hz: 50,
        baro_hz: 25,
        mag_hz: 25,
        range_hz: 50,
        battery_hz: 25,
        gnss_hz: 10,
        rc_hz: 100,
        output_raw_imu_divisor: 0,
    });
    #[cfg(all(feature = "synthetic-imu", not(feature = "release-loop-bench")))]
    world.set_telemetry_rates(TelemetryRates {
        imu_hz: 1,
        attitude_hz: 1,
        output_raw_hz: 1,
        diff_pressure_hz: 1,
        baro_hz: 1,
        mag_hz: 1,
        range_hz: 1,
        battery_hz: 1,
        gnss_hz: 1,
        rc_hz: 1,
        output_raw_imu_divisor: 0,
    });
    #[cfg(not(feature = "synthetic-imu"))]
    world.set_telemetry_rates(TelemetryRates::bounded_high_rate_transport());
    world
}

fn hal_config() -> HalConfig {
    HalConfig::new(ClockConfig::system_freq(300_000_000).unwrap())
}

#[cfg(feature = "release-loop-bench")]
struct LoopBench {
    mailbox: SharedMavlinkMailbox,
    next_report_us: u64,
    sequence: u8,
    count: u32,
    sum_us: u64,
    max_us: u32,
    missed_250us: u32,
    buckets: [u32; LOOP_BENCH_BUCKETS],
}

#[cfg(feature = "release-loop-bench")]
impl LoopBench {
    fn new(mailbox: SharedMavlinkMailbox) -> Self {
        Self {
            mailbox,
            next_report_us: Instant::now()
                .as_micros()
                .saturating_add(LOOP_BENCH_REPORT_US),
            sequence: 0,
            count: 0,
            sum_us: 0,
            max_us: 0,
            missed_250us: 0,
            buckets: [0; LOOP_BENCH_BUCKETS],
        }
    }

    fn record(&mut self, elapsed_us: u32, now_us: u64) {
        self.count = self.count.wrapping_add(1);
        self.sum_us = self.sum_us.saturating_add(elapsed_us as u64);
        self.max_us = self.max_us.max(elapsed_us);
        if elapsed_us > LOOP_BENCH_BUDGET_US {
            self.missed_250us = self.missed_250us.wrapping_add(1);
        }

        let bucket =
            (elapsed_us / LOOP_BENCH_BUCKET_US).min((LOOP_BENCH_BUCKETS - 1) as u32) as usize;
        self.buckets[bucket] = self.buckets[bucket].wrapping_add(1);

        if now_us >= self.next_report_us {
            self.report();
            self.reset(now_us);
        }
    }

    fn report(&mut self) {
        if self.count == 0 {
            return;
        }

        let avg_us = (self.sum_us / self.count as u64).min(u32::MAX as u64) as u32;
        let p90_us = self.percentile_us(90);
        let p99_us = self.percentile_us(99);
        let mut text = [0_u8; 50];
        let mut pos = 0;
        bench_write_bytes(&mut text, &mut pos, b"RLB n");
        bench_write_num(&mut text, &mut pos, self.count);
        bench_write_bytes(&mut text, &mut pos, b" a");
        bench_write_num(&mut text, &mut pos, avg_us);
        bench_write_bytes(&mut text, &mut pos, b" p90");
        bench_write_num(&mut text, &mut pos, p90_us);
        bench_write_bytes(&mut text, &mut pos, b" p99");
        bench_write_num(&mut text, &mut pos, p99_us);
        bench_write_bytes(&mut text, &mut pos, b" x");
        bench_write_num(&mut text, &mut pos, self.max_us);
        bench_write_bytes(&mut text, &mut pos, b" m");
        bench_write_num(&mut text, &mut pos, self.missed_250us);

        let frame = statustext_frame(self.sequence, &text);
        self.sequence = self.sequence.wrapping_add(1);
        let _ = self
            .mailbox
            .write_from_priority(&frame, SerialTxPriority::REPLACEABLE_TELEMETRY);
    }

    fn reset(&mut self, now_us: u64) {
        self.next_report_us = now_us.saturating_add(LOOP_BENCH_REPORT_US);
        self.count = 0;
        self.sum_us = 0;
        self.max_us = 0;
        self.missed_250us = 0;
        self.buckets = [0; LOOP_BENCH_BUCKETS];
    }

    fn percentile_us(&self, percentile: u32) -> u32 {
        let target = self.count.saturating_mul(percentile).saturating_add(99) / 100;
        let mut seen = 0_u32;
        for (index, count) in self.buckets.iter().enumerate() {
            seen = seen.saturating_add(*count);
            if seen >= target {
                return (index as u32).saturating_mul(LOOP_BENCH_BUCKET_US);
            }
        }
        self.max_us
    }
}

#[cfg(feature = "release-loop-bench")]
fn bench_write_bytes(out: &mut [u8; 50], pos: &mut usize, bytes: &[u8]) {
    for byte in bytes {
        if *pos >= out.len() {
            return;
        }
        out[*pos] = *byte;
        *pos += 1;
    }
}

#[cfg(feature = "release-loop-bench")]
fn bench_write_num(out: &mut [u8; 50], pos: &mut usize, mut value: u32) {
    let mut digits = [0_u8; 10];
    let mut len = 0;
    loop {
        digits[len] = b'0' + (value % 10) as u8;
        len += 1;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    while len > 0 {
        len -= 1;
        bench_write_bytes(out, pos, &digits[len..=len]);
    }
}

#[cfg(feature = "release-loop-bench")]
fn statustext_frame(sequence: u8, text: &[u8; 50]) -> [u8; MAVLINK_STATUSTEXT_LEN + 8] {
    let mut frame = [0_u8; MAVLINK_STATUSTEXT_LEN + 8];
    frame[0] = MAVLINK_V1_STX;
    frame[1] = MAVLINK_STATUSTEXT_LEN as u8;
    frame[2] = sequence;
    frame[3] = MAVLINK_SYSTEM_ID;
    frame[4] = MAVLINK_COMPONENT_ID;
    frame[5] = MAVLINK_STATUSTEXT_ID;
    frame[6] = 6;
    frame[7..57].copy_from_slice(text);

    let checksum = mavlink_x25(&frame[1..57], MAVLINK_STATUSTEXT_CRC_EXTRA);
    frame[57] = checksum as u8;
    frame[58] = (checksum >> 8) as u8;
    frame
}

#[cfg(feature = "release-loop-bench")]
fn mavlink_x25(bytes: &[u8], extra: u8) -> u16 {
    let mut crc = 0xffff_u16;
    for byte in bytes.iter().copied().chain(core::iter::once(extra)) {
        let tmp = byte ^ (crc as u8);
        let tmp = tmp ^ (tmp << 4);
        crc = (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4);
    }
    crc
}

#[entry]
fn main() -> ! {
    let peripherals = rp::init(hal_config());
    let config = Pico2WConfig::default();
    let mailbox = SHARED_MAVLINK_MAILBOX;

    spawn_core1_services(
        Core1Resources {
            core1: peripherals.CORE1,
            uart0: peripherals.UART0,
            uart1: peripherals.UART1,
            #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
            spi1: peripherals.SPI1,
            dma_ch0: peripherals.DMA_CH0,
            dma_ch1: peripherals.DMA_CH1,
            dma_ch2: peripherals.DMA_CH2,
            dma_ch3: peripherals.DMA_CH3,
            pin0: peripherals.PIN_0,
            pin1: peripherals.PIN_1,
            pin8: peripherals.PIN_8,
            pin9: peripherals.PIN_9,
            #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
            pin10: peripherals.PIN_10,
            #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
            pin11: peripherals.PIN_11,
            #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
            pin12: peripherals.PIN_12,
            #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
            pin13: peripherals.PIN_13,
        },
        mailbox,
    );

    let (mut board, pwm_driver) = board::Board::new_uart(config, None);

    let mut params = Params::default();
    if !board.read_params(&mut params) {
        params.set_defaults();
        let _ = board.write_params(&params);
    }

    let mut world = init_world(board, params, pwm_driver);
    #[cfg(feature = "release-loop-bench")]
    let mut loop_bench = LoopBench::new(mailbox);
    loop {
        #[cfg(feature = "release-loop-bench")]
        {
            let start_us = Instant::now().as_micros();
            let _ = world.run_once();
            let end_us = Instant::now().as_micros();
            loop_bench.record(
                end_us.saturating_sub(start_us).min(u32::MAX as u64) as u32,
                end_us,
            );
        }
        #[cfg(not(feature = "release-loop-bench"))]
        let _ = world.run_once();
    }
}
