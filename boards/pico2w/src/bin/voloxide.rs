#![no_std]
#![no_main]
#![cfg_attr(
    any(feature = "raw-swi-smoke", feature = "interrupt-executor-smoke"),
    allow(dead_code, unused_imports)
)]

use core::ptr::addr_of_mut;

use cortex_m_rt::entry;
use embassy_time::{Duration, Instant, Timer};
use panic_halt as _;
use pico2w::comms_core::{SHARED_MAVLINK_MAILBOX, SharedMavlinkMailbox};
use pico2w::gps::{
    SHARED_GNSS_QUEUE, UbxNavPvtParser, make_ubx_packet, record_gps_byte, record_nav_pvt,
};
#[cfg(any(
    feature = "synthetic-imu",
    all(
        feature = "ism330dhcx-driver",
        not(feature = "synthetic-imu"),
        not(feature = "interrupt-executor-smoke"),
        not(feature = "raw-swi-smoke")
    )
))]
use pico2w::ism330dhcx::SHARED_ISM330DHCX_IMU_QUEUE;
#[cfg(all(
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
use pico2w::ism330dhcx::{
    record_ism330dhcx_drdy_edge, record_ism330dhcx_init_attempt, record_ism330dhcx_init_failure,
    record_ism330dhcx_init_ok, record_ism330dhcx_read_error, record_ism330dhcx_read_ok,
};
use pico2w::pio_uart_dma::{PioUartDmaRx, PioUartDmaRxProgram};
use pico2w::rc_receiver::{
    CRSF_BAUDRATE, CrsfRcParser, SHARED_CRSF_RC_QUEUE, record_crsf_bytes, record_crsf_frame,
    record_crsf_read_error,
};
use pico2w::{board, config::Pico2WConfig, pwm::PioPwmDriver};
use rp2350_platform::hal::clocks::ClockConfig;
use rp2350_platform::hal::dma;
#[cfg(all(
    any(
        feature = "imu-producer-interrupt-executor",
        feature = "interrupt-executor-smoke"
    ),
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu")
))]
use rp2350_platform::hal::executor::InterruptExecutor;
#[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
use rp2350_platform::hal::gpio::{Input, Pull};
#[cfg(any(
    feature = "scope-timing-pins",
    all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu"))
))]
use rp2350_platform::hal::gpio::{Level, Output};
#[cfg(all(
    feature = "imu-producer-scope",
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu")
))]
use rp2350_platform::hal::peripherals::PIN_22;
#[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
use rp2350_platform::hal::peripherals::{PIN_10, PIN_11, PIN_12, PIN_13, PIN_14, SPI1};
#[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
use rp2350_platform::hal::spi::{Blocking, Config as SpiConfig, Phase, Polarity, Spi};
use rp2350_platform::hal::{
    self as rp, Peri, bind_interrupts,
    config::Config as HalConfig,
    executor::Executor,
    multicore::{Stack, spawn_core1},
    peripherals::{
        CORE1, DMA_CH0, DMA_CH1, DMA_CH2, DMA_CH3, DMA_CH4, PIN_0, PIN_1, PIN_6, PIN_7, PIN_8,
        PIN_9, PIO0, UART0, UART1,
    },
    pio::{InterruptHandler as PioInterruptHandler, Pio},
    pio_programs::uart::{PioUartTx, PioUartTxProgram},
    uart::{
        Async as UartAsync, Config as UartConfig, InterruptHandler as UartInterruptHandler, Uart,
        UartRx, UartTx,
    },
};
#[cfg(all(
    any(
        feature = "imu-producer-interrupt-executor",
        feature = "interrupt-executor-smoke",
        feature = "raw-swi-smoke"
    ),
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu")
))]
use rp2350_platform::hal::{
    interrupt,
    interrupt::{InterruptExt, Priority},
};
use static_cell::StaticCell;
#[cfg(feature = "release-loop-bench")]
use voloxide_core::board::SerialTxPriority;
#[cfg(any(
    feature = "synthetic-imu",
    all(
        feature = "ism330dhcx-driver",
        not(feature = "synthetic-imu"),
        not(feature = "interrupt-executor-smoke"),
        not(feature = "raw-swi-smoke")
    )
))]
use voloxide_core::packets::{ImuPacket, RosflightPacketHeader};
use voloxide_core::world::RealtimeSchedulerStep;
#[cfg(any(
    feature = "release-loop-classifier",
    feature = "release-loop-spike-counter"
))]
use voloxide_core::world::WorldRunClass;
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
#[cfg(all(
    any(
        feature = "imu-producer-interrupt-executor",
        feature = "interrupt-executor-smoke"
    ),
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu")
))]
static CORE1_IMU_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

const UART_TX_BATCH_BYTES: usize = 512;
const UART_RX_CHUNK_BYTES: usize = 16;
const UART_IDLE_DELAY_US: u64 = 50;
const MAVLINK_UART_BAUDRATE: u32 = 2_000_000;
const CRSF_RX_CHUNK_BYTES: usize = 32;
const GPS_UART_BAUDRATE: u32 = 115_200;
const MAIN_LOOP_MAX_SERVICE_DEFERRAL_US: u64 = 250;
#[cfg(feature = "synthetic-imu")]
const SYNTHETIC_IMU_PERIOD_US: u64 = synthetic_imu_period_us();
#[cfg(all(
    feature = "synthetic-imu-1khz",
    not(feature = "synthetic-imu-2khz"),
    not(feature = "synthetic-imu-3333hz"),
    not(feature = "synthetic-imu-4khz")
))]
const fn synthetic_imu_period_us() -> u64 {
    1_000
}
#[cfg(all(
    feature = "synthetic-imu-2khz",
    not(feature = "synthetic-imu-1khz"),
    not(feature = "synthetic-imu-3333hz"),
    not(feature = "synthetic-imu-4khz")
))]
const fn synthetic_imu_period_us() -> u64 {
    500
}
#[cfg(all(
    feature = "synthetic-imu-3333hz",
    not(feature = "synthetic-imu-1khz"),
    not(feature = "synthetic-imu-2khz"),
    not(feature = "synthetic-imu-4khz")
))]
const fn synthetic_imu_period_us() -> u64 {
    300
}
#[cfg(all(
    feature = "synthetic-imu-4khz",
    not(feature = "synthetic-imu-1khz"),
    not(feature = "synthetic-imu-2khz"),
    not(feature = "synthetic-imu-3333hz")
))]
const fn synthetic_imu_period_us() -> u64 {
    250
}
#[cfg(all(
    feature = "synthetic-imu",
    not(feature = "synthetic-imu-1khz"),
    not(feature = "synthetic-imu-2khz"),
    not(feature = "synthetic-imu-3333hz"),
    not(feature = "synthetic-imu-4khz")
))]
const fn synthetic_imu_period_us() -> u64 {
    125
}
#[cfg(any(
    all(feature = "synthetic-imu-1khz", feature = "synthetic-imu-2khz"),
    all(feature = "synthetic-imu-1khz", feature = "synthetic-imu-3333hz"),
    all(feature = "synthetic-imu-1khz", feature = "synthetic-imu-4khz"),
    all(feature = "synthetic-imu-2khz", feature = "synthetic-imu-3333hz"),
    all(feature = "synthetic-imu-2khz", feature = "synthetic-imu-4khz"),
    all(feature = "synthetic-imu-3333hz", feature = "synthetic-imu-4khz")
))]
compile_error!("select only one synthetic IMU rate feature");
#[cfg(any(
    all(feature = "raw-swi-smoke", feature = "interrupt-executor-smoke"),
    all(feature = "raw-swi-smoke", feature = "imu-producer-interrupt-executor"),
    all(
        feature = "interrupt-executor-smoke",
        feature = "imu-producer-interrupt-executor"
    )
))]
compile_error!(
    "select only one RP interrupt experiment feature: raw-swi-smoke, interrupt-executor-smoke, or imu-producer-interrupt-executor"
);

#[cfg(any(
    all(feature = "pre-control-scope", feature = "imu-producer-scope"),
    all(feature = "pre-control-scope", feature = "rc-command-scope"),
    all(feature = "pre-control-scope", feature = "control-scope-estimator"),
    all(feature = "pre-control-scope", feature = "control-scope-controller"),
    all(feature = "pre-control-scope", feature = "control-scope-mixer"),
    all(feature = "pre-control-scope", feature = "control-scope-pwm"),
    all(feature = "rc-command-scope", feature = "imu-producer-scope"),
    all(feature = "rc-command-scope", feature = "control-scope-estimator"),
    all(feature = "rc-command-scope", feature = "control-scope-controller"),
    all(feature = "rc-command-scope", feature = "control-scope-mixer"),
    all(feature = "rc-command-scope", feature = "control-scope-pwm"),
))]
compile_error!(
    "pre-control-scope and rc-command-scope use GP22 and cannot be combined with other GP22 scope modes"
);
#[cfg(all(
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
#[cfg(all(feature = "ism330dhcx-1k666", feature = "ism330dhcx-3k333"))]
compile_error!("select only one ISM330DHCX ODR feature: ism330dhcx-1k666 or ism330dhcx-3k333");
#[cfg(any(feature = "ism330dhcx-1k666", feature = "ism330dhcx-3k333"))]
const ISM330DHCX_IMU_PERIOD_US: u64 = 0;
#[cfg(all(
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
#[cfg(not(any(feature = "ism330dhcx-1k666", feature = "ism330dhcx-3k333")))]
const ISM330DHCX_IMU_PERIOD_US: u64 = 250;
#[cfg(all(
    feature = "ism330dhcx-driver",
    feature = "ism330dhcx-1k666",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
const ISM330DHCX_CTRL1_XL: u8 = 0x84;
#[cfg(all(
    feature = "ism330dhcx-driver",
    feature = "ism330dhcx-3k333",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
const ISM330DHCX_CTRL1_XL: u8 = 0x94;
#[cfg(all(
    feature = "ism330dhcx-driver",
    not(feature = "ism330dhcx-1k666"),
    not(feature = "ism330dhcx-3k333"),
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
const ISM330DHCX_CTRL1_XL: u8 = 0xa4;
#[cfg(all(
    feature = "ism330dhcx-driver",
    feature = "ism330dhcx-1k666",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
const ISM330DHCX_CTRL2_G: u8 = 0x8c;
#[cfg(all(
    feature = "ism330dhcx-driver",
    feature = "ism330dhcx-3k333",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
const ISM330DHCX_CTRL2_G: u8 = 0x9c;
#[cfg(all(
    feature = "ism330dhcx-driver",
    not(feature = "ism330dhcx-1k666"),
    not(feature = "ism330dhcx-3k333"),
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
const ISM330DHCX_CTRL2_G: u8 = 0xac;
#[cfg(all(
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
const ISM330DHCX_SPI_HZ: u32 = 10_000_000;
#[cfg(all(
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
const ISM330DHCX_WHO_AM_I: u8 = 0x6b;
#[cfg(feature = "release-loop-bench")]
const LOOP_BENCH_REPORT_US: u64 = 1_000_000;
#[cfg(all(
    feature = "release-loop-bench",
    feature = "ism330dhcx-1k666",
    not(feature = "synthetic-imu")
))]
const LOOP_BENCH_BUDGET_US: u32 = 600;
#[cfg(all(
    feature = "release-loop-bench",
    feature = "ism330dhcx-3k333",
    not(feature = "synthetic-imu")
))]
const LOOP_BENCH_BUDGET_US: u32 = 300;
#[cfg(all(feature = "release-loop-bench", feature = "synthetic-imu-3333hz"))]
const LOOP_BENCH_BUDGET_US: u32 = 300;
#[cfg(all(
    feature = "release-loop-bench",
    not(feature = "ism330dhcx-1k666"),
    not(feature = "ism330dhcx-3k333"),
    not(feature = "synthetic-imu-3333hz")
))]
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
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 =>
        dma::InterruptHandler<DMA_CH0>,
        dma::InterruptHandler<DMA_CH1>,
        dma::InterruptHandler<DMA_CH2>,
        dma::InterruptHandler<DMA_CH3>,
        dma::InterruptHandler<DMA_CH4>;
});

#[cfg(all(
    any(
        feature = "imu-producer-interrupt-executor",
        feature = "interrupt-executor-smoke"
    ),
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu")
))]
#[interrupt]
unsafe fn SIO_IRQ_BELL() {
    unsafe { CORE1_IMU_EXECUTOR.on_interrupt() };
}

#[cfg(all(
    feature = "raw-swi-smoke",
    feature = "imu-producer-scope",
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu")
))]
#[interrupt]
unsafe fn SIO_IRQ_BELL() {
    const SIO_GPIO_OUT_XOR0: *mut u32 = 0xd000_0028 as *mut u32;
    unsafe { core::ptr::write_volatile(SIO_GPIO_OUT_XOR0, 1 << 22) };
}

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

#[cfg(all(
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
#[embassy_executor::task]
async fn ism330dhcx_imu_task(
    mut spi: Spi<'static, SPI1, Blocking>,
    mut cs: Output<'static>,
    mut drdy: Input<'static>,
    #[cfg(feature = "imu-producer-scope")] mut imu_scope: Output<'static>,
) -> ! {
    let mut seq = 0_u32;
    loop {
        record_ism330dhcx_init_attempt();
        match ism330dhcx_init(&mut spi, &mut cs) {
            Ok(who_am_i) => {
                record_ism330dhcx_init_ok(who_am_i);
            }
            Err(who_am_i) => {
                record_ism330dhcx_init_failure(who_am_i);
                Timer::after_millis(1_000).await;
                continue;
            }
        }

        {
            loop {
                if ISM330DHCX_IMU_PERIOD_US == 0 {
                    drdy.wait_for_rising_edge().await;
                    record_ism330dhcx_drdy_edge();
                    #[cfg(feature = "imu-producer-scope")]
                    imu_scope.set_high();
                    let now_us = Instant::now().as_micros();
                    match ism330dhcx_read_packet(&mut spi, &mut cs, now_us, seq) {
                        Ok(packet) => {
                            record_ism330dhcx_read_ok();
                            SHARED_ISM330DHCX_IMU_QUEUE.push_from_interrupt(packet);
                            seq = seq.wrapping_add(1);
                        }
                        Err(()) => record_ism330dhcx_read_error(),
                    }
                    #[cfg(feature = "imu-producer-scope")]
                    imu_scope.set_low();
                } else {
                    Timer::after(Duration::from_micros(ISM330DHCX_IMU_PERIOD_US)).await;
                    #[cfg(feature = "imu-producer-scope")]
                    imu_scope.set_high();
                    let now_us = Instant::now().as_micros();
                    match ism330dhcx_read_packet(&mut spi, &mut cs, now_us, seq) {
                        Ok(packet) => {
                            record_ism330dhcx_read_ok();
                            SHARED_ISM330DHCX_IMU_QUEUE.push_from_interrupt(packet);
                            seq = seq.wrapping_add(1);
                        }
                        Err(()) => record_ism330dhcx_read_error(),
                    }
                    #[cfg(feature = "imu-producer-scope")]
                    imu_scope.set_low();
                };
            }
        }
    }
}

#[cfg(all(
    feature = "interrupt-executor-smoke",
    feature = "imu-producer-scope",
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu")
))]
#[embassy_executor::task]
async fn interrupt_executor_smoke_task(mut scope: Output<'static>) -> ! {
    loop {
        scope.set_high();
        Timer::after(Duration::from_micros(200)).await;
        scope.set_low();
        Timer::after(Duration::from_micros(800)).await;
    }
}

#[cfg(all(
    any(feature = "interrupt-executor-smoke", feature = "raw-swi-smoke"),
    feature = "imu-producer-scope",
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu")
))]
fn smoke_marker_delay() {
    for _ in 0..60_000 {
        cortex_m::asm::nop();
    }
}

#[cfg(all(
    feature = "raw-swi-smoke",
    feature = "imu-producer-scope",
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu")
))]
#[inline(never)]
fn run_raw_swi_smoke(mut scope: Output<'static>) -> ! {
    scope.set_high();
    smoke_marker_delay();
    unsafe { cortex_m::interrupt::enable() };
    interrupt::SIO_IRQ_BELL.set_priority(Priority::P1);
    interrupt::SIO_IRQ_BELL.unpend();
    unsafe { interrupt::SIO_IRQ_BELL.enable() };
    loop {
        interrupt::SIO_IRQ_BELL.pend();
        smoke_marker_delay();
    }
}

#[cfg(all(
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
fn ism330dhcx_init(
    spi: &mut Spi<'static, SPI1, Blocking>,
    cs: &mut Output<'static>,
) -> Result<u8, Option<u8>> {
    let who_am_i = ism330dhcx_read_reg(spi, cs, 0x0f).map_err(|_| None)?;
    if who_am_i != ISM330DHCX_WHO_AM_I {
        return Err(Some(who_am_i));
    }
    ism330dhcx_write_reg(spi, cs, 0x12, 0x44).map_err(|_| Some(who_am_i))?;
    ism330dhcx_write_reg(spi, cs, 0x10, ISM330DHCX_CTRL1_XL).map_err(|_| Some(who_am_i))?;
    ism330dhcx_write_reg(spi, cs, 0x11, ISM330DHCX_CTRL2_G).map_err(|_| Some(who_am_i))?;
    ism330dhcx_write_reg(spi, cs, 0x0b, 0x80).map_err(|_| Some(who_am_i))?;
    ism330dhcx_write_reg(spi, cs, 0x0d, 0x03).map_err(|_| Some(who_am_i))?;
    Ok(who_am_i)
}

#[cfg(all(
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
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

#[cfg(all(
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
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

#[cfg(all(
    feature = "ism330dhcx-driver",
    not(feature = "synthetic-imu"),
    not(feature = "interrupt-executor-smoke"),
    not(feature = "raw-swi-smoke")
))]
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
            record_crsf_bytes(rx.len());
            if let Some(packet) = parser.push_bytes(&rx, Instant::now().as_micros()) {
                record_crsf_frame();
                SHARED_CRSF_RC_QUEUE.push_from_receiver_task(packet);
            }
        } else {
            record_crsf_read_error();
            Timer::after(Duration::from_micros(UART_IDLE_DELAY_US)).await;
        }
    }
}

#[embassy_executor::task]
async fn gps_pio_task(
    mut gps_rx: PioUartDmaRx<'static, PIO0, 0>,
    mut gps_tx: PioUartTx<'static, PIO0, 1>,
    mut gps_dma: dma::Channel<'static>,
) -> ! {
    gps_configure_nav_pvt(&mut gps_tx).await;

    let mut parser = UbxNavPvtParser::new();
    let mut next_poll_us = Instant::now().as_micros().saturating_add(250_000);
    let mut rx_words = [0_u32; 128];
    loop {
        gps_rx.read_words_dma(&mut gps_dma, &mut rx_words).await;
        let mut now_us = Instant::now().as_micros();
        for word in rx_words {
            let byte = word as u8;
            record_gps_byte(byte);
            if let Some(packet) = parser.feed_byte(byte, now_us) {
                record_nav_pvt();
                SHARED_GNSS_QUEUE.push_from_receiver_task(Ok(packet));
            }
            now_us = Instant::now().as_micros();
        }
        if gps_rx.stalled() {
            next_poll_us = now_us.saturating_add(1_000_000);
        } else if now_us >= next_poll_us {
            gps_poll_nav_pvt(&mut gps_tx).await;
            next_poll_us = now_us.saturating_add(1_000_000);
        }
    }
}

async fn gps_configure_nav_pvt(gps_tx: &mut PioUartTx<'static, PIO0, 1>) {
    let mut packet = [0_u8; 40];

    if let Some(len) = make_ubx_packet(0x06, 0x01, &[0x01, 0x07, 1], &mut packet) {
        gps_write_packet(gps_tx, &packet[..len]).await;
    }
    Timer::after_millis(50).await;

    if let Some(len) = make_ubx_packet(0x06, 0x01, &[0x01, 0x07, 0, 1, 0, 0, 0, 0], &mut packet) {
        gps_write_packet(gps_tx, &packet[..len]).await;
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
        gps_write_packet(gps_tx, &packet[..len]).await;
    }
}

async fn gps_poll_nav_pvt(gps_tx: &mut PioUartTx<'static, PIO0, 1>) {
    let mut packet = [0_u8; 8];
    if let Some(len) = make_ubx_packet(0x01, 0x07, &[], &mut packet) {
        gps_write_packet(gps_tx, &packet[..len]).await;
    }
}

async fn gps_write_packet(gps_tx: &mut PioUartTx<'static, PIO0, 1>, bytes: &[u8]) {
    for byte in bytes {
        gps_tx.write_u8(*byte).await;
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
    pio0: Peri<'static, PIO0>,
    #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
    spi1: Peri<'static, SPI1>,
    dma_ch0: Peri<'static, DMA_CH0>,
    dma_ch1: Peri<'static, DMA_CH1>,
    dma_ch2: Peri<'static, DMA_CH2>,
    dma_ch3: Peri<'static, DMA_CH3>,
    dma_ch4: Peri<'static, DMA_CH4>,
    pin0: Peri<'static, PIN_0>,
    pin1: Peri<'static, PIN_1>,
    pin6: Peri<'static, PIN_6>,
    pin7: Peri<'static, PIN_7>,
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
    #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
    pin14: Peri<'static, PIN_14>,
    #[cfg(all(
        feature = "imu-producer-scope",
        feature = "ism330dhcx-driver",
        not(feature = "synthetic-imu")
    ))]
    pin22: Peri<'static, PIN_22>,
}

#[cfg(feature = "scope-timing-pins")]
struct ScopeTimingPins {
    whole_loop: Output<'static>,
    whole_loop_high: bool,
}

#[cfg(all(
    feature = "scope-timing-pins",
    not(feature = "imu-producer-scope"),
    not(feature = "pre-control-scope"),
    not(feature = "rc-command-scope"),
    not(any(
        feature = "control-scope-estimator",
        feature = "control-scope-controller",
        feature = "control-scope-mixer",
        feature = "control-scope-pwm"
    ))
))]
const SCOPE_GP22_MARKS_SERVICE: bool = true;

#[cfg(all(
    feature = "scope-timing-pins",
    any(
        feature = "imu-producer-scope",
        feature = "pre-control-scope",
        feature = "rc-command-scope",
        feature = "control-scope-estimator",
        feature = "control-scope-controller",
        feature = "control-scope-mixer",
        feature = "control-scope-pwm"
    )
))]
const SCOPE_GP22_MARKS_SERVICE: bool = false;

#[cfg(feature = "scope-timing-pins")]
impl ScopeTimingPins {
    fn new(whole_loop: Output<'static>) -> Self {
        Self {
            whole_loop,
            whole_loop_high: false,
        }
    }

    fn mark_loop_boundary(&mut self) {
        self.whole_loop_high = !self.whole_loop_high;
        if self.whole_loop_high {
            self.whole_loop.set_high();
        } else {
            self.whole_loop.set_low();
        }
    }
}

fn spawn_core1_services(resources: Core1Resources, mailbox: SharedMavlinkMailbox) {
    spawn_core1(
        resources.core1,
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        move || {
            mailbox.set_comms_state(20);
            #[cfg(all(
                feature = "raw-swi-smoke",
                feature = "imu-producer-scope",
                feature = "ism330dhcx-driver",
                not(feature = "synthetic-imu")
            ))]
            {
                let imu_scope = Output::new(resources.pin22, Level::Low);
                run_raw_swi_smoke(imu_scope);
            }

            #[cfg(not(all(
                feature = "raw-swi-smoke",
                feature = "imu-producer-scope",
                feature = "ism330dhcx-driver",
                not(feature = "synthetic-imu")
            )))]
            {
                #[cfg(not(feature = "imu-producer-isolation"))]
                configure_core1_transport_interrupt_priorities();

                #[cfg(not(feature = "imu-producer-isolation"))]
                let mavlink_uart = Uart::new(
                    resources.uart0,
                    resources.pin0,
                    resources.pin1,
                    Irqs,
                    resources.dma_ch0,
                    resources.dma_ch1,
                    mavlink_uart_config(),
                );
                #[cfg(not(feature = "imu-producer-isolation"))]
                let (uart_tx, uart_rx) = mavlink_uart.split();

                #[cfg(not(feature = "imu-producer-isolation"))]
                let crsf_uart = Uart::new(
                    resources.uart1,
                    resources.pin8,
                    resources.pin9,
                    Irqs,
                    resources.dma_ch2,
                    resources.dma_ch3,
                    crsf_uart_config(),
                );
                #[cfg(not(feature = "imu-producer-isolation"))]
                let (_crsf_tx, crsf_rx) = crsf_uart.split();

                #[cfg(not(feature = "imu-producer-isolation"))]
                let mut pio = Pio::new(resources.pio0, Irqs);
                #[cfg(not(feature = "imu-producer-isolation"))]
                let gps_rx_program = PioUartDmaRxProgram::new(&mut pio.common);
                #[cfg(not(feature = "imu-producer-isolation"))]
                let gps_tx_program = PioUartTxProgram::new(&mut pio.common);
                #[cfg(not(feature = "imu-producer-isolation"))]
                let gps_rx = PioUartDmaRx::new(
                    GPS_UART_BAUDRATE,
                    &mut pio.common,
                    pio.sm0,
                    resources.pin7,
                    &gps_rx_program,
                );
                #[cfg(not(feature = "imu-producer-isolation"))]
                let gps_tx = PioUartTx::new(
                    GPS_UART_BAUDRATE,
                    &mut pio.common,
                    pio.sm1,
                    resources.pin6,
                    &gps_tx_program,
                );
                #[cfg(not(feature = "imu-producer-isolation"))]
                let gps_dma = dma::Channel::new(resources.dma_ch4, Irqs);

                #[cfg(all(
                    feature = "ism330dhcx-driver",
                    not(feature = "synthetic-imu"),
                    not(feature = "interrupt-executor-smoke")
                ))]
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
                #[cfg(all(
                    feature = "ism330dhcx-driver",
                    not(feature = "synthetic-imu"),
                    not(feature = "interrupt-executor-smoke")
                ))]
                let imu_cs = Output::new(resources.pin13, Level::High);
                #[cfg(all(
                    feature = "ism330dhcx-driver",
                    not(feature = "synthetic-imu"),
                    not(feature = "interrupt-executor-smoke")
                ))]
                let imu_drdy = Input::new(resources.pin14, Pull::Down);
                #[cfg(all(
                    feature = "imu-producer-scope",
                    feature = "ism330dhcx-driver",
                    not(feature = "synthetic-imu")
                ))]
                let imu_scope = Output::new(resources.pin22, Level::Low);
                #[cfg(feature = "interrupt-executor-smoke")]
                let mut imu_scope = imu_scope;

                let executor = CORE1_EXECUTOR.init(Executor::new());
                #[cfg(all(
                    feature = "interrupt-executor-smoke",
                    feature = "imu-producer-scope",
                    feature = "ism330dhcx-driver",
                    not(feature = "synthetic-imu")
                ))]
                {
                    imu_scope.set_high();
                    smoke_marker_delay();
                    imu_scope.set_low();
                    smoke_marker_delay();
                    interrupt::SIO_IRQ_BELL.set_priority(Priority::P1);
                    let imu_spawner = CORE1_IMU_EXECUTOR.start(interrupt::SIO_IRQ_BELL);
                    imu_scope.set_high();
                    smoke_marker_delay();
                    imu_scope.set_low();
                    smoke_marker_delay();
                    if let Ok(token) = interrupt_executor_smoke_task(imu_scope) {
                        imu_spawner.spawn(token);
                    }
                    interrupt::SIO_IRQ_BELL.pend();
                }
                #[cfg(all(
                    feature = "imu-producer-interrupt-executor",
                    not(feature = "interrupt-executor-smoke"),
                    feature = "ism330dhcx-driver",
                    not(feature = "synthetic-imu")
                ))]
                {
                    interrupt::SIO_IRQ_BELL.set_priority(Priority::P1);
                    let imu_spawner = CORE1_IMU_EXECUTOR.start(interrupt::SIO_IRQ_BELL);
                    if let Ok(token) = ism330dhcx_imu_task(
                        imu_spi,
                        imu_cs,
                        imu_drdy,
                        #[cfg(feature = "imu-producer-scope")]
                        imu_scope,
                    ) {
                        imu_spawner.spawn(token);
                    }
                }
                mailbox.set_comms_state(21);
                executor.run(|spawner| {
                    #[cfg(not(any(
                        feature = "imu-producer-isolation",
                        feature = "core1-disable-heartbeat"
                    )))]
                    if let Ok(token) = core1_heartbeat_task(mailbox) {
                        spawner.spawn(token);
                    }
                    #[cfg(not(any(
                        feature = "imu-producer-isolation",
                        feature = "core1-disable-mavlink-tx"
                    )))]
                    if let Ok(token) = uart_tx_task(uart_tx, mailbox) {
                        spawner.spawn(token);
                    }
                    #[cfg(not(any(
                        feature = "imu-producer-isolation",
                        feature = "core1-disable-mavlink-rx"
                    )))]
                    if let Ok(token) = uart_rx_task(uart_rx, mailbox) {
                        spawner.spawn(token);
                    }
                    #[cfg(not(any(
                        feature = "imu-producer-isolation",
                        feature = "core1-disable-crsf"
                    )))]
                    if let Ok(token) = crsf_rx_task(crsf_rx) {
                        spawner.spawn(token);
                    }
                    #[cfg(not(any(
                        feature = "imu-producer-isolation",
                        feature = "core1-disable-gps"
                    )))]
                    if let Ok(token) = gps_pio_task(gps_rx, gps_tx, gps_dma) {
                        spawner.spawn(token);
                    }
                    #[cfg(feature = "synthetic-imu")]
                    if let Ok(token) = synthetic_imu_task() {
                        spawner.spawn(token);
                    }
                    #[cfg(all(
                        feature = "ism330dhcx-driver",
                        not(feature = "synthetic-imu"),
                        not(feature = "imu-producer-interrupt-executor"),
                        not(feature = "interrupt-executor-smoke")
                    ))]
                    if let Ok(token) = ism330dhcx_imu_task(
                        imu_spi,
                        imu_cs,
                        imu_drdy,
                        #[cfg(feature = "imu-producer-scope")]
                        imu_scope,
                    ) {
                        spawner.spawn(token);
                    }
                    mailbox.set_comms_state(22);
                })
            }
        },
    );
}

#[cfg(not(feature = "imu-producer-isolation"))]
fn configure_core1_transport_interrupt_priorities() {
    interrupt::UART0_IRQ.set_priority(Priority::P3);
    interrupt::UART1_IRQ.set_priority(Priority::P3);
    interrupt::PIO0_IRQ_0.set_priority(Priority::P3);
    interrupt::DMA_IRQ_0.set_priority(Priority::P3);
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
        imu_hz: 400,
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
    #[cfg(all(
        feature = "release-loop-bench",
        feature = "ism330dhcx-driver",
        not(feature = "synthetic-imu")
    ))]
    world.set_telemetry_rates(TelemetryRates {
        imu_hz: 50,
        attitude_hz: 1,
        output_raw_hz: 1,
        diff_pressure_hz: 1,
        baro_hz: 5,
        mag_hz: 1,
        range_hz: 1,
        battery_hz: 1,
        gnss_hz: 1,
        rc_hz: 50,
        output_raw_imu_divisor: 0,
    });
    #[cfg(all(
        not(feature = "synthetic-imu"),
        not(feature = "release-loop-bench")
    ))]
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
    #[cfg(any(
        feature = "release-loop-classifier",
        feature = "release-loop-spike-counter"
    ))]
    slow_rx: u32,
    #[cfg(any(
        feature = "release-loop-classifier",
        feature = "release-loop-spike-counter"
    ))]
    slow_raw_imu: u32,
    #[cfg(any(
        feature = "release-loop-classifier",
        feature = "release-loop-spike-counter"
    ))]
    slow_raw_baro: u32,
    #[cfg(any(
        feature = "release-loop-classifier",
        feature = "release-loop-spike-counter"
    ))]
    slow_raw_rc: u32,
    #[cfg(any(
        feature = "release-loop-classifier",
        feature = "release-loop-spike-counter"
    ))]
    slow_telemetry_due: u32,
    #[cfg(any(
        feature = "release-loop-classifier",
        feature = "release-loop-spike-counter"
    ))]
    slow_telemetry_deferred: u32,
    #[cfg(any(
        feature = "release-loop-classifier",
        feature = "release-loop-spike-counter"
    ))]
    slow_after_control_over_budget: u32,
    #[cfg(any(
        feature = "release-loop-classifier",
        feature = "release-loop-spike-counter"
    ))]
    slow_control: u32,
    #[cfg(feature = "release-loop-classifier")]
    sum_estimator_us: u32,
    #[cfg(feature = "release-loop-classifier")]
    sum_controller_us: u32,
    #[cfg(feature = "release-loop-classifier")]
    sum_mixer_us: u32,
    #[cfg(feature = "release-loop-classifier")]
    sum_pwm_us: u32,
    #[cfg(feature = "release-loop-classifier")]
    max_estimator_us: u16,
    #[cfg(feature = "release-loop-classifier")]
    max_controller_us: u16,
    #[cfg(feature = "release-loop-classifier")]
    max_mixer_us: u16,
    #[cfg(feature = "release-loop-classifier")]
    max_pwm_us: u16,
    #[cfg(feature = "release-loop-classifier")]
    control_count: u32,
    #[cfg(feature = "release-loop-classifier")]
    control_sum_us: u64,
    #[cfg(feature = "release-loop-classifier")]
    control_max_us: u32,
    #[cfg(feature = "release-loop-classifier")]
    control_missed_budget: u32,
    #[cfg(feature = "release-loop-classifier")]
    control_buckets: [u32; LOOP_BENCH_BUCKETS],
    #[cfg(feature = "release-loop-classifier")]
    no_control_count: u32,
    #[cfg(feature = "release-loop-classifier")]
    no_control_sum_us: u64,
    #[cfg(feature = "release-loop-classifier")]
    no_control_max_us: u32,
    #[cfg(feature = "release-loop-classifier")]
    no_control_missed_budget: u32,
    #[cfg(feature = "release-loop-classifier")]
    no_control_buckets: [u32; LOOP_BENCH_BUCKETS],
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
            #[cfg(any(
                feature = "release-loop-classifier",
                feature = "release-loop-spike-counter"
            ))]
            slow_rx: 0,
            #[cfg(any(
                feature = "release-loop-classifier",
                feature = "release-loop-spike-counter"
            ))]
            slow_raw_imu: 0,
            #[cfg(any(
                feature = "release-loop-classifier",
                feature = "release-loop-spike-counter"
            ))]
            slow_raw_baro: 0,
            #[cfg(any(
                feature = "release-loop-classifier",
                feature = "release-loop-spike-counter"
            ))]
            slow_raw_rc: 0,
            #[cfg(any(
                feature = "release-loop-classifier",
                feature = "release-loop-spike-counter"
            ))]
            slow_telemetry_due: 0,
            #[cfg(any(
                feature = "release-loop-classifier",
                feature = "release-loop-spike-counter"
            ))]
            slow_telemetry_deferred: 0,
            #[cfg(any(
                feature = "release-loop-classifier",
                feature = "release-loop-spike-counter"
            ))]
            slow_after_control_over_budget: 0,
            #[cfg(any(
                feature = "release-loop-classifier",
                feature = "release-loop-spike-counter"
            ))]
            slow_control: 0,
            #[cfg(feature = "release-loop-classifier")]
            sum_estimator_us: 0,
            #[cfg(feature = "release-loop-classifier")]
            sum_controller_us: 0,
            #[cfg(feature = "release-loop-classifier")]
            sum_mixer_us: 0,
            #[cfg(feature = "release-loop-classifier")]
            sum_pwm_us: 0,
            #[cfg(feature = "release-loop-classifier")]
            max_estimator_us: 0,
            #[cfg(feature = "release-loop-classifier")]
            max_controller_us: 0,
            #[cfg(feature = "release-loop-classifier")]
            max_mixer_us: 0,
            #[cfg(feature = "release-loop-classifier")]
            max_pwm_us: 0,
            #[cfg(feature = "release-loop-classifier")]
            control_count: 0,
            #[cfg(feature = "release-loop-classifier")]
            control_sum_us: 0,
            #[cfg(feature = "release-loop-classifier")]
            control_max_us: 0,
            #[cfg(feature = "release-loop-classifier")]
            control_missed_budget: 0,
            #[cfg(feature = "release-loop-classifier")]
            control_buckets: [0; LOOP_BENCH_BUCKETS],
            #[cfg(feature = "release-loop-classifier")]
            no_control_count: 0,
            #[cfg(feature = "release-loop-classifier")]
            no_control_sum_us: 0,
            #[cfg(feature = "release-loop-classifier")]
            no_control_max_us: 0,
            #[cfg(feature = "release-loop-classifier")]
            no_control_missed_budget: 0,
            #[cfg(feature = "release-loop-classifier")]
            no_control_buckets: [0; LOOP_BENCH_BUCKETS],
            buckets: [0; LOOP_BENCH_BUCKETS],
        }
    }

    #[cfg(not(any(
        feature = "release-loop-classifier",
        feature = "release-loop-spike-counter"
    )))]
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

    #[cfg(any(
        feature = "release-loop-classifier",
        feature = "release-loop-spike-counter"
    ))]
    fn record(&mut self, elapsed_us: u32, now_us: u64, class: WorldRunClass) {
        self.count = self.count.wrapping_add(1);
        self.sum_us = self.sum_us.saturating_add(elapsed_us as u64);
        self.max_us = self.max_us.max(elapsed_us);
        #[cfg(feature = "release-loop-classifier")]
        self.record_loop_closure_class(elapsed_us, class.ran_control);
        if elapsed_us > LOOP_BENCH_BUDGET_US {
            self.missed_250us = self.missed_250us.wrapping_add(1);
            self.slow_rx = self.slow_rx.wrapping_add(class.had_rx as u32);
            self.slow_raw_imu = self.slow_raw_imu.wrapping_add(class.had_raw_imu as u32);
            self.slow_raw_baro = self.slow_raw_baro.wrapping_add(class.had_raw_baro as u32);
            self.slow_raw_rc = self.slow_raw_rc.wrapping_add(class.had_raw_rc as u32);
            self.slow_telemetry_due = self
                .slow_telemetry_due
                .wrapping_add(class.telemetry_due as u32);
            self.slow_telemetry_deferred = self
                .slow_telemetry_deferred
                .wrapping_add(class.telemetry_deferred as u32);
            self.slow_after_control_over_budget = self
                .slow_after_control_over_budget
                .wrapping_add((class.elapsed_after_control_us > LOOP_BENCH_BUDGET_US) as u32);
            self.slow_control = self.slow_control.wrapping_add(class.ran_control as u32);
            #[cfg(feature = "release-loop-classifier")]
            {
                self.sum_estimator_us = self
                    .sum_estimator_us
                    .wrapping_add(class.estimator_us as u32);
                self.sum_controller_us = self
                    .sum_controller_us
                    .wrapping_add(class.controller_us as u32);
                self.sum_mixer_us = self.sum_mixer_us.wrapping_add(class.mixer_us as u32);
                self.sum_pwm_us = self.sum_pwm_us.wrapping_add(class.pwm_us as u32);
                self.max_estimator_us = self.max_estimator_us.max(class.estimator_us);
                self.max_controller_us = self.max_controller_us.max(class.controller_us);
                self.max_mixer_us = self.max_mixer_us.max(class.mixer_us);
                self.max_pwm_us = self.max_pwm_us.max(class.pwm_us);
            }
        }

        let bucket =
            (elapsed_us / LOOP_BENCH_BUCKET_US).min((LOOP_BENCH_BUCKETS - 1) as u32) as usize;
        self.buckets[bucket] = self.buckets[bucket].wrapping_add(1);

        if now_us >= self.next_report_us {
            self.report();
            self.reset(now_us);
        }
    }

    #[cfg(feature = "release-loop-classifier")]
    fn record_loop_closure_class(&mut self, elapsed_us: u32, ran_control: bool) {
        let bucket =
            (elapsed_us / LOOP_BENCH_BUCKET_US).min((LOOP_BENCH_BUCKETS - 1) as u32) as usize;
        if ran_control {
            self.control_count = self.control_count.wrapping_add(1);
            self.control_sum_us = self.control_sum_us.saturating_add(elapsed_us as u64);
            self.control_max_us = self.control_max_us.max(elapsed_us);
            self.control_buckets[bucket] = self.control_buckets[bucket].wrapping_add(1);
            if elapsed_us > LOOP_BENCH_BUDGET_US {
                self.control_missed_budget = self.control_missed_budget.wrapping_add(1);
            }
        } else {
            self.no_control_count = self.no_control_count.wrapping_add(1);
            self.no_control_sum_us = self.no_control_sum_us.saturating_add(elapsed_us as u64);
            self.no_control_max_us = self.no_control_max_us.max(elapsed_us);
            self.no_control_buckets[bucket] = self.no_control_buckets[bucket].wrapping_add(1);
            if elapsed_us > LOOP_BENCH_BUDGET_US {
                self.no_control_missed_budget = self.no_control_missed_budget.wrapping_add(1);
            }
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

        #[cfg(any(
            feature = "release-loop-classifier",
            feature = "release-loop-spike-counter"
        ))]
        {
            let mut text = [0_u8; 50];
            let mut pos = 0;
            bench_write_bytes(&mut text, &mut pos, b"SLC m");
            bench_write_num(&mut text, &mut pos, self.missed_250us);
            bench_write_bytes(&mut text, &mut pos, b" rx");
            bench_write_num(&mut text, &mut pos, self.slow_rx);
            bench_write_bytes(&mut text, &mut pos, b" im");
            bench_write_num(&mut text, &mut pos, self.slow_raw_imu);
            bench_write_bytes(&mut text, &mut pos, b" br");
            bench_write_num(&mut text, &mut pos, self.slow_raw_baro);
            bench_write_bytes(&mut text, &mut pos, b" rc");
            bench_write_num(&mut text, &mut pos, self.slow_raw_rc);
            bench_write_bytes(&mut text, &mut pos, b" te");
            bench_write_num(&mut text, &mut pos, self.slow_telemetry_due);
            bench_write_bytes(&mut text, &mut pos, b" df");
            bench_write_num(&mut text, &mut pos, self.slow_telemetry_deferred);
            bench_write_bytes(&mut text, &mut pos, b" ac");
            bench_write_num(&mut text, &mut pos, self.slow_after_control_over_budget);

            let frame = statustext_frame(self.sequence, &text);
            self.sequence = self.sequence.wrapping_add(1);
            let _ = self
                .mailbox
                .write_from_priority(&frame, SerialTxPriority::REPLACEABLE_TELEMETRY);
        }

        #[cfg(feature = "release-loop-classifier")]
        {
            let control_buckets = self.control_buckets;
            let no_control_buckets = self.no_control_buckets;
            self.report_loop_closure_class(
                b'C',
                self.control_count,
                self.control_sum_us,
                self.control_max_us,
                self.control_missed_budget,
                &control_buckets,
            );
            self.report_loop_closure_class(
                b'N',
                self.no_control_count,
                self.no_control_sum_us,
                self.no_control_max_us,
                self.no_control_missed_budget,
                &no_control_buckets,
            );

            let mut text = [0_u8; 50];
            let mut pos = 0;
            let slow_control = self.slow_control.max(1);
            bench_write_bytes(&mut text, &mut pos, b"CLC e");
            bench_write_count_max(
                &mut text,
                &mut pos,
                self.sum_estimator_us / slow_control,
                self.max_estimator_us as u32,
            );
            bench_write_bytes(&mut text, &mut pos, b" c");
            bench_write_count_max(
                &mut text,
                &mut pos,
                self.sum_controller_us / slow_control,
                self.max_controller_us as u32,
            );
            bench_write_bytes(&mut text, &mut pos, b" m");
            bench_write_count_max(
                &mut text,
                &mut pos,
                self.sum_mixer_us / slow_control,
                self.max_mixer_us as u32,
            );
            bench_write_bytes(&mut text, &mut pos, b" p");
            bench_write_count_max(
                &mut text,
                &mut pos,
                self.sum_pwm_us / slow_control,
                self.max_pwm_us as u32,
            );

            let frame = statustext_frame(self.sequence, &text);
            self.sequence = self.sequence.wrapping_add(1);
            let _ = self
                .mailbox
                .write_from_priority(&frame, SerialTxPriority::REPLACEABLE_TELEMETRY);
        }
    }

    #[cfg(feature = "release-loop-classifier")]
    fn report_loop_closure_class(
        &mut self,
        class: u8,
        count: u32,
        sum_us: u64,
        max_us: u32,
        missed_budget: u32,
        buckets: &[u32; LOOP_BENCH_BUCKETS],
    ) {
        if count == 0 {
            return;
        }

        let avg_us = (sum_us / count as u64).min(u32::MAX as u64) as u32;
        let p90_us = Self::percentile_from(count, buckets, 90, max_us);
        let p99_us = Self::percentile_from(count, buckets, 99, max_us);
        let mut text = [0_u8; 50];
        let mut pos = 0;
        bench_write_bytes(&mut text, &mut pos, b"RLC ");
        bench_write_bytes(&mut text, &mut pos, &[class]);
        bench_write_bytes(&mut text, &mut pos, b" n");
        bench_write_num(&mut text, &mut pos, count);
        bench_write_bytes(&mut text, &mut pos, b" a");
        bench_write_num(&mut text, &mut pos, avg_us);
        bench_write_bytes(&mut text, &mut pos, b" p90");
        bench_write_num(&mut text, &mut pos, p90_us);
        bench_write_bytes(&mut text, &mut pos, b" p99");
        bench_write_num(&mut text, &mut pos, p99_us);
        bench_write_bytes(&mut text, &mut pos, b" x");
        bench_write_num(&mut text, &mut pos, max_us);
        bench_write_bytes(&mut text, &mut pos, b" m");
        bench_write_num(&mut text, &mut pos, missed_budget);

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
        #[cfg(any(
            feature = "release-loop-classifier",
            feature = "release-loop-spike-counter"
        ))]
        {
            self.slow_rx = 0;
            self.slow_raw_imu = 0;
            self.slow_raw_baro = 0;
            self.slow_raw_rc = 0;
            self.slow_telemetry_due = 0;
            self.slow_telemetry_deferred = 0;
            self.slow_after_control_over_budget = 0;
            self.slow_control = 0;
        }
        #[cfg(feature = "release-loop-classifier")]
        {
            self.sum_estimator_us = 0;
            self.sum_controller_us = 0;
            self.sum_mixer_us = 0;
            self.sum_pwm_us = 0;
            self.max_estimator_us = 0;
            self.max_controller_us = 0;
            self.max_mixer_us = 0;
            self.max_pwm_us = 0;
            self.control_count = 0;
            self.control_sum_us = 0;
            self.control_max_us = 0;
            self.control_missed_budget = 0;
            self.control_buckets = [0; LOOP_BENCH_BUCKETS];
            self.no_control_count = 0;
            self.no_control_sum_us = 0;
            self.no_control_max_us = 0;
            self.no_control_missed_budget = 0;
            self.no_control_buckets = [0; LOOP_BENCH_BUCKETS];
        }
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

    #[cfg(feature = "release-loop-classifier")]
    fn percentile_from(
        count: u32,
        buckets: &[u32; LOOP_BENCH_BUCKETS],
        percentile: u32,
        max_us: u32,
    ) -> u32 {
        let target = count.saturating_mul(percentile).saturating_add(99) / 100;
        let mut seen = 0_u32;
        for (index, count) in buckets.iter().enumerate() {
            seen = seen.saturating_add(*count);
            if seen >= target {
                return (index as u32).saturating_mul(LOOP_BENCH_BUCKET_US);
            }
        }
        max_us
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

#[cfg(feature = "release-loop-classifier")]
fn bench_write_count_max(out: &mut [u8; 50], pos: &mut usize, count: u32, max_us: u32) {
    bench_write_num(out, pos, count);
    bench_write_bytes(out, pos, b"/");
    bench_write_num(out, pos, max_us);
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

    #[cfg(feature = "scope-timing-pins")]
    let mut scope_timing_pins = ScopeTimingPins::new(Output::new(peripherals.PIN_18, Level::Low));
    #[cfg(feature = "scope-timing-pins")]
    let control_scope_pin = Output::new(peripherals.PIN_19, Level::Low);
    #[cfg(all(feature = "scope-timing-pins", not(feature = "imu-producer-scope")))]
    let non_control_scope_pin = Output::new(peripherals.PIN_22, Level::Low);

    spawn_core1_services(
        Core1Resources {
            core1: peripherals.CORE1,
            uart0: peripherals.UART0,
            uart1: peripherals.UART1,
            pio0: peripherals.PIO0,
            #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
            spi1: peripherals.SPI1,
            dma_ch0: peripherals.DMA_CH0,
            dma_ch1: peripherals.DMA_CH1,
            dma_ch2: peripherals.DMA_CH2,
            dma_ch3: peripherals.DMA_CH3,
            dma_ch4: peripherals.DMA_CH4,
            pin0: peripherals.PIN_0,
            pin1: peripherals.PIN_1,
            pin6: peripherals.PIN_6,
            pin7: peripherals.PIN_7,
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
            #[cfg(all(feature = "ism330dhcx-driver", not(feature = "synthetic-imu")))]
            pin14: peripherals.PIN_14,
            #[cfg(all(
                feature = "imu-producer-scope",
                feature = "ism330dhcx-driver",
                not(feature = "synthetic-imu")
            ))]
            pin22: peripherals.PIN_22,
        },
        mailbox,
    );

    let (mut board, pwm_driver) = board::Board::new_uart(
        config,
        None,
        #[cfg(feature = "scope-timing-pins")]
        control_scope_pin,
        #[cfg(all(feature = "scope-timing-pins", not(feature = "imu-producer-scope")))]
        non_control_scope_pin,
    );

    let mut params = Params::default();
    if !board.read_params(&mut params) {
        params.set_defaults();
        let _ = board.write_params(&params);
    }

    let mut world = init_world(board, params, pwm_driver);
    #[cfg(feature = "release-loop-bench")]
    let mut loop_bench = LoopBench::new(mailbox);
    loop {
        match world.realtime_scheduler_step() {
            RealtimeSchedulerStep::ImuControl => {
                #[cfg(feature = "scope-timing-pins")]
                scope_timing_pins.mark_loop_boundary();
                #[cfg(feature = "release-loop-bench")]
                {
                    let start_us = Instant::now().as_micros();
                    #[cfg(any(
                        feature = "release-loop-classifier",
                        feature = "release-loop-spike-counter"
                    ))]
                    let class = world.run_imu_control_tick_classified();
                    #[cfg(not(any(
                        feature = "release-loop-classifier",
                        feature = "release-loop-spike-counter"
                    )))]
                    let _ = world.run_imu_control_tick();
                    let end_us = Instant::now().as_micros();
                    #[cfg(any(
                        feature = "release-loop-classifier",
                        feature = "release-loop-spike-counter"
                    ))]
                    loop_bench.record(
                        end_us.saturating_sub(start_us).min(u32::MAX as u64) as u32,
                        end_us,
                        class,
                    );
                    #[cfg(not(any(
                        feature = "release-loop-classifier",
                        feature = "release-loop-spike-counter"
                    )))]
                    loop_bench.record(
                        end_us.saturating_sub(start_us).min(u32::MAX as u64) as u32,
                        end_us,
                    );
                }
                #[cfg(not(feature = "release-loop-bench"))]
                let _ = world.run_imu_control_tick();
            }
            RealtimeSchedulerStep::Service => {
                #[cfg(feature = "scope-timing-pins")]
                if SCOPE_GP22_MARKS_SERVICE {
                    world.set_test_pin_3(true);
                }
                #[cfg(feature = "release-loop-bench")]
                {
                    let start_us = Instant::now().as_micros();
                    #[cfg(any(
                        feature = "release-loop-classifier",
                        feature = "release-loop-spike-counter"
                    ))]
                    let class =
                        world.run_service_step_with_deferral(MAIN_LOOP_MAX_SERVICE_DEFERRAL_US);
                    #[cfg(not(any(
                        feature = "release-loop-classifier",
                        feature = "release-loop-spike-counter"
                    )))]
                    let _ = world.run_service_step_with_deferral(MAIN_LOOP_MAX_SERVICE_DEFERRAL_US);
                    let end_us = Instant::now().as_micros();
                    #[cfg(any(
                        feature = "release-loop-classifier",
                        feature = "release-loop-spike-counter"
                    ))]
                    loop_bench.record(
                        end_us.saturating_sub(start_us).min(u32::MAX as u64) as u32,
                        end_us,
                        class,
                    );
                    #[cfg(not(any(
                        feature = "release-loop-classifier",
                        feature = "release-loop-spike-counter"
                    )))]
                    loop_bench.record(
                        end_us.saturating_sub(start_us).min(u32::MAX as u64) as u32,
                        end_us,
                    );
                }
                #[cfg(not(feature = "release-loop-bench"))]
                let _ = world.run_service_step_with_deferral(MAIN_LOOP_MAX_SERVICE_DEFERRAL_US);
                #[cfg(feature = "scope-timing-pins")]
                if SCOPE_GP22_MARKS_SERVICE {
                    world.set_test_pin_3(false);
                }
            }
            RealtimeSchedulerStep::Idle => {
                core::hint::spin_loop();
            }
        }
    }
}
