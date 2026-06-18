#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use cortex_m_rt::entry;
use embassy_time::{Duration, Instant, Timer};
use panic_halt as _;
use pico2w::comms_core::{SHARED_MAVLINK_MAILBOX, SharedMavlinkMailbox};
use pico2w::gps::{
    SHARED_GNSS_QUEUE, UbxNavPvtParser, make_ubx_packet, record_gps_byte, record_nav_pvt,
};
#[cfg(any(all(feature = "ism330dhcx-driver")))]
use pico2w::ism330dhcx::SHARED_ISM330DHCX_IMU_QUEUE;
#[cfg(all(feature = "ism330dhcx-driver"))]
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
    any(feature = "imu-producer-interrupt-executor"),
    feature = "ism330dhcx-driver"
))]
use rp2350_platform::hal::executor::InterruptExecutor;
#[cfg(feature = "ism330dhcx-driver")]
use rp2350_platform::hal::gpio::{Input, Pull};
#[cfg(any(feature = "scope-timing-pins", feature = "ism330dhcx-driver"))]
use rp2350_platform::hal::gpio::{Level, Output};
#[cfg(all(feature = "imu-producer-scope", feature = "ism330dhcx-driver"))]
use rp2350_platform::hal::peripherals::PIN_22;
#[cfg(feature = "ism330dhcx-driver")]
use rp2350_platform::hal::peripherals::{PIN_10, PIN_11, PIN_12, PIN_13, PIN_14, SPI1};
#[cfg(feature = "ism330dhcx-driver")]
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
    any(feature = "imu-producer-interrupt-executor"),
    feature = "ism330dhcx-driver"
))]
use rp2350_platform::hal::{
    interrupt,
    interrupt::{InterruptExt, Priority},
};
use static_cell::StaticCell;
#[cfg(any(all(feature = "ism330dhcx-driver")))]
use veloxity_core::packets::{ImuPacket, RosflightPacketHeader};
use veloxity_core::world::RealtimeSchedulerStep;
use veloxity_core::{
    board::{BoardIo, SerialRxPriority},
    comm::TelemetryRates,
    params::Params,
    state_machine::StateManager,
    vehicle::quadrotor,
    world::{ControlLoopRates, RealtimeServicePolicy, World},
};
use veloxity_mavlink::{MavlinkFrameEncoder, MavlinkInterface, parser::MavlinkParser};

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
    any(feature = "imu-producer-interrupt-executor"),
    feature = "ism330dhcx-driver"
))]
static CORE1_IMU_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

const UART_TX_BATCH_BYTES: usize = 512;
const UART_RX_CHUNK_BYTES: usize = 16;
const UART_IDLE_DELAY_US: u64 = 50;
const MAVLINK_UART_BAUDRATE: u32 = 2_000_000;
const CRSF_RX_CHUNK_BYTES: usize = 32;
const GPS_UART_BAUDRATE: u32 = 115_200;
const PICO2W_TELEMETRY_STREAMS_PER_SERVICE_PHASE: usize = 2;
const PICO2W_CONTROL_LOOP_HZ: u16 = 1_500;
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
#[cfg(all(feature = "ism330dhcx-driver"))]
const ISM330DHCX_IMU_PERIOD_US: u64 = 0;
#[cfg(all(feature = "ism330dhcx-driver", feature = "imu-odr-1666hz"))]
const ISM330DHCX_ODR_CONFIG: Ism330dhcxOdrConfig = Ism330dhcxOdrConfig::ODR_1666HZ;
#[cfg(all(feature = "ism330dhcx-driver", not(feature = "imu-odr-1666hz")))]
const ISM330DHCX_ODR_CONFIG: Ism330dhcxOdrConfig = Ism330dhcxOdrConfig::ODR_3333HZ;
#[cfg(all(feature = "ism330dhcx-driver"))]
const ISM330DHCX_SPI_HZ: u32 = 10_000_000;
#[cfg(all(feature = "ism330dhcx-driver"))]
const ISM330DHCX_WHO_AM_I: u8 = 0x6b;
#[cfg(all(feature = "ism330dhcx-driver"))]
#[derive(Clone, Copy)]
struct Ism330dhcxOdrConfig {
    accel_ctrl1_xl: u8,
    gyro_ctrl2_g: u8,
}

#[cfg(all(feature = "ism330dhcx-driver"))]
impl Ism330dhcxOdrConfig {
    #[allow(dead_code)]
    const ODR_1666HZ: Self = Self {
        accel_ctrl1_xl: 0x84,
        gyro_ctrl2_g: 0x8c,
    };
    #[allow(dead_code)]
    const ODR_3333HZ: Self = Self {
        accel_ctrl1_xl: 0x94,
        gyro_ctrl2_g: 0x9c,
    };
}
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
    any(feature = "imu-producer-interrupt-executor"),
    feature = "ism330dhcx-driver"
))]
#[interrupt]
unsafe fn SIO_IRQ_BELL() {
    unsafe { CORE1_IMU_EXECUTOR.on_interrupt() };
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

#[cfg(all(feature = "ism330dhcx-driver"))]
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

#[cfg(all(feature = "ism330dhcx-driver"))]
fn ism330dhcx_init(
    spi: &mut Spi<'static, SPI1, Blocking>,
    cs: &mut Output<'static>,
) -> Result<u8, Option<u8>> {
    let who_am_i = ism330dhcx_read_reg(spi, cs, 0x0f).map_err(|_| None)?;
    if who_am_i != ISM330DHCX_WHO_AM_I {
        return Err(Some(who_am_i));
    }
    ism330dhcx_write_reg(spi, cs, 0x12, 0x44).map_err(|_| Some(who_am_i))?;
    ism330dhcx_write_reg(spi, cs, 0x10, ISM330DHCX_ODR_CONFIG.accel_ctrl1_xl)
        .map_err(|_| Some(who_am_i))?;
    ism330dhcx_write_reg(spi, cs, 0x11, ISM330DHCX_ODR_CONFIG.gyro_ctrl2_g)
        .map_err(|_| Some(who_am_i))?;
    ism330dhcx_write_reg(spi, cs, 0x0b, 0x80).map_err(|_| Some(who_am_i))?;
    ism330dhcx_write_reg(spi, cs, 0x0d, 0x03).map_err(|_| Some(who_am_i))?;
    Ok(who_am_i)
}

#[cfg(all(feature = "ism330dhcx-driver"))]
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

#[cfg(all(feature = "ism330dhcx-driver"))]
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

#[cfg(all(feature = "ism330dhcx-driver"))]
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
    let mut encoder = MavlinkFrameEncoder::new();
    loop {
        let mut n = mailbox.drain_tx_batch_into(&mut tx);
        if n == 0 {
            if let Some(queued) = mailbox.pop_downlink_message() {
                n = encoder
                    .encode_downlink(queued.system_id, queued.msg, &mut tx)
                    .unwrap_or(0);
            }
        }
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
    #[cfg(feature = "ism330dhcx-driver")]
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
    #[cfg(feature = "ism330dhcx-driver")]
    pin10: Peri<'static, PIN_10>,
    #[cfg(feature = "ism330dhcx-driver")]
    pin11: Peri<'static, PIN_11>,
    #[cfg(feature = "ism330dhcx-driver")]
    pin12: Peri<'static, PIN_12>,
    #[cfg(feature = "ism330dhcx-driver")]
    pin13: Peri<'static, PIN_13>,
    #[cfg(feature = "ism330dhcx-driver")]
    pin14: Peri<'static, PIN_14>,
    #[cfg(all(feature = "imu-producer-scope", feature = "ism330dhcx-driver"))]
    pin22: Peri<'static, PIN_22>,
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

fn spawn_core1_services(resources: Core1Resources, mailbox: SharedMavlinkMailbox) {
    spawn_core1(
        resources.core1,
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        move || {
            mailbox.set_comms_state(20);
            configure_core1_transport_interrupt_priorities();

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

            let mut pio = Pio::new(resources.pio0, Irqs);
            let gps_rx_program = PioUartDmaRxProgram::new(&mut pio.common);
            let gps_tx_program = PioUartTxProgram::new(&mut pio.common);
            let gps_rx = PioUartDmaRx::new(
                GPS_UART_BAUDRATE,
                &mut pio.common,
                pio.sm0,
                resources.pin7,
                &gps_rx_program,
            );
            let gps_tx = PioUartTx::new(
                GPS_UART_BAUDRATE,
                &mut pio.common,
                pio.sm1,
                resources.pin6,
                &gps_tx_program,
            );
            let gps_dma = dma::Channel::new(resources.dma_ch4, Irqs);

            #[cfg(feature = "ism330dhcx-driver")]
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
            #[cfg(feature = "ism330dhcx-driver")]
            let imu_cs = Output::new(resources.pin13, Level::High);
            #[cfg(feature = "ism330dhcx-driver")]
            let imu_drdy = Input::new(resources.pin14, Pull::Down);
            #[cfg(all(feature = "imu-producer-scope", feature = "ism330dhcx-driver"))]
            let imu_scope = Output::new(resources.pin22, Level::Low);

            let executor = CORE1_EXECUTOR.init(Executor::new());
            #[cfg(all(
                feature = "imu-producer-interrupt-executor",
                feature = "ism330dhcx-driver"
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
                #[cfg(not(feature = "core1-disable-heartbeat"))]
                if let Ok(token) = core1_heartbeat_task(mailbox) {
                    spawner.spawn(token);
                }
                #[cfg(not(feature = "core1-disable-mavlink-tx"))]
                if let Ok(token) = uart_tx_task(uart_tx, mailbox) {
                    spawner.spawn(token);
                }
                #[cfg(not(feature = "core1-disable-mavlink-rx"))]
                if let Ok(token) = uart_rx_task(uart_rx, mailbox) {
                    spawner.spawn(token);
                }
                #[cfg(not(feature = "core1-disable-crsf"))]
                if let Ok(token) = crsf_rx_task(crsf_rx) {
                    spawner.spawn(token);
                }
                #[cfg(not(feature = "core1-disable-gps"))]
                if let Ok(token) = gps_pio_task(gps_rx, gps_tx, gps_dma) {
                    spawner.spawn(token);
                }
                #[cfg(all(
                    feature = "ism330dhcx-driver",
                    not(feature = "imu-producer-interrupt-executor")
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
        },
    );
}

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
    world.set_telemetry_rates(TelemetryRates::bounded_high_rate_transport());
    world.set_control_loop_rates(ControlLoopRates::fixed_rate_hz(PICO2W_CONTROL_LOOP_HZ));
    world
}

fn hal_config() -> HalConfig {
    HalConfig::new(ClockConfig::system_freq(300_000_000).unwrap())
}

#[entry]
fn main() -> ! {
    let peripherals = rp::init(hal_config());
    let config = Pico2WConfig::default();
    let mailbox = SHARED_MAVLINK_MAILBOX;

    #[cfg(feature = "scope-timing-pins")]
    let deadline_scope_pin = Output::new(peripherals.PIN_18, Level::Low);
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
            #[cfg(feature = "ism330dhcx-driver")]
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
            #[cfg(feature = "ism330dhcx-driver")]
            pin10: peripherals.PIN_10,
            #[cfg(feature = "ism330dhcx-driver")]
            pin11: peripherals.PIN_11,
            #[cfg(feature = "ism330dhcx-driver")]
            pin12: peripherals.PIN_12,
            #[cfg(feature = "ism330dhcx-driver")]
            pin13: peripherals.PIN_13,
            #[cfg(feature = "ism330dhcx-driver")]
            pin14: peripherals.PIN_14,
            #[cfg(all(feature = "imu-producer-scope", feature = "ism330dhcx-driver"))]
            pin22: peripherals.PIN_22,
        },
        mailbox,
    );

    let (mut board, pwm_driver) = board::Board::new_uart(
        config,
        None,
        #[cfg(feature = "scope-timing-pins")]
        deadline_scope_pin,
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
    loop {
        match world.realtime_scheduler_step() {
            RealtimeSchedulerStep::ImuControl => {
                let _ = world.run_imu_control_tick();
            }
            RealtimeSchedulerStep::ControlUpdate => {
                let _ = world.run_control_update_tick();
            }
            RealtimeSchedulerStep::Service => {
                #[cfg(feature = "scope-timing-pins")]
                if SCOPE_GP22_MARKS_SERVICE {
                    world.set_test_pin_3(true);
                }
                let _ = world.run_prioritized_service_steps_with_policy(
                    RealtimeServicePolicy::continuous(PICO2W_TELEMETRY_STREAMS_PER_SERVICE_PHASE),
                );
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
