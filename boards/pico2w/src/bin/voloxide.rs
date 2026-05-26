#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use cortex_m_rt::entry;
use embassy_executor::Executor;
use embassy_time::{Duration, Instant, Timer};
use panic_halt as _;
use pico2w::comms_core::{SHARED_MAVLINK_MAILBOX, SharedMavlinkMailbox};
#[cfg(feature = "synthetic-imu")]
use pico2w::ism330dhcx::SHARED_ISM330DHCX_IMU_QUEUE;
use pico2w::rc_receiver::{CRSF_BAUDRATE, CrsfRcParser, SHARED_CRSF_RC_QUEUE};
use pico2w::{board, config::Pico2WConfig, pwm::PioPwmDriver};
use rp2350_platform::hal::clocks::ClockConfig;
use rp2350_platform::hal::dma;
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
use static_cell::StaticCell;
#[cfg(feature = "synthetic-imu")]
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
const CRSF_RX_CHUNK_BYTES: usize = 8;
#[cfg(feature = "synthetic-imu")]
const SYNTHETIC_IMU_PERIOD_US: u64 = 125;

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
    config.baudrate = 921_600;
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
    dma_ch0: Peri<'static, DMA_CH0>,
    dma_ch1: Peri<'static, DMA_CH1>,
    dma_ch2: Peri<'static, DMA_CH2>,
    dma_ch3: Peri<'static, DMA_CH3>,
    pin0: Peri<'static, PIN_0>,
    pin1: Peri<'static, PIN_1>,
    pin8: Peri<'static, PIN_8>,
    pin9: Peri<'static, PIN_9>,
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
    #[cfg(feature = "synthetic-imu")]
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
            dma_ch0: peripherals.DMA_CH0,
            dma_ch1: peripherals.DMA_CH1,
            dma_ch2: peripherals.DMA_CH2,
            dma_ch3: peripherals.DMA_CH3,
            pin0: peripherals.PIN_0,
            pin1: peripherals.PIN_1,
            pin8: peripherals.PIN_8,
            pin9: peripherals.PIN_9,
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
    loop {
        let _ = world.run_once();
    }
}
