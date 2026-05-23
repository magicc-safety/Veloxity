#![no_std]
#![no_main]

#[cfg(feature = "wifi-mavlink")]
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::{
    asm,
    peripheral::{SYST, syst::SystClkSource},
};
use cortex_m_rt::{entry, exception};
#[cfg(feature = "wifi-mavlink")]
use cyw43::{JoinOptions, aligned_bytes};
#[cfg(feature = "wifi-mavlink")]
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
#[cfg(feature = "wifi-mavlink")]
use embassy_executor::{Executor, Spawner};
#[cfg(feature = "wifi-mavlink")]
use embassy_futures::select::{Either, select};
#[cfg(feature = "wifi-mavlink")]
use embassy_net::{
    Config as NetConfig, IpAddress, IpEndpoint, StackResources,
    udp::{PacketMetadata, UdpMetadata, UdpSocket},
};
#[cfg(feature = "wifi-mavlink")]
use embassy_time::{Duration, Instant, Timer};
use panic_halt as _;
#[cfg(feature = "wifi-mavlink")]
use pico2w::comms_core::{SHARED_MAVLINK_MAILBOX, SharedMavlinkMailbox};
use pico2w::{board, config::Pico2WConfig, gy91::Gy91, pwm::PioPwmDriver};
use rp2350_platform::hal::{
    self as rp,
    gpio::{Level, Output},
    spi::{Config as SpiConfig, Phase, Polarity, Spi},
    uart::{Config as UartConfig, Uart},
};
#[cfg(feature = "wifi-mavlink")]
use rp2350_platform::hal::{
    Peri, bind_interrupts, dma,
    multicore::{Stack, spawn_core1},
    peripherals::{CORE1, DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0},
    pio::{InterruptHandler as PioInterruptHandler, Pio},
};
#[cfg(feature = "wifi-mavlink")]
use static_cell::StaticCell;
use voloxide_core::{
    board::BoardIo, comm::TelemetryRates, params::Params, state_machine::StateManager,
    vehicle::quadrotor, world::World,
};
use voloxide_mavlink::MavlinkInterface;

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

#[cfg(feature = "wifi-mavlink")]
const WIFI_MAVLINK_TELEMETRY_RATES: TelemetryRates = TelemetryRates {
    imu_hz: 200,
    attitude_hz: 50,
    output_raw_hz: 0,
    diff_pressure_hz: 25,
    baro_hz: 25,
    mag_hz: 25,
    range_hz: 25,
    battery_hz: 10,
    gnss_hz: 10,
    rc_hz: 50,
    output_raw_imu_divisor: 0,
};

#[cfg(feature = "wifi-mavlink")]
static mut CORE1_STACK: Stack<65536> = Stack::new();
#[cfg(feature = "wifi-mavlink")]
static CORE1_EXECUTOR: StaticCell<Executor> = StaticCell::new();
#[cfg(feature = "wifi-mavlink")]
static mut CYW43_STATE: cyw43::State = cyw43::State::new();

#[cfg(feature = "wifi-mavlink")]
const UDP_LATENCY_MAGIC: &[u8; 4] = b"VXL1";

const CORE0_SCHEDULER_HZ: u32 = 4_000;
const MAX_PENDING_CORE0_TICKS: u32 = 4;

static CORE0_PENDING_TICKS: AtomicU32 = AtomicU32::new(0);

#[cfg(feature = "wifi-mavlink")]
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

#[cfg(feature = "wifi-mavlink")]
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

#[cfg(feature = "wifi-mavlink")]
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[cfg(feature = "wifi-mavlink")]
#[embassy_executor::task]
async fn wifi_mavlink_task(
    mailbox: SharedMavlinkMailbox,
    config: Pico2WConfig,
    pio0: Peri<'static, PIO0>,
    dma_ch0: Peri<'static, DMA_CH0>,
    pwr_pin: Peri<'static, PIN_23>,
    dio_pin: Peri<'static, PIN_24>,
    cs_pin: Peri<'static, PIN_25>,
    clk_pin: Peri<'static, PIN_29>,
    spawner: Spawner,
) {
    mailbox.set_wifi_state(1);
    let ssid = option_env!("VOLOXIDE_WIFI_SSID").unwrap_or("");
    let passphrase = option_env!("VOLOXIDE_WIFI_PASSWORD").unwrap_or("");
    mailbox.set_wifi_state(31);

    if ssid.is_empty() {
        mailbox.set_wifi_state(10);
        loop {
            mailbox.record_core1_heartbeat();
            Timer::after_secs(1).await;
        }
    }

    let fw = aligned_bytes!("../../firmware/43439A0.bin");
    let clm = aligned_bytes!("../../firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../../firmware/nvram_rp2040.bin");
    mailbox.set_wifi_state(32);

    let pwr = Output::new(pwr_pin, Level::Low);
    let cs = Output::new(cs_pin, Level::High);
    mailbox.set_wifi_state(33);
    let mut pio = Pio::new(pio0, Irqs);
    mailbox.set_wifi_state(34);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        dio_pin,
        clk_pin,
        dma::Channel::new(dma_ch0, Irqs),
    );
    mailbox.set_wifi_state(35);

    let state = unsafe { &mut *addr_of_mut!(CYW43_STATE) };
    mailbox.set_wifi_state(2);
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;
    mailbox.set_wifi_state(36);
    if let Ok(token) = cyw43_task(runner) {
        spawner.spawn(token);
    }

    control.init(clm).await;
    mailbox.set_wifi_state(37);
    control
        .set_power_management(cyw43::PowerManagementMode::None)
        .await;

    loop {
        mailbox.set_wifi_state(3);
        if control
            .join(ssid, JoinOptions::new(passphrase.as_bytes()))
            .await
            .is_ok()
        {
            break;
        }
        mailbox.set_wifi_state(4);
        mailbox.record_core1_heartbeat();
        Timer::after_secs(1).await;
    }
    mailbox.set_wifi_state(5);

    static NET_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let seed = 0x5eed_2350_u64;
    let (stack, runner) = embassy_net::new(
        net_device,
        NetConfig::dhcpv4(Default::default()),
        NET_RESOURCES.init(StackResources::new()),
        seed,
    );
    if let Ok(token) = net_task(runner) {
        spawner.spawn(token);
    }

    mailbox.set_wifi_state(6);
    stack.wait_link_up().await;
    mailbox.set_wifi_state(7);
    stack.wait_config_up().await;
    mailbox.set_wifi_state(8);

    let mut rx_meta = [PacketMetadata::EMPTY; 8];
    let mut tx_meta = [PacketMetadata::EMPTY; 8];
    let mut rx_buffer = [0_u8; 4096];
    let mut tx_buffer = [0_u8; 4096];
    let mut udp_rx = [0_u8; 512];
    let mut udp_tx = [0_u8; 512];
    let mut peer: Option<UdpMetadata> = None;
    let discovery = IpEndpoint::new(IpAddress::v4(255, 255, 255, 255), config.wifi.udp.peer_port);

    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );
    let _ = socket.bind(config.wifi.udp.bind_port);
    mailbox.set_wifi_state(9);
    let mut next_discovery = Instant::now();
    let mut next_heartbeat = Instant::now();

    loop {
        match select(
            socket.recv_from(&mut udp_rx),
            Timer::after(Duration::from_millis(1)),
        )
        .await
        {
            Either::First(Ok((n, metadata))) => {
                mailbox.record_wifi_rx_datagram();
                if udp_rx[..n].starts_with(UDP_LATENCY_MAGIC) {
                    if socket.send_to(&udp_rx[..n], metadata).await.is_ok() {
                        mailbox.record_wifi_tx_datagram();
                    }
                } else {
                    peer = Some(metadata);
                    let _ = mailbox.push_rx_priority(
                        &udp_rx[..n],
                        voloxide_core::board::SerialRxPriority::NORMAL,
                    );
                }
            }
            Either::First(Err(_)) | Either::Second(()) => {}
        }

        if let Some(remote) = peer {
            let n = mailbox.drain_tx_into(&mut udp_tx);
            if n > 0 && socket.send_to(&udp_tx[..n], remote).await.is_ok() {
                mailbox.record_wifi_tx_datagram();
            }
        } else if Instant::now() >= next_discovery {
            let _ = socket.send_to(b"voloxide-pico2w-mavlink", discovery).await;
            next_discovery += Duration::from_secs(1);
        }

        if Instant::now() >= next_heartbeat {
            mailbox.record_core1_heartbeat();
            next_heartbeat += Duration::from_millis(500);
        }
        Timer::after(Duration::from_millis(1)).await;
    }
}

#[cfg(feature = "wifi-mavlink")]
struct WifiCoreResources {
    core1: Peri<'static, CORE1>,
    pio0: Peri<'static, PIO0>,
    dma_ch0: Peri<'static, DMA_CH0>,
    pwr_pin: Peri<'static, PIN_23>,
    dio_pin: Peri<'static, PIN_24>,
    cs_pin: Peri<'static, PIN_25>,
    clk_pin: Peri<'static, PIN_29>,
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
    #[cfg(feature = "wifi-mavlink")]
    world.set_telemetry_rates(WIFI_MAVLINK_TELEMETRY_RATES);
    #[cfg(not(feature = "wifi-mavlink"))]
    world.set_telemetry_rates(TelemetryRates::bounded_high_rate_transport());
    world
}

#[cfg(feature = "wifi-mavlink")]
fn trace(uart: &mut Uart<'_, rp::uart::Blocking>, message: &[u8]) {
    let _ = uart.blocking_write(message);
    let _ = uart.blocking_flush();
}

#[cfg(not(feature = "wifi-mavlink"))]
fn trace(uart: &mut Uart<'_, rp::uart::Blocking>, message: &[u8]) {
    let _ = uart.blocking_write(message);
    let _ = uart.blocking_flush();
}

#[cfg(any(feature = "wifi-mavlink", not(feature = "wifi-mavlink")))]
fn trace_delay() {
    for _ in 0..100_000 {
        core::hint::spin_loop();
    }
}

#[cfg(feature = "wifi-mavlink")]
fn spawn_wifi_core(
    resources: WifiCoreResources,
    config: Pico2WConfig,
    mailbox: SharedMavlinkMailbox,
) {
    spawn_core1(
        resources.core1,
        unsafe { &mut *addr_of_mut!(CORE1_STACK) },
        move || {
            mailbox.set_wifi_state(20);
            let executor = CORE1_EXECUTOR.init(Executor::new());
            mailbox.set_wifi_state(21);
            executor.run(|spawner| {
                match wifi_mavlink_task(
                    mailbox,
                    config,
                    resources.pio0,
                    resources.dma_ch0,
                    resources.pwr_pin,
                    resources.dio_pin,
                    resources.cs_pin,
                    resources.clk_pin,
                    spawner,
                ) {
                    Ok(token) => {
                        mailbox.set_wifi_state(22);
                        spawner.spawn(token);
                        mailbox.set_wifi_state(23);
                    }
                    Err(_) => mailbox.set_wifi_state(24),
                }
            })
        },
    );
}

#[cfg(not(feature = "wifi-mavlink"))]
fn mavlink_uart_config() -> UartConfig {
    let mut config = UartConfig::default();
    config.baudrate = 921_600;
    config
}

fn gy91_spi_config() -> SpiConfig {
    let mut config = SpiConfig::default();
    config.frequency = 1_000_000;
    config.polarity = Polarity::IdleLow;
    config.phase = Phase::CaptureOnFirstTransition;
    config
}

fn configure_core0_scheduler(mut syst: SYST) {
    let clk_sys_hz = rp::clocks::clk_sys_freq();
    let ticks_per_period = clk_sys_hz / CORE0_SCHEDULER_HZ;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(ticks_per_period.saturating_sub(1));
    syst.clear_current();
    syst.enable_interrupt();
    syst.enable_counter();
}

fn run_world_on_core0_scheduler(mut world: Pico2WWorld) -> ! {
    loop {
        let pending = CORE0_PENDING_TICKS.swap(0, Ordering::AcqRel);
        if pending == 0 {
            asm::wfi();
            continue;
        }

        for _ in 0..pending {
            world.run_once();
        }
    }
}

#[exception]
fn SysTick() {
    let mut current = CORE0_PENDING_TICKS.load(Ordering::Relaxed);
    while current < MAX_PENDING_CORE0_TICKS {
        match CORE0_PENDING_TICKS.compare_exchange_weak(
            current,
            current + 1,
            Ordering::Release,
            Ordering::Relaxed,
        ) {
            Ok(_) => break,
            Err(next) => current = next,
        }
    }
}

#[entry]
fn main() -> ! {
    let peripherals = rp::init(Default::default());
    let core = cortex_m::Peripherals::take().unwrap();
    configure_core0_scheduler(core.SYST);

    #[cfg(feature = "wifi-mavlink")]
    {
        let mut debug_uart = Uart::new_blocking(
            peripherals.UART0,
            peripherals.PIN_0,
            peripherals.PIN_1,
            UartConfig::default(),
        );
        for _ in 0..5 {
            trace(&mut debug_uart, b"voloxide pico2w uart preflight\r\n");
            trace_delay();
        }

        let config = Pico2WConfig::default();
        trace(&mut debug_uart, b"config ok\r\n");
        let gy91 = Gy91::new(
            Spi::new_blocking(
                peripherals.SPI1,
                peripherals.PIN_10,
                peripherals.PIN_11,
                peripherals.PIN_12,
                gy91_spi_config(),
            ),
            Output::new(peripherals.PIN_13, Level::High),
            Output::new(peripherals.PIN_14, Level::High),
        );
        trace(&mut debug_uart, b"gy91 spi ok\r\n");

        let mailbox = SHARED_MAVLINK_MAILBOX;
        spawn_wifi_core(
            WifiCoreResources {
                core1: peripherals.CORE1,
                pio0: peripherals.PIO0,
                dma_ch0: peripherals.DMA_CH0,
                pwr_pin: peripherals.PIN_23,
                dio_pin: peripherals.PIN_24,
                cs_pin: peripherals.PIN_25,
                clk_pin: peripherals.PIN_29,
            },
            config,
            mailbox,
        );
        trace(&mut debug_uart, b"core1 wifi-mavlink ok\r\n");

        let (mut board, pwm_driver) = board::Board::new_wifi(config, Some(gy91));
        trace(&mut debug_uart, b"board ok\r\n");

        let mut params = Params::default();
        if !board.read_params(&mut params) {
            trace(&mut debug_uart, b"params defaulting\r\n");
            params.set_defaults();
            let _ = board.write_params(&params);
        }
        trace(&mut debug_uart, b"params ok\r\n");

        let world = init_world(board, params, pwm_driver);
        trace(&mut debug_uart, b"world ok\r\n");

        run_world_on_core0_scheduler(world);
    }

    #[cfg(not(feature = "wifi-mavlink"))]
    {
        let mut mavlink_uart = Uart::new_blocking(
            peripherals.UART0,
            peripherals.PIN_0,
            peripherals.PIN_1,
            mavlink_uart_config(),
        );
        for _ in 0..5 {
            trace(&mut mavlink_uart, b"voloxide pico2w uart preflight\r\n");
            trace_delay();
        }

        let config = Pico2WConfig::default();
        trace(&mut mavlink_uart, b"config ok\r\n");
        trace(&mut mavlink_uart, b"mavlink uart ok\r\n");
        let gy91 = Gy91::new(
            Spi::new_blocking(
                peripherals.SPI1,
                peripherals.PIN_10,
                peripherals.PIN_11,
                peripherals.PIN_12,
                gy91_spi_config(),
            ),
            Output::new(peripherals.PIN_13, Level::High),
            Output::new(peripherals.PIN_14, Level::High),
        );
        let (mut board, pwm_driver) = board::Board::new_uart(config, mavlink_uart, Some(gy91));

        let mut params = Params::default();
        if !board.read_params(&mut params) {
            params.set_defaults();
            let _ = board.write_params(&params);
        }

        let world = init_world(board, params, pwm_driver);
        run_world_on_core0_scheduler(world);
    }
}
