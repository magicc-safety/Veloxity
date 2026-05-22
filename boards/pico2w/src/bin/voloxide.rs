#![no_std]
#![no_main]

#[cfg(feature = "wifi-mavlink")]
use core::ptr::addr_of_mut;

use cortex_m_rt::entry;
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
use pico2w::{board, config::Pico2WConfig, pwm::PioPwmDriver};
use rp2350_platform::hal::{
    self as rp,
    uart::{Config as UartConfig, Uart},
};
#[cfg(feature = "wifi-mavlink")]
use rp2350_platform::hal::{
    Peri, bind_interrupts, dma,
    gpio::{Level, Output},
    multicore::{Stack, spawn_core1},
    peripherals::{CORE1, DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0},
    pio::{InterruptHandler as PioInterruptHandler, Pio},
};
#[cfg(feature = "wifi-mavlink")]
use static_cell::StaticCell;
use voloxide_core::{
    board::BoardIo, params::Params, state_machine::StateManager, vehicle::quadrotor, world::World,
};
use voloxide_mavlink::MavlinkInterface;

type Pico2WWorld = World<
    board::Board,
    quadrotor::Estimator,
    quadrotor::Controller,
    quadrotor::Mixer,
    MavlinkInterface,
    PioPwmDriver,
>;

#[cfg(feature = "wifi-mavlink")]
static mut CORE1_STACK: Stack<65536> = Stack::new();
#[cfg(feature = "wifi-mavlink")]
static CORE1_EXECUTOR: StaticCell<Executor> = StaticCell::new();
#[cfg(feature = "wifi-mavlink")]
static mut CYW43_STATE: cyw43::State = cyw43::State::new();

#[cfg(feature = "wifi-mavlink")]
const UDP_LATENCY_MAGIC: &[u8; 4] = b"VXL1";

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
                    let _ = mailbox.push_rx(&udp_rx[..n]);
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
    Pico2WWorld::init(
        board,
        params,
        MavlinkInterface::new(),
        StateManager::new(),
        quadrotor::Estimator::default(),
        quadrotor::Controller::default(),
        mixer,
        pwm_driver,
    )
}

#[cfg(feature = "wifi-mavlink")]
fn trace(uart: &mut Uart<'_, rp::uart::Blocking>, message: &[u8]) {
    let _ = uart.blocking_write(message);
    let _ = uart.blocking_flush();
}

#[cfg(feature = "wifi-mavlink")]
fn trace_delay() {
    for _ in 0..100_000 {
        core::hint::spin_loop();
    }
}

#[cfg(feature = "wifi-mavlink")]
fn trace_wifi_state(uart: &mut Uart<'_, rp::uart::Blocking>, state: u32) {
    match state {
        1 => trace(uart, b"wifi init\r\n"),
        2 => trace(uart, b"wifi cyw43 new\r\n"),
        3 => trace(uart, b"wifi joining\r\n"),
        4 => trace(uart, b"wifi join retry\r\n"),
        5 => trace(uart, b"wifi joined\r\n"),
        6 => trace(uart, b"wifi wait link\r\n"),
        7 => trace(uart, b"wifi wait dhcp\r\n"),
        8 => trace(uart, b"wifi dhcp ok\r\n"),
        9 => trace(uart, b"wifi udp ok\r\n"),
        10 => trace(uart, b"wifi missing config\r\n"),
        20 => trace(uart, b"core1 entry\r\n"),
        21 => trace(uart, b"core1 executor\r\n"),
        22 => trace(uart, b"wifi task token ok\r\n"),
        23 => trace(uart, b"wifi task spawn ok\r\n"),
        24 => trace(uart, b"wifi task token failed\r\n"),
        25 => trace(uart, b"wifi task spawn failed\r\n"),
        31 => trace(uart, b"wifi config ok\r\n"),
        32 => trace(uart, b"wifi firmware refs ok\r\n"),
        33 => trace(uart, b"wifi pins ok\r\n"),
        34 => trace(uart, b"wifi pio ok\r\n"),
        35 => trace(uart, b"wifi spi ok\r\n"),
        36 => trace(uart, b"wifi cyw43 ready\r\n"),
        37 => trace(uart, b"wifi control init ok\r\n"),
        _ => {}
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

#[entry]
fn main() -> ! {
    let peripherals = rp::init(Default::default());

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

        let (mut board, pwm_driver) = board::Board::new_wifi(config);
        trace(&mut debug_uart, b"board ok\r\n");

        let mut params = Params::default();
        if !board.read_params(&mut params) {
            trace(&mut debug_uart, b"params defaulting\r\n");
            params.set_defaults();
            let _ = board.write_params(&params);
        }
        trace(&mut debug_uart, b"params ok\r\n");

        let mut world = init_world(board, params, pwm_driver);
        trace(&mut debug_uart, b"world ok\r\n");

        let mut loops = 0_u32;
        let mut last_core1_heartbeat = 0_u32;
        let mut last_wifi_state = 0_u32;
        loop {
            world.run_once();
            loops = loops.wrapping_add(1);
            if loops <= 10 || loops % 10_000 == 0 {
                trace(&mut debug_uart, b"world tick\r\n");
            }
            let stats = mailbox.stats();
            if stats.core1_heartbeats != last_core1_heartbeat {
                trace(&mut debug_uart, b"core1 tick\r\n");
                last_core1_heartbeat = stats.core1_heartbeats;
            }
            if stats.wifi_state != last_wifi_state {
                trace_wifi_state(&mut debug_uart, stats.wifi_state);
                last_wifi_state = stats.wifi_state;
            }
        }
    }

    #[cfg(not(feature = "wifi-mavlink"))]
    {
        let config = Pico2WConfig::default();
        let mavlink_uart = Uart::new_blocking(
            peripherals.UART0,
            peripherals.PIN_0,
            peripherals.PIN_1,
            mavlink_uart_config(),
        );
        let (mut board, pwm_driver) = board::Board::new_uart(config, mavlink_uart);

        let mut params = Params::default();
        if !board.read_params(&mut params) {
            params.set_defaults();
            let _ = board.write_params(&params);
        }

        let mut world = init_world(board, params, pwm_driver);
        loop {
            world.run_once();
        }
    }
}
