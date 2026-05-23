use core::{
    cell::RefCell,
    sync::atomic::{AtomicU32, Ordering},
};

use critical_section::Mutex;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe::Pipe};
use rp2350_platform::wifi::WifiCommsConfig;
use voloxide_core::board::{SerialRxPriority, SerialTxPriority};

pub static MAVLINK_MAILBOX: Mutex<RefCell<MavlinkMailbox>> =
    Mutex::new(RefCell::new(MavlinkMailbox::new()));
static WIFI_STATE: AtomicU32 = AtomicU32::new(0);

static RX_HIGH: Pipe<CriticalSectionRawMutex, 1024> = Pipe::new();
static RX_NORMAL: Pipe<CriticalSectionRawMutex, 4096> = Pipe::new();
static RX_LOW: Pipe<CriticalSectionRawMutex, 1024> = Pipe::new();
static TX_HIGH: Pipe<CriticalSectionRawMutex, 2048> = Pipe::new();
static TX_NORMAL: Pipe<CriticalSectionRawMutex, 1024> = Pipe::new();
static TX_LOW: Pipe<CriticalSectionRawMutex, 2048> = Pipe::new();

#[derive(Clone, Copy)]
pub struct SharedMavlinkMailbox {
    inner: &'static Mutex<RefCell<MavlinkMailbox>>,
}

unsafe impl Send for SharedMavlinkMailbox {}
unsafe impl Sync for SharedMavlinkMailbox {}

impl SharedMavlinkMailbox {
    pub const fn new(inner: &'static Mutex<RefCell<MavlinkMailbox>>) -> Self {
        Self { inner }
    }

    pub fn read_into(&self, out: &mut [u8]) -> usize {
        let n = read_priority(&RX_HIGH, &RX_NORMAL, &RX_LOW, out);
        self.update_stats(|stats| stats.rx_read = stats.rx_read.wrapping_add(n as u32));
        n
    }

    pub fn write_from(&self, bytes: &[u8]) -> usize {
        self.write_from_priority(bytes, SerialTxPriority::NORMAL)
    }

    pub fn write_from_priority(&self, bytes: &[u8], priority: SerialTxPriority) -> usize {
        let sent = if priority >= SerialTxPriority::HIGH {
            write_all_if_fits(&TX_HIGH, bytes)
        } else if priority >= SerialTxPriority::NORMAL {
            write_all_if_fits(&TX_NORMAL, bytes)
        } else {
            write_all_if_fits(&TX_LOW, bytes)
        };

        if sent {
            self.update_stats(|stats| {
                stats.tx_written = stats.tx_written.wrapping_add(bytes.len() as u32)
            });
            bytes.len()
        } else {
            self.update_stats(|stats| {
                stats.tx_dropped = stats.tx_dropped.wrapping_add(bytes.len() as u32)
            });
            0
        }
    }

    pub fn push_rx(&self, bytes: &[u8]) -> usize {
        self.push_rx_priority(bytes, SerialRxPriority::NORMAL)
    }

    pub fn push_rx_priority(&self, bytes: &[u8], priority: SerialRxPriority) -> usize {
        let sent = if priority >= SerialRxPriority::HIGH {
            write_all_if_fits(&RX_HIGH, bytes)
        } else if priority >= SerialRxPriority::NORMAL {
            write_all_if_fits(&RX_NORMAL, bytes)
        } else {
            write_all_if_fits(&RX_LOW, bytes)
        };

        if sent {
            self.update_stats(|stats| {
                stats.rx_pushed = stats.rx_pushed.wrapping_add(bytes.len() as u32)
            });
            bytes.len()
        } else {
            self.update_stats(|stats| {
                stats.rx_dropped = stats.rx_dropped.wrapping_add(bytes.len() as u32)
            });
            0
        }
    }

    pub fn drain_tx_into(&self, out: &mut [u8]) -> usize {
        let n = read_priority(&TX_HIGH, &TX_NORMAL, &TX_LOW, out);
        self.update_stats(|stats| stats.tx_drained = stats.tx_drained.wrapping_add(n as u32));
        n
    }

    pub fn drain_tx_batch_into(&self, out: &mut [u8]) -> usize {
        let mut total = 0;
        while total < out.len() {
            let n = read_priority(&TX_HIGH, &TX_NORMAL, &TX_LOW, &mut out[total..]);
            if n == 0 {
                break;
            }
            total += n;
        }

        self.update_stats(|stats| stats.tx_drained = stats.tx_drained.wrapping_add(total as u32));
        total
    }

    pub fn record_core1_heartbeat(&self) {
        self.update_stats(|stats| stats.core1_heartbeats = stats.core1_heartbeats.wrapping_add(1));
    }

    pub fn record_wifi_rx_datagram(&self) {
        self.update_stats(|stats| {
            stats.wifi_rx_datagrams = stats.wifi_rx_datagrams.wrapping_add(1)
        });
    }

    pub fn record_wifi_tx_datagram(&self) {
        self.update_stats(|stats| {
            stats.wifi_tx_datagrams = stats.wifi_tx_datagrams.wrapping_add(1)
        });
    }

    pub fn set_wifi_state(&self, state: u32) {
        WIFI_STATE.store(state, Ordering::Release);
    }

    pub fn stats(&self) -> MavlinkMailboxStats {
        let mut stats = critical_section::with(|cs| self.inner.borrow_ref(cs).stats);
        stats.wifi_state = WIFI_STATE.load(Ordering::Acquire);
        stats
    }

    fn update_stats(&self, update: impl FnOnce(&mut MavlinkMailboxStats)) {
        critical_section::with(|cs| update(&mut self.inner.borrow_ref_mut(cs).stats));
    }
}

pub const SHARED_MAVLINK_MAILBOX: SharedMavlinkMailbox =
    SharedMavlinkMailbox::new(&MAVLINK_MAILBOX);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MavlinkMailboxStats {
    pub rx_pushed: u32,
    pub rx_read: u32,
    pub rx_dropped: u32,
    pub tx_written: u32,
    pub tx_drained: u32,
    pub tx_dropped: u32,
    pub wifi_rx_datagrams: u32,
    pub wifi_tx_datagrams: u32,
    pub wifi_state: u32,
    pub core1_heartbeats: u32,
}

pub struct MavlinkMailbox {
    stats: MavlinkMailboxStats,
}

impl MavlinkMailbox {
    pub const fn new() -> Self {
        Self {
            stats: MavlinkMailboxStats {
                rx_pushed: 0,
                rx_read: 0,
                rx_dropped: 0,
                tx_written: 0,
                tx_drained: 0,
                tx_dropped: 0,
                wifi_rx_datagrams: 0,
                wifi_tx_datagrams: 0,
                wifi_state: 0,
                core1_heartbeats: 0,
            },
        }
    }
}

impl Default for MavlinkMailbox {
    fn default() -> Self {
        Self::new()
    }
}

fn read_priority<const H: usize, const N: usize, const L: usize>(
    high: &Pipe<CriticalSectionRawMutex, H>,
    normal: &Pipe<CriticalSectionRawMutex, N>,
    low: &Pipe<CriticalSectionRawMutex, L>,
    out: &mut [u8],
) -> usize {
    high.try_read(out)
        .or_else(|_| normal.try_read(out))
        .or_else(|_| low.try_read(out))
        .unwrap_or(0)
}

fn write_all_if_fits<const N: usize>(
    pipe: &Pipe<CriticalSectionRawMutex, N>,
    bytes: &[u8],
) -> bool {
    if bytes.len() > pipe.free_capacity() {
        return false;
    }

    let mut written = 0;
    while written < bytes.len() {
        match pipe.try_write(&bytes[written..]) {
            Ok(0) | Err(_) => return false,
            Ok(n) => written += n,
        }
    }
    true
}

pub struct WifiMavlinkCore {
    pub config: WifiCommsConfig,
}

impl WifiMavlinkCore {
    pub fn new(config: WifiCommsConfig) -> Self {
        Self { config }
    }

    pub fn run_forever(&mut self, mailbox: SharedMavlinkMailbox) -> ! {
        let mut tx_bytes = [0_u8; 256];
        let mut heartbeat_divider = 0_u32;
        loop {
            let _ = mailbox.drain_tx_into(&mut tx_bytes);
            heartbeat_divider = heartbeat_divider.wrapping_add(1);
            if heartbeat_divider == 50_000 {
                mailbox.record_core1_heartbeat();
                heartbeat_divider = 0;
            }
            core::hint::spin_loop();
        }
    }
}
