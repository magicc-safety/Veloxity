use core::{
    cell::RefCell,
    sync::atomic::{AtomicU32, Ordering},
};

use critical_section::Mutex;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe::Pipe};
use rp2350_platform::wifi::WifiCommsConfig;
use voloxide_core::board::{SerialRxPriority, SerialTxPriority};

const MAVLINK_V1_MAX_FRAME_BYTES: usize = 263;
const TX_LOW_FRAME_CAPACITY: usize = 48;

pub static MAVLINK_MAILBOX: Mutex<RefCell<MavlinkMailbox>> =
    Mutex::new(RefCell::new(MavlinkMailbox::new()));
static WIFI_STATE: AtomicU32 = AtomicU32::new(0);

static RX_HIGH: Pipe<CriticalSectionRawMutex, 1024> = Pipe::new();
static RX_NORMAL: Pipe<CriticalSectionRawMutex, 4096> = Pipe::new();
static RX_LOW: Pipe<CriticalSectionRawMutex, 1024> = Pipe::new();
static TX_HIGH: Pipe<CriticalSectionRawMutex, 2048> = Pipe::new();
static TX_NORMAL: Pipe<CriticalSectionRawMutex, 1024> = Pipe::new();

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
            self.push_low_frame(bytes)
        };

        if sent {
            self.update_stats(|stats| {
                stats.tx_written = stats.tx_written.wrapping_add(bytes.len() as u32);
                if priority >= SerialTxPriority::HIGH {
                    stats.tx_high_written = stats.tx_high_written.wrapping_add(bytes.len() as u32);
                } else if priority >= SerialTxPriority::NORMAL {
                    stats.tx_normal_written =
                        stats.tx_normal_written.wrapping_add(bytes.len() as u32);
                } else {
                    stats.tx_low_written = stats.tx_low_written.wrapping_add(bytes.len() as u32);
                }
            });
            bytes.len()
        } else {
            self.update_stats(|stats| {
                stats.tx_dropped = stats.tx_dropped.wrapping_add(bytes.len() as u32);
                if priority >= SerialTxPriority::HIGH {
                    stats.tx_high_dropped = stats.tx_high_dropped.wrapping_add(bytes.len() as u32);
                } else if priority >= SerialTxPriority::NORMAL {
                    stats.tx_normal_dropped =
                        stats.tx_normal_dropped.wrapping_add(bytes.len() as u32);
                } else {
                    stats.tx_low_dropped = stats.tx_low_dropped.wrapping_add(bytes.len() as u32);
                }
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
        let n = self.read_tx_priority(out);
        self.update_stats(|stats| stats.tx_drained = stats.tx_drained.wrapping_add(n as u32));
        n
    }

    pub fn drain_tx_batch_into(&self, out: &mut [u8]) -> usize {
        let mut total = 0;
        while total < out.len() {
            let n = self.read_tx_priority(&mut out[total..]);
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

    pub fn record_wifi_rx_datagram(&self, bytes: usize) {
        self.update_stats(|stats| {
            stats.wifi_rx_datagrams = stats.wifi_rx_datagrams.wrapping_add(1);
            stats.wifi_rx_bytes = stats.wifi_rx_bytes.wrapping_add(bytes as u32);
        });
    }

    pub fn record_wifi_tx_datagram(&self, bytes: usize) {
        self.update_stats(|stats| {
            stats.wifi_tx_datagrams = stats.wifi_tx_datagrams.wrapping_add(1);
            stats.wifi_tx_bytes = stats.wifi_tx_bytes.wrapping_add(bytes as u32);
            stats.wifi_tx_max_datagram = stats.wifi_tx_max_datagram.max(bytes as u32);
        });
    }

    pub fn record_wifi_tx_error(&self) {
        self.update_stats(|stats| stats.wifi_tx_errors = stats.wifi_tx_errors.wrapping_add(1));
    }

    pub fn record_uart_tx_batch(&self, bytes: usize) {
        self.update_stats(|stats| {
            stats.uart_tx_batches = stats.uart_tx_batches.wrapping_add(1);
            stats.uart_tx_bytes = stats.uart_tx_bytes.wrapping_add(bytes as u32);
            stats.uart_tx_max_batch = stats.uart_tx_max_batch.max(bytes as u32);
        });
    }

    pub fn record_uart_rx_chunk(&self, bytes: usize) {
        self.update_stats(|stats| {
            stats.uart_rx_chunks = stats.uart_rx_chunks.wrapping_add(1);
            stats.uart_rx_bytes = stats.uart_rx_bytes.wrapping_add(bytes as u32);
        });
    }

    pub fn record_uart_tx_error(&self) {
        self.update_stats(|stats| stats.uart_tx_errors = stats.uart_tx_errors.wrapping_add(1));
    }

    pub fn record_uart_rx_error(&self) {
        self.update_stats(|stats| stats.uart_rx_errors = stats.uart_rx_errors.wrapping_add(1));
    }

    pub fn set_wifi_state(&self, state: u32) {
        WIFI_STATE.store(state, Ordering::Release);
    }

    pub fn stats(&self) -> MavlinkMailboxStats {
        let mut stats = critical_section::with(|cs| {
            let mailbox = self.inner.borrow_ref(cs);
            let mut stats = mailbox.stats;
            stats.tx_low_pending_frames = mailbox.tx_low_len as u32;
            stats.tx_pending = tx_pending_bytes()
                + low_frame_pending_bytes(
                    &mailbox.tx_low_frame_lens,
                    mailbox.tx_low_head,
                    mailbox.tx_low_len,
                );
            stats
        });
        stats.wifi_state = WIFI_STATE.load(Ordering::Acquire);
        stats
    }

    pub fn has_pending_tx(&self) -> bool {
        self.stats().tx_pending != 0
    }

    fn update_stats(&self, update: impl FnOnce(&mut MavlinkMailboxStats)) {
        critical_section::with(|cs| update(&mut self.inner.borrow_ref_mut(cs).stats));
    }

    fn push_low_frame(&self, bytes: &[u8]) -> bool {
        critical_section::with(|cs| {
            let mut mailbox = self.inner.borrow_ref_mut(cs);
            mailbox.push_low_frame(bytes)
        })
    }

    fn read_tx_priority(&self, out: &mut [u8]) -> usize {
        if let Ok(n) = TX_HIGH.try_read(out) {
            return n;
        }
        if let Ok(n) = TX_NORMAL.try_read(out) {
            return n;
        }

        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).pop_low_frame(out))
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
    pub tx_high_written: u32,
    pub tx_normal_written: u32,
    pub tx_low_written: u32,
    pub tx_drained: u32,
    pub tx_dropped: u32,
    pub tx_high_dropped: u32,
    pub tx_normal_dropped: u32,
    pub tx_low_dropped: u32,
    pub tx_low_replaced: u32,
    pub tx_pending: u32,
    pub tx_low_pending_frames: u32,
    pub wifi_rx_datagrams: u32,
    pub wifi_rx_bytes: u32,
    pub wifi_tx_datagrams: u32,
    pub wifi_tx_bytes: u32,
    pub wifi_tx_max_datagram: u32,
    pub wifi_tx_errors: u32,
    pub wifi_state: u32,
    pub core1_heartbeats: u32,
    pub uart_tx_batches: u32,
    pub uart_tx_bytes: u32,
    pub uart_tx_max_batch: u32,
    pub uart_rx_chunks: u32,
    pub uart_rx_bytes: u32,
    pub uart_tx_errors: u32,
    pub uart_rx_errors: u32,
}

pub struct MavlinkMailbox {
    stats: MavlinkMailboxStats,
    tx_low_frames: [[u8; MAVLINK_V1_MAX_FRAME_BYTES]; TX_LOW_FRAME_CAPACITY],
    tx_low_frame_lens: [u16; TX_LOW_FRAME_CAPACITY],
    tx_low_head: usize,
    tx_low_len: usize,
}

impl MavlinkMailbox {
    pub const fn new() -> Self {
        Self {
            stats: MavlinkMailboxStats {
                rx_pushed: 0,
                rx_read: 0,
                rx_dropped: 0,
                tx_written: 0,
                tx_high_written: 0,
                tx_normal_written: 0,
                tx_low_written: 0,
                tx_drained: 0,
                tx_dropped: 0,
                tx_high_dropped: 0,
                tx_normal_dropped: 0,
                tx_low_dropped: 0,
                tx_low_replaced: 0,
                tx_pending: 0,
                tx_low_pending_frames: 0,
                wifi_rx_datagrams: 0,
                wifi_rx_bytes: 0,
                wifi_tx_datagrams: 0,
                wifi_tx_bytes: 0,
                wifi_tx_max_datagram: 0,
                wifi_tx_errors: 0,
                wifi_state: 0,
                core1_heartbeats: 0,
                uart_tx_batches: 0,
                uart_tx_bytes: 0,
                uart_tx_max_batch: 0,
                uart_rx_chunks: 0,
                uart_rx_bytes: 0,
                uart_tx_errors: 0,
                uart_rx_errors: 0,
            },
            tx_low_frames: [[0; MAVLINK_V1_MAX_FRAME_BYTES]; TX_LOW_FRAME_CAPACITY],
            tx_low_frame_lens: [0; TX_LOW_FRAME_CAPACITY],
            tx_low_head: 0,
            tx_low_len: 0,
        }
    }

    fn push_low_frame(&mut self, bytes: &[u8]) -> bool {
        if bytes.len() > MAVLINK_V1_MAX_FRAME_BYTES {
            return false;
        }

        if self.tx_low_len == TX_LOW_FRAME_CAPACITY {
            self.tx_low_head = (self.tx_low_head + 1) % TX_LOW_FRAME_CAPACITY;
            self.tx_low_len -= 1;
            self.stats.tx_low_replaced = self.stats.tx_low_replaced.wrapping_add(1);
        }

        let tail = (self.tx_low_head + self.tx_low_len) % TX_LOW_FRAME_CAPACITY;
        self.tx_low_frames[tail][..bytes.len()].copy_from_slice(bytes);
        self.tx_low_frame_lens[tail] = bytes.len() as u16;
        self.tx_low_len += 1;
        true
    }

    fn pop_low_frame(&mut self, out: &mut [u8]) -> usize {
        if self.tx_low_len == 0 {
            return 0;
        }

        let len = self.tx_low_frame_lens[self.tx_low_head] as usize;
        if len > out.len() {
            return 0;
        }

        out[..len].copy_from_slice(&self.tx_low_frames[self.tx_low_head][..len]);
        self.tx_low_head = (self.tx_low_head + 1) % TX_LOW_FRAME_CAPACITY;
        self.tx_low_len -= 1;
        len
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

fn tx_pending_bytes() -> u32 {
    (TX_HIGH.capacity() - TX_HIGH.free_capacity()) as u32
        + (TX_NORMAL.capacity() - TX_NORMAL.free_capacity()) as u32
}

fn low_frame_pending_bytes(lens: &[u16; TX_LOW_FRAME_CAPACITY], head: usize, len: usize) -> u32 {
    let mut total = 0_u32;
    let mut i = 0;
    while i < len {
        total = total.wrapping_add(lens[(head + i) % TX_LOW_FRAME_CAPACITY] as u32);
        i += 1;
    }
    total
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
