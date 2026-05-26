use core::{
    cell::RefCell,
    sync::atomic::{AtomicU32, Ordering},
};

use critical_section::Mutex;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe::Pipe};
use voloxide_core::board::{SerialRxPriority, SerialTxPriority};

const MAVLINK_V1_MAX_FRAME_BYTES: usize = 263;
const TX_FRAME_CAPACITY: usize = 64;

pub static MAVLINK_MAILBOX: Mutex<RefCell<MavlinkMailbox>> =
    Mutex::new(RefCell::new(MavlinkMailbox::new()));
static COMMS_STATE: AtomicU32 = AtomicU32::new(0);

static RX_BYTES: Pipe<CriticalSectionRawMutex, 4096> = Pipe::new();

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
        let n = RX_BYTES.try_read(out).unwrap_or(0);
        self.update_stats(|stats| stats.rx_read = stats.rx_read.wrapping_add(n as u32));
        n
    }

    pub fn write_from(&self, bytes: &[u8]) -> usize {
        self.write_from_priority(bytes, SerialTxPriority::DEFAULT)
    }

    pub fn write_from_priority(&self, bytes: &[u8], priority: SerialTxPriority) -> usize {
        let sent = critical_section::with(|cs| {
            self.inner
                .borrow_ref_mut(cs)
                .push_tx_frame(bytes, priority.0)
        });

        if sent {
            self.update_stats(|stats| {
                stats.tx_written = stats.tx_written.wrapping_add(bytes.len() as u32);
                stats.tx_priority_min = priority_min(stats.tx_priority_min, priority.0);
                stats.tx_priority_max = stats.tx_priority_max.max(priority.0);
            });
            bytes.len()
        } else {
            self.update_stats(|stats| {
                stats.tx_dropped = stats.tx_dropped.wrapping_add(bytes.len() as u32);
                stats.tx_drop_priority_min = priority_min(stats.tx_drop_priority_min, priority.0);
                stats.tx_drop_priority_max = stats.tx_drop_priority_max.max(priority.0);
            });
            0
        }
    }

    pub fn push_rx(&self, bytes: &[u8]) -> usize {
        self.push_rx_priority(bytes, SerialRxPriority::DEFAULT)
    }

    pub fn push_rx_priority(&self, bytes: &[u8], priority: SerialRxPriority) -> usize {
        let sent = write_all_if_fits(&RX_BYTES, bytes);

        if sent {
            self.update_stats(|stats| {
                stats.rx_pushed = stats.rx_pushed.wrapping_add(bytes.len() as u32);
                stats.rx_priority_min = priority_min(stats.rx_priority_min, priority.0);
                stats.rx_priority_max = stats.rx_priority_max.max(priority.0);
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
        let n = critical_section::with(|cs| self.inner.borrow_ref_mut(cs).pop_tx_frame(out));
        self.update_stats(|stats| stats.tx_drained = stats.tx_drained.wrapping_add(n as u32));
        n
    }

    pub fn drain_tx_batch_into(&self, out: &mut [u8]) -> usize {
        let mut total = 0;
        while total < out.len() {
            let n = critical_section::with(|cs| {
                self.inner
                    .borrow_ref_mut(cs)
                    .pop_tx_frame(&mut out[total..])
            });
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

    pub fn set_comms_state(&self, state: u32) {
        COMMS_STATE.store(state, Ordering::Release);
    }

    pub fn stats(&self) -> MavlinkMailboxStats {
        let mut stats = critical_section::with(|cs| {
            let mailbox = self.inner.borrow_ref(cs);
            let mut stats = mailbox.stats;
            stats.tx_pending = mailbox.pending_bytes();
            stats.tx_pending_frames = mailbox.tx_len as u32;
            stats
        });
        stats.comms_state = COMMS_STATE.load(Ordering::Acquire);
        stats
    }

    pub fn has_pending_tx(&self) -> bool {
        self.stats().tx_pending != 0
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
    pub rx_priority_min: u8,
    pub rx_priority_max: u8,
    pub tx_written: u32,
    pub tx_drained: u32,
    pub tx_dropped: u32,
    pub tx_replaced: u32,
    pub tx_pending: u32,
    pub tx_pending_frames: u32,
    pub tx_priority_min: u8,
    pub tx_priority_max: u8,
    pub tx_drop_priority_min: u8,
    pub tx_drop_priority_max: u8,
    pub comms_state: u32,
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
    tx_frames: [[u8; MAVLINK_V1_MAX_FRAME_BYTES]; TX_FRAME_CAPACITY],
    tx_frame_lens: [u16; TX_FRAME_CAPACITY],
    tx_frame_priorities: [u8; TX_FRAME_CAPACITY],
    tx_len: usize,
}

impl MavlinkMailbox {
    pub const fn new() -> Self {
        Self {
            stats: MavlinkMailboxStats {
                rx_pushed: 0,
                rx_read: 0,
                rx_dropped: 0,
                rx_priority_min: 0,
                rx_priority_max: 0,
                tx_written: 0,
                tx_drained: 0,
                tx_dropped: 0,
                tx_replaced: 0,
                tx_pending: 0,
                tx_pending_frames: 0,
                tx_priority_min: 0,
                tx_priority_max: 0,
                tx_drop_priority_min: 0,
                tx_drop_priority_max: 0,
                comms_state: 0,
                core1_heartbeats: 0,
                uart_tx_batches: 0,
                uart_tx_bytes: 0,
                uart_tx_max_batch: 0,
                uart_rx_chunks: 0,
                uart_rx_bytes: 0,
                uart_tx_errors: 0,
                uart_rx_errors: 0,
            },
            tx_frames: [[0; MAVLINK_V1_MAX_FRAME_BYTES]; TX_FRAME_CAPACITY],
            tx_frame_lens: [0; TX_FRAME_CAPACITY],
            tx_frame_priorities: [0; TX_FRAME_CAPACITY],
            tx_len: 0,
        }
    }

    fn push_tx_frame(&mut self, bytes: &[u8], priority: u8) -> bool {
        if bytes.len() > MAVLINK_V1_MAX_FRAME_BYTES {
            return false;
        }

        let slot = if self.tx_len < TX_FRAME_CAPACITY {
            let slot = self.tx_len;
            self.tx_len += 1;
            slot
        } else {
            let Some(slot) = self.lowest_priority_slot() else {
                return false;
            };
            if priority <= self.tx_frame_priorities[slot] {
                return false;
            }
            self.stats.tx_replaced = self.stats.tx_replaced.wrapping_add(1);
            slot
        };

        self.tx_frames[slot][..bytes.len()].copy_from_slice(bytes);
        self.tx_frame_lens[slot] = bytes.len() as u16;
        self.tx_frame_priorities[slot] = priority;
        true
    }

    fn pop_tx_frame(&mut self, out: &mut [u8]) -> usize {
        if self.tx_len == 0 {
            return 0;
        }

        let slot = self.highest_priority_slot();
        let len = self.tx_frame_lens[slot] as usize;
        if len > out.len() {
            return 0;
        }

        out[..len].copy_from_slice(&self.tx_frames[slot][..len]);
        self.remove_tx_slot(slot);
        len
    }

    fn highest_priority_slot(&self) -> usize {
        let mut best = 0;
        let mut index = 1;
        while index < self.tx_len {
            if self.tx_frame_priorities[index] > self.tx_frame_priorities[best] {
                best = index;
            }
            index += 1;
        }
        best
    }

    fn lowest_priority_slot(&self) -> Option<usize> {
        if self.tx_len == 0 {
            return None;
        }
        let mut lowest = 0;
        let mut index = 1;
        while index < self.tx_len {
            if self.tx_frame_priorities[index] < self.tx_frame_priorities[lowest] {
                lowest = index;
            }
            index += 1;
        }
        Some(lowest)
    }

    fn remove_tx_slot(&mut self, slot: usize) {
        let last = self.tx_len - 1;
        if slot != last {
            self.tx_frames[slot] = self.tx_frames[last];
            self.tx_frame_lens[slot] = self.tx_frame_lens[last];
            self.tx_frame_priorities[slot] = self.tx_frame_priorities[last];
        }
        self.tx_len -= 1;
    }

    fn pending_bytes(&self) -> u32 {
        let mut total = 0_u32;
        let mut i = 0;
        while i < self.tx_len {
            total = total.wrapping_add(self.tx_frame_lens[i] as u32);
            i += 1;
        }
        total
    }
}

impl Default for MavlinkMailbox {
    fn default() -> Self {
        Self::new()
    }
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

fn priority_min(current: u8, value: u8) -> u8 {
    if current == 0 {
        value
    } else {
        current.min(value)
    }
}
