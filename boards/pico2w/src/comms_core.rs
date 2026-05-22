use core::{
    cell::RefCell,
    sync::atomic::{AtomicU32, Ordering},
};

use critical_section::Mutex;
use rp2350_platform::wifi::WifiCommsConfig;

pub static MAVLINK_MAILBOX: Mutex<RefCell<MavlinkMailbox>> =
    Mutex::new(RefCell::new(MavlinkMailbox::new()));
static WIFI_STATE: AtomicU32 = AtomicU32::new(0);

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
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).read_into(out))
    }

    pub fn write_from(&self, bytes: &[u8]) -> usize {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).write_from(bytes))
    }

    pub fn push_rx(&self, bytes: &[u8]) -> usize {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).push_rx(bytes))
    }

    pub fn drain_tx_into(&self, out: &mut [u8]) -> usize {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).drain_tx_into(out))
    }

    pub fn record_core1_heartbeat(&self) {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).record_core1_heartbeat());
    }

    pub fn record_wifi_rx_datagram(&self) {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).record_wifi_rx_datagram());
    }

    pub fn record_wifi_tx_datagram(&self) {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).record_wifi_tx_datagram());
    }

    pub fn set_wifi_state(&self, state: u32) {
        WIFI_STATE.store(state, Ordering::Release);
    }

    pub fn stats(&self) -> MavlinkMailboxStats {
        let mut stats = critical_section::with(|cs| self.inner.borrow_ref_mut(cs).stats());
        stats.wifi_state = WIFI_STATE.load(Ordering::Acquire);
        stats
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
    rx: [u8; 4096],
    rx_len: usize,
    tx: [u8; 4096],
    tx_len: usize,
    stats: MavlinkMailboxStats,
}

impl MavlinkMailbox {
    pub const fn new() -> Self {
        Self {
            rx: [0; 4096],
            rx_len: 0,
            tx: [0; 4096],
            tx_len: 0,
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

    pub fn read_into(&mut self, out: &mut [u8]) -> usize {
        let n = out.len().min(self.rx_len);
        out[..n].copy_from_slice(&self.rx[..n]);
        if n < self.rx_len {
            self.rx.copy_within(n..self.rx_len, 0);
        }
        self.rx_len -= n;
        self.stats.rx_read = self.stats.rx_read.wrapping_add(n as u32);
        n
    }

    pub fn write_from(&mut self, bytes: &[u8]) -> usize {
        let available = self.tx.len().saturating_sub(self.tx_len);
        let n = bytes.len().min(available);
        self.tx[self.tx_len..self.tx_len + n].copy_from_slice(&bytes[..n]);
        self.tx_len += n;
        self.stats.tx_written = self.stats.tx_written.wrapping_add(n as u32);
        self.stats.tx_dropped = self
            .stats
            .tx_dropped
            .wrapping_add(bytes.len().saturating_sub(n) as u32);
        n
    }

    pub fn pending_tx(&self) -> &[u8] {
        &self.tx[..self.tx_len]
    }

    pub fn clear_tx(&mut self, n: usize) {
        let n = n.min(self.tx_len);
        if n < self.tx_len {
            self.tx.copy_within(n..self.tx_len, 0);
        }
        self.tx_len -= n;
    }

    pub fn drain_tx_into(&mut self, out: &mut [u8]) -> usize {
        let n = out.len().min(self.tx_len);
        out[..n].copy_from_slice(&self.tx[..n]);
        self.clear_tx(n);
        self.stats.tx_drained = self.stats.tx_drained.wrapping_add(n as u32);
        n
    }

    pub fn push_rx(&mut self, bytes: &[u8]) -> usize {
        let available = self.rx.len().saturating_sub(self.rx_len);
        let n = bytes.len().min(available);
        self.rx[self.rx_len..self.rx_len + n].copy_from_slice(&bytes[..n]);
        self.rx_len += n;
        self.stats.rx_pushed = self.stats.rx_pushed.wrapping_add(n as u32);
        self.stats.rx_dropped = self
            .stats
            .rx_dropped
            .wrapping_add(bytes.len().saturating_sub(n) as u32);
        n
    }

    pub fn record_core1_heartbeat(&mut self) {
        self.stats.core1_heartbeats = self.stats.core1_heartbeats.wrapping_add(1);
    }

    pub fn record_wifi_rx_datagram(&mut self) {
        self.stats.wifi_rx_datagrams = self.stats.wifi_rx_datagrams.wrapping_add(1);
    }

    pub fn record_wifi_tx_datagram(&mut self) {
        self.stats.wifi_tx_datagrams = self.stats.wifi_tx_datagrams.wrapping_add(1);
    }

    pub fn set_wifi_state(&mut self, state: u32) {
        self.stats.wifi_state = state;
    }

    pub fn stats(&self) -> MavlinkMailboxStats {
        self.stats
    }
}

impl Default for MavlinkMailbox {
    fn default() -> Self {
        Self::new()
    }
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
