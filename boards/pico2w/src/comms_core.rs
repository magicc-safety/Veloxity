use rp2350_platform::wifi::WifiCommsConfig;

pub struct MavlinkMailbox {
    rx: [u8; 4096],
    rx_len: usize,
    tx: [u8; 4096],
    tx_len: usize,
}

impl MavlinkMailbox {
    pub const fn new() -> Self {
        Self {
            rx: [0; 4096],
            rx_len: 0,
            tx: [0; 4096],
            tx_len: 0,
        }
    }

    pub fn read_into(&mut self, out: &mut [u8]) -> usize {
        let n = out.len().min(self.rx_len);
        out[..n].copy_from_slice(&self.rx[..n]);
        if n < self.rx_len {
            self.rx.copy_within(n..self.rx_len, 0);
        }
        self.rx_len -= n;
        n
    }

    pub fn write_from(&mut self, bytes: &[u8]) -> usize {
        let available = self.tx.len().saturating_sub(self.tx_len);
        let n = bytes.len().min(available);
        self.tx[self.tx_len..self.tx_len + n].copy_from_slice(&bytes[..n]);
        self.tx_len += n;
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

    pub fn push_rx(&mut self, bytes: &[u8]) -> usize {
        let available = self.rx.len().saturating_sub(self.rx_len);
        let n = bytes.len().min(available);
        self.rx[self.rx_len..self.rx_len + n].copy_from_slice(&bytes[..n]);
        self.rx_len += n;
        n
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

    pub fn run_forever(&mut self, _mailbox: &mut MavlinkMailbox) -> ! {
        loop {
            core::hint::spin_loop();
        }
    }
}
