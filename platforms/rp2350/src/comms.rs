use core::{
    cell::RefCell,
    sync::atomic::{AtomicU32, Ordering},
};

use critical_section::Mutex;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe::Pipe};
use heapless::Deque;
use veloxity_core::board::{
    SERIAL_RX_FRAME_MAX_BYTES, SerialRxFrame, SerialRxPriority, SerialTxPriority,
};
use veloxity_core::comm::messages::messages::DownlinkMessage;

const MAVLINK_V1_MAX_FRAME_BYTES: usize = 263;
const TX_FRAME_CAPACITY: usize = 64;
const RX_FRAME_CAPACITY: usize = 32;
const DOWNLINK_CRITICAL_CAPACITY: usize = 64;
const DOWNLINK_TELEMETRY_STREAMS: usize = 10;

#[derive(Clone, Copy, Debug)]
pub struct QueuedDownlinkMessage {
    pub system_id: u8,
    pub msg: DownlinkMessage,
}

fn downlink_telemetry_stream(msg: &DownlinkMessage) -> Option<usize> {
    match msg {
        DownlinkMessage::OutputRaw(_) => Some(0),
        DownlinkMessage::Attitude(_) => Some(1),
        DownlinkMessage::Baro(_) => Some(2),
        DownlinkMessage::DiffPressure(_) => Some(3),
        DownlinkMessage::Imu(_) => Some(4),
        DownlinkMessage::Mag(_) => Some(5),
        DownlinkMessage::RcRaw(_) | DownlinkMessage::RcChannels(_) => Some(6),
        DownlinkMessage::Range(_) => Some(7),
        DownlinkMessage::Gnss(_) => Some(8),
        DownlinkMessage::BatteryStatus(_) => Some(9),
        DownlinkMessage::Heartbeat(_)
        | DownlinkMessage::ParamValue(_)
        | DownlinkMessage::Status(_)
        | DownlinkMessage::Timesync(_)
        | DownlinkMessage::Version(_)
        | DownlinkMessage::CmdAck(_)
        | DownlinkMessage::Statustext(_)
        | DownlinkMessage::HardError(_) => None,
    }
}

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

    pub fn enqueue_downlink(
        &self,
        system_id: u8,
        msg: DownlinkMessage,
        priority: SerialTxPriority,
    ) -> usize {
        let queued = QueuedDownlinkMessage { system_id, msg };
        let sent = critical_section::with(|cs| {
            self.inner
                .borrow_ref_mut(cs)
                .push_downlink_message(queued, priority.0)
        });

        if sent {
            self.update_stats(|stats| {
                stats.downlink_enqueued = stats.downlink_enqueued.wrapping_add(1);
                stats.downlink_priority_min = priority_min(stats.downlink_priority_min, priority.0);
                stats.downlink_priority_max = stats.downlink_priority_max.max(priority.0);
            });
            1
        } else {
            self.update_stats(|stats| {
                stats.downlink_dropped = stats.downlink_dropped.wrapping_add(1);
                stats.downlink_drop_priority_min =
                    priority_min(stats.downlink_drop_priority_min, priority.0);
                stats.downlink_drop_priority_max = stats.downlink_drop_priority_max.max(priority.0);
            });
            0
        }
    }

    pub fn pop_downlink_message(&self) -> Option<QueuedDownlinkMessage> {
        let msg = critical_section::with(|cs| self.inner.borrow_ref_mut(cs).pop_downlink_message());
        if msg.is_some() {
            self.update_stats(|stats| {
                stats.downlink_drained = stats.downlink_drained.wrapping_add(1)
            });
        }
        msg
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

    pub fn push_rx_frame_priority(&self, bytes: &[u8], priority: SerialRxPriority) -> usize {
        let sent = critical_section::with(|cs| {
            self.inner
                .borrow_ref_mut(cs)
                .push_rx_frame(bytes, priority.0)
        });

        if sent {
            self.update_stats(|stats| {
                stats.rx_frames_pushed = stats.rx_frames_pushed.wrapping_add(1);
                stats.rx_frame_bytes = stats.rx_frame_bytes.wrapping_add(bytes.len() as u32);
                stats.rx_priority_min = priority_min(stats.rx_priority_min, priority.0);
                stats.rx_priority_max = stats.rx_priority_max.max(priority.0);
            });
            bytes.len()
        } else {
            self.update_stats(|stats| {
                stats.rx_frames_dropped = stats.rx_frames_dropped.wrapping_add(1);
                stats.rx_drop_priority_min = priority_min(stats.rx_drop_priority_min, priority.0);
                stats.rx_drop_priority_max = stats.rx_drop_priority_max.max(priority.0);
            });
            0
        }
    }

    pub fn pop_rx_frame(&self) -> Option<SerialRxFrame> {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).pop_rx_frame())
    }

    pub fn has_pending_rx_frame(&self) -> bool {
        critical_section::with(|cs| self.inner.borrow_ref(cs).rx_len != 0)
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

    pub fn record_uart_rx_parse_error(&self) {
        self.update_stats(|stats| {
            stats.uart_rx_parse_errors = stats.uart_rx_parse_errors.wrapping_add(1)
        });
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
            stats.rx_pending_frames = mailbox.rx_len as u32;
            stats.downlink_pending =
                (mailbox.downlink_critical.len() + mailbox.telemetry_pending_count()) as u32;
            stats
        });
        stats.comms_state = COMMS_STATE.load(Ordering::Acquire);
        stats
    }

    pub fn has_pending_tx(&self) -> bool {
        let stats = self.stats();
        stats.tx_pending != 0 || stats.downlink_pending != 0
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
    pub rx_drop_priority_min: u8,
    pub rx_drop_priority_max: u8,
    pub rx_frames_pushed: u32,
    pub rx_frames_dropped: u32,
    pub rx_frames_replaced: u32,
    pub rx_frame_bytes: u32,
    pub rx_pending_frames: u32,
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
    pub uart_rx_parse_errors: u32,
    pub downlink_enqueued: u32,
    pub downlink_drained: u32,
    pub downlink_dropped: u32,
    pub downlink_replaced: u32,
    pub downlink_pending: u32,
    pub downlink_priority_min: u8,
    pub downlink_priority_max: u8,
    pub downlink_drop_priority_min: u8,
    pub downlink_drop_priority_max: u8,
}

pub struct MavlinkMailbox {
    stats: MavlinkMailboxStats,
    tx_frames: [[u8; MAVLINK_V1_MAX_FRAME_BYTES]; TX_FRAME_CAPACITY],
    tx_frame_lens: [u16; TX_FRAME_CAPACITY],
    tx_frame_priorities: [u8; TX_FRAME_CAPACITY],
    tx_len: usize,
    rx_frames: [[u8; SERIAL_RX_FRAME_MAX_BYTES]; RX_FRAME_CAPACITY],
    rx_frame_lens: [u16; RX_FRAME_CAPACITY],
    rx_frame_priorities: [u8; RX_FRAME_CAPACITY],
    rx_len: usize,
    downlink_critical: Deque<QueuedDownlinkMessage, DOWNLINK_CRITICAL_CAPACITY>,
    downlink_telemetry: [Option<QueuedDownlinkMessage>; DOWNLINK_TELEMETRY_STREAMS],
    downlink_telemetry_ready: Deque<usize, DOWNLINK_TELEMETRY_STREAMS>,
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
                rx_drop_priority_min: 0,
                rx_drop_priority_max: 0,
                rx_frames_pushed: 0,
                rx_frames_dropped: 0,
                rx_frames_replaced: 0,
                rx_frame_bytes: 0,
                rx_pending_frames: 0,
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
                uart_rx_parse_errors: 0,
                downlink_enqueued: 0,
                downlink_drained: 0,
                downlink_dropped: 0,
                downlink_replaced: 0,
                downlink_pending: 0,
                downlink_priority_min: 0,
                downlink_priority_max: 0,
                downlink_drop_priority_min: 0,
                downlink_drop_priority_max: 0,
            },
            tx_frames: [[0; MAVLINK_V1_MAX_FRAME_BYTES]; TX_FRAME_CAPACITY],
            tx_frame_lens: [0; TX_FRAME_CAPACITY],
            tx_frame_priorities: [0; TX_FRAME_CAPACITY],
            tx_len: 0,
            rx_frames: [[0; SERIAL_RX_FRAME_MAX_BYTES]; RX_FRAME_CAPACITY],
            rx_frame_lens: [0; RX_FRAME_CAPACITY],
            rx_frame_priorities: [0; RX_FRAME_CAPACITY],
            rx_len: 0,
            downlink_critical: Deque::new(),
            downlink_telemetry: [None; DOWNLINK_TELEMETRY_STREAMS],
            downlink_telemetry_ready: Deque::new(),
        }
    }

    fn push_downlink_message(&mut self, msg: QueuedDownlinkMessage, priority: u8) -> bool {
        if priority >= SerialTxPriority::DEFAULT.0 {
            return self.downlink_critical.push_back(msg).is_ok();
        }

        let Some(stream) = downlink_telemetry_stream(&msg.msg) else {
            return self.downlink_critical.push_back(msg).is_ok();
        };

        let replaced = self.downlink_telemetry[stream].is_some();
        self.downlink_telemetry[stream] = Some(msg);
        if replaced {
            self.stats.downlink_replaced = self.stats.downlink_replaced.wrapping_add(1);
            return true;
        }

        self.downlink_telemetry_ready.push_back(stream).is_ok()
    }

    fn pop_downlink_message(&mut self) -> Option<QueuedDownlinkMessage> {
        if let Some(msg) = self.downlink_critical.pop_front() {
            return Some(msg);
        }

        while let Some(stream) = self.downlink_telemetry_ready.pop_front() {
            if let Some(msg) = self.downlink_telemetry[stream].take() {
                return Some(msg);
            }
        }

        None
    }

    fn telemetry_pending_count(&self) -> usize {
        let mut count = 0;
        let mut i = 0;
        while i < DOWNLINK_TELEMETRY_STREAMS {
            if self.downlink_telemetry[i].is_some() {
                count += 1;
            }
            i += 1;
        }
        count
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
            let slot = self.lowest_tx_priority_slot();
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

        let slot = self.highest_tx_priority_slot();
        let len = self.tx_frame_lens[slot] as usize;
        if len > out.len() {
            return 0;
        }

        out[..len].copy_from_slice(&self.tx_frames[slot][..len]);
        self.remove_tx_slot(slot);
        len
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

    fn highest_tx_priority_slot(&self) -> usize {
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

    fn lowest_tx_priority_slot(&self) -> usize {
        let mut lowest = 0;
        let mut index = 1;
        while index < self.tx_len {
            if self.tx_frame_priorities[index] < self.tx_frame_priorities[lowest] {
                lowest = index;
            }
            index += 1;
        }
        lowest
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

    fn push_rx_frame(&mut self, bytes: &[u8], priority: u8) -> bool {
        if bytes.len() > SERIAL_RX_FRAME_MAX_BYTES {
            return false;
        }

        let slot = if self.rx_len < RX_FRAME_CAPACITY {
            let slot = self.rx_len;
            self.rx_len += 1;
            slot
        } else {
            let Some(slot) = self.lowest_rx_priority_slot() else {
                return false;
            };
            if priority <= self.rx_frame_priorities[slot] {
                return false;
            }
            self.stats.rx_frames_replaced = self.stats.rx_frames_replaced.wrapping_add(1);
            slot
        };

        self.rx_frames[slot][..bytes.len()].copy_from_slice(bytes);
        self.rx_frame_lens[slot] = bytes.len() as u16;
        self.rx_frame_priorities[slot] = priority;
        true
    }

    fn pop_rx_frame(&mut self) -> Option<SerialRxFrame> {
        if self.rx_len == 0 {
            return None;
        }

        let slot = self.highest_rx_priority_slot();
        let len = self.rx_frame_lens[slot] as usize;
        let mut frame = SerialRxFrame::default();
        frame.data[..len].copy_from_slice(&self.rx_frames[slot][..len]);
        frame.len = len;
        self.remove_rx_slot(slot);
        Some(frame)
    }

    fn highest_rx_priority_slot(&self) -> usize {
        let mut best = 0;
        let mut index = 1;
        while index < self.rx_len {
            if self.rx_frame_priorities[index] > self.rx_frame_priorities[best] {
                best = index;
            }
            index += 1;
        }
        best
    }

    fn lowest_rx_priority_slot(&self) -> Option<usize> {
        if self.rx_len == 0 {
            return None;
        }
        let mut lowest = 0;
        let mut index = 1;
        while index < self.rx_len {
            if self.rx_frame_priorities[index] < self.rx_frame_priorities[lowest] {
                lowest = index;
            }
            index += 1;
        }
        Some(lowest)
    }

    fn remove_rx_slot(&mut self, slot: usize) {
        let last = self.rx_len - 1;
        if slot != last {
            self.rx_frames[slot] = self.rx_frames[last];
            self.rx_frame_lens[slot] = self.rx_frame_lens[last];
            self.rx_frame_priorities[slot] = self.rx_frame_priorities[last];
        }
        self.rx_len -= 1;
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

#[cfg(test)]
mod tests {
    use super::*;
    use veloxity_core::comm::messages::enums::{OffboardControlIgnore, OffboardControlMode};
    use veloxity_core::comm::messages::messages::{
        AttitudeQuaternionMsg, RosflightStatusMsg, SmallImuMsg,
    };
    use veloxity_core::state_machine::ErrorFlag;

    #[test]
    fn tx_frames_pop_in_priority_order() {
        let mut mailbox = MavlinkMailbox::new();
        assert!(mailbox.push_tx_frame(&[1], SerialTxPriority::DEFAULT.0));
        assert!(mailbox.push_tx_frame(&[2], SerialTxPriority::CRITICAL.0));
        assert!(mailbox.push_tx_frame(&[3], SerialTxPriority::REPLACEABLE_TELEMETRY.0));

        let mut out = [0u8; MAVLINK_V1_MAX_FRAME_BYTES];
        assert_eq!(mailbox.pop_tx_frame(&mut out), 1);
        assert_eq!(out[0], 2);
        assert_eq!(mailbox.pop_tx_frame(&mut out), 1);
        assert_eq!(out[0], 1);
        assert_eq!(mailbox.pop_tx_frame(&mut out), 1);
        assert_eq!(out[0], 3);
    }

    #[test]
    fn full_tx_queue_replaces_lowest_priority_frame_only_for_higher_priority() {
        let mut mailbox = MavlinkMailbox::new();
        for value in 0..TX_FRAME_CAPACITY {
            assert!(mailbox.push_tx_frame(&[value as u8], SerialTxPriority::DEFAULT.0));
        }

        assert!(!mailbox.push_tx_frame(&[99], SerialTxPriority::REPLACEABLE_TELEMETRY.0));
        assert!(mailbox.push_tx_frame(&[42], SerialTxPriority::CRITICAL.0));

        let mut out = [0u8; MAVLINK_V1_MAX_FRAME_BYTES];
        assert_eq!(mailbox.pop_tx_frame(&mut out), 1);
        assert_eq!(out[0], 42);
        assert_eq!(mailbox.stats.tx_replaced, 1);
    }

    #[test]
    fn replaceable_downlink_keeps_latest_per_telemetry_stream() {
        let mut mailbox = MavlinkMailbox::new();

        assert!(
            mailbox
                .push_downlink_message(queued_imu(1), SerialTxPriority::REPLACEABLE_TELEMETRY.0,)
        );
        assert!(
            mailbox
                .push_downlink_message(queued_imu(2), SerialTxPriority::REPLACEABLE_TELEMETRY.0,)
        );

        assert_eq!(mailbox.stats.downlink_replaced, 1);
        assert_eq!(mailbox.telemetry_pending_count(), 1);

        let msg = mailbox.pop_downlink_message().expect("latest IMU");
        match msg.msg {
            DownlinkMessage::Imu(imu) => assert_eq!(imu.time_boot_us, 2),
            _ => panic!("expected IMU downlink"),
        }
        assert!(mailbox.pop_downlink_message().is_none());
    }

    #[test]
    fn replaceable_downlink_preserves_distinct_streams() {
        let mut mailbox = MavlinkMailbox::new();

        assert!(
            mailbox
                .push_downlink_message(queued_imu(1), SerialTxPriority::REPLACEABLE_TELEMETRY.0,)
        );
        assert!(mailbox.push_downlink_message(
            queued_attitude(2),
            SerialTxPriority::REPLACEABLE_TELEMETRY.0,
        ));

        assert_eq!(mailbox.stats.downlink_replaced, 0);
        assert_eq!(mailbox.telemetry_pending_count(), 2);

        assert!(matches!(
            mailbox.pop_downlink_message().map(|queued| queued.msg),
            Some(DownlinkMessage::Imu(_))
        ));
        assert!(matches!(
            mailbox.pop_downlink_message().map(|queued| queued.msg),
            Some(DownlinkMessage::Attitude(_))
        ));
        assert!(mailbox.pop_downlink_message().is_none());
    }

    #[test]
    fn non_replaceable_downlink_uses_critical_fifo_even_at_low_priority() {
        let mut mailbox = MavlinkMailbox::new();

        assert!(mailbox.push_downlink_message(
            queued_status(1),
            SerialTxPriority::REPLACEABLE_TELEMETRY.0,
        ));
        assert!(mailbox.push_downlink_message(
            queued_status(2),
            SerialTxPriority::REPLACEABLE_TELEMETRY.0,
        ));

        assert_eq!(mailbox.stats.downlink_replaced, 0);
        assert_eq!(mailbox.telemetry_pending_count(), 0);
        assert_eq!(mailbox.downlink_critical.len(), 2);

        let first = mailbox.pop_downlink_message().expect("first status");
        let second = mailbox.pop_downlink_message().expect("second status");
        assert!(matches!(first.msg, DownlinkMessage::Status(status) if status.num_errors == 1));
        assert!(matches!(second.msg, DownlinkMessage::Status(status) if status.num_errors == 2));
    }

    fn queued_imu(time_boot_us: u64) -> QueuedDownlinkMessage {
        QueuedDownlinkMessage {
            system_id: 1,
            msg: DownlinkMessage::Imu(SmallImuMsg {
                time_boot_us,
                xacc: 0.0,
                yacc: 0.0,
                zacc: 0.0,
                xgyro: 0.0,
                ygyro: 0.0,
                zgyro: 0.0,
                temperature: 0.0,
            }),
        }
    }

    fn queued_attitude(time_boot_ms: u32) -> QueuedDownlinkMessage {
        QueuedDownlinkMessage {
            system_id: 1,
            msg: DownlinkMessage::Attitude(AttitudeQuaternionMsg {
                time_boot_ms,
                q1: 1.0,
                q2: 0.0,
                q3: 0.0,
                q4: 0.0,
                rollspeed: 0.0,
                pitchspeed: 0.0,
                yawspeed: 0.0,
            }),
        }
    }

    fn queued_status(num_errors: i16) -> QueuedDownlinkMessage {
        QueuedDownlinkMessage {
            system_id: 1,
            msg: DownlinkMessage::Status(RosflightStatusMsg {
                armed: 0,
                failsafe: 0,
                rc_override: 0,
                offboard: 0,
                error_code: ErrorFlag::empty(),
                control_mode: OffboardControlMode::ModePassThrough,
                num_errors,
                loop_time_us: 0,
            }),
        }
    }
}
