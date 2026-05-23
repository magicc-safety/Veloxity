use core::cell::RefCell;

use critical_section::Mutex;
use crsf::{Packet, PacketParser, RcChannels};
use voloxide_core::packets::{RC_PACKET_CHANNELS, RcPacket, RosflightPacketHeader};

pub const CRSF_BAUDRATE: u32 = 420_000;
pub const CRSF_MAX_CHANNELS: usize = 16;
pub const CRSF_PARSER_CAPACITY: usize = 128;

const RC_QUEUE_CAPACITY: usize = 4;

const EMPTY_RC_PACKET: RcPacket = RcPacket {
    header: RosflightPacketHeader {
        timestamp: 0,
        status: 0,
    },
    n_chan: 0,
    chan: [0.0; RC_PACKET_CHANNELS],
    lol: true,
};

pub static CRSF_RC_QUEUE: Mutex<RefCell<CrsfRcQueue>> =
    Mutex::new(RefCell::new(CrsfRcQueue::new()));

#[derive(Clone, Copy)]
pub struct SharedCrsfRcQueue {
    inner: &'static Mutex<RefCell<CrsfRcQueue>>,
}

unsafe impl Send for SharedCrsfRcQueue {}
unsafe impl Sync for SharedCrsfRcQueue {}

impl SharedCrsfRcQueue {
    pub const fn new(inner: &'static Mutex<RefCell<CrsfRcQueue>>) -> Self {
        Self { inner }
    }

    pub fn push_from_receiver_task(&self, packet: RcPacket) {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).push(packet));
    }

    pub fn take_latest(&self) -> Option<RcPacket> {
        critical_section::with(|cs| self.inner.borrow_ref_mut(cs).take_latest())
    }
}

pub const SHARED_CRSF_RC_QUEUE: SharedCrsfRcQueue = SharedCrsfRcQueue::new(&CRSF_RC_QUEUE);

pub struct CrsfRcParser {
    parser: PacketParser<CRSF_PARSER_CAPACITY>,
}

impl CrsfRcParser {
    pub const fn new() -> Self {
        Self {
            parser: PacketParser::new(),
        }
    }

    pub fn push_bytes(&mut self, bytes: &[u8], now_us: u64) -> Option<RcPacket> {
        self.parser.push_bytes(bytes);
        let mut latest = None;

        while let Some(result) = self.parser.next_packet() {
            if let Ok((_destination, Packet::RcChannels(channels))) = result {
                latest = Some(rc_channels_to_packet(&channels, now_us));
            }
        }

        latest
    }
}

impl Default for CrsfRcParser {
    fn default() -> Self {
        Self::new()
    }
}

fn rc_channels_to_packet(channels: &RcChannels, now_us: u64) -> RcPacket {
    let mut chan = [0.0_f32; RC_PACKET_CHANNELS];
    for (out, raw) in chan.iter_mut().zip(channels.0.iter()) {
        *out = crsf_channel_to_unit(*raw);
    }

    RcPacket {
        header: RosflightPacketHeader {
            timestamp: now_us,
            status: 0,
        },
        n_chan: CRSF_MAX_CHANNELS as u32,
        chan,
        lol: false,
    }
}

fn crsf_channel_to_unit(raw: u16) -> f32 {
    let span = (RcChannels::CHANNEL_VALUE_2000 - RcChannels::CHANNEL_VALUE_1000) as f32;
    ((raw.saturating_sub(RcChannels::CHANNEL_VALUE_1000)) as f32 / span).clamp(0.0, 1.0)
}

pub struct CrsfRcQueue {
    packets: [RcPacket; RC_QUEUE_CAPACITY],
    head: usize,
    len: usize,
    dropped_oldest: u32,
}

impl CrsfRcQueue {
    pub const fn new() -> Self {
        Self {
            packets: [EMPTY_RC_PACKET; RC_QUEUE_CAPACITY],
            head: 0,
            len: 0,
            dropped_oldest: 0,
        }
    }

    fn push(&mut self, packet: RcPacket) {
        if self.len == RC_QUEUE_CAPACITY {
            self.head = (self.head + 1) % RC_QUEUE_CAPACITY;
            self.len -= 1;
            self.dropped_oldest = self.dropped_oldest.wrapping_add(1);
        }

        let tail = (self.head + self.len) % RC_QUEUE_CAPACITY;
        self.packets[tail] = packet;
        self.len += 1;
    }

    fn take_latest(&mut self) -> Option<RcPacket> {
        if self.len == 0 {
            return None;
        }

        let latest = (self.head + self.len - 1) % RC_QUEUE_CAPACITY;
        let packet = self.packets[latest];
        self.head = (latest + 1) % RC_QUEUE_CAPACITY;
        self.len = 0;
        Some(packet)
    }

    pub fn dropped_oldest(&self) -> u32 {
        self.dropped_oldest
    }
}

impl Default for CrsfRcQueue {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use crsf::{Packet, PacketAddress, RcChannels};

    use super::*;

    #[test]
    fn crsf_parser_converts_rc_channels_to_unit_packet() {
        let mut channels = [RcChannels::CHANNEL_VALUE_1000; 16];
        channels[0] = RcChannels::CHANNEL_VALUE_1000;
        channels[1] = RcChannels::CHANNEL_VALUE_MID;
        channels[2] = RcChannels::CHANNEL_VALUE_2000;

        let raw = Packet::RcChannels(RcChannels(channels)).into_raw(PacketAddress::Controller);
        let mut parser = CrsfRcParser::new();
        let packet = parser.push_bytes(raw.data(), 1234).unwrap();

        assert_eq!(packet.header.timestamp, 1234);
        assert_eq!(packet.n_chan, 16);
        assert!((packet.chan[0] - 0.0).abs() < 0.001);
        assert!((packet.chan[1] - 0.5).abs() < 0.01);
        assert!((packet.chan[2] - 1.0).abs() < 0.001);
        assert!(!packet.lol);
    }

    #[test]
    fn crsf_parser_returns_latest_rc_packet_from_byte_stream() {
        let first = Packet::RcChannels(RcChannels([RcChannels::CHANNEL_VALUE_1000; 16]))
            .into_raw(PacketAddress::Controller);
        let second = Packet::RcChannels(RcChannels([RcChannels::CHANNEL_VALUE_2000; 16]))
            .into_raw(PacketAddress::Controller);

        let mut parser = CrsfRcParser::new();
        let mut bytes = [0_u8; 128];
        let first_len = first.data().len();
        bytes[..first_len].copy_from_slice(first.data());
        let second_len = second.data().len();
        bytes[first_len..first_len + second_len].copy_from_slice(second.data());

        let packet = parser
            .push_bytes(&bytes[..first_len + second_len], 5678)
            .unwrap();
        assert_eq!(packet.header.timestamp, 5678);
        assert!((packet.chan[0] - 1.0).abs() < 0.001);
    }
}
