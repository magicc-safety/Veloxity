// /**
// ******************************************************************************
// * File     : mavlink_parser.rs
// * Date     : May 8, 2025
// ******************************************************************************
// *
// * Copyright (c) 2023, AeroVironment, Inc.
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// * 1.Redistributions of source code must retain the above copyright notice, this
// * list of conditions and the following disclaimer.
// *
// * 2.Redistributions in binary form must reproduce the above copyright notice,
// * this list of conditions and the following disclaimer in the documentation
// * and/or other materials provided with the distribution.
// *
// * 3.Neither the name of the copyright holder nor the names of its
// * contributors may be used to endorse or promote products derived from
// * this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// ******************************************************************************
// **/
use crate::mavlink::dialects::rosflight::Rosflight;
use mavio::prelude::*;
use mavio::{Frame, Receiver, Sender};

static MAX_FRAME_SIZE_BYTES: usize = 280; // it's ok that this isn't as large as RX_BUF_SIZE
                                          // because we reset the index if we receive a whole
                                          // frame, so we'll never run out of space... we can just
                                          // read in an arbitrary number of bytes in process_bytes

#[derive(Clone, Copy)]
pub struct CompleteFrame {
    pub data: [u8; 280],
    pub len: usize,
}

pub fn process_mavlink_frame(frame: CompleteFrame) -> Option<Rosflight> {
    //defmt::trace!("MAVLink frame: {} bytes", frame.len);

    let reader = EmbeddedIoReader::new(&frame.data[..frame.len]);
    let mut receiver = Receiver::new::<V1>(reader);

    match receiver.recv() {
        Ok(parsed_frame) => {
            //defmt::trace!(
            //    "Frame - SysID: {}, CompID: {}, MsgID: {}",
            //    parsed_frame.header().system_id(),
            //    parsed_frame.header().component_id(),
            //    parsed_frame.header().message_id()
            //);

            if let Ok(message) = parsed_frame.decode::<Rosflight>() {
                return Some(message);
            } else {
                return None;
            }
        }
        Err(_) => {
            //defmt::trace!("System: Failed to parse Mavlink frame");
            return None;
        }
    }
}

#[derive(PartialEq)]
enum ParseState {
    WaitingForStart,
    ReadingHeader,
    ReadingPayload { expected_len: usize },
    ReadingChecksum { bytes_remaining: usize },
}

pub struct MavlinkParser {
    frame_buf: [u8; MAX_FRAME_SIZE_BYTES],
    frame_pos: usize,
    state: ParseState,
}

impl MavlinkParser {
    pub fn new() -> Self {
        Self {
            frame_buf: [0u8; MAX_FRAME_SIZE_BYTES],
            frame_pos: 0,
            state: ParseState::WaitingForStart,
        }
    }

    // CRC calculation for MAVLink V1
    fn calculate_crc(&self, payload_len: usize) -> u16 {
        let mut crc = 0xFFFF_u16;

        // CRC over payload length + sequence + sysid + compid + msgid + payload
        for i in 1..6 + payload_len {
            crc = self.crc_accumulate(self.frame_buf[i], crc);
        }

        // MAVLink uses a message-specific CRC extra byte
        let msg_id = self.frame_buf[5];

        //defmt::trace!("Got msg_id: {}", msg_id);

        let crc_extra = match msg_id {
            // Must define for all messages in the future
            0 => 50,   // HEARTBEAT
            21 => 159, // PARAM_REQUEST_LIST
            20 => 214, // PARAM_REQUEST_READ
            111 => 34, // TIMESYNC
            _ => {
                // Unknown message, will fail CRC
                //defmt::trace!("Unknown message ID: {}", msg_id);
                0
            }
        };

        if crc_extra != 0 {
            crc = self.crc_accumulate(crc_extra, crc);
        }

        crc
    }

    fn crc_accumulate(&self, data: u8, crc: u16) -> u16 {
        let tmp = data ^ (crc as u8); // Take low byte of crc
        let tmp = tmp ^ (tmp << 4);
        let tmp16 = tmp as u16; // Convert to u16 for final calculation

        (crc >> 8) ^ (tmp16 << 8) ^ (tmp16 << 3) ^ (tmp16 >> 4)
    }

    pub fn feed_byte(&mut self, byte: u8) -> Option<CompleteFrame> {
        match self.state {
            ParseState::WaitingForStart => {
                if byte == 0xFE {
                    //defmt::trace!("Got start byte!!!");
                    self.frame_buf[0] = byte;
                    self.frame_pos = 1;
                    self.state = ParseState::ReadingHeader;
                }
                None
            }
            ParseState::ReadingHeader => {
                //defmt::trace!("Got header!");
                self.frame_buf[self.frame_pos] = byte;
                self.frame_pos += 1;

                if self.frame_pos >= 6 {
                    // MAVLink v1 header complete
                    let payload_len = self.frame_buf[1] as usize;

                    // Validate payload length (MAVLink v1 max is 255)
                    if payload_len > 255 {
                        self.reset();
                        return None;
                    }

                    if payload_len == 0 {
                        self.state = ParseState::ReadingChecksum { bytes_remaining: 2 };
                    } else {
                        self.state = ParseState::ReadingPayload {
                            expected_len: payload_len,
                        };
                    }
                }
                None
            }
            ParseState::ReadingPayload { expected_len } => {
                //defmt::trace!("Got to payload");
                self.frame_buf[self.frame_pos] = byte;
                self.frame_pos += 1;
                let payload_bytes_read = self.frame_pos - 6;

                if payload_bytes_read >= expected_len {
                    self.state = ParseState::ReadingChecksum { bytes_remaining: 2 };
                }
                None
            }
            ParseState::ReadingChecksum { bytes_remaining } => {
                //defmt::trace!("Got to reading checksum!");
                self.frame_buf[self.frame_pos] = byte;
                self.frame_pos += 1;

                if bytes_remaining == 1 {
                    //defmt::trace!("Got structurally complete frame!");
                    // Frame structurally complete, validate checksum
                    let payload_len = self.frame_buf[1] as usize;
                    let crc_low = self.frame_buf[self.frame_pos - 2];
                    let crc_high = self.frame_buf[self.frame_pos - 1];
                    let received_crc = (crc_high as u16) << 8 | (crc_low as u16);

                    let calculated_crc = self.calculate_crc(payload_len);

                    //defmt::trace!(
                    //    "Got received crc: {}, calculated crc: {}",
                    //    received_crc,
                    //    calculated_crc
                    //);

                    if received_crc == calculated_crc {
                        // Valid frame!
                        let frame_len = self.frame_pos;
                        let mut frame = CompleteFrame {
                            data: [0u8; 280],
                            len: frame_len,
                        };

                        // Copy frame data
                        for i in 0..frame_len {
                            frame.data[i] = self.frame_buf[i];
                        }

                        //defmt::trace!("Got frame: copied {} bytes!!!", frame_len);

                        self.reset();
                        Some(frame)
                    } else {
                        // Invalid CRC, this is not a valid frame
                        //defmt::trace!(
                        //    "CRC mismatch: got {:04x}, expected {:04x}",
                        //    received_crc,
                        //    calculated_crc
                        //);

                        // Search for next potential start byte
                        let mut found_start = false;
                        let mut start_pos = 0;

                        for i in 1..self.frame_pos {
                            if self.frame_buf[i] == 0xFE {
                                found_start = true;
                                start_pos = i;
                                break;
                            }
                        }

                        if found_start {
                            // Shift buffer manually (no_std compatible)
                            let remaining = self.frame_pos - start_pos;
                            for i in 0..remaining {
                                self.frame_buf[i] = self.frame_buf[start_pos + i];
                            }
                            self.frame_pos = remaining;
                            self.state = ParseState::ReadingHeader;
                        } else {
                            // No start byte found, reset completely
                            self.reset();
                        }

                        None
                    }
                } else {
                    self.state = ParseState::ReadingChecksum {
                        bytes_remaining: bytes_remaining - 1,
                    };
                    None
                }
            }
        }
    }

    fn reset(&mut self) {
        self.frame_pos = 0;
        self.state = ParseState::WaitingForStart;
    }
}
