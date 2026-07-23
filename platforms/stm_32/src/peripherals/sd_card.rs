// ******************************************************************************
// * File     : platforms/stm_32/src/peripherals/sd_card.rs
// * Date     : July 23, 2026
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

use core::cmp::min;

use embassy_futures::select::{Either, select};
use embassy_stm32::gpio::Input;
use embassy_stm32::sdmmc::sd::{Addressable, CmdBlock, DataBlock, StorageDevice};
use embassy_stm32::sdmmc::{Error, Sdmmc};
use embassy_stm32::time::mhz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use veloxity_core::errors;
use veloxity_core::packets::{self, PARAM_PACKET_SIZE, ParamPacket};
use veloxity_core::params::{PARAM_DEFINITIONS, PARAMS_COUNT, ParamDefinition, ParamValue, Params};

const BLOCK_SIZE: usize = 512;
const CHECKSUM_SIZE: usize = size_of::<u32>();
const MAX_BLOCKS: usize = 16;
const MAX_PAYLOAD_SIZE: usize = MAX_BLOCKS * BLOCK_SIZE - CHECKSUM_SIZE;
const PARAM_STORAGE_MAGIC: [u8; 4] = *b"VLXP";
const PARAM_STORAGE_VERSION: u16 = 1;
const PARAM_STORAGE_HEADER_SIZE: usize = 12;
const PARAM_STORAGE_VALUE_SIZE: usize = size_of::<u32>();

const _: () = assert!(
    PARAM_STORAGE_HEADER_SIZE + PARAMS_COUNT * PARAM_STORAGE_VALUE_SIZE <= PARAM_PACKET_SIZE
);

pub static SD_WRITE_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::ParamPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::ParamPacket, errors::SensorError>>::new();

pub static SD_READ_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::ParamPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::ParamPacket, errors::SensorError>>::new();

pub static SD_READ_REQUEST_SIGNAL: Signal<CriticalSectionRawMutex, u64> =
    Signal::<CriticalSectionRawMutex, u64>::new();

/// Errors returned while accessing the ROSflight parameter image.
#[derive(Debug)]
pub enum SdCardError {
    /// The STM32 SDMMC peripheral or the card reported an error.
    Device(Error),
    /// The requested image and its checksum exceed ROSflight's 16-block limit.
    ImageTooLarge,
    /// The inserted card cannot hold the requested image.
    CardTooSmall,
    /// The stored image did not pass ROSflight's CRC check.
    ChecksumMismatch,
}

impl From<Error> for SdCardError {
    fn from(error: Error) -> Self {
        Self::Device(error)
    }
}

fn param_type_tag(value: ParamValue) -> u8 {
    match value {
        ParamValue::Float(_) => 1,
        ParamValue::Int(_) => 2,
        ParamValue::Uint(_) => 3,
        ParamValue::Bool(_) => 4,
    }
}

fn hash_byte(hash: u32, byte: u8) -> u32 {
    (hash ^ u32::from(byte)).wrapping_mul(0x0100_0193)
}

fn param_schema_hash() -> u32 {
    let mut hash = 0x811C_9DC5;

    for definition in PARAM_DEFINITIONS.iter() {
        for byte in definition.name.as_bytes() {
            hash = hash_byte(hash, *byte);
        }
        hash = hash_byte(hash, 0);
        hash = hash_byte(hash, param_type_tag(definition.default));
    }

    hash
}

fn encode_param_value(value: ParamValue, definition: &ParamDefinition) -> Option<[u8; 4]> {
    match (value, definition.default) {
        (ParamValue::Float(value), ParamValue::Float(_)) => Some(value.to_bits().to_le_bytes()),
        (ParamValue::Int(value), ParamValue::Int(_)) => Some(value.to_le_bytes()),
        (ParamValue::Uint(value), ParamValue::Uint(_)) => Some(value.to_le_bytes()),
        (ParamValue::Bool(value), ParamValue::Bool(_)) => Some(u32::from(value).to_le_bytes()),
        _ => None,
    }
}

fn decode_param_value(bytes: [u8; 4], definition: &ParamDefinition) -> Option<ParamValue> {
    Some(match definition.default {
        ParamValue::Float(_) => ParamValue::Float(f32::from_bits(u32::from_le_bytes(bytes))),
        ParamValue::Int(_) => ParamValue::Int(i32::from_le_bytes(bytes)),
        ParamValue::Uint(_) => ParamValue::Uint(u32::from_le_bytes(bytes)),
        ParamValue::Bool(_) => match u32::from_le_bytes(bytes) {
            0 => ParamValue::Bool(false),
            1 => ParamValue::Bool(true),
            _ => return None,
        },
    })
}

/// Encodes the current parameter table into the versioned STM32 storage image.
pub fn encode_params(params: &Params) -> Option<ParamPacket> {
    let mut packet = ParamPacket::default();
    packet.values[..4].copy_from_slice(&PARAM_STORAGE_MAGIC);
    packet.values[4..6].copy_from_slice(&PARAM_STORAGE_VERSION.to_le_bytes());
    packet.values[6..8].copy_from_slice(&(PARAMS_COUNT as u16).to_le_bytes());
    packet.values[8..12].copy_from_slice(&param_schema_hash().to_le_bytes());

    for (index, definition) in PARAM_DEFINITIONS.iter().enumerate() {
        let offset = PARAM_STORAGE_HEADER_SIZE + index * PARAM_STORAGE_VALUE_SIZE;
        let encoded = encode_param_value(params.get_by_id(definition.id), definition)?;
        packet.values[offset..offset + PARAM_STORAGE_VALUE_SIZE].copy_from_slice(&encoded);
    }

    Some(packet)
}

/// Decodes an SD response after validating its format and parameter schema.
pub fn decode_params(packet: &ParamPacket) -> Option<Params> {
    if packet.header.status != 1
        || packet.values[..4] != PARAM_STORAGE_MAGIC
        || u16::from_le_bytes(packet.values[4..6].try_into().ok()?) != PARAM_STORAGE_VERSION
        || usize::from(u16::from_le_bytes(packet.values[6..8].try_into().ok()?)) != PARAMS_COUNT
        || u32::from_le_bytes(packet.values[8..12].try_into().ok()?) != param_schema_hash()
    {
        return None;
    }

    let mut params = Params::default();
    for (index, definition) in PARAM_DEFINITIONS.iter().enumerate() {
        let offset = PARAM_STORAGE_HEADER_SIZE + index * PARAM_STORAGE_VALUE_SIZE;
        let bytes = packet.values[offset..offset + PARAM_STORAGE_VALUE_SIZE]
            .try_into()
            .ok()?;
        params.set_by_id(definition.id, decode_param_value(bytes, definition)?);
    }

    Some(params)
}

pub struct SdCard {
    pub sdmmc: Sdmmc<'static>,
    pub detect: Input<'static>,
}

impl SdCard {
    /// Reads and verifies a ROSflight parameter image from the SD card.
    pub async fn read(&mut self, destination: &mut [u8]) -> Result<(), SdCardError> {
        validate_image_size(destination.len())?;

        let mut cmd_block = CmdBlock::new();
        let mut storage =
            StorageDevice::new_sd_card(&mut self.sdmmc, &mut cmd_block, mhz(4)).await?;
        validate_card_size(&storage, destination.len())?;

        let checksum = read_image(&mut storage, destination).await?;
        if checksum != rosflight_crc32(destination) {
            return Err(SdCardError::ChecksumMismatch);
        }

        Ok(())
    }

    /// Writes a ROSflight parameter image and its CRC to the SD card.
    pub async fn write(&mut self, source: &[u8]) -> Result<(), SdCardError> {
        validate_image_size(source.len())?;

        let mut cmd_block = CmdBlock::new();
        let mut storage =
            StorageDevice::new_sd_card(&mut self.sdmmc, &mut cmd_block, mhz(4)).await?;
        validate_card_size(&storage, source.len())?;

        write_image(&mut storage, source, rosflight_crc32(source)).await
    }

    async fn run(&mut self) {
        loop {
            match select(SD_READ_REQUEST_SIGNAL.wait(), SD_WRITE_SIGNAL.wait()).await {
                Either::First(request_id) => {
                    let mut packet = packets::ParamPacket {
                        header: packets::RosflightPacketHeader {
                            timestamp: request_id,
                            status: 0,
                        },
                        values: [0u8; packets::PARAM_PACKET_SIZE],
                    };
                    packet.header.status = u16::from(self.read(&mut packet.values).await.is_ok());
                    SD_READ_SIGNAL.signal(Ok(packet));
                }
                Either::Second(Ok(mut packet)) => {
                    packet.header.status = u16::from(self.write(&packet.values).await.is_ok());
                    SD_READ_SIGNAL.signal(Ok(packet));
                }
                Either::Second(Err(error)) => SD_READ_SIGNAL.signal(Err(error)),
            }
        }
    }
}

fn image_blocks(image_size: usize) -> usize {
    (image_size + CHECKSUM_SIZE).div_ceil(BLOCK_SIZE)
}

fn validate_image_size(image_size: usize) -> Result<(), SdCardError> {
    if image_size > MAX_PAYLOAD_SIZE {
        Err(SdCardError::ImageTooLarge)
    } else {
        Ok(())
    }
}

fn validate_card_size(
    storage: &StorageDevice<'_, '_, impl Addressable>,
    image_size: usize,
) -> Result<(), SdCardError> {
    let available_blocks = storage.card().size() as usize / BLOCK_SIZE;
    if image_blocks(image_size) > available_blocks {
        Err(SdCardError::CardTooSmall)
    } else {
        Ok(())
    }
}

async fn read_image(
    storage: &mut StorageDevice<'_, '_, impl Addressable>,
    destination: &mut [u8],
) -> Result<u32, SdCardError> {
    let image_size = destination.len();
    let mut stored_checksum = [0u8; CHECKSUM_SIZE];

    for block_index in 0..image_blocks(image_size) {
        let mut block = DataBlock::new();
        storage.read_block(block_index as u32, &mut block).await?;

        let block_start = block_index * BLOCK_SIZE;
        let payload_end = min(image_size.saturating_sub(block_start), BLOCK_SIZE);
        if payload_end != 0 {
            destination[block_start..block_start + payload_end]
                .copy_from_slice(&block[..payload_end]);
        }

        copy_checksum_overlap(&block, block_start, image_size, &mut stored_checksum);
    }

    Ok(u32::from_le_bytes(stored_checksum))
}

async fn write_image(
    storage: &mut StorageDevice<'_, '_, impl Addressable>,
    source: &[u8],
    checksum: u32,
) -> Result<(), SdCardError> {
    let image_size = source.len();
    let checksum = checksum.to_le_bytes();

    for block_index in 0..image_blocks(image_size) {
        let mut block = DataBlock::new();
        let block_start = block_index * BLOCK_SIZE;
        let payload_end = min(image_size.saturating_sub(block_start), BLOCK_SIZE);
        if payload_end != 0 {
            block[..payload_end].copy_from_slice(&source[block_start..block_start + payload_end]);
        }

        write_checksum_overlap(&mut block, block_start, image_size, &checksum);
        storage.write_block(block_index as u32, &block).await?;
    }

    Ok(())
}

fn copy_checksum_overlap(
    block: &[u8; BLOCK_SIZE],
    block_start: usize,
    image_size: usize,
    checksum: &mut [u8; CHECKSUM_SIZE],
) {
    let checksum_start = image_size;
    let checksum_end = image_size + CHECKSUM_SIZE;
    let block_end = block_start + BLOCK_SIZE;
    let overlap_start = block_start.max(checksum_start);
    let overlap_end = block_end.min(checksum_end);

    if overlap_start < overlap_end {
        checksum[overlap_start - checksum_start..overlap_end - checksum_start]
            .copy_from_slice(&block[overlap_start - block_start..overlap_end - block_start]);
    }
}

fn write_checksum_overlap(
    block: &mut [u8; BLOCK_SIZE],
    block_start: usize,
    image_size: usize,
    checksum: &[u8; CHECKSUM_SIZE],
) {
    let checksum_start = image_size;
    let checksum_end = image_size + CHECKSUM_SIZE;
    let block_end = block_start + BLOCK_SIZE;
    let overlap_start = block_start.max(checksum_start);
    let overlap_end = block_end.min(checksum_end);

    if overlap_start < overlap_end {
        block[overlap_start - block_start..overlap_end - block_start].copy_from_slice(
            &checksum[overlap_start - checksum_start..overlap_end - checksum_start],
        );
    }
}

// STM32's default CRC peripheral configuration: polynomial 0x04C11DB7,
// initial value 0xFFFFFFFF, no input/output reflection, and no final XOR.
fn rosflight_crc32(bytes: &[u8]) -> u32 {
    let mut crc = u32::MAX;

    for byte in bytes {
        crc ^= u32::from(*byte) << 24;
        for _ in 0..8 {
            crc = if crc & 0x8000_0000 != 0 {
                (crc << 1) ^ 0x04C1_1DB7
            } else {
                crc << 1
            };
        }
    }

    crc
}

#[embassy_executor::task]
pub async fn task(mut sd_card: SdCard) {
    sd_card.run().await;
}

#[cfg(test)]
mod tests {
    use super::*;
    use veloxity_core::params::ParamId;

    #[test]
    fn crc_matches_stm32_default_crc_for_standard_vector() {
        assert_eq!(rosflight_crc32(b"123456789"), 0x0376_E6E7);
    }

    #[test]
    fn block_count_includes_checksum() {
        assert_eq!(image_blocks(0), 1);
        assert_eq!(image_blocks(508), 1);
        assert_eq!(image_blocks(509), 2);
        assert_eq!(image_blocks(2048), 5);
    }

    #[test]
    fn parameter_storage_format_round_trips_values() {
        let mut params = Params::default();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        params.set_by_id(ParamId::PARAM_GYRO_X_BIAS, ParamValue::Float(-0.25));

        let mut packet = encode_params(&params).expect("parameter table must fit");
        packet.header.status = 1;
        let decoded = decode_params(&packet).expect("valid packet must decode");

        assert_eq!(
            decoded.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );
        assert_eq!(
            decoded.get_by_id(ParamId::PARAM_GYRO_X_BIAS),
            ParamValue::Float(-0.25)
        );
    }

    #[test]
    fn parameter_storage_format_rejects_schema_mismatch() {
        let mut packet = encode_params(&Params::default()).expect("parameter table must fit");
        packet.header.status = 1;
        packet.values[8] ^= 1;

        assert!(decode_params(&packet).is_none());
    }
}
