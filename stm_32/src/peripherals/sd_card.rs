// /**
// ******************************************************************************
// * File     : sdcard.rs
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
// THIS CODE HAS BEEN MADE SAFE BUT SAFETY HAS NOT BEEN TESTED
use embassy_stm32::gpio::Input;
use embassy_stm32::peripherals::SDMMC1;
use embassy_stm32::sdmmc::DataBlock;
use embassy_stm32::sdmmc::Error;
use embassy_stm32::sdmmc::Sdmmc;
use embassy_stm32::time::mhz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;

//use defmt::trace;

use voloxide_core::errors;
use voloxide_core::packets;

pub static SD_WRITE_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::ParamPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::ParamPacket, errors::SensorError>>::new();

pub static SD_READ_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::ParamPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::ParamPacket, errors::SensorError>>::new();

pub struct SdCard {
    pub sdmmc: Sdmmc<'static, SDMMC1>,
    pub detect: Input<'static>,
}

impl SdCard {
    async fn read(&mut self, p: &mut packets::ParamPacket, max_blocks: usize) -> Result<(), Error> {
        let block_size = 512; // this is a fixed value
        // number of blocks to write
        let p_size = p.values.len();
        let p_blocks = (p_size + block_size - 1) / block_size;

        let mut blocks = p_blocks;
        if blocks > max_blocks {
            blocks = max_blocks;
        };

        for i in 0..blocks {
            let mut block = DataBlock([0u8; 512]);
            let result = self.sdmmc.read_block(i as u32, &mut block).await?;
            p.values[i * 512..(i + 1) * 512].copy_from_slice(&block.0);
        }
        Ok(())
    }

    async fn write(&mut self, p: &packets::ParamPacket, max_blocks: usize) -> Result<(), Error> {
        let block_size = 512; // this is a fixed value
        // number of blocks to write
        let p_size = p.values.len();
        let p_blocks = (p_size + block_size - 1) / block_size;

        let mut blocks = p_blocks;
        if blocks > max_blocks {
            blocks = max_blocks;
        };

        for i in 0..blocks {
            let mut block = DataBlock([0u8; 512]);
            block
                .0
                .copy_from_slice(&p.values[(i * 512)..((i + 1) * 512)]);
            let result = self.sdmmc.write_block(i as u32, &block).await?;
        }
        Ok(())
    }

    async fn run(&mut self) {
        // Initialize
        let mut card_blocks = 0usize;
        let mut card_size = 0usize;

        // Should print 400kHz for initialization
        //trace!("uSD: Configured clock: {}", self.sdmmc.clock().0);

        // Initialize the SD card
        if let Err(e) = self.sdmmc.init_card(mhz(4)).await {
            //trace!("uSD: Failed to initialize SD card: {:?}", e);
        } else {
            //trace!("uSD: Initialized SD card");
        }

        // Get card information
        match self.sdmmc.card() {
            Ok(card) => {
                card_blocks = card.csd.block_count() as usize;
                card_size = card.csd.card_size() as usize;

                //defmt::trace!("uSD: SD card initialized.");
            }
            Err(e) => {
                //defmt::error!("uSD: Failed to get card details: {:?}", e);
            }
        }
        let block_size = card_size / card_blocks;
        // any block_size other than 512 is an error!

        //defmt::trace!(
        //    "uSD: ( {} blocks ) * ( {} bytes/block) = card size {} bytes",
        //    card_blocks,
        //    block_size,
        //    card_size
        //);

        //trace!("uSD: Done Init Card");

        // Read stored values from sd and push to SD_READ_SIGNAL

        let mut header = packets::RosflightPacketHeader {
            timestamp: Instant::now().as_micros(),
            status: 0u16,
        };
        let mut values = [0u8; packets::PARAM_PACKET_SIZE];
        let mut param_packet = packets::ParamPacket { header, values };

        let result = self.read(&mut param_packet, card_blocks).await;

        SD_READ_SIGNAL.signal(Ok(param_packet));

        loop {
            // if there is new data, write to disk
            match SD_WRITE_SIGNAL.wait().await {
                Ok(mut packet) => {
                    let result = self.write(&packet, card_blocks).await;
                    match result {
                        Ok(()) => {
                            packet.header.status = 1;
                        }
                        Err(e) => {
                            packet.header.status = 0;
                        }
                    }
                    SD_READ_SIGNAL.signal(Ok(packet));
                }
                Err(e) => {
                    //trace!("uSD: Error here");
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut sd_card: SdCard) {
    //defmt::trace!("uSD task");
    sd_card.run().await;
}
