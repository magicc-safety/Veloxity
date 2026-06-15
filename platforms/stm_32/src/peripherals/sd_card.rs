use embassy_stm32::gpio::Input;
use embassy_stm32::sdmmc::Error;
use embassy_stm32::sdmmc::Sdmmc;
use embassy_stm32::sdmmc::sd::{Addressable, CmdBlock, DataBlock, StorageDevice};
use embassy_stm32::time::mhz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;

use veloxity_core::errors;
use veloxity_core::packets;

pub static SD_WRITE_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::ParamPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::ParamPacket, errors::SensorError>>::new();

pub static SD_READ_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::ParamPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::ParamPacket, errors::SensorError>>::new();

pub struct SdCard {
    pub sdmmc: Sdmmc<'static>,
    pub detect: Input<'static>,
}

impl SdCard {
    async fn read(&mut self, p: &mut packets::ParamPacket, max_blocks: usize) -> Result<(), Error> {
        let mut cmd_block = CmdBlock::new();
        let mut storage =
            StorageDevice::new_sd_card(&mut self.sdmmc, &mut cmd_block, mhz(4)).await?;
        read(&mut storage, p, max_blocks).await
    }

    async fn write(&mut self, p: &packets::ParamPacket, max_blocks: usize) -> Result<(), Error> {
        let mut cmd_block = CmdBlock::new();
        let mut storage =
            StorageDevice::new_sd_card(&mut self.sdmmc, &mut cmd_block, mhz(4)).await?;
        write(&mut storage, p, max_blocks).await
    }

    async fn run(&mut self) {
        let mut card_blocks = 0usize;
        let mut cmd_block = CmdBlock::new();

        if let Ok(storage) =
            StorageDevice::new_sd_card(&mut self.sdmmc, &mut cmd_block, mhz(4)).await
        {
            let card = storage.card();
            card_blocks = (card.size() / 512) as usize;
        }

        let header = packets::RosflightPacketHeader {
            timestamp: Instant::now().as_micros(),
            status: 0u16,
        };
        let values = [0u8; packets::PARAM_PACKET_SIZE];
        let mut param_packet = packets::ParamPacket { header, values };

        let _result = self.read(&mut param_packet, card_blocks).await;

        SD_READ_SIGNAL.signal(Ok(param_packet));

        loop {
            match SD_WRITE_SIGNAL.wait().await {
                Ok(mut packet) => {
                    let result = self.write(&packet, card_blocks).await;
                    match result {
                        Ok(()) => {
                            packet.header.status = 1;
                        }
                        Err(_e) => {
                            packet.header.status = 0;
                        }
                    }
                    SD_READ_SIGNAL.signal(Ok(packet));
                }
                Err(_e) => {}
            }
        }
    }
}

async fn read(
    storage: &mut StorageDevice<'_, '_, impl Addressable>,
    p: &mut packets::ParamPacket,
    max_blocks: usize,
) -> Result<(), Error> {
    let block_size = 512; // this is a fixed value
    // number of blocks to write
    let p_size = p.values.len();
    let p_blocks = (p_size + block_size - 1) / block_size;

    let mut blocks = p_blocks;
    if blocks > max_blocks {
        blocks = max_blocks;
    };

    for i in 0..blocks {
        let mut block = DataBlock::new();
        storage.read_block(i as u32, &mut block).await?;
        p.values[i * 512..(i + 1) * 512].copy_from_slice(&block[..]);
    }
    Ok(())
}

async fn write(
    storage: &mut StorageDevice<'_, '_, impl Addressable>,
    p: &packets::ParamPacket,
    max_blocks: usize,
) -> Result<(), Error> {
    let block_size = 512; // this is a fixed value
    // number of blocks to write
    let p_size = p.values.len();
    let p_blocks = (p_size + block_size - 1) / block_size;

    let mut blocks = p_blocks;
    if blocks > max_blocks {
        blocks = max_blocks;
    };

    for i in 0..blocks {
        let mut block = DataBlock::new();
        block.copy_from_slice(&p.values[(i * 512)..((i + 1) * 512)]);
        storage.write_block(i as u32, &block).await?;
    }
    Ok(())
}

#[embassy_executor::task]
pub async fn task(mut sd_card: SdCard) {
    sd_card.run().await;
}
