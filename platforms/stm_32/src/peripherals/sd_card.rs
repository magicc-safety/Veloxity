use embassy_stm32::gpio::Input;
use embassy_stm32::peripherals::SDMMC1;
use embassy_stm32::sdmmc::DataBlock;
use embassy_stm32::sdmmc::Error;
use embassy_stm32::sdmmc::Sdmmc;
use embassy_stm32::time::mhz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;

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
            let _result = self.sdmmc.read_block(i as u32, &mut block).await?;
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
            let _result = self.sdmmc.write_block(i as u32, &block).await?;
        }
        Ok(())
    }

    async fn run(&mut self) {
        // Initialize
        let mut card_blocks = 0usize;
        let mut card_size = 0usize;

        // Should print 400kHz for initialization

        // Initialize the SD card
        if let Err(_e) = self.sdmmc.init_card(mhz(4)).await {
        } else {
        }

        // Get card information
        match self.sdmmc.card() {
            Ok(card) => {
                card_blocks = card.csd.block_count() as usize;
                card_size = card.csd.card_size() as usize;
            }
            Err(_e) => {}
        }
        let _block_size = card_size / card_blocks;
        // any block_size other than 512 is an error!
        //    "uSD: ( {} blocks ) * ( {} bytes/block) = card size {} bytes",
        //    card_blocks,
        //    block_size,
        //    card_size
        //);

        // Read stored values from sd and push to SD_READ_SIGNAL

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

#[embassy_executor::task]
pub async fn task(mut sd_card: SdCard) {
    sd_card.run().await;
}
