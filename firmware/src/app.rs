use crate::init;
use crate::drivers::feram::{self, FeRam};
use crate::drivers::spi::FramSpi;
use crate::storage::{
    visible_block_count_from_physical,
    MetadataJournalStorage,
    JOURNAL_RESERVED_BLOCKS,
};
use crate::usb;
use embedded_hal::digital::OutputPin;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb as stm32_usb;
use rtt_target::rprintln;

fn spi_config() -> spi::Config {
    let mut config = spi::Config::default();
    config.frequency = Hertz(4_000_000);
    config
}

fn log_capacity(physical_blocks: u32, visible_blocks: u32) {
    rprintln!(
        "FeRAM capacity: physical={} blocks, visible={} blocks",
        physical_blocks,
        visible_blocks
    );
}

async fn log_chip_ids<'d, CS0, CS1, CS2, CS3>(fram: &mut FeRam<FramSpi<'d, CS0, CS1, CS2, CS3>>)
where
    CS0: OutputPin,
    CS1: OutputPin<Error = CS0::Error>,
    CS2: OutputPin<Error = CS0::Error>,
    CS3: OutputPin<Error = CS0::Error>,
{
    for chip_idx in 0..feram::CHIP_COUNT {
        match fram.read_id(chip_idx).await {
            Ok(id) => {
                rprintln!(
                    "Chip {}: Device ID = 0x{:02X}{:02X}{:02X}",
                    chip_idx,
                    id[0],
                    id[1],
                    id[2]
                );
            }
            Err(e) => {
                rprintln!("Chip {}: ID read failed - {:?}", chip_idx, e);
            }
        }
    }
}

async fn ensure_mass_storage_volume<'d, CS0, CS1, CS2, CS3>(
    fram: &mut FeRam<FramSpi<'d, CS0, CS1, CS2, CS3>>,
    visible_blocks: u32,
)
where
    CS0: OutputPin,
    CS1: OutputPin<Error = CS0::Error>,
    CS2: OutputPin<Error = CS0::Error>,
    CS3: OutputPin<Error = CS0::Error>,
{
    match fram
        .ensure_mass_storage_volume_for_total_blocks_at_offset(visible_blocks, JOURNAL_RESERVED_BLOCKS)
        .await
    {
        Ok(true) => rprintln!("Initialized FAT12 volume for Windows"),
        Ok(false) => rprintln!("FAT12 volume already present"),
        Err(_) => rprintln!("Volume initialization failed"),
    }
}

pub async fn run() {
    init::init_logging();
    let p = init::init_peripherals();

    let spi = Spi::new(
        p.SPI1,
        p.PA5,
        p.PA7,
        p.PA6,
        p.DMA1_CH1,
        p.DMA1_CH2,
        init::Irqs,
        spi_config(),
    );

    let cs0 = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    let cs1 = Output::new(p.PA3, Level::High, Speed::VeryHigh);
    let cs2 = Output::new(p.PA2, Level::High, Speed::VeryHigh);
    let cs3 = Output::new(p.PA1, Level::High, Speed::VeryHigh);

    let fram_spi = FramSpi::new(spi, cs0, cs1, cs2, cs3);
    let mut fram = FeRam::new(fram_spi);
    let physical_blocks = fram.block_count();
    let visible_blocks = visible_block_count_from_physical(physical_blocks);

    log_capacity(physical_blocks, visible_blocks);
    log_chip_ids(&mut fram).await;
    ensure_mass_storage_volume(&mut fram, visible_blocks).await;

    let mut storage = MetadataJournalStorage::new(fram);
    if storage.initialize().await.is_err() {
        rprintln!("Metadata journal initialization failed");
    }

    let usb_driver = stm32_usb::Driver::new(p.USB, init::Irqs, p.PA12, p.PA11);
    usb::device::run(usb_driver, storage).await;
}