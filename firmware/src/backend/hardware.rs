use crate::drivers::feram::{self, FeRam};
use crate::drivers::spi::FramSpi;
use crate::init;
use crate::storage::{
    visible_block_count_from_physical,
    MetadataJournalStorage,
    JOURNAL_RESERVED_BLOCKS,
};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals;
use embassy_stm32::spi::{self, mode::Master, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb as stm32_usb;
use embassy_stm32::{Peri, Peripherals};
use rtt_target::rprintln;

pub type HardwareStorage = MetadataJournalStorage<
    FeRam<FramSpi<'static, Output<'static>, Output<'static>, Output<'static>, Output<'static>>>,
>;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum HardwareInitError {
    MissingResource(&'static str),
    VolumeInitFailed,
    JournalInitFailed,
}

pub struct HardwareParts {
    spi: Option<Spi<'static, Async, Master>>,
    cs0: Option<Output<'static>>,
    cs1: Option<Output<'static>>,
    cs2: Option<Output<'static>>,
    cs3: Option<Output<'static>>,
    usb: Option<Peri<'static, peripherals::USB>>,
    usb_dp: Option<Peri<'static, peripherals::PA12>>,
    usb_dm: Option<Peri<'static, peripherals::PA11>>,
}

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

pub fn init_hardware_parts(p: Peripherals) -> HardwareParts {
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

    HardwareParts {
        spi: Some(spi),
        cs0: Some(cs0),
        cs1: Some(cs1),
        cs2: Some(cs2),
        cs3: Some(cs3),
        usb: Some(p.USB),
        usb_dp: Some(p.PA12),
        usb_dm: Some(p.PA11),
    }
}

pub async fn init_storage(parts: &mut HardwareParts) -> Result<HardwareStorage, HardwareInitError> {
    let fram_spi = FramSpi::new(
        parts
            .spi
            .take()
            .ok_or(HardwareInitError::MissingResource("SPI already consumed"))?,
        parts
            .cs0
            .take()
            .ok_or(HardwareInitError::MissingResource("CS0 already consumed"))?,
        parts
            .cs1
            .take()
            .ok_or(HardwareInitError::MissingResource("CS1 already consumed"))?,
        parts
            .cs2
            .take()
            .ok_or(HardwareInitError::MissingResource("CS2 already consumed"))?,
        parts
            .cs3
            .take()
            .ok_or(HardwareInitError::MissingResource("CS3 already consumed"))?,
    );

    let mut fram = FeRam::new(fram_spi);
    let physical_blocks = fram.block_count();
    let visible_blocks = visible_block_count_from_physical(physical_blocks);

    log_capacity(physical_blocks, visible_blocks);

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

    match fram
        .ensure_mass_storage_volume_for_total_blocks_at_offset(visible_blocks, JOURNAL_RESERVED_BLOCKS)
        .await
    {
        Ok(true) => rprintln!("Initialized FAT12 volume for Windows"),
        Ok(false) => rprintln!("FAT12 volume already present"),
        Err(e) => {
            rprintln!("Volume initialization failed: {:?}", e);
            return Err(HardwareInitError::VolumeInitFailed);
        }
    }

    let mut storage = MetadataJournalStorage::new(fram);
    if let Err(e) = storage.initialize().await {
        rprintln!("Metadata journal initialization failed: {:?}", e);
        return Err(HardwareInitError::JournalInitFailed);
    }

    Ok(storage)
}

pub fn init_usb_driver(parts: &mut HardwareParts) -> Result<stm32_usb::Driver<'static, peripherals::USB>, HardwareInitError> {
    Ok(stm32_usb::Driver::new(
        parts
            .usb
            .take()
            .ok_or(HardwareInitError::MissingResource("USB peripheral already consumed"))?,
        init::Irqs,
        parts
            .usb_dp
            .take()
            .ok_or(HardwareInitError::MissingResource("USB DP already consumed"))?,
        parts
            .usb_dm
            .take()
            .ok_or(HardwareInitError::MissingResource("USB DM already consumed"))?,
    ))
}
