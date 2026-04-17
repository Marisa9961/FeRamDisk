use crate::feram::{self, FeRam};
use crate::init;
use crate::spi::FramSpi;
use crate::usb;
use crate::usb::storage::{
    visible_block_count_from_physical,
    MetadataJournalStorage,
    JOURNAL_RESERVED_BLOCKS,
};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, dma, peripherals, usb as stm32_usb, Peripherals};
use rtt_target::{rprintln, rtt_init_print};

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    USB_LP => stm32_usb::InterruptHandler<peripherals::USB>;
});

pub async fn run() {
    rtt_init_print!();
    let p: Peripherals = init::init();

    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(4_000_000);
    let spi = Spi::new(
        p.SPI1,
        p.PA5,
        p.PA7,
        p.PA6,
        p.DMA1_CH1,
        p.DMA1_CH2,
        Irqs,
        spi_cfg,
    );

    let cs0 = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    let cs1 = Output::new(p.PA3, Level::High, Speed::VeryHigh);
    let cs2 = Output::new(p.PA2, Level::High, Speed::VeryHigh);
    let cs3 = Output::new(p.PA1, Level::High, Speed::VeryHigh);

    let fram_spi = FramSpi::new(spi, cs0, cs1, cs2, cs3);
    let mut fram = FeRam::new(fram_spi);
    let visible_blocks = visible_block_count_from_physical(fram.block_count());

    rprintln!(
        "FeRAM capacity: physical={} blocks, visible={} blocks",
        fram.block_count(),
        visible_blocks
    );

    for chip_idx in 0..feram::CHIP_COUNT {
        match fram.read_id(chip_idx).await {
            Ok(id) => {
                rprintln!("Chip {}: Device ID = 0x{:02X}{:02X}{:02X}",
                    chip_idx, id[0], id[1], id[2]);
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
        Err(_) => rprintln!("Volume initialization failed"),
    }

    let mut storage = MetadataJournalStorage::new(fram);
    if storage.initialize().await.is_err() {
        rprintln!("Metadata journal initialization failed");
    }

    let usb_driver = stm32_usb::Driver::new(p.USB, Irqs, p.PA12, p.PA11);
    usb::device::run(usb_driver, storage).await;
}