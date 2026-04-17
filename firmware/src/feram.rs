#![allow(dead_code)]

use core::cmp::min;

use crate::spi::{FramSpi, FramSpiError};
use embedded_hal::digital::OutputPin;

pub const BLOCK_SIZE: usize = 512;
pub const CHIP_COUNT: usize = 4;
pub const CHIP_SIZE_BYTES: usize = 32 * 1024;
pub const TOTAL_SIZE_BYTES: usize = CHIP_COUNT * CHIP_SIZE_BYTES;
pub const TOTAL_BLOCKS: u32 = (TOTAL_SIZE_BYTES / BLOCK_SIZE) as u32;

const PARTITION_START_BLOCK: u32 = 1;
const FAT_COUNT: u8 = 2;
const FAT12_MAX_CLUSTERS: u32 = 0x0FF4;
const ROOT_DIR_ENTRIES: u16 = 32;
const RESERVED_SECTORS: u16 = 1;
const PARTITION_TYPE_FAT12: u8 = 0x01;
const MEDIA_DESCRIPTOR: u8 = 0xF8;
const BOOT_OEM_NAME: &[u8; 8] = b"FRAMDISK";
const VOLUME_LABEL: &[u8; 11] = b"FERAMDISK  ";
const FILE_SYSTEM_TYPE: &[u8; 8] = b"FAT12   ";

#[derive(Copy, Clone)]
struct Fat12Geometry {
    sectors_per_cluster: u8,
    fat_sectors: u16,
    root_dir_sectors: u16,
}

const CMD_WREN: u8 = 0x06;
const CMD_WRDI: u8 = 0x04;
const CMD_RDSR: u8 = 0x05;
const CMD_WRSR: u8 = 0x01;
const CMD_READ: u8 = 0x03;
const CMD_FAST_READ: u8 = 0x0B;
const CMD_WRITE: u8 = 0x02;
const CMD_RDID: u8 = 0x9F;
const CMD_SLEEP: u8 = 0xB9;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum FeRamError<SpiError, CsError> {
    OutOfRange,
    Bus(crate::spi::FramSpiError<SpiError, CsError>),
}

pub struct FeRam<BUS> {
    bus: BUS,
}

impl<BUS> FeRam<BUS> {
    pub const fn new(bus: BUS) -> Self {
        Self { bus }
    }

    pub fn capacity_bytes(&self) -> usize {
        TOTAL_SIZE_BYTES
    }

    pub fn block_count(&self) -> u32 {
        TOTAL_BLOCKS
    }
}

impl<'d, CS0, CS1, CS2, CS3> FeRam<FramSpi<'d, CS0, CS1, CS2, CS3>>
where
    CS0: OutputPin,
    CS1: OutputPin<Error = CS0::Error>,
    CS2: OutputPin<Error = CS0::Error>,
    CS3: OutputPin<Error = CS0::Error>,
{
    pub async fn read(
        &mut self,
        address: usize,
        out: &mut [u8],
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.check_range(address, out.len())?;

        let mut remaining = out;
        let mut current_addr = address;

        while !remaining.is_empty() {
            let chip = current_addr / CHIP_SIZE_BYTES;
            let chip_addr = (current_addr % CHIP_SIZE_BYTES) as u32;
            let chunk_len = min(CHIP_SIZE_BYTES - (current_addr % CHIP_SIZE_BYTES), remaining.len());

            let (chunk, tail) = remaining.split_at_mut(chunk_len);
            self.read_on_chip(chip, chip_addr, chunk).await?;

            current_addr += chunk_len;
            remaining = tail;
        }

        Ok(())
    }

    pub async fn fast_read(
        &mut self,
        address: usize,
        out: &mut [u8],
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.check_range(address, out.len())?;

        let mut remaining = out;
        let mut current_addr = address;

        while !remaining.is_empty() {
            let chip = current_addr / CHIP_SIZE_BYTES;
            let chip_addr = (current_addr % CHIP_SIZE_BYTES) as u32;
            let chunk_len = min(CHIP_SIZE_BYTES - (current_addr % CHIP_SIZE_BYTES), remaining.len());

            let (chunk, tail) = remaining.split_at_mut(chunk_len);
            self.fast_read_on_chip(chip, chip_addr, chunk).await?;

            current_addr += chunk_len;
            remaining = tail;
        }

        Ok(())
    }

    pub async fn write(
        &mut self,
        address: usize,
        data: &[u8],
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.check_range(address, data.len())?;

        let mut remaining = data;
        let mut current_addr = address;

        while !remaining.is_empty() {
            let chip = current_addr / CHIP_SIZE_BYTES;
            let chip_addr = (current_addr % CHIP_SIZE_BYTES) as u32;
            let chunk_len = min(CHIP_SIZE_BYTES - (current_addr % CHIP_SIZE_BYTES), remaining.len());

            let (chunk, tail) = remaining.split_at(chunk_len);
            self.write_on_chip(chip, chip_addr, chunk).await?;

            current_addr += chunk_len;
            remaining = tail;
        }

        Ok(())
    }

    pub async fn read_block(
        &mut self,
        block_index: u32,
        out: &mut [u8; BLOCK_SIZE],
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.read(block_index as usize * BLOCK_SIZE, out).await
    }

    pub async fn write_block(
        &mut self,
        block_index: u32,
        data: &[u8; BLOCK_SIZE],
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.write(block_index as usize * BLOCK_SIZE, data).await
    }

    pub async fn ensure_mass_storage_volume(
        &mut self,
    ) -> Result<bool, FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.ensure_mass_storage_volume_for_total_blocks(TOTAL_BLOCKS).await
    }

    pub async fn ensure_mass_storage_volume_for_total_blocks(
        &mut self,
        total_blocks: u32,
    ) -> Result<bool, FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.ensure_mass_storage_volume_for_total_blocks_at_offset(total_blocks, 0)
            .await
    }

    pub async fn ensure_mass_storage_volume_for_total_blocks_at_offset(
        &mut self,
        total_blocks: u32,
        volume_start_block: u32,
    ) -> Result<bool, FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        if total_blocks <= PARTITION_START_BLOCK || total_blocks > TOTAL_BLOCKS {
            return Err(FeRamError::OutOfRange);
        }

        let end_block = volume_start_block
            .checked_add(total_blocks)
            .ok_or(FeRamError::OutOfRange)?;
        if end_block > TOTAL_BLOCKS {
            return Err(FeRamError::OutOfRange);
        }

        let partition_blocks = total_blocks - PARTITION_START_BLOCK;
        let geometry = calculate_fat12_geometry(partition_blocks);

        if self
            .has_valid_volume_layout(volume_start_block, partition_blocks, geometry)
            .await?
        {
            return Ok(false);
        }

        self.format_mass_storage_volume(volume_start_block, partition_blocks, geometry)
            .await?;
        Ok(true)
    }

    async fn has_valid_volume_layout(
        &mut self,
        volume_start_block: u32,
        partition_blocks: u32,
        geometry: Fat12Geometry,
    ) -> Result<bool, FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        let mut mbr = [0u8; BLOCK_SIZE];
        self.read_block(volume_start_block, &mut mbr).await?;

        if mbr[510] != 0x55 || mbr[511] != 0xAA {
            return Ok(false);
        }

        let partition = &mbr[446..462];
        if partition[4] != PARTITION_TYPE_FAT12 {
            return Ok(false);
        }

        if u32::from_le_bytes([partition[8], partition[9], partition[10], partition[11]]) != PARTITION_START_BLOCK {
            return Ok(false);
        }

        if u32::from_le_bytes([partition[12], partition[13], partition[14], partition[15]]) != partition_blocks {
            return Ok(false);
        }

        let mut boot_sector = [0u8; BLOCK_SIZE];
        self.read_block(volume_start_block + PARTITION_START_BLOCK, &mut boot_sector)
            .await?;

        if boot_sector[510] != 0x55 || boot_sector[511] != 0xAA {
            return Ok(false);
        }

        if u16::from_le_bytes([boot_sector[11], boot_sector[12]]) != BLOCK_SIZE as u16 {
            return Ok(false);
        }

        if boot_sector[13] != geometry.sectors_per_cluster
            || u16::from_le_bytes([boot_sector[14], boot_sector[15]]) != RESERVED_SECTORS
        {
            return Ok(false);
        }

        if boot_sector[16] != FAT_COUNT
            || u16::from_le_bytes([boot_sector[17], boot_sector[18]]) != ROOT_DIR_ENTRIES
        {
            return Ok(false);
        }

        if boot_sector[21] != MEDIA_DESCRIPTOR {
            return Ok(false);
        }

        if u16::from_le_bytes([boot_sector[22], boot_sector[23]]) != geometry.fat_sectors {
            return Ok(false);
        }

        let total_sectors_16 = u16::from_le_bytes([boot_sector[19], boot_sector[20]]);
        let total_sectors_32 = u32::from_le_bytes([boot_sector[32], boot_sector[33], boot_sector[34], boot_sector[35]]);
        if partition_blocks <= u16::MAX as u32 {
            if total_sectors_16 != partition_blocks as u16 {
                return Ok(false);
            }
        } else if total_sectors_16 != 0 || total_sectors_32 != partition_blocks {
            return Ok(false);
        }

        Ok(true)
    }

    async fn format_mass_storage_volume(
        &mut self,
        volume_start_block: u32,
        partition_blocks: u32,
        geometry: Fat12Geometry,
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.write_block(volume_start_block, &build_mbr(partition_blocks)).await?;
        self.write_block(
            volume_start_block + PARTITION_START_BLOCK,
            &build_boot_sector(partition_blocks, geometry),
        )
        .await?;

        for sector_offset in 0..geometry.fat_sectors as u32 {
            let mut sector = [0u8; BLOCK_SIZE];
            if sector_offset == 0 {
                sector[0] = MEDIA_DESCRIPTOR;
                sector[1] = 0xFF;
                sector[2] = 0xFF;
            }

            let fat1_block = volume_start_block + PARTITION_START_BLOCK + RESERVED_SECTORS as u32 + sector_offset;
            let fat2_block = fat1_block + geometry.fat_sectors as u32;
            self.write_block(fat1_block, &sector).await?;
            self.write_block(fat2_block, &sector).await?;
        }

        let zero_sector = [0u8; BLOCK_SIZE];
        let root_dir_start = volume_start_block
            + PARTITION_START_BLOCK
            + RESERVED_SECTORS as u32
            + geometry.fat_sectors as u32 * FAT_COUNT as u32;
        for sector_index in 0..geometry.root_dir_sectors as u32 {
            self.write_block(
                root_dir_start + sector_index,
                &zero_sector,
            )
            .await?;
        }

        Ok(())
    }

    pub async fn write_enable(&mut self, chip: usize) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.command(chip, CMD_WREN).await
    }

    pub async fn write_disable(&mut self, chip: usize) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.command(chip, CMD_WRDI).await
    }

    pub async fn read_status(&mut self, chip: usize) -> Result<u8, FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        let mut sr = [0u8; 1];
        self.command_read(chip, CMD_RDSR, &mut sr).await?;
        Ok(sr[0])
    }

    pub async fn write_status(
        &mut self,
        chip: usize,
        status: u8,
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.write_enable(chip).await?;
        self.command_write(chip, CMD_WRSR, &[status]).await
    }

    pub async fn read_id(
        &mut self,
        chip: usize,
    ) -> Result<[u8; 3], FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        let mut id = [0u8; 3];
        self.command_read(chip, CMD_RDID, &mut id).await?;
        Ok(id)
    }

    pub async fn sleep(&mut self, chip: usize) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.command(chip, CMD_SLEEP).await
    }

    async fn command(
        &mut self,
        chip: usize,
        opcode: u8,
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.bus.select_chip(chip).map_err(FeRamError::Bus)?;
        let spi_result = self
            .bus
            .spi
            .write(&[opcode])
            .await
            .map_err(|error| FeRamError::Bus(FramSpiError::Spi(error)));
        let cs_result = self.bus.deselect_chip(chip);
        Self::finish(spi_result, cs_result)
    }

    async fn command_write(
        &mut self,
        chip: usize,
        opcode: u8,
        payload: &[u8],
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.bus.select_chip(chip).map_err(FeRamError::Bus)?;
        let spi_result = async {
            self.bus.spi.write(&[opcode]).await?;
            self.bus.spi.write(payload).await
        }
        .await
        .map_err(|error| FeRamError::Bus(FramSpiError::Spi(error)));
        let cs_result = self.bus.deselect_chip(chip);
        Self::finish(spi_result, cs_result)
    }

    async fn command_read(
        &mut self,
        chip: usize,
        opcode: u8,
        out: &mut [u8],
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        self.bus.select_chip(chip).map_err(FeRamError::Bus)?;
        let spi_result = async {
            self.bus.spi.write(&[opcode]).await?;
            self.bus.spi.read(out).await
        }
        .await
        .map_err(|error| FeRamError::Bus(FramSpiError::Spi(error)));
        let cs_result = self.bus.deselect_chip(chip);
        Self::finish(spi_result, cs_result)
    }

    async fn read_on_chip(
        &mut self,
        chip: usize,
        address: u32,
        out: &mut [u8],
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        let addr = (address & 0xFFFF) as u16;
        let header = [CMD_READ, (addr >> 8) as u8, addr as u8];

        self.bus.select_chip(chip).map_err(FeRamError::Bus)?;
        let spi_result = async {
            self.bus.spi.write(&header).await?;
            self.bus.spi.read(out).await
        }
        .await
        .map_err(|error| FeRamError::Bus(FramSpiError::Spi(error)));
        let cs_result = self.bus.deselect_chip(chip);
        Self::finish(spi_result, cs_result)
    }

    async fn fast_read_on_chip(
        &mut self,
        chip: usize,
        address: u32,
        out: &mut [u8],
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        let addr = (address & 0xFFFF) as u16;
        let header = [
            CMD_FAST_READ,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
            0x00,
        ];

        self.bus.select_chip(chip).map_err(FeRamError::Bus)?;
        let spi_result = async {
            self.bus.spi.write(&header).await?;
            self.bus.spi.read(out).await
        }
        .await
        .map_err(|error| FeRamError::Bus(FramSpiError::Spi(error)));
        let cs_result = self.bus.deselect_chip(chip);
        Self::finish(spi_result, cs_result)
    }

    async fn write_on_chip(
        &mut self,
        chip: usize,
        address: u32,
        data: &[u8],
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        let addr = (address & 0xFFFF) as u16;
        self.write_enable(chip).await?;

        let header = [
            CMD_WRITE,
            ((addr >> 8) & 0xFF) as u8,
            (addr & 0xFF) as u8,
        ];

        self.bus.select_chip(chip).map_err(FeRamError::Bus)?;
        let spi_result = async {
            self.bus.spi.write(&header).await?;
            self.bus.spi.write(data).await
        }
        .await
        .map_err(|error| FeRamError::Bus(FramSpiError::Spi(error)));
        let cs_result = self.bus.deselect_chip(chip);
        Self::finish(spi_result, cs_result)
    }

    fn check_range(
        &self,
        address: usize,
        len: usize,
    ) -> Result<(), FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        if len == 0 {
            return Ok(());
        }

        let end = address.checked_add(len).ok_or(FeRamError::OutOfRange)?;
        if end > TOTAL_SIZE_BYTES {
            Err(FeRamError::OutOfRange)
        } else {
            Ok(())
        }
    }

    fn finish<R>(
        spi_result: Result<R, FeRamError<embassy_stm32::spi::Error, CS0::Error>>,
        cs_result: Result<(), FramSpiError<embassy_stm32::spi::Error, CS0::Error>>,
    ) -> Result<R, FeRamError<embassy_stm32::spi::Error, CS0::Error>> {
        match (spi_result, cs_result) {
            (Err(error), _) => Err(error),
            (Ok(_), Err(error)) => Err(FeRamError::Bus(error)),
            (Ok(value), Ok(())) => Ok(value),
        }
    }
}

fn build_mbr(partition_blocks: u32) -> [u8; BLOCK_SIZE] {
    let mut sector = [0u8; BLOCK_SIZE];
    sector[446] = 0x00;
    sector[447] = 0x01;
    sector[448] = 0x01;
    sector[449] = 0x00;
    sector[450] = PARTITION_TYPE_FAT12;
    sector[451] = 0xFE;
    sector[452] = 0xFF;
    sector[453] = 0xFF;
    sector[454..458].copy_from_slice(&PARTITION_START_BLOCK.to_le_bytes());
    sector[458..462].copy_from_slice(&partition_blocks.to_le_bytes());
    sector[510] = 0x55;
    sector[511] = 0xAA;
    sector
}

fn build_boot_sector(partition_blocks: u32, geometry: Fat12Geometry) -> [u8; BLOCK_SIZE] {
    let mut sector = [0u8; BLOCK_SIZE];
    sector[0] = 0xEB;
    sector[1] = 0x3C;
    sector[2] = 0x90;
    sector[3..11].copy_from_slice(BOOT_OEM_NAME);
    sector[11..13].copy_from_slice(&(BLOCK_SIZE as u16).to_le_bytes());
    sector[13] = geometry.sectors_per_cluster;
    sector[14..16].copy_from_slice(&RESERVED_SECTORS.to_le_bytes());
    sector[16] = FAT_COUNT;
    sector[17..19].copy_from_slice(&ROOT_DIR_ENTRIES.to_le_bytes());
    if partition_blocks <= u16::MAX as u32 {
        sector[19..21].copy_from_slice(&(partition_blocks as u16).to_le_bytes());
        sector[32..36].copy_from_slice(&0u32.to_le_bytes());
    } else {
        sector[19..21].copy_from_slice(&0u16.to_le_bytes());
        sector[32..36].copy_from_slice(&partition_blocks.to_le_bytes());
    }
    sector[21] = MEDIA_DESCRIPTOR;
    sector[22..24].copy_from_slice(&geometry.fat_sectors.to_le_bytes());
    sector[24..26].copy_from_slice(&0x20u16.to_le_bytes());
    sector[26..28].copy_from_slice(&0x40u16.to_le_bytes());
    sector[28..32].copy_from_slice(&PARTITION_START_BLOCK.to_le_bytes());
    sector[36] = 0x80;
    sector[38] = 0x29;
    sector[39..43].copy_from_slice(&0x4652_4d31u32.to_le_bytes());
    sector[43..54].copy_from_slice(VOLUME_LABEL);
    sector[54..62].copy_from_slice(FILE_SYSTEM_TYPE);
    sector[510] = 0x55;
    sector[511] = 0xAA;
    sector
}

fn calculate_fat12_geometry(partition_blocks: u32) -> Fat12Geometry {
    let root_dir_sectors = ((ROOT_DIR_ENTRIES as u32) * 32).div_ceil(BLOCK_SIZE as u32) as u16;

    for sectors_per_cluster in [1u8, 2, 4, 8, 16, 32, 64] {
        let fat_sectors = calculate_fat_sectors(partition_blocks, sectors_per_cluster, root_dir_sectors);
        let data_sectors = partition_blocks.saturating_sub(
            RESERVED_SECTORS as u32 + FAT_COUNT as u32 * fat_sectors as u32 + root_dir_sectors as u32,
        );
        let cluster_count = data_sectors / sectors_per_cluster as u32;

        if cluster_count <= FAT12_MAX_CLUSTERS {
            return Fat12Geometry {
                sectors_per_cluster,
                fat_sectors,
                root_dir_sectors,
            };
        }
    }

    let sectors_per_cluster = 64;
    let fat_sectors = calculate_fat_sectors(partition_blocks, sectors_per_cluster, root_dir_sectors);
    Fat12Geometry {
        sectors_per_cluster,
        fat_sectors,
        root_dir_sectors,
    }
}

fn calculate_fat_sectors(partition_blocks: u32, sectors_per_cluster: u8, root_dir_sectors: u16) -> u16 {
    let mut fat_sectors = 1u32;

    loop {
        let non_data = RESERVED_SECTORS as u32 + root_dir_sectors as u32 + FAT_COUNT as u32 * fat_sectors;
        let data_sectors = partition_blocks.saturating_sub(non_data);
        let cluster_count = data_sectors / sectors_per_cluster as u32;

        // FAT12 uses 12-bit entries: required bytes = ceil(entries * 1.5).
        let fat_entries = cluster_count + 2;
        let required_fat_bytes = (fat_entries * 3).div_ceil(2);
        let required_fat_sectors = required_fat_bytes.div_ceil(BLOCK_SIZE as u32);

        if required_fat_sectors <= fat_sectors {
            return fat_sectors as u16;
        }

        fat_sectors = required_fat_sectors;
    }
}