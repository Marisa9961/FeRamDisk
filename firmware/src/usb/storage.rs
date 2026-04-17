#![allow(dead_code)]

use core::cmp::min;

use crate::feram::{FeRam, FeRamError, BLOCK_SIZE, TOTAL_BLOCKS};
use crate::spi::FramSpi;
use embedded_hal::digital::OutputPin;

/// Errors surfaced by the logical block backend and mapped to SCSI sense data.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum StorageError {
    NotReady,
    MediumError,
    WriteProtect,
    HardwareError,
}

pub const JOURNAL_RESERVED_BLOCKS: u32 = 2;

const JOURNAL_STATE_CLEAN: u8 = 0x00;
const JOURNAL_STATE_COMMITTED: u8 = 0xA5;
const JOURNAL_MAGIC: [u8; 3] = *b"JNL";
const JOURNAL_HEADER_STATE_OFFSET: usize = 0;
const JOURNAL_HEADER_MAGIC_OFFSET: usize = 1;
const JOURNAL_HEADER_TARGET_LBA_OFFSET: usize = 4;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
struct LbaRange {
    start: u32,
    end_exclusive: u32,
}

impl LbaRange {
    const fn empty() -> Self {
        Self {
            start: 0,
            end_exclusive: 0,
        }
    }

    fn contains(&self, lba: u32) -> bool {
        lba >= self.start && lba < self.end_exclusive
    }
}

pub const fn visible_block_count_from_physical(physical_blocks: u32) -> u32 {
    physical_blocks.saturating_sub(JOURNAL_RESERVED_BLOCKS)
}

/// Byte-addressable physical backend used by the metadata journal layer.
pub trait JournalBackend {
    fn physical_block_count(&self) -> u32;
    async fn read_physical_block(&mut self, block_index: u32, out: &mut [u8; BLOCK_SIZE]) -> Result<(), StorageError>;
    async fn write_physical_block(&mut self, block_index: u32, data: &[u8; BLOCK_SIZE]) -> Result<(), StorageError>;
    async fn read_bytes(&mut self, address: usize, out: &mut [u8]) -> Result<(), StorageError>;
    async fn write_bytes(&mut self, address: usize, data: &[u8]) -> Result<(), StorageError>;
}

/// Metadata-only atomic journal wrapper.
///
/// Protected LBAs (FAT tables + root directory sectors) use journaled writes,
/// while all other LBAs directly passthrough to keep bulk data throughput high.
pub struct MetadataJournalStorage<B> {
    backend: B,
    logical_block_count: u32,
    data_start_lba: u32,
    journal_header_lba: u32,
    journal_shadow_lba: u32,
    protected_lbas: LbaRange,
    ready: bool,
}

impl<B> MetadataJournalStorage<B>
where
    B: JournalBackend,
{
    pub fn new(backend: B) -> Self {
        let physical_blocks = backend.physical_block_count();
        let data_start_lba = min(JOURNAL_RESERVED_BLOCKS, physical_blocks);
        let journal_shadow_lba = if physical_blocks > 1 { 1 } else { 0 };

        Self {
            backend,
            logical_block_count: visible_block_count_from_physical(physical_blocks),
            data_start_lba,
            journal_header_lba: 0,
            journal_shadow_lba,
            protected_lbas: LbaRange::empty(),
            ready: false,
        }
    }

    pub async fn initialize(&mut self) -> Result<(), StorageError> {
        if self.logical_block_count == 0 {
            return Err(StorageError::NotReady);
        }

        self.initialize_journal_header().await?;
        self.protected_lbas = self.detect_protected_lba_range().await?;
        self.recover_pending_metadata_write().await?;
        self.ready = true;
        Ok(())
    }

    fn is_logical_lba(&self, lba: u32) -> bool {
        lba < self.logical_block_count
    }

    fn is_protected_lba(&self, lba: u32) -> bool {
        self.protected_lbas.contains(lba)
    }

    fn logical_to_physical_lba(&self, logical_lba: u32) -> u32 {
        self.data_start_lba + logical_lba
    }

    fn journal_header_address(&self, offset: usize) -> usize {
        self.journal_header_lba as usize * BLOCK_SIZE + offset
    }

    async fn initialize_journal_header(&mut self) -> Result<(), StorageError> {
        let mut magic = [0u8; 3];
        self.backend
            .read_bytes(self.journal_header_address(JOURNAL_HEADER_MAGIC_OFFSET), &mut magic)
            .await?;

        if magic != JOURNAL_MAGIC {
            self.write_journal_state(JOURNAL_STATE_CLEAN).await?;
            self.backend
                .write_bytes(self.journal_header_address(JOURNAL_HEADER_MAGIC_OFFSET), &JOURNAL_MAGIC)
                .await?;
        }

        Ok(())
    }

    async fn read_journal_state(&mut self) -> Result<u8, StorageError> {
        let mut state = [0u8; 1];
        self.backend
            .read_bytes(self.journal_header_address(JOURNAL_HEADER_STATE_OFFSET), &mut state)
            .await?;
        Ok(state[0])
    }

    async fn write_journal_state(&mut self, state: u8) -> Result<(), StorageError> {
        self.backend
            .write_bytes(self.journal_header_address(JOURNAL_HEADER_STATE_OFFSET), &[state])
            .await
    }

    async fn read_journal_target_lba(&mut self) -> Result<u32, StorageError> {
        let mut lba = [0u8; 4];
        self.backend
            .read_bytes(
                self.journal_header_address(JOURNAL_HEADER_TARGET_LBA_OFFSET),
                &mut lba,
            )
            .await?;
        Ok(u32::from_le_bytes(lba))
    }

    async fn write_journal_target_lba(&mut self, lba: u32) -> Result<(), StorageError> {
        self.backend
            .write_bytes(
                self.journal_header_address(JOURNAL_HEADER_TARGET_LBA_OFFSET),
                &lba.to_le_bytes(),
            )
            .await
    }

    async fn recover_pending_metadata_write(&mut self) -> Result<(), StorageError> {
        if self.read_journal_state().await? != JOURNAL_STATE_COMMITTED {
            return Ok(());
        }

        let target_lba = self.read_journal_target_lba().await?;
        if self.is_logical_lba(target_lba) && self.is_protected_lba(target_lba) {
            let mut shadow = [0u8; BLOCK_SIZE];
            self.backend
                .read_physical_block(self.journal_shadow_lba, &mut shadow)
                .await?;
            self.backend
                .write_physical_block(self.logical_to_physical_lba(target_lba), &shadow)
                .await?;
        }

        self.write_journal_state(JOURNAL_STATE_CLEAN).await
    }

    async fn journaled_write_block(&mut self, block_index: u32, data: &[u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        self.recover_pending_metadata_write().await?;

        self.write_journal_state(JOURNAL_STATE_CLEAN).await?;
        self.write_journal_target_lba(block_index).await?;

        self.backend
            .write_physical_block(self.journal_shadow_lba, data)
            .await?;

        // Single-byte commit marker is written last to make replay decision atomic.
        self.write_journal_state(JOURNAL_STATE_COMMITTED).await?;

        self.backend
            .write_physical_block(self.logical_to_physical_lba(block_index), data)
            .await?;

        self.write_journal_state(JOURNAL_STATE_CLEAN).await
    }

    async fn detect_protected_lba_range(&mut self) -> Result<LbaRange, StorageError> {
        let mut mbr = [0u8; BLOCK_SIZE];
        self.backend
            .read_physical_block(self.logical_to_physical_lba(0), &mut mbr)
            .await?;

        let (partition_start, partition_blocks) = self.parse_partition_geometry(&mbr);
        if partition_blocks == 0 || partition_start >= self.logical_block_count {
            return Ok(LbaRange::empty());
        }

        let mut boot_sector = [0u8; BLOCK_SIZE];
        self.backend
            .read_physical_block(self.logical_to_physical_lba(partition_start), &mut boot_sector)
            .await?;

        if boot_sector[510] != 0x55 || boot_sector[511] != 0xAA {
            return Ok(LbaRange::empty());
        }

        let bytes_per_sector = u16::from_le_bytes([boot_sector[11], boot_sector[12]]) as u32;
        if bytes_per_sector != BLOCK_SIZE as u32 {
            return Ok(LbaRange::empty());
        }

        let reserved_sectors = u16::from_le_bytes([boot_sector[14], boot_sector[15]]) as u32;
        let fat_count = boot_sector[16] as u32;
        let root_dir_entries = u16::from_le_bytes([boot_sector[17], boot_sector[18]]) as u32;
        let fat_sectors_16 = u16::from_le_bytes([boot_sector[22], boot_sector[23]]) as u32;
        let fat_sectors_32 = u32::from_le_bytes([boot_sector[36], boot_sector[37], boot_sector[38], boot_sector[39]]);
        let fat_sectors = if fat_sectors_16 != 0 { fat_sectors_16 } else { fat_sectors_32 };

        if fat_count == 0 || fat_sectors == 0 {
            return Ok(LbaRange::empty());
        }

        let root_dir_sectors = (root_dir_entries * 32).div_ceil(BLOCK_SIZE as u32);
        let start = partition_start.saturating_add(reserved_sectors);
        let protected_len = fat_count
            .saturating_mul(fat_sectors)
            .saturating_add(root_dir_sectors);
        let end = min(start.saturating_add(protected_len), self.logical_block_count);

        if start >= end {
            Ok(LbaRange::empty())
        } else {
            Ok(LbaRange {
                start,
                end_exclusive: end,
            })
        }
    }

    fn parse_partition_geometry(&self, mbr: &[u8; BLOCK_SIZE]) -> (u32, u32) {
        if mbr[510] == 0x55 && mbr[511] == 0xAA {
            let entry = &mbr[446..462];
            let partition_type = entry[4];
            let lba_start = u32::from_le_bytes([entry[8], entry[9], entry[10], entry[11]]);
            let lba_size = u32::from_le_bytes([entry[12], entry[13], entry[14], entry[15]]);

            if partition_type != 0 && lba_size != 0 {
                let clamped_size = min(
                    lba_size,
                    self.logical_block_count.saturating_sub(min(lba_start, self.logical_block_count)),
                );
                return (lba_start, clamped_size);
            }
        }

        (0, self.logical_block_count)
    }
}

/// Logical block-device contract used by the MSC BOT command engine.
pub trait BlockStorage {
    fn block_count(&self) -> u32;

    /// Report whether the logical unit is ready to accept media commands.
    ///
    /// IMPORTANT: real hardware backends should override this and return actual
    /// readiness instead of relying on the default true.
    ///
    /// Backends that need async initialization should return false until the
    /// medium is actually usable.
    fn is_ready(&self) -> bool {
        true
    }

    fn is_write_protected(&self) -> bool {
        false
    }

    async fn read_block(&mut self, block_index: u32, out: &mut [u8; BLOCK_SIZE]) -> Result<(), StorageError>;
    async fn write_block(&mut self, block_index: u32, data: &[u8; BLOCK_SIZE]) -> Result<(), StorageError>;
}

impl<'d, CS0, CS1, CS2, CS3> BlockStorage for FeRam<FramSpi<'d, CS0, CS1, CS2, CS3>>
where
    CS0: OutputPin,
    CS1: OutputPin<Error = CS0::Error>,
    CS2: OutputPin<Error = CS0::Error>,
    CS3: OutputPin<Error = CS0::Error>,
{
    fn block_count(&self) -> u32 {
        TOTAL_BLOCKS
    }

    fn is_ready(&self) -> bool {
        true
    }

    fn is_write_protected(&self) -> bool {
        false
    }

    async fn read_block(&mut self, block_index: u32, out: &mut [u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        self.read_block(block_index, out)
            .await
            .map_err(map_feram_error)
    }

    async fn write_block(&mut self, block_index: u32, data: &[u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        self.write_block(block_index, data)
            .await
            .map_err(map_feram_error)
    }
}

impl<B> BlockStorage for MetadataJournalStorage<B>
where
    B: JournalBackend,
{
    fn block_count(&self) -> u32 {
        self.logical_block_count
    }

    fn is_ready(&self) -> bool {
        self.ready
    }

    fn is_write_protected(&self) -> bool {
        false
    }

    async fn read_block(&mut self, block_index: u32, out: &mut [u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        if !self.ready {
            return Err(StorageError::NotReady);
        }

        if !self.is_logical_lba(block_index) {
            return Err(StorageError::MediumError);
        }

        self.backend
            .read_physical_block(self.logical_to_physical_lba(block_index), out)
            .await
    }

    async fn write_block(&mut self, block_index: u32, data: &[u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        if !self.ready {
            return Err(StorageError::NotReady);
        }

        if !self.is_logical_lba(block_index) {
            return Err(StorageError::MediumError);
        }

        if self.is_protected_lba(block_index) {
            self.journaled_write_block(block_index, data).await
        } else {
            self.backend
                .write_physical_block(self.logical_to_physical_lba(block_index), data)
                .await
        }
    }
}

impl<'d, CS0, CS1, CS2, CS3> JournalBackend for FeRam<FramSpi<'d, CS0, CS1, CS2, CS3>>
where
    CS0: OutputPin,
    CS1: OutputPin<Error = CS0::Error>,
    CS2: OutputPin<Error = CS0::Error>,
    CS3: OutputPin<Error = CS0::Error>,
{
    fn physical_block_count(&self) -> u32 {
        TOTAL_BLOCKS
    }

    async fn read_physical_block(&mut self, block_index: u32, out: &mut [u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        self.read_block(block_index, out)
            .await
            .map_err(map_feram_error)
    }

    async fn write_physical_block(&mut self, block_index: u32, data: &[u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        self.write_block(block_index, data)
            .await
            .map_err(map_feram_error)
    }

    async fn read_bytes(&mut self, address: usize, out: &mut [u8]) -> Result<(), StorageError> {
        self.read(address, out).await.map_err(map_feram_error)
    }

    async fn write_bytes(&mut self, address: usize, data: &[u8]) -> Result<(), StorageError> {
        self.write(address, data).await.map_err(map_feram_error)
    }
}

fn map_feram_error<SpiError, CsError>(error: FeRamError<SpiError, CsError>) -> StorageError {
    match error {
        FeRamError::OutOfRange => StorageError::MediumError,
        FeRamError::Bus(_) => StorageError::HardwareError,
    }
}
