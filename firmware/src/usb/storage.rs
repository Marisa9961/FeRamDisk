#![allow(dead_code)]

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

fn map_feram_error<SpiError, CsError>(error: FeRamError<SpiError, CsError>) -> StorageError {
    match error {
        FeRamError::OutOfRange => StorageError::MediumError,
        FeRamError::Bus(_) => StorageError::HardwareError,
    }
}
