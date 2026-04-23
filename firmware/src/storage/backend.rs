#![allow(dead_code)]
#![allow(async_fn_in_trait)]

use crate::storage::BLOCK_SIZE;
#[cfg(feature = "hardware")]
use crate::drivers::feram::{FeRam, FeRamError, TOTAL_BLOCKS};
#[cfg(feature = "hardware")]
use crate::drivers::spi::FramSpi;
use crate::storage::error::StorageError;
#[cfg(feature = "hardware")]
use embedded_hal::digital::OutputPin;

/// Byte-addressable physical backend used by the metadata journal layer.
pub trait JournalBackend {
    fn physical_block_count(&self) -> u32;
    async fn read_physical_block(&mut self, block_index: u32, out: &mut [u8; BLOCK_SIZE]) -> Result<(), StorageError>;
    async fn write_physical_block(&mut self, block_index: u32, data: &[u8; BLOCK_SIZE]) -> Result<(), StorageError>;
    async fn read_bytes(&mut self, address: usize, out: &mut [u8]) -> Result<(), StorageError>;
    async fn write_bytes(&mut self, address: usize, data: &[u8]) -> Result<(), StorageError>;
}

#[cfg(feature = "hardware")]
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

#[cfg(feature = "hardware")]
pub(crate) fn map_feram_error<SpiError, CsError>(error: FeRamError<SpiError, CsError>) -> StorageError {
    match error {
        FeRamError::OutOfRange => StorageError::MediumError,
        FeRamError::Bus(_) => StorageError::HardwareError,
    }
}
