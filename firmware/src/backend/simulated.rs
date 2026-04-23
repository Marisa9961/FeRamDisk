#![allow(dead_code)]

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use crate::storage::BlockStorage;
use crate::usb::commands::{
    execute_command as execute_command_inner, SenseData as InnerSenseData,
    StallAfterCsw as InnerStallAfterCsw,
};
use crate::usb::core::{
    Cbw as InnerCbw, Csw as InnerCsw, DataDirection as InnerDataDirection,
    DataEndpoint as InnerDataEndpoint,
};
use embassy_usb_driver::EndpointError;

pub use crate::storage::backend::JournalBackend;
pub use crate::storage::error::StorageError;

pub const BLOCK_SIZE: usize = crate::storage::BLOCK_SIZE;
pub const USB_PACKET_SIZE: usize = crate::usb::constants::USB_PACKET_SIZE;

pub const CSW_STATUS_PASSED: u8 = crate::usb::constants::CSW_STATUS_PASSED;
pub const CSW_STATUS_FAILED: u8 = crate::usb::constants::CSW_STATUS_FAILED;
pub const CSW_STATUS_PHASE_ERROR: u8 = crate::usb::constants::CSW_STATUS_PHASE_ERROR;

pub const SCSI_TEST_UNIT_READY: u8 = crate::usb::constants::SCSI_TEST_UNIT_READY;
pub const SCSI_REQUEST_SENSE: u8 = crate::usb::constants::SCSI_REQUEST_SENSE;
pub const SCSI_INQUIRY: u8 = crate::usb::constants::SCSI_INQUIRY;
pub const SCSI_MODE_SENSE_6: u8 = crate::usb::constants::SCSI_MODE_SENSE_6;
pub const SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL: u8 = crate::usb::constants::SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL;
pub const SCSI_READ_FORMAT_CAPACITIES: u8 = crate::usb::constants::SCSI_READ_FORMAT_CAPACITIES;
pub const SCSI_READ_CAPACITY_10: u8 = crate::usb::constants::SCSI_READ_CAPACITY_10;
pub const SCSI_READ_10: u8 = crate::usb::constants::SCSI_READ_10;
pub const SCSI_WRITE_10: u8 = crate::usb::constants::SCSI_WRITE_10;
pub const SCSI_VERIFY_10: u8 = crate::usb::constants::SCSI_VERIFY_10;
pub const SCSI_SYNCHRONIZE_CACHE_10: u8 = crate::usb::constants::SCSI_SYNCHRONIZE_CACHE_10;
pub const SCSI_MODE_SENSE_10: u8 = crate::usb::constants::SCSI_MODE_SENSE_10;
pub const SCSI_START_STOP_UNIT: u8 = crate::usb::constants::SCSI_START_STOP_UNIT;

pub const BOT_EVENT_BULK_RESET: u8 = crate::usb::constants::BOT_EVENT_BULK_RESET;

pub type Storage = RamStorage;

#[allow(async_fn_in_trait)]
pub trait DataEndpoint {
    async fn endpoint_read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError>;
    async fn endpoint_write(&mut self, buf: &[u8]) -> Result<(), EndpointError>;
}

impl<T> InnerDataEndpoint for T
where
    T: DataEndpoint,
{
    async fn endpoint_read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        DataEndpoint::endpoint_read(self, buf).await
    }

    async fn endpoint_write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        DataEndpoint::endpoint_write(self, buf).await
    }
}

#[derive(Copy, Clone)]
pub struct Cbw {
    inner: InnerCbw,
}

impl Cbw {
    pub fn new(
        tag: u32,
        data_transfer_length: u32,
        flags: u8,
        lun: u8,
        command_length: u8,
        command: [u8; 16],
    ) -> Self {
        Self {
            inner: InnerCbw {
                packet_len: 31,
                signature_valid: true,
                tag,
                data_transfer_length,
                flags,
                lun,
                command_length,
                command,
            },
        }
    }

    pub fn data_direction(&self) -> DataDirection {
        match self.inner.data_direction() {
            InnerDataDirection::None => DataDirection::None,
            InnerDataDirection::In => DataDirection::In,
            InnerDataDirection::Out => DataDirection::Out,
        }
    }

    fn into_inner(self) -> InnerCbw {
        self.inner
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum DataDirection {
    None,
    In,
    Out,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct Csw {
    pub tag: u32,
    pub residue: u32,
    pub status: u8,
}

impl From<InnerCsw> for Csw {
    fn from(value: InnerCsw) -> Self {
        Self {
            tag: value.tag,
            residue: value.residue,
            status: value.status,
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum StallAfterCsw {
    None,
    In,
    Out,
    Both,
}

impl From<InnerStallAfterCsw> for StallAfterCsw {
    fn from(value: InnerStallAfterCsw) -> Self {
        match value {
            InnerStallAfterCsw::None => Self::None,
            InnerStallAfterCsw::In => Self::In,
            InnerStallAfterCsw::Out => Self::Out,
            InnerStallAfterCsw::Both => Self::Both,
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct CommandOutcome {
    pub csw: Csw,
    pub stall_after_csw: StallAfterCsw,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct SenseData {
    inner: InnerSenseData,
}

impl SenseData {
    pub fn good() -> Self {
        Self {
            inner: InnerSenseData::good(),
        }
    }

    pub fn to_response(self) -> [u8; 18] {
        self.inner.to_response()
    }
}

pub async fn execute_command<OUT, IN, S>(
    storage: &mut S,
    out_ep: &mut OUT,
    in_ep: &mut IN,
    sense: &mut SenseData,
    prevent_medium_removal: &mut bool,
    cbw: Cbw,
) -> Result<CommandOutcome, EndpointError>
where
    OUT: DataEndpoint,
    IN: DataEndpoint,
    S: BlockStorage,
{
    let outcome = execute_command_inner(
        storage,
        out_ep,
        in_ep,
        &mut sense.inner,
        prevent_medium_removal,
        cbw.into_inner(),
    )
    .await?;

    Ok(CommandOutcome {
        csw: outcome.csw.into(),
        stall_after_csw: outcome.stall_after_csw.into(),
    })
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BackendOp {
    ReadPhysicalBlock { block_index: u32 },
    WritePhysicalBlock { block_index: u32, data: Vec<u8> },
    ReadBytes { address: usize, len: usize },
    WriteBytes { address: usize, data: Vec<u8> },
}

#[derive(Debug)]
pub struct RamStorage {
    blocks: Vec<[u8; BLOCK_SIZE]>,
    ready: bool,
    write_protected: bool,
    next_read_error: Option<StorageError>,
    next_write_error: Option<StorageError>,
    read_errors: BTreeMap<u32, StorageError>,
    write_errors: BTreeMap<u32, StorageError>,
}

impl RamStorage {
    pub fn new(block_count: u32) -> Self {
        Self {
            blocks: vec![[0u8; BLOCK_SIZE]; block_count as usize],
            ready: true,
            write_protected: false,
            next_read_error: None,
            next_write_error: None,
            read_errors: BTreeMap::new(),
            write_errors: BTreeMap::new(),
        }
    }

    pub fn set_ready(&mut self, ready: bool) {
        self.ready = ready;
    }

    pub fn set_write_protected(&mut self, write_protected: bool) {
        self.write_protected = write_protected;
    }

    pub fn set_block(&mut self, index: u32, data: [u8; BLOCK_SIZE]) {
        self.blocks[index as usize] = data;
    }

    pub fn block(&self, index: u32) -> [u8; BLOCK_SIZE] {
        self.blocks[index as usize]
    }

    pub fn inject_next_read_error(&mut self, error: StorageError) {
        self.next_read_error = Some(error);
    }

    pub fn inject_next_write_error(&mut self, error: StorageError) {
        self.next_write_error = Some(error);
    }

    pub fn inject_read_error_at(&mut self, block_index: u32, error: StorageError) {
        self.read_errors.insert(block_index, error);
    }

    pub fn inject_write_error_at(&mut self, block_index: u32, error: StorageError) {
        self.write_errors.insert(block_index, error);
    }

    fn consume_read_error(&mut self, block_index: u32) -> Option<StorageError> {
        self.next_read_error.take().or_else(|| self.read_errors.remove(&block_index))
    }

    fn consume_write_error(&mut self, block_index: u32) -> Option<StorageError> {
        self.next_write_error
            .take()
            .or_else(|| self.write_errors.remove(&block_index))
    }
}

impl BlockStorage for RamStorage {
    fn block_count(&self) -> u32 {
        self.blocks.len() as u32
    }

    fn is_ready(&self) -> bool {
        self.ready
    }

    fn is_write_protected(&self) -> bool {
        self.write_protected
    }

    async fn read_block(&mut self, block_index: u32, out: &mut [u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        if !self.ready {
            return Err(StorageError::NotReady);
        }

        if block_index >= self.block_count() {
            return Err(StorageError::MediumError);
        }

        if let Some(error) = self.consume_read_error(block_index) {
            return Err(error);
        }

        out.copy_from_slice(&self.blocks[block_index as usize]);
        Ok(())
    }

    async fn write_block(&mut self, block_index: u32, data: &[u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        if !self.ready {
            return Err(StorageError::NotReady);
        }

        if self.write_protected {
            return Err(StorageError::WriteProtect);
        }

        if block_index >= self.block_count() {
            return Err(StorageError::MediumError);
        }

        if let Some(error) = self.consume_write_error(block_index) {
            return Err(error);
        }

        self.blocks[block_index as usize].copy_from_slice(data);
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct SharedRamStorage {
    inner: Arc<Mutex<RamStorage>>,
}

impl SharedRamStorage {
    pub fn new(block_count: u32) -> Self {
        Self {
            inner: Arc::new(Mutex::new(RamStorage::new(block_count))),
        }
    }

    pub fn with_inner(inner: Arc<Mutex<RamStorage>>) -> Self {
        Self { inner }
    }

    pub fn inner(&self) -> Arc<Mutex<RamStorage>> {
        Arc::clone(&self.inner)
    }
}

impl BlockStorage for SharedRamStorage {
    fn block_count(&self) -> u32 {
        self.inner.lock().expect("ram storage poisoned").block_count()
    }

    fn is_ready(&self) -> bool {
        self.inner.lock().expect("ram storage poisoned").is_ready()
    }

    fn is_write_protected(&self) -> bool {
        self.inner
            .lock()
            .expect("ram storage poisoned")
            .is_write_protected()
    }

    async fn read_block(&mut self, block_index: u32, out: &mut [u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        self.inner
            .lock()
            .expect("ram storage poisoned")
            .read_block(block_index, out)
            .await
    }

    async fn write_block(&mut self, block_index: u32, data: &[u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        self.inner
            .lock()
            .expect("ram storage poisoned")
            .write_block(block_index, data)
            .await
    }
}

#[derive(Debug)]
pub struct RamBackend {
    bytes: Vec<u8>,
    write_protected: bool,
    next_read_block_error: Option<StorageError>,
    next_write_block_error: Option<StorageError>,
    next_read_bytes_error: Option<StorageError>,
    next_write_bytes_error: Option<StorageError>,
    read_block_errors: BTreeMap<u32, StorageError>,
    write_block_errors: BTreeMap<u32, StorageError>,
    read_bytes_errors: BTreeMap<usize, StorageError>,
    write_bytes_errors: BTreeMap<usize, StorageError>,
    operations: Vec<BackendOp>,
}

impl RamBackend {
    pub fn new(physical_blocks: u32) -> Self {
        Self {
            bytes: vec![0u8; physical_blocks as usize * BLOCK_SIZE],
            write_protected: false,
            next_read_block_error: None,
            next_write_block_error: None,
            next_read_bytes_error: None,
            next_write_bytes_error: None,
            read_block_errors: BTreeMap::new(),
            write_block_errors: BTreeMap::new(),
            read_bytes_errors: BTreeMap::new(),
            write_bytes_errors: BTreeMap::new(),
            operations: Vec::new(),
        }
    }

    pub fn set_write_protected(&mut self, write_protected: bool) {
        self.write_protected = write_protected;
    }

    pub fn inject_next_read_block_error(&mut self, error: StorageError) {
        self.next_read_block_error = Some(error);
    }

    pub fn inject_next_write_block_error(&mut self, error: StorageError) {
        self.next_write_block_error = Some(error);
    }

    pub fn inject_next_read_bytes_error(&mut self, error: StorageError) {
        self.next_read_bytes_error = Some(error);
    }

    pub fn inject_next_write_bytes_error(&mut self, error: StorageError) {
        self.next_write_bytes_error = Some(error);
    }

    pub fn inject_read_block_error_at(&mut self, block_index: u32, error: StorageError) {
        self.read_block_errors.insert(block_index, error);
    }

    pub fn inject_write_block_error_at(&mut self, block_index: u32, error: StorageError) {
        self.write_block_errors.insert(block_index, error);
    }

    pub fn inject_read_bytes_error_at(&mut self, address: usize, error: StorageError) {
        self.read_bytes_errors.insert(address, error);
    }

    pub fn inject_write_bytes_error_at(&mut self, address: usize, error: StorageError) {
        self.write_bytes_errors.insert(address, error);
    }

    pub fn set_physical_block(&mut self, block_index: u32, data: [u8; BLOCK_SIZE]) {
        let start = block_index as usize * BLOCK_SIZE;
        self.bytes[start..start + BLOCK_SIZE].copy_from_slice(&data);
    }

    pub fn physical_block(&self, block_index: u32) -> [u8; BLOCK_SIZE] {
        let start = block_index as usize * BLOCK_SIZE;
        let mut out = [0u8; BLOCK_SIZE];
        out.copy_from_slice(&self.bytes[start..start + BLOCK_SIZE]);
        out
    }

    pub fn bytes_at(&self, address: usize, len: usize) -> Vec<u8> {
        self.bytes[address..address + len].to_vec()
    }

    pub fn operations(&self) -> &[BackendOp] {
        &self.operations
    }

    pub fn clear_operations(&mut self) {
        self.operations.clear();
    }

    fn physical_blocks(&self) -> u32 {
        (self.bytes.len() / BLOCK_SIZE) as u32
    }

    fn consume_read_block_error(&mut self, block_index: u32) -> Option<StorageError> {
        self.next_read_block_error
            .take()
            .or_else(|| self.read_block_errors.remove(&block_index))
    }

    fn consume_write_block_error(&mut self, block_index: u32) -> Option<StorageError> {
        self.next_write_block_error
            .take()
            .or_else(|| self.write_block_errors.remove(&block_index))
    }

    fn consume_read_bytes_error(&mut self, address: usize) -> Option<StorageError> {
        self.next_read_bytes_error
            .take()
            .or_else(|| self.read_bytes_errors.remove(&address))
    }

    fn consume_write_bytes_error(&mut self, address: usize) -> Option<StorageError> {
        self.next_write_bytes_error
            .take()
            .or_else(|| self.write_bytes_errors.remove(&address))
    }

    fn check_block_range(&self, block_index: u32) -> Result<usize, StorageError> {
        if block_index >= self.physical_blocks() {
            return Err(StorageError::MediumError);
        }

        Ok(block_index as usize * BLOCK_SIZE)
    }

    fn check_bytes_range(&self, address: usize, len: usize) -> Result<(), StorageError> {
        if address
            .checked_add(len)
            .is_none_or(|end| end > self.bytes.len())
        {
            return Err(StorageError::MediumError);
        }

        Ok(())
    }
}

impl JournalBackend for RamBackend {
    fn physical_block_count(&self) -> u32 {
        self.physical_blocks()
    }

    async fn read_physical_block(&mut self, block_index: u32, out: &mut [u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        self.operations
            .push(BackendOp::ReadPhysicalBlock { block_index });

        if let Some(error) = self.consume_read_block_error(block_index) {
            return Err(error);
        }

        let start = self.check_block_range(block_index)?;
        out.copy_from_slice(&self.bytes[start..start + BLOCK_SIZE]);
        Ok(())
    }

    async fn write_physical_block(&mut self, block_index: u32, data: &[u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        self.operations.push(BackendOp::WritePhysicalBlock {
            block_index,
            data: data.to_vec(),
        });

        if self.write_protected {
            return Err(StorageError::WriteProtect);
        }

        if let Some(error) = self.consume_write_block_error(block_index) {
            return Err(error);
        }

        let start = self.check_block_range(block_index)?;
        self.bytes[start..start + BLOCK_SIZE].copy_from_slice(data);
        Ok(())
    }

    async fn read_bytes(&mut self, address: usize, out: &mut [u8]) -> Result<(), StorageError> {
        self.operations.push(BackendOp::ReadBytes {
            address,
            len: out.len(),
        });

        if let Some(error) = self.consume_read_bytes_error(address) {
            return Err(error);
        }

        self.check_bytes_range(address, out.len())?;
        out.copy_from_slice(&self.bytes[address..address + out.len()]);
        Ok(())
    }

    async fn write_bytes(&mut self, address: usize, data: &[u8]) -> Result<(), StorageError> {
        self.operations.push(BackendOp::WriteBytes {
            address,
            data: data.to_vec(),
        });

        if self.write_protected {
            return Err(StorageError::WriteProtect);
        }

        if let Some(error) = self.consume_write_bytes_error(address) {
            return Err(error);
        }

        self.check_bytes_range(address, data.len())?;
        self.bytes[address..address + data.len()].copy_from_slice(data);
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct SharedRamBackend {
    inner: Arc<Mutex<RamBackend>>,
}

impl SharedRamBackend {
    pub fn new(physical_blocks: u32) -> Self {
        Self {
            inner: Arc::new(Mutex::new(RamBackend::new(physical_blocks))),
        }
    }

    pub fn inner(&self) -> Arc<Mutex<RamBackend>> {
        Arc::clone(&self.inner)
    }
}

impl JournalBackend for SharedRamBackend {
    fn physical_block_count(&self) -> u32 {
        self.inner
            .lock()
            .expect("ram backend poisoned")
            .physical_block_count()
    }

    async fn read_physical_block(&mut self, block_index: u32, out: &mut [u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        self.inner
            .lock()
            .expect("ram backend poisoned")
            .read_physical_block(block_index, out)
            .await
    }

    async fn write_physical_block(&mut self, block_index: u32, data: &[u8; BLOCK_SIZE]) -> Result<(), StorageError> {
        self.inner
            .lock()
            .expect("ram backend poisoned")
            .write_physical_block(block_index, data)
            .await
    }

    async fn read_bytes(&mut self, address: usize, out: &mut [u8]) -> Result<(), StorageError> {
        self.inner
            .lock()
            .expect("ram backend poisoned")
            .read_bytes(address, out)
            .await
    }

    async fn write_bytes(&mut self, address: usize, data: &[u8]) -> Result<(), StorageError> {
        self.inner
            .lock()
            .expect("ram backend poisoned")
            .write_bytes(address, data)
            .await
    }
}

pub async fn init_storage() -> RamStorage {
    RamStorage::new(256)
}
