#![allow(dead_code)]

use core::cmp::min;
use core::sync::atomic::{AtomicU8, Ordering};

use crate::feram::{FeRam, FeRamError, BLOCK_SIZE, TOTAL_BLOCKS};
use crate::spi::FramSpi;
use embassy_time::{with_timeout, Duration};
use embassy_usb_driver::{EndpointError, EndpointIn, EndpointOut};
use embedded_hal::digital::OutputPin;

const CBW_SIGNATURE: u32 = 0x4342_5355;
const CSW_SIGNATURE: u32 = 0x5342_5355;

const CSW_STATUS_PASSED: u8 = 0;
const CSW_STATUS_FAILED: u8 = 1;
const CSW_STATUS_PHASE_ERROR: u8 = 2;

const SCSI_TEST_UNIT_READY: u8 = 0x00;
const SCSI_REQUEST_SENSE: u8 = 0x03;
const SCSI_INQUIRY: u8 = 0x12;
const SCSI_MODE_SENSE_6: u8 = 0x1A;
const SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL: u8 = 0x1E;
const SCSI_READ_FORMAT_CAPACITIES: u8 = 0x23;
const SCSI_READ_CAPACITY_10: u8 = 0x25;
const SCSI_READ_10: u8 = 0x28;
const SCSI_WRITE_10: u8 = 0x2A;
const SCSI_VERIFY_10: u8 = 0x2F;
const SCSI_SYNCHRONIZE_CACHE_10: u8 = 0x35;
const SCSI_MODE_SENSE_10: u8 = 0x5A;
const SCSI_START_STOP_UNIT: u8 = 0x1B;

const SENSE_NOT_READY: u8 = 0x02;
const SENSE_MEDIUM_ERROR: u8 = 0x03;
const SENSE_HARDWARE_ERROR: u8 = 0x04;
const SENSE_ILLEGAL_REQUEST: u8 = 0x05;
const SENSE_DATA_PROTECT: u8 = 0x07;

const ASC_INVALID_COMMAND_OPCODE: u8 = 0x20;
const ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE: u8 = 0x21;
const ASC_INVALID_FIELD_IN_CDB: u8 = 0x24;
const ASC_WRITE_PROTECTED: u8 = 0x27;
const ASC_LOGICAL_UNIT_NOT_READY: u8 = 0x04;
const ASCQ_INITIALIZING_COMMAND_REQUIRED: u8 = 0x02;
const ASC_UNRECOVERED_READ_ERROR: u8 = 0x11;
const ASC_WRITE_ERROR: u8 = 0x0C;
const ASC_INTERNAL_TARGET_FAILURE: u8 = 0x44;

const MODE_PAGE_CACHING: u8 = 0x08;

const CBW_READ_TIMEOUT_MS: u64 = 1500;
const DATA_OUT_TIMEOUT_MS: u64 = 1500;
const OVERFLOW_DRAIN_TIMEOUT_MS: u64 = 20;
const OVERFLOW_DRAIN_MAX_PACKETS: usize = 128;

const LUN_COUNT: u8 = 1;
const USB_PACKET_SIZE: usize = 64;

pub const BOT_ACTION_STALL_IN: u8 = 1 << 0;
pub const BOT_ACTION_STALL_OUT: u8 = 1 << 1;
pub const BOT_EVENT_BULK_RESET: u8 = 1 << 0;

pub struct BotControl {
    bus_actions: AtomicU8,
    msc_events: AtomicU8,
}

impl BotControl {
    pub const fn new() -> Self {
        Self {
            bus_actions: AtomicU8::new(0),
            msc_events: AtomicU8::new(0),
        }
    }

    pub fn request_stall_in(&self) {
        self.bus_actions.fetch_or(BOT_ACTION_STALL_IN, Ordering::Release);
    }

    pub fn request_stall_out(&self) {
        self.bus_actions.fetch_or(BOT_ACTION_STALL_OUT, Ordering::Release);
    }

    pub fn signal_bulk_reset(&self) {
        self.msc_events.fetch_or(BOT_EVENT_BULK_RESET, Ordering::Release);
    }

    pub fn take_bus_actions(&self) -> u8 {
        self.bus_actions.swap(0, Ordering::Acquire)
    }

    pub fn take_msc_events(&self) -> u8 {
        self.msc_events.swap(0, Ordering::Acquire)
    }
}

#[derive(Copy, Clone, Eq, PartialEq)]
enum DataDirection {
    None,
    In,
    Out,
}

#[derive(Copy, Clone, Debug, Default)]
struct SenseData {
    key: u8,
    asc: u8,
    ascq: u8,
    valid: bool,
    information: u32,
    ili: bool,
    sksv: bool,
    sense_key_specific: [u8; 3],
}

impl SenseData {
    fn good() -> Self {
        Self::default()
    }

    fn illegal_request(asc: u8) -> Self {
        Self {
            key: SENSE_ILLEGAL_REQUEST,
            asc,
            ascq: 0,
            valid: false,
            information: 0,
            ili: false,
            sksv: false,
            sense_key_specific: [0; 3],
        }
    }

    fn illegal_field_in_cdb(field_pointer: u16) -> Self {
        Self {
            key: SENSE_ILLEGAL_REQUEST,
            asc: ASC_INVALID_FIELD_IN_CDB,
            ascq: 0,
            valid: false,
            information: 0,
            ili: false,
            sksv: true,
            sense_key_specific: [0x40, (field_pointer >> 8) as u8, field_pointer as u8],
        }
    }

    fn lba_out_of_range(lba: u32) -> Self {
        Self {
            key: SENSE_ILLEGAL_REQUEST,
            asc: ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE,
            ascq: 0,
            valid: true,
            information: lba,
            ili: false,
            sksv: false,
            sense_key_specific: [0; 3],
        }
    }

    fn not_ready_initializing() -> Self {
        Self {
            key: SENSE_NOT_READY,
            asc: ASC_LOGICAL_UNIT_NOT_READY,
            ascq: ASCQ_INITIALIZING_COMMAND_REQUIRED,
            valid: false,
            information: 0,
            ili: false,
            sksv: false,
            sense_key_specific: [0; 3],
        }
    }

    fn from_storage_error(error: StorageError, is_write: bool) -> Self {
        match error {
            StorageError::NotReady => Self::not_ready_initializing(),
            StorageError::MediumError => Self {
                key: SENSE_MEDIUM_ERROR,
                asc: if is_write { ASC_WRITE_ERROR } else { ASC_UNRECOVERED_READ_ERROR },
                ascq: 0,
                valid: false,
                information: 0,
                ili: false,
                sksv: false,
                sense_key_specific: [0; 3],
            },
            StorageError::WriteProtect => Self {
                key: SENSE_DATA_PROTECT,
                asc: ASC_WRITE_PROTECTED,
                ascq: 0,
                valid: false,
                information: 0,
                ili: false,
                sksv: false,
                sense_key_specific: [0; 3],
            },
            StorageError::HardwareError => Self {
                key: SENSE_HARDWARE_ERROR,
                asc: ASC_INTERNAL_TARGET_FAILURE,
                ascq: 0,
                valid: false,
                information: 0,
                ili: false,
                sksv: false,
                sense_key_specific: [0; 3],
            },
        }
    }

    fn transfer_length_mismatch(residue: u32, cdb_field_pointer: u16) -> Self {
        Self {
            key: SENSE_ILLEGAL_REQUEST,
            asc: ASC_INVALID_FIELD_IN_CDB,
            ascq: 0,
            valid: true,
            information: residue,
            ili: true,
            sksv: true,
            sense_key_specific: [0x40, (cdb_field_pointer >> 8) as u8, cdb_field_pointer as u8],
        }
    }

    fn to_response(self) -> [u8; 18] {
        let mut response = [0u8; 18];
        response[0] = 0x70 | if self.valid { 0x80 } else { 0x00 };
        response[2] = (self.key & 0x0F) | if self.ili { 0x20 } else { 0x00 };
        response[3..7].copy_from_slice(&self.information.to_be_bytes());
        response[7] = 10;
        response[12] = self.asc;
        response[13] = self.ascq;
        if self.sksv {
            response[15] = self.sense_key_specific[0] | 0x80;
            response[16] = self.sense_key_specific[1];
            response[17] = self.sense_key_specific[2];
        }
        response
    }
}

#[derive(Debug, Clone, Copy)]
struct Cbw {
    packet_len: usize,
    signature_valid: bool,
    tag: u32,
    data_transfer_length: u32,
    flags: u8,
    lun: u8,
    command_length: u8,
    command: [u8; 16],
}

impl Cbw {
    fn parse(packet: &[u8]) -> Self {
        let mut bytes = [0u8; 31];
        let copy_len = min(packet.len(), bytes.len());
        bytes[..copy_len].copy_from_slice(&packet[..copy_len]);

        let mut command = [0u8; 16];
        command.copy_from_slice(&bytes[15..31]);

        Self {
            packet_len: packet.len(),
            signature_valid: u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) == CBW_SIGNATURE,
            tag: u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
            data_transfer_length: u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
            flags: bytes[12],
            lun: bytes[13],
            command_length: bytes[14],
            command,
        }
    }

    fn is_valid(&self) -> bool {
        self.packet_len == 31
            && self.signature_valid
            && (self.flags & 0x7F) == 0
            && (self.lun & 0xF0) == 0
            && self.lun < LUN_COUNT
            && (1..=16).contains(&self.command_length)
    }

    fn opcode(&self) -> u8 {
        self.command[0]
    }

    fn expects_in(&self) -> bool {
        self.flags & 0x80 != 0
    }

    fn data_direction(&self) -> DataDirection {
        if self.data_transfer_length == 0 {
            DataDirection::None
        } else if self.expects_in() {
            DataDirection::In
        } else {
            DataDirection::Out
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Csw {
    tag: u32,
    residue: u32,
    status: u8,
}

impl Csw {
    fn to_bytes(self) -> [u8; 13] {
        let mut response = [0u8; 13];
        response[0..4].copy_from_slice(&CSW_SIGNATURE.to_le_bytes());
        response[4..8].copy_from_slice(&self.tag.to_le_bytes());
        response[8..12].copy_from_slice(&self.residue.to_le_bytes());
        response[12] = self.status;
        response
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum StorageError {
    NotReady,
    MediumError,
    WriteProtect,
    HardwareError,
}

pub trait BlockStorage {
    fn block_count(&self) -> u32;

    /// Report whether the logical unit is ready to accept media commands.
    ///
    /// IMPORTANT: real hardware backends should override this and return actual
    /// readiness instead of relying on the default `true`.
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

#[derive(Copy, Clone)]
enum BotState {
    WaitingForCbw,
    Executing(Cbw),
    SendingCsw { csw: Csw, stall_after_csw: StallAfterCsw },
}

#[derive(Copy, Clone)]
enum StallAfterCsw {
    None,
    In,
    Out,
    Both,
}

struct CommandOutcome {
    csw: Csw,
    stall_after_csw: StallAfterCsw,
}

#[derive(Copy, Clone)]
enum TransferError {
    Endpoint(EndpointError),
    Storage(StorageError),
}

struct WriteTransfer {
    written_bytes: u32,
    consumed_bytes: u32,
    short_packet: bool,
    packet_overflow: bool,
}

pub async fn run<OUT, IN, S>(mut out_ep: OUT, mut in_ep: IN, mut storage: S, bot_control: &BotControl)
where
    OUT: EndpointOut,
    IN: EndpointIn,
    S: BlockStorage,
{
    let mut sense = SenseData::good();
    let mut prevent_medium_removal = false;

    loop {
        in_ep.wait_enabled().await;
        out_ep.wait_enabled().await;

        let mut state = BotState::WaitingForCbw;

        'session: loop {
            if bot_control.take_msc_events() & BOT_EVENT_BULK_RESET != 0 {
                state = BotState::WaitingForCbw;
                sense = SenseData::good();
                continue;
            }

            match state {
                BotState::WaitingForCbw => {
                    let cbw = match read_cbw(&mut out_ep).await {
                        Ok(Some(cbw)) => cbw,
                        Ok(None) => continue,
                        Err(EndpointError::Disabled) => break 'session,
                        Err(_) => break 'session,
                    };

                    if !cbw.is_valid() {
                        state = BotState::SendingCsw {
                            csw: Csw {
                                tag: cbw.tag,
                                residue: cbw.data_transfer_length,
                                status: CSW_STATUS_PHASE_ERROR,
                            },
                            stall_after_csw: StallAfterCsw::Both,
                        };
                    } else {
                        state = BotState::Executing(cbw);
                    }
                }
                BotState::Executing(cbw) => {
                    let outcome = match execute_command(
                        &mut storage,
                        &mut out_ep,
                        &mut in_ep,
                        &mut sense,
                        &mut prevent_medium_removal,
                        cbw,
                    )
                    .await
                    {
                        Ok(outcome) => outcome,
                        Err(EndpointError::Disabled) => break 'session,
                        Err(_) => break 'session,
                    };

                    state = BotState::SendingCsw {
                        csw: outcome.csw,
                        stall_after_csw: outcome.stall_after_csw,
                    };
                }
                BotState::SendingCsw { csw, stall_after_csw } => {
                    if send_csw(&mut in_ep, csw).await.is_err() {
                        break 'session;
                    }

                    request_stall_after_csw(bot_control, stall_after_csw);
                    state = BotState::WaitingForCbw;
                }
            }
        }
    }
}

fn request_stall_after_csw(bot_control: &BotControl, stall_after_csw: StallAfterCsw) {
    match stall_after_csw {
        StallAfterCsw::None => {}
        StallAfterCsw::In => bot_control.request_stall_in(),
        StallAfterCsw::Out => bot_control.request_stall_out(),
        StallAfterCsw::Both => {
            bot_control.request_stall_in();
            bot_control.request_stall_out();
        }
    }
}

async fn read_cbw<OUT>(out_ep: &mut OUT) -> Result<Option<Cbw>, EndpointError>
where
    OUT: EndpointOut,
{
    let mut packet = [0u8; USB_PACKET_SIZE];

    let read_result = with_timeout(Duration::from_millis(CBW_READ_TIMEOUT_MS), out_ep.read(&mut packet)).await;
    let packet_len = match read_result {
        Ok(Ok(length)) => length,
        Ok(Err(error)) => return Err(error),
        Err(_) => return Ok(None),
    };

    if packet_len == 0 {
        return Ok(None);
    }

    Ok(Some(Cbw::parse(&packet[..packet_len])))
}

async fn execute_command<OUT, IN, S>(
    storage: &mut S,
    out_ep: &mut OUT,
    in_ep: &mut IN,
    sense: &mut SenseData,
    prevent_medium_removal: &mut bool,
    cbw: Cbw,
) -> Result<CommandOutcome, EndpointError>
where
    OUT: EndpointOut,
    IN: EndpointIn,
    S: BlockStorage,
{
    let expected_length = cbw.data_transfer_length;
    let opcode = cbw.opcode();

    let mut transferred = 0u32;
    let mut status = CSW_STATUS_PASSED;
    let mut stall_after_csw = StallAfterCsw::None;

    match opcode {
        SCSI_TEST_UNIT_READY => {
            if has_phase_mismatch(cbw, DataDirection::None) {
                return Ok(phase_error(cbw));
            }

            // Storage backends should override BlockStorage::is_ready() if they require
            // deferred media init/probe before they can serve requests.
            if !storage.is_ready() {
                *sense = SenseData::not_ready_initializing();
                status = CSW_STATUS_FAILED;
            }
        }
        SCSI_INQUIRY => {
            if has_phase_mismatch(cbw, DataDirection::In) {
                return Ok(phase_error(cbw));
            }

            let evpd = cbw.command[1] & 0x01 != 0;
            let page_code = cbw.command[2];
            if evpd && page_code != 0x00 {
                *sense = SenseData::illegal_field_in_cdb(2);
                status = CSW_STATUS_FAILED;
            } else {
            let response = build_inquiry_response();
                let transfer_len = min(expected_length, cbw.command[4] as u32);
                transferred = send_in_data(in_ep, &response, transfer_len, cbw.expects_in()).await?;
            }
        }
        SCSI_REQUEST_SENSE => {
            if has_phase_mismatch(cbw, DataDirection::In) {
                return Ok(phase_error(cbw));
            }

            if cbw.command[1] & 0x01 != 0 {
                *sense = SenseData::illegal_field_in_cdb(1);
                status = CSW_STATUS_FAILED;
            } else {
            let response = sense.to_response();
                let transfer_len = min(expected_length, cbw.command[4] as u32);
                transferred = send_in_data(in_ep, &response, transfer_len, cbw.expects_in()).await?;
                *sense = SenseData::good();
            }
        }
        SCSI_READ_CAPACITY_10 => {
            if has_phase_mismatch(cbw, DataDirection::In) {
                return Ok(phase_error(cbw));
            }

            let response = build_read_capacity_10_response(storage.block_count());
            transferred = send_in_data(in_ep, &response, expected_length, cbw.expects_in()).await?;
        }
        SCSI_READ_FORMAT_CAPACITIES => {
            if has_phase_mismatch(cbw, DataDirection::In) {
                return Ok(phase_error(cbw));
            }

            let response = build_read_format_capacities_response(storage.block_count());
            let allocation_length = u16::from_be_bytes([cbw.command[7], cbw.command[8]]) as u32;
            transferred = send_in_data(
                in_ep,
                &response,
                min(expected_length, allocation_length),
                cbw.expects_in(),
            )
            .await?;
        }
        SCSI_MODE_SENSE_6 => {
            if has_phase_mismatch(cbw, DataDirection::In) {
                return Ok(phase_error(cbw));
            }

            let page_control = (cbw.command[2] >> 6) & 0x03;
            let page_code = cbw.command[2] & 0x3F;
            let subpage = cbw.command[3];

            if page_control != 0 || !mode_page_supported(page_code, subpage) {
                *sense = SenseData::illegal_field_in_cdb(2);
                status = CSW_STATUS_FAILED;
            } else {
                let response = build_mode_sense_6_response(storage.is_write_protected());
                let allocation_length = cbw.command[4] as u32;
                transferred = send_in_data(
                    in_ep,
                    &response,
                    min(expected_length, allocation_length),
                    cbw.expects_in(),
                )
                .await?;
            }
        }
        SCSI_MODE_SENSE_10 => {
            if has_phase_mismatch(cbw, DataDirection::In) {
                return Ok(phase_error(cbw));
            }

            let page_control = (cbw.command[2] >> 6) & 0x03;
            let page_code = cbw.command[2] & 0x3F;
            let subpage = cbw.command[3];

            if page_control != 0 || !mode_page_supported(page_code, subpage) {
                *sense = SenseData::illegal_field_in_cdb(2);
                status = CSW_STATUS_FAILED;
            } else {
                let response = build_mode_sense_10_response(storage.is_write_protected());
                let allocation_length = u16::from_be_bytes([cbw.command[7], cbw.command[8]]) as u32;
                transferred = send_in_data(
                    in_ep,
                    &response,
                    min(expected_length, allocation_length),
                    cbw.expects_in(),
                )
                .await?;
            }
        }
        SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL => {
            if has_phase_mismatch(cbw, DataDirection::None) {
                return Ok(phase_error(cbw));
            }

            *prevent_medium_removal = cbw.command[4] & 0x01 != 0;
        }
        SCSI_SYNCHRONIZE_CACHE_10 | SCSI_VERIFY_10 | SCSI_START_STOP_UNIT => {
            if has_phase_mismatch(cbw, DataDirection::None) {
                return Ok(phase_error(cbw));
            }
        }
        SCSI_READ_10 => {
            if has_phase_mismatch(cbw, DataDirection::In) {
                return Ok(phase_error(cbw));
            }

            if !storage.is_ready() {
                *sense = SenseData::not_ready_initializing();
                status = CSW_STATUS_FAILED;
            } else {
                let (lba, block_count) = parse_read_write_10(&cbw.command);
                if !block_range_valid(storage, lba, block_count) {
                    *sense = SenseData::lba_out_of_range(lba);
                    status = CSW_STATUS_FAILED;
                } else {
                    match read_blocks(storage, in_ep, lba, block_count, expected_length, cbw.expects_in()).await {
                        Ok(bytes) => transferred = bytes,
                        Err(TransferError::Endpoint(error)) => return Err(error),
                        Err(TransferError::Storage(error)) => {
                            *sense = SenseData::from_storage_error(error, false);
                            status = CSW_STATUS_FAILED;
                            stall_after_csw = StallAfterCsw::In;
                        }
                    }
                }
            }
        }
        SCSI_WRITE_10 => {
            if has_phase_mismatch(cbw, DataDirection::Out) {
                return Ok(phase_error(cbw));
            }

            if !storage.is_ready() {
                *sense = SenseData::not_ready_initializing();
                status = CSW_STATUS_FAILED;
            } else if storage.is_write_protected() {
                *sense = SenseData::from_storage_error(StorageError::WriteProtect, true);
                status = CSW_STATUS_FAILED;
            } else {
                let (lba, block_count) = parse_read_write_10(&cbw.command);
                if !block_range_valid(storage, lba, block_count) {
                    *sense = SenseData::lba_out_of_range(lba);
                    status = CSW_STATUS_FAILED;
                } else {
                    match write_blocks(storage, out_ep, lba, block_count, expected_length).await {
                        Ok(transfer) => {
                            transferred = transfer.written_bytes;

                            let requested_bytes = block_count as u32 * BLOCK_SIZE as u32;
                            let mismatch = transfer.short_packet
                                || transfer.packet_overflow
                                || transfer.written_bytes < requested_bytes
                                || transfer.consumed_bytes < expected_length
                                || expected_length != requested_bytes;

                            if mismatch {
                                *sense = SenseData::transfer_length_mismatch(
                                    expected_length.saturating_sub(transfer.written_bytes),
                                    7,
                                );
                                status = CSW_STATUS_FAILED;
                            }
                        }
                        Err(TransferError::Endpoint(error)) => return Err(error),
                        Err(TransferError::Storage(error)) => {
                            *sense = SenseData::from_storage_error(error, true);
                            status = CSW_STATUS_FAILED;
                        }
                    }
                }
            }
        }
        _ => {
            *sense = SenseData::illegal_request(ASC_INVALID_COMMAND_OPCODE);
            status = CSW_STATUS_FAILED;
        }
    }

    if status == CSW_STATUS_FAILED && expected_length > 0 && matches!(stall_after_csw, StallAfterCsw::None) {
        stall_after_csw = match cbw.data_direction() {
            DataDirection::None => StallAfterCsw::None,
            DataDirection::In => StallAfterCsw::In,
            DataDirection::Out => StallAfterCsw::Out,
        };
    }

    let residue = expected_length.saturating_sub(transferred);
    Ok(CommandOutcome {
        csw: Csw {
            tag: cbw.tag,
            residue,
            status,
        },
        stall_after_csw,
    })
}

fn phase_error(cbw: Cbw) -> CommandOutcome {
    CommandOutcome {
        csw: Csw {
            tag: cbw.tag,
            residue: cbw.data_transfer_length,
            status: CSW_STATUS_PHASE_ERROR,
        },
        stall_after_csw: StallAfterCsw::Both,
    }
}

fn has_phase_mismatch(cbw: Cbw, command_direction: DataDirection) -> bool {
    if cbw.data_transfer_length == 0 {
        return false;
    }

    match command_direction {
        DataDirection::None => true,
        DataDirection::In => !cbw.expects_in(),
        DataDirection::Out => cbw.expects_in(),
    }
}

async fn send_csw<IN>(in_ep: &mut IN, csw: Csw) -> Result<(), EndpointError>
where
    IN: EndpointIn,
{
    let response = csw.to_bytes();
    let mut offset = 0usize;

    while offset < response.len() {
        let end = min(offset + USB_PACKET_SIZE, response.len());
        in_ep.write(&response[offset..end]).await?;
        offset = end;
    }

    Ok(())
}

async fn send_in_data<IN>(
    in_ep: &mut IN,
    data: &[u8],
    expected_length: u32,
    expects_in: bool,
) -> Result<u32, EndpointError>
where
    IN: EndpointIn,
{
    let total_length = min(data.len(), expected_length as usize);
    let mut offset = 0usize;

    while offset < total_length {
        let end = min(offset + USB_PACKET_SIZE, total_length);
        in_ep.write(&data[offset..end]).await?;
        offset = end;
    }

    if expects_in && total_length > 0 && total_length < expected_length as usize && total_length % USB_PACKET_SIZE == 0 {
        in_ep.write(&[]).await?;
    }

    Ok(total_length as u32)
}

fn block_range_valid<S: BlockStorage>(storage: &S, lba: u32, block_count: u16) -> bool {
    if block_count == 0 {
        return true;
    }

    let total_blocks = storage.block_count();
    lba.checked_add(block_count as u32)
        .map_or(false, |end| end <= total_blocks)
}

async fn read_blocks<IN, S>(
    storage: &mut S,
    in_ep: &mut IN,
    lba: u32,
    block_count: u16,
    expected_length: u32,
    expects_in: bool,
) -> Result<u32, TransferError>
where
    IN: EndpointIn,
    S: BlockStorage,
{
    let total_payload = block_count as usize * BLOCK_SIZE;
    let total_to_send = min(total_payload, expected_length as usize);
    if total_to_send == 0 {
        return Ok(0);
    }

    let mut sent = 0usize;
    let mut remaining = total_to_send;
    let mut block_buffer = [0u8; BLOCK_SIZE];

    for block_offset in 0..block_count {
        if remaining == 0 {
            break;
        }

        storage
            .read_block(lba + block_offset as u32, &mut block_buffer)
            .await
            .map_err(TransferError::Storage)?;

        let chunk_len = min(remaining, BLOCK_SIZE);
        let mut chunk_offset = 0usize;
        while chunk_offset < chunk_len {
            let end = min(chunk_offset + USB_PACKET_SIZE, chunk_len);
            in_ep
                .write(&block_buffer[chunk_offset..end])
                .await
                .map_err(TransferError::Endpoint)?;
            chunk_offset = end;
        }

        sent += chunk_len;
        remaining -= chunk_len;
    }

    if expects_in && sent > 0 && sent < expected_length as usize && sent % USB_PACKET_SIZE == 0 {
        in_ep.write(&[]).await.map_err(TransferError::Endpoint)?;
    }

    Ok(sent as u32)
}

async fn write_blocks<OUT, S>(
    storage: &mut S,
    out_ep: &mut OUT,
    lba: u32,
    block_count: u16,
    expected_length: u32,
) -> Result<WriteTransfer, TransferError>
where
    OUT: EndpointOut,
    S: BlockStorage,
{
    if expected_length == 0 {
        return Ok(WriteTransfer {
            written_bytes: 0,
            consumed_bytes: 0,
            short_packet: false,
            packet_overflow: false,
        });
    }

    let mut consumed_bytes = 0u32;
    let mut written_bytes = 0u32;
    let mut current_block = 0u16;
    let mut short_packet = false;
    let mut packet_overflow = false;

    let mut packet = [0u8; USB_PACKET_SIZE];
    let mut block_buffer = [0u8; BLOCK_SIZE];
    let mut block_fill = 0usize;

    while consumed_bytes < expected_length {
        let packet_len = match with_timeout(Duration::from_millis(DATA_OUT_TIMEOUT_MS), out_ep.read(&mut packet)).await {
            Ok(Ok(length)) => length,
            Ok(Err(error)) => return Err(TransferError::Endpoint(error)),
            Err(_) => {
                short_packet = true;
                break;
            }
        };

        if packet_len == 0 {
            continue;
        }

        let remaining_expected = (expected_length - consumed_bytes) as usize;
        let usable_len = min(packet_len, remaining_expected);
        if packet_len > remaining_expected {
            packet_overflow = true;
        }

        consumed_bytes = consumed_bytes.saturating_add(usable_len as u32);

        let mut packet_offset = 0usize;
        while packet_offset < usable_len {
            let copy_len = min(BLOCK_SIZE - block_fill, usable_len - packet_offset);
            block_buffer[block_fill..block_fill + copy_len]
                .copy_from_slice(&packet[packet_offset..packet_offset + copy_len]);

            block_fill += copy_len;
            packet_offset += copy_len;

            if block_fill == BLOCK_SIZE {
                if current_block < block_count {
                    storage
                        .write_block(lba + current_block as u32, &block_buffer)
                        .await
                        .map_err(TransferError::Storage)?;
                    written_bytes = written_bytes.saturating_add(BLOCK_SIZE as u32);
                    current_block = current_block.saturating_add(1);
                }
                block_fill = 0;
            }
        }

        if packet_len < USB_PACKET_SIZE && consumed_bytes < expected_length {
            short_packet = true;
            break;
        }

        if packet_overflow {
            discard_overflow_packets(out_ep)
                .await
                .map_err(TransferError::Endpoint)?;
            break;
        }
    }

    Ok(WriteTransfer {
        written_bytes,
        consumed_bytes,
        short_packet,
        packet_overflow,
    })
}

async fn discard_overflow_packets<OUT>(out_ep: &mut OUT) -> Result<(), EndpointError>
where
    OUT: EndpointOut,
{
    let mut packet = [0u8; USB_PACKET_SIZE];
    let mut drained = 0usize;

    loop {
        if drained >= OVERFLOW_DRAIN_MAX_PACKETS {
            break;
        }

        let packet_len = match with_timeout(Duration::from_millis(OVERFLOW_DRAIN_TIMEOUT_MS), out_ep.read(&mut packet)).await {
            Ok(Ok(length)) => length,
            Ok(Err(error)) => return Err(error),
            Err(_) => break,
        };

        if packet_len == 0 {
            continue;
        }

        drained += 1;
        if packet_len < USB_PACKET_SIZE {
            break;
        }
    }

    Ok(())
}

fn parse_read_write_10(command: &[u8; 16]) -> (u32, u16) {
    let lba = u32::from_be_bytes([command[2], command[3], command[4], command[5]]);
    let block_count = u16::from_be_bytes([command[7], command[8]]);
    (lba, block_count)
}

fn build_inquiry_response() -> [u8; 36] {
    let mut response = [0u8; 36];
    response[0] = 0x00;
    response[1] = 0x80;
    response[2] = 0x06;
    response[3] = 0x02;
    response[4] = 31;
    response[8..16].copy_from_slice(b"FeRam   ");
    response[16..32].copy_from_slice(b"FeRamDisk       ");
    response[32..36].copy_from_slice(b"1.00");
    response
}

fn build_read_capacity_10_response(block_count: u32) -> [u8; 8] {
    let mut response = [0u8; 8];
    let last_block = block_count.saturating_sub(1);
    response[0..4].copy_from_slice(&last_block.to_be_bytes());
    response[4..8].copy_from_slice(&(BLOCK_SIZE as u32).to_be_bytes());
    response
}

fn build_read_format_capacities_response(block_count: u32) -> [u8; 12] {
    let mut response = [0u8; 12];
    response[0] = 0x00;
    response[1] = 0x00;
    response[2] = 0x00;
    response[3] = 8;
    response[4..8].copy_from_slice(&block_count.to_be_bytes());
    response[8] = 0x02;
    response[9..12].copy_from_slice(&[
        ((BLOCK_SIZE as u32 >> 16) & 0xFF) as u8,
        ((BLOCK_SIZE as u32 >> 8) & 0xFF) as u8,
        (BLOCK_SIZE as u32 & 0xFF) as u8,
    ]);
    response
}

fn mode_page_supported(page_code: u8, subpage: u8) -> bool {
    // MODE SENSE page code 0x3F means "return all supported pages".
    // This device currently supports only Caching Page (08h), so 0x3F returns
    // the same single page payload as an explicit 08h request.
    (page_code == MODE_PAGE_CACHING || page_code == 0x3F) && (subpage == 0x00 || subpage == 0xFF)
}

fn build_caching_page() -> [u8; 20] {
    let mut page = [0u8; 20];
    page[0] = MODE_PAGE_CACHING;
    page[1] = 0x12;

    let write_cache_enabled = false;
    let read_cache_disabled = true;
    page[2] = (if write_cache_enabled { 0x04 } else { 0x00 }) | if read_cache_disabled { 0x01 } else { 0x00 };

    page
}

fn build_mode_sense_6_response(write_protected: bool) -> [u8; 24] {
    let mut response = [0u8; 24];
    let caching_page = build_caching_page();

    response[0] = (response.len() - 1) as u8;
    response[1] = 0x00;
    response[2] = if write_protected { 0x80 } else { 0x00 };
    response[3] = 0x00;
    response[4..].copy_from_slice(&caching_page);

    response
}

fn build_mode_sense_10_response(write_protected: bool) -> [u8; 28] {
    let mut response = [0u8; 28];
    let caching_page = build_caching_page();

    let mode_data_length = (response.len() - 2) as u16;
    response[0..2].copy_from_slice(&mode_data_length.to_be_bytes());
    response[2] = 0x00;
    response[3] = if write_protected { 0x80 } else { 0x00 };
    response[4] = 0x00;
    response[5] = 0x00;
    response[6] = 0x00;
    response[7] = 0x00;
    response[8..].copy_from_slice(&caching_page);

    response
}
