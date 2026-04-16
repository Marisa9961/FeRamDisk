#![allow(dead_code)]

use core::cmp::min;

use crate::usb::constants::{
    BLOCK_SIZE,
    ASC_INVALID_COMMAND_OPCODE, ASC_INVALID_FIELD_IN_CDB, ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE,
    ASC_LOGICAL_UNIT_NOT_READY, ASC_INTERNAL_TARGET_FAILURE, ASCQ_INITIALIZING_COMMAND_REQUIRED,
    ASC_UNRECOVERED_READ_ERROR, ASC_WRITE_ERROR, ASC_WRITE_PROTECTED,
    CSW_STATUS_FAILED, CSW_STATUS_PASSED, CSW_STATUS_PHASE_ERROR,
    DATA_OUT_TIMEOUT_MS, OVERFLOW_DRAIN_MAX_PACKETS, OVERFLOW_DRAIN_TIMEOUT_MS,
    SENSE_ADDITIONAL_LENGTH, SENSE_FIXED_RESPONSE_LEN,
    SCSI_INQUIRY, SCSI_MODE_SENSE_10, SCSI_MODE_SENSE_6, SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL,
    SCSI_READ_10, SCSI_READ_CAPACITY_10, SCSI_READ_FORMAT_CAPACITIES, SCSI_REQUEST_SENSE,
    SCSI_START_STOP_UNIT, SCSI_SYNCHRONIZE_CACHE_10, SCSI_TEST_UNIT_READY, SCSI_VERIFY_10,
    SCSI_WRITE_10, SENSE_DATA_PROTECT, SENSE_HARDWARE_ERROR, SENSE_ILLEGAL_REQUEST,
    SENSE_MEDIUM_ERROR, SENSE_NOT_READY, USB_PACKET_SIZE,
};
use crate::usb::core::{send_in_data, Cbw, Csw, DataDirection};
use crate::usb::scsi::{
    build_inquiry_response, build_mode_sense_10_response, build_mode_sense_6_response,
    build_read_capacity_10_response, build_read_format_capacities_response, mode_page_supported,
};
use crate::usb::storage::{BlockStorage, StorageError};
use embassy_time::{with_timeout, Duration};
use embassy_usb_driver::{EndpointError, EndpointIn, EndpointOut};

/// Fixed-format request-sense working state for the current command stream.
#[derive(Copy, Clone, Debug, Default)]
pub(crate) struct SenseData {
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
    pub(crate) fn good() -> Self {
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

    pub(crate) fn to_response(self) -> [u8; SENSE_FIXED_RESPONSE_LEN] {
        let mut response = [0u8; SENSE_FIXED_RESPONSE_LEN];
        response[0] = 0x70 | if self.valid { 0x80 } else { 0x00 };
        response[2] = (self.key & 0x0F) | if self.ili { 0x20 } else { 0x00 };
        response[3..7].copy_from_slice(&self.information.to_be_bytes());
        response[7] = SENSE_ADDITIONAL_LENGTH;
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

#[derive(Copy, Clone)]
pub(crate) enum StallAfterCsw {
    None,
    In,
    Out,
    Both,
}

pub(crate) struct CommandOutcome {
    pub(crate) csw: Csw,
    pub(crate) stall_after_csw: StallAfterCsw,
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

pub(crate) async fn execute_command<OUT, IN, S>(
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
