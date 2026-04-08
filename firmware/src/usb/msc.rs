#![allow(dead_code)]

use core::cmp::min;

use crate::feram::{FeRam, BLOCK_SIZE, TOTAL_BLOCKS};
use crate::spi::FramSpi;
use embassy_usb_driver::{EndpointError, EndpointIn, EndpointOut};
use embedded_hal::digital::OutputPin;

const CBW_SIGNATURE: u32 = 0x4342_5355;
const CSW_SIGNATURE: u32 = 0x5342_5355;

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

const SENSE_ILLEGAL_REQUEST: u8 = 0x05;

const ASC_INVALID_COMMAND_OPCODE: u8 = 0x20;
const ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE: u8 = 0x21;

const LUN_COUNT: u8 = 1;
const USB_PACKET_SIZE: usize = 64;

#[derive(Copy, Clone, Debug, Default)]
struct SenseData {
    key: u8,
    asc: u8,
    ascq: u8,
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
        }
    }

    fn to_response(self) -> [u8; 18] {
        let mut response = [0u8; 18];
        response[0] = 0x70;
        response[2] = self.key;
        response[7] = 10;
        response[12] = self.asc;
        response[13] = self.ascq;
        response
    }
}

#[derive(Debug, Clone, Copy)]
struct Cbw {
    tag: u32,
    data_transfer_length: u32,
    flags: u8,
    lun: u8,
    command_length: u8,
    command: [u8; 16],
    valid: bool,
}

impl Cbw {
    fn parse(bytes: [u8; 31]) -> Self {
        let mut command = [0u8; 16];
        command.copy_from_slice(&bytes[15..31]);

        Self {
            valid: u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) == CBW_SIGNATURE,
            tag: u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
            data_transfer_length: u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
            flags: bytes[12],
            lun: bytes[13],
            command_length: bytes[14],
            command,
        }
    }

    fn opcode(&self) -> u8 {
        self.command[0]
    }

    fn expects_in(&self) -> bool {
        self.flags & 0x80 != 0
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

pub trait BlockStorage {
    fn block_count(&self) -> u32;
    async fn read_block(&mut self, block_index: u32, out: &mut [u8; BLOCK_SIZE]) -> Result<(), ()>;
    async fn write_block(&mut self, block_index: u32, data: &[u8; BLOCK_SIZE]) -> Result<(), ()>;
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

    async fn read_block(&mut self, block_index: u32, out: &mut [u8; BLOCK_SIZE]) -> Result<(), ()> {
        self.read_block(block_index, out).await.map_err(|_| ())
    }

    async fn write_block(&mut self, block_index: u32, data: &[u8; BLOCK_SIZE]) -> Result<(), ()> {
        self.write_block(block_index, data).await.map_err(|_| ())
    }
}

pub async fn run<OUT, IN, S>(mut out_ep: OUT, mut in_ep: IN, mut storage: S)
where
    OUT: EndpointOut,
    IN: EndpointIn,
    S: BlockStorage,
{
    loop {
        in_ep.wait_enabled().await;
        out_ep.wait_enabled().await;

        let mut sense = SenseData::good();

        loop {
            let cbw = match read_cbw(&mut out_ep).await {
                Ok(cbw) => cbw,
                Err(EndpointError::Disabled) => break,
                Err(_) => break,
            };

            let csw = match execute_command(&mut storage, &mut out_ep, &mut in_ep, &mut sense, cbw).await {
                Ok(csw) => csw,
                Err(EndpointError::Disabled) => break,
                Err(_) => break,
            };

            if send_csw(&mut in_ep, csw).await.is_err() {
                break;
            }
        }
    }
}

async fn read_cbw<OUT>(out_ep: &mut OUT) -> Result<Cbw, EndpointError>
where
    OUT: EndpointOut,
{
    let mut buffer = [0u8; 31];
    let mut offset = 0usize;

    while offset < buffer.len() {
        let length = out_ep.read(&mut buffer[offset..]).await?;
        if length == 0 {
            continue;
        }
        offset += length;
    }

    Ok(Cbw::parse(buffer))
}

async fn execute_command<OUT, IN, S>(
    storage: &mut S,
    out_ep: &mut OUT,
    in_ep: &mut IN,
    sense: &mut SenseData,
    cbw: Cbw,
) -> Result<Csw, EndpointError>
where
    OUT: EndpointOut,
    IN: EndpointIn,
    S: BlockStorage,
{
    let expected_length = cbw.data_transfer_length;

    if !cbw.valid || cbw.lun >= LUN_COUNT || cbw.command_length == 0 || cbw.command_length > 16 {
        *sense = SenseData::illegal_request(ASC_INVALID_COMMAND_OPCODE);
        return Ok(Csw {
            tag: cbw.tag,
            residue: expected_length,
            status: 2,
        });
    }

    let opcode = cbw.opcode();
    let mut transferred = 0u32;
    let mut status = 0u8;

    match opcode {
        SCSI_TEST_UNIT_READY => {}
        SCSI_INQUIRY => {
            let response = build_inquiry_response();
            transferred = send_in_data(in_ep, &response, expected_length).await?;
        }
        SCSI_REQUEST_SENSE => {
            let response = sense.to_response();
            *sense = SenseData::good();
            transferred = send_in_data(in_ep, &response, expected_length).await?;
        }
        SCSI_READ_CAPACITY_10 => {
            let response = build_read_capacity_10_response(storage.block_count());
            transferred = send_in_data(in_ep, &response, expected_length).await?;
        }
        SCSI_READ_FORMAT_CAPACITIES => {
            let response = build_read_format_capacities_response(storage.block_count());
            transferred = send_in_data(in_ep, &response, expected_length).await?;
        }
        SCSI_MODE_SENSE_6 => {
            let response = build_mode_sense_6_response();
            transferred = send_in_data(in_ep, &response, expected_length).await?;
        }
        SCSI_MODE_SENSE_10 => {
            let response = build_mode_sense_10_response();
            transferred = send_in_data(in_ep, &response, expected_length).await?;
        }
        SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL | SCSI_SYNCHRONIZE_CACHE_10 | SCSI_VERIFY_10 | SCSI_START_STOP_UNIT => {}
        SCSI_READ_10 => {
            let (lba, block_count) = parse_read_write_10(&cbw.command)?;
            if !block_range_valid(storage, lba, block_count) {
                *sense = SenseData::illegal_request(ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE);
                status = 1;
            } else {
                transferred = read_blocks(storage, in_ep, lba, block_count, expected_length).await?;
            }
        }
        SCSI_WRITE_10 => {
            let (lba, block_count) = parse_read_write_10(&cbw.command)?;
            if !block_range_valid(storage, lba, block_count) {
                *sense = SenseData::illegal_request(ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE);
                status = 1;
            } else {
                transferred = write_blocks(storage, out_ep, lba, block_count, expected_length).await?;
            }
        }
        _ => {
            *sense = SenseData::illegal_request(ASC_INVALID_COMMAND_OPCODE);
            status = 1;
        }
    }

    let residue = expected_length.saturating_sub(transferred);
    Ok(Csw {
        tag: cbw.tag,
        residue,
        status,
    })
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

async fn send_in_data<IN>(in_ep: &mut IN, data: &[u8], expected_length: u32) -> Result<u32, EndpointError>
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
) -> Result<u32, EndpointError>
where
    IN: EndpointIn,
    S: BlockStorage,
{
    if block_count == 0 {
        return Ok(0);
    }

    let mut total_transferred = 0u32;
    let mut block_buffer = [0u8; BLOCK_SIZE];

    for block_offset in 0..block_count {
        storage.read_block(lba + block_offset as u32, &mut block_buffer).await.map_err(|_| EndpointError::Disabled)?;
        let remaining = expected_length.saturating_sub(total_transferred);
        if remaining == 0 {
            break;
        }
        let sent = send_in_data(in_ep, &block_buffer, remaining).await?;
        total_transferred = total_transferred.saturating_add(sent);
    }

    Ok(total_transferred)
}

async fn write_blocks<OUT, S>(
    storage: &mut S,
    out_ep: &mut OUT,
    lba: u32,
    block_count: u16,
    expected_length: u32,
) -> Result<u32, EndpointError>
where
    OUT: EndpointOut,
    S: BlockStorage,
{
    if block_count == 0 {
        return Ok(0);
    }

    let mut total_transferred = 0u32;
    let mut block_buffer = [0u8; BLOCK_SIZE];

    for block_offset in 0..block_count {
        let remaining = expected_length.saturating_sub(total_transferred);
        if remaining == 0 {
            break;
        }

        read_exact_into(out_ep, &mut block_buffer).await?;
        storage.write_block(lba + block_offset as u32, &block_buffer).await.map_err(|_| EndpointError::Disabled)?;
        total_transferred = total_transferred.saturating_add(BLOCK_SIZE as u32);
    }

    Ok(total_transferred)
}

async fn read_exact_into<OUT>(out_ep: &mut OUT, buffer: &mut [u8; BLOCK_SIZE]) -> Result<(), EndpointError>
where
    OUT: EndpointOut,
{
    let mut offset = 0usize;

    while offset < buffer.len() {
        let mut packet = [0u8; USB_PACKET_SIZE];
        let length = out_ep.read(&mut packet).await?;
        if length == 0 {
            continue;
        }

        let remaining = buffer.len() - offset;
        let copy_length = min(length, remaining);
        buffer[offset..offset + copy_length].copy_from_slice(&packet[..copy_length]);
        offset += copy_length;
    }

    Ok(())
}

fn parse_read_write_10(command: &[u8; 16]) -> Result<(u32, u16), EndpointError> {
    let lba = u32::from_be_bytes([command[2], command[3], command[4], command[5]]);
    let block_count = u16::from_be_bytes([command[7], command[8]]);
    Ok((lba, block_count))
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
    response[3] = 8;
    response[4..8].copy_from_slice(&block_count.to_be_bytes());
    response[8] = 0x02;
    response[9..12].copy_from_slice(&[(BLOCK_SIZE >> 16) as u8, (BLOCK_SIZE >> 8) as u8, BLOCK_SIZE as u8]);
    response
}

fn build_mode_sense_6_response() -> [u8; 4] {
    [3, 0, 0, 0]
}

fn build_mode_sense_10_response() -> [u8; 8] {
    [0, 6, 0, 0, 0, 0, 0, 0]
}
