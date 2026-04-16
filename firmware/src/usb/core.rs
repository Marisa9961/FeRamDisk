#![allow(dead_code)]

use core::cmp::min;
use core::sync::atomic::{AtomicU8, Ordering};

use crate::usb::commands::{execute_command, SenseData, StallAfterCsw};
use crate::usb::constants::{
    BOT_ACTION_STALL_IN, BOT_ACTION_STALL_OUT, BOT_EVENT_BULK_RESET,
    CBW_READ_TIMEOUT_MS, CBW_SIGNATURE, CSW_SIGNATURE, CSW_STATUS_PHASE_ERROR,
    LUN_COUNT, USB_PACKET_SIZE,
};
use crate::usb::storage::BlockStorage;
use embassy_time::{with_timeout, Duration};
use embassy_usb_driver::{EndpointError, EndpointIn, EndpointOut};

/// Bulk-Only Transport control structure shared between MSC data path and control path.
pub struct BotControl {
    bus_actions: AtomicU8,
    msc_events: AtomicU8,
}

impl BotControl {
    /// Create a new BOT shared control state.
    pub const fn new() -> Self {
        Self {
            bus_actions: AtomicU8::new(0),
            msc_events: AtomicU8::new(0),
        }
    }

    /// Request that the control task stall the bulk-IN endpoint.
    pub fn request_stall_in(&self) {
        self.bus_actions.fetch_or(BOT_ACTION_STALL_IN, Ordering::Release);
    }

    /// Request that the control task stall the bulk-OUT endpoint.
    pub fn request_stall_out(&self) {
        self.bus_actions.fetch_or(BOT_ACTION_STALL_OUT, Ordering::Release);
    }

    /// Signal a BOT reset event to the MSC state machine.
    pub fn signal_bulk_reset(&self) {
        self.msc_events.fetch_or(BOT_EVENT_BULK_RESET, Ordering::Release);
    }

    /// Consume and clear pending endpoint actions for the control task.
    pub fn take_bus_actions(&self) -> u8 {
        self.bus_actions.swap(0, Ordering::Acquire)
    }

    /// Consume and clear pending BOT events for the MSC task.
    pub fn take_msc_events(&self) -> u8 {
        self.msc_events.swap(0, Ordering::Acquire)
    }
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub(crate) enum DataDirection {
    None,
    In,
    Out,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Cbw {
    pub(crate) packet_len: usize,
    pub(crate) signature_valid: bool,
    pub(crate) tag: u32,
    pub(crate) data_transfer_length: u32,
    pub(crate) flags: u8,
    pub(crate) lun: u8,
    pub(crate) command_length: u8,
    pub(crate) command: [u8; 16],
}

impl Cbw {
    pub(crate) fn parse(packet: &[u8]) -> Self {
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

    pub(crate) fn is_valid(&self) -> bool {
        self.packet_len == 31
            && self.signature_valid
            && (self.flags & 0x7F) == 0
            && (self.lun & 0xF0) == 0
            && self.lun < LUN_COUNT
            && (1..=16).contains(&self.command_length)
    }

    pub(crate) fn opcode(&self) -> u8 {
        self.command[0]
    }

    pub(crate) fn expects_in(&self) -> bool {
        self.flags & 0x80 != 0
    }

    pub(crate) fn data_direction(&self) -> DataDirection {
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
pub(crate) struct Csw {
    pub(crate) tag: u32,
    pub(crate) residue: u32,
    pub(crate) status: u8,
}

impl Csw {
    pub(crate) fn to_bytes(self) -> [u8; 13] {
        let mut response = [0u8; 13];
        response[0..4].copy_from_slice(&CSW_SIGNATURE.to_le_bytes());
        response[4..8].copy_from_slice(&self.tag.to_le_bytes());
        response[8..12].copy_from_slice(&self.residue.to_le_bytes());
        response[12] = self.status;
        response
    }
}

#[derive(Copy, Clone)]
enum BotState {
    WaitingForCbw,
    Executing(Cbw),
    SendingCsw { csw: Csw, stall_after_csw: StallAfterCsw },
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

pub(crate) async fn send_in_data<IN>(
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
