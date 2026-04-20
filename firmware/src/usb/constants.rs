#![allow(dead_code)]

pub use crate::drivers::feram::BLOCK_SIZE;

pub(crate) const CBW_SIGNATURE: u32 = 0x4342_5355;
pub(crate) const CSW_SIGNATURE: u32 = 0x5342_5355;

pub(crate) const CSW_STATUS_PASSED: u8 = 0;
pub(crate) const CSW_STATUS_FAILED: u8 = 1;
pub(crate) const CSW_STATUS_PHASE_ERROR: u8 = 2;

pub(crate) const SCSI_TEST_UNIT_READY: u8 = 0x00;
pub(crate) const SCSI_REQUEST_SENSE: u8 = 0x03;
pub(crate) const SCSI_INQUIRY: u8 = 0x12;
pub(crate) const SCSI_MODE_SENSE_6: u8 = 0x1A;
pub(crate) const SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL: u8 = 0x1E;
pub(crate) const SCSI_READ_FORMAT_CAPACITIES: u8 = 0x23;
pub(crate) const SCSI_READ_CAPACITY_10: u8 = 0x25;
pub(crate) const SCSI_READ_10: u8 = 0x28;
pub(crate) const SCSI_WRITE_10: u8 = 0x2A;
pub(crate) const SCSI_VERIFY_10: u8 = 0x2F;
pub(crate) const SCSI_SYNCHRONIZE_CACHE_10: u8 = 0x35;
pub(crate) const SCSI_MODE_SENSE_10: u8 = 0x5A;
pub(crate) const SCSI_START_STOP_UNIT: u8 = 0x1B;

pub(crate) const SENSE_NOT_READY: u8 = 0x02;
pub(crate) const SENSE_MEDIUM_ERROR: u8 = 0x03;
pub(crate) const SENSE_HARDWARE_ERROR: u8 = 0x04;
pub(crate) const SENSE_ILLEGAL_REQUEST: u8 = 0x05;
pub(crate) const SENSE_DATA_PROTECT: u8 = 0x07;

pub(crate) const SENSE_FIXED_RESPONSE_LEN: usize = 18;
pub(crate) const SENSE_ADDITIONAL_LENGTH: u8 = 10;

pub(crate) const ASC_INVALID_COMMAND_OPCODE: u8 = 0x20;
pub(crate) const ASC_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE: u8 = 0x21;
pub(crate) const ASC_INVALID_FIELD_IN_CDB: u8 = 0x24;
pub(crate) const ASC_WRITE_PROTECTED: u8 = 0x27;
pub(crate) const ASC_LOGICAL_UNIT_NOT_READY: u8 = 0x04;
pub(crate) const ASCQ_INITIALIZING_COMMAND_REQUIRED: u8 = 0x02;
pub(crate) const ASC_UNRECOVERED_READ_ERROR: u8 = 0x11;
pub(crate) const ASC_WRITE_ERROR: u8 = 0x0C;
pub(crate) const ASC_INTERNAL_TARGET_FAILURE: u8 = 0x44;

pub(crate) const MODE_PAGE_CACHING: u8 = 0x08;

pub(crate) const CBW_READ_TIMEOUT_MS: u64 = 1500;
pub(crate) const DATA_OUT_TIMEOUT_MS: u64 = 1500;
pub(crate) const OVERFLOW_DRAIN_TIMEOUT_MS: u64 = 20;
pub(crate) const OVERFLOW_DRAIN_MAX_PACKETS: usize = 128;

pub(crate) const LUN_COUNT: u8 = 1;
pub(crate) const USB_PACKET_SIZE: usize = 64;

pub(crate) const BOT_ACTION_STALL_IN: u8 = 1 << 0;
pub(crate) const BOT_ACTION_STALL_OUT: u8 = 1 << 1;
pub(crate) const BOT_EVENT_BULK_RESET: u8 = 1 << 0;
