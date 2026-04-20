#![allow(dead_code)]

/// Errors surfaced by the logical block backend and mapped to SCSI sense data.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum StorageError {
    NotReady,
    MediumError,
    WriteProtect,
    HardwareError,
}
