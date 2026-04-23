#![cfg_attr(feature = "hardware", no_std)]

#[cfg(feature = "hardware")]
pub mod app;
#[cfg(feature = "hardware")]
pub mod drivers;
#[cfg(feature = "hardware")]
pub mod init;

pub mod backend;
pub mod storage;
pub mod usb;
