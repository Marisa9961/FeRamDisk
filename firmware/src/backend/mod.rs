#[cfg(feature = "hardware")]
pub mod hardware;
#[cfg(not(feature = "hardware"))]
pub mod simulated;

#[cfg(feature = "hardware")]
pub use hardware::{init_hardware_parts, init_storage, init_usb_driver, HardwareStorage};
#[cfg(not(feature = "hardware"))]
pub use simulated::{init_storage, RamStorage as Storage};
