use crate::backend::{init_hardware_parts, init_storage, init_usb_driver};
use crate::usb;
use embassy_stm32::Peripherals;

pub async fn run(peripherals: Peripherals) {
    let mut hardware = init_hardware_parts(peripherals);
    let storage = init_storage(&mut hardware).await;
    let usb_driver = init_usb_driver(&mut hardware);
    usb::device::run(usb_driver, storage).await;
}