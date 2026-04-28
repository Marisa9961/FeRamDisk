use crate::backend::{init_hardware_parts, init_storage, init_usb_driver};
use crate::usb;
use embassy_stm32::Peripherals;
use rtt_target::rprintln;

pub async fn run(peripherals: Peripherals) {
    let mut hardware = init_hardware_parts(peripherals);
    let storage = match init_storage(&mut hardware).await {
        Ok(storage) => storage,
        Err(e) => {
            rprintln!("Hardware storage init failed: {:?}", e);
            return;
        }
    };

    let usb_driver = match init_usb_driver(&mut hardware) {
        Ok(driver) => driver,
        Err(e) => {
            rprintln!("USB driver init failed: {:?}", e);
            return;
        }
    };

    usb::device::run(usb_driver, storage).await;
}