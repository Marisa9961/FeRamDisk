#![no_std]
#![no_main]

use feramdisk_firmware::{app, init};
use panic_halt as _;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let peripherals = init::init_peripherals();
    app::run(peripherals).await;
}
