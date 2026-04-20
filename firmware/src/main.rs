#![no_std]
#![no_main]

mod app;
mod drivers;
mod init;
mod storage;
mod usb;

use panic_halt as _;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    app::run().await;
}
