#![no_std]
#![no_main]

use embassy_executor::Spawner;
use panic_halt as _;

mod app;
mod storage;
mod usb_msc;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    app::run().await;
}
