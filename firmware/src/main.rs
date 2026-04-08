#![no_std]
#![no_main]

mod app;
mod feram;
mod init;
mod spi;
mod usb;

use panic_halt as _;

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    app::run().await;
}
