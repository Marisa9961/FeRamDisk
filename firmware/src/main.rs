#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

mod app;
mod storage;
mod usb_msc;

#[entry]
fn main() -> ! {
    app::run()
}
