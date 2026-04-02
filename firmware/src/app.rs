use crate::storage::Storage;
use crate::usb_msc::MscStack;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::Peripherals;
use embassy_time::{Duration, Timer};
use rtt_target::{rprintln, rtt_init_print};

pub async fn run() {
    rtt_init_print!();
    rprintln!("Embassy app start");

    let p: Peripherals = embassy_stm32::init(Default::default());
    let mut led0 = Output::new(p.PB0, Level::Low, Speed::Low);
    let mut led1 = Output::new(p.PB1, Level::High, Speed::Low);

    let mut storage = Storage::new();
    let mut msc = MscStack::new();
    let mut led_on = false;

    rprintln!("Hardware initialized (embassy). Entering main loop.");

    loop {
        storage.poll();
        msc.poll(&mut storage);

        led_on = !led_on;
        if led_on {
            led0.set_high();
            led1.set_low();
        } else {
            led0.set_low();
            led1.set_high();
        }

        rprintln!("Heartbeat - LED ON: {}", led_on);
        Timer::after(Duration::from_millis(1000)).await;
    }
}
