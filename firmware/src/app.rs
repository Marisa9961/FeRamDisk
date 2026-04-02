use crate::storage::Storage;
use crate::usb_msc::MscStack;
use stm32g4xx_hal as hal;
use rtt_target::{rprintln, rtt_init_print};

use hal::prelude::*;
use hal::pwr::PwrExt;
use hal::rcc::Config;
use hal::stm32;
use hal::time::ExtU32;

pub fn run() -> ! {
    rtt_init_print!();
    rprintln!("System starting...");

    let dp = stm32::Peripherals::take().expect("cannot take device peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = dp.RCC.freeze(Config::hsi(), pwr);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let mut led0 = gpiob.pb0.into_push_pull_output();
    let mut led1 = gpiob.pb1.into_push_pull_output();
    let mut delay = cp.SYST.delay(&rcc.clocks);

    let mut storage = Storage::new();
    let mut msc = MscStack::new();
    let mut led_on = false;

    rprintln!("Hardware initialized. Entering main loop.");

    loop {
        storage.poll();
        msc.poll(&mut storage);

        led_on = !led_on;
        if led_on {
            let _ = led0.set_high();
            let _ = led1.set_low();
        } else {
            let _ = led0.set_low();
            let _ = led1.set_high();
        }

        rprintln!("Heartbeat - LED ON: {}", led_on);
        delay.delay(1000.millis());
    }
}
