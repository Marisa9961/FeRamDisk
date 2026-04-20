use embassy_stm32::bind_interrupts;
use embassy_stm32::dma;
use embassy_stm32::peripherals;
use embassy_stm32::rcc::mux::Clk48sel;
use embassy_stm32::rcc::{Hsi48Config, Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk};
use embassy_stm32::usb as stm32_usb;
use embassy_stm32::{Config, Peripherals};
use rtt_target::rtt_init_print;

bind_interrupts!(pub(crate) struct Irqs {
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    USB_LP => stm32_usb::InterruptHandler<peripherals::USB>;
});

pub fn init_logging() {
    rtt_init_print!();
}

pub fn init_peripherals() -> Peripherals {
    let mut config = Config::default();
    config.rcc.pll = Some(Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL85,
        divp: None,
        divq: None,
        divr: Some(PllRDiv::DIV2),
    });
    config.rcc.sys = Sysclk::PLL1_R;
    config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true });
    config.rcc.mux.clk48sel = Clk48sel::HSI48;

    embassy_stm32::init(config)
}