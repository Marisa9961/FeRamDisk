use embassy_stm32::rcc::mux::Clk48sel;
use embassy_stm32::rcc::{Hsi48Config, Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk};
use embassy_stm32::{Config, Peripherals};

pub fn init() -> Peripherals {
    let mut stm32_cfg = Config::default();
    stm32_cfg.rcc.pll = Some(Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL85,
        divp: None,
        divq: None,
        divr: Some(PllRDiv::DIV2),
    });
    stm32_cfg.rcc.sys = Sysclk::PLL1_R;
    stm32_cfg.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true });
    stm32_cfg.rcc.mux.clk48sel = Clk48sel::HSI48;

    embassy_stm32::init(stm32_cfg)
}