#![allow(dead_code)]

use embassy_stm32::mode::Async;
use embassy_stm32::spi::mode::Master;
use embassy_stm32::spi::Spi;
use embedded_hal::digital::OutputPin;

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum FramSpiError<SpiError, CsError> {
    InvalidChip,
    Spi(SpiError),
    Cs(CsError),
}

pub struct FramSpi<'d, CS0, CS1, CS2, CS3> {
    pub(crate) spi: Spi<'d, Async, Master>,
    pub(crate) cs0: CS0,
    pub(crate) cs1: CS1,
    pub(crate) cs2: CS2,
    pub(crate) cs3: CS3,
}

impl<'d, CS0, CS1, CS2, CS3> FramSpi<'d, CS0, CS1, CS2, CS3>
where
    CS0: OutputPin,
    CS1: OutputPin<Error = CS0::Error>,
    CS2: OutputPin<Error = CS0::Error>,
    CS3: OutputPin<Error = CS0::Error>,
{
    pub fn new(spi: Spi<'d, Async, Master>, cs0: CS0, cs1: CS1, cs2: CS2, cs3: CS3) -> Self {
        Self {
            spi,
            cs0,
            cs1,
            cs2,
            cs3,
        }
    }

    pub(crate) fn select_chip(
        &mut self,
        chip: usize,
    ) -> Result<(), FramSpiError<embassy_stm32::spi::Error, CS0::Error>> {
        match chip {
            0 => self.cs0.set_low().map_err(FramSpiError::Cs),
            1 => self.cs1.set_low().map_err(FramSpiError::Cs),
            2 => self.cs2.set_low().map_err(FramSpiError::Cs),
            3 => self.cs3.set_low().map_err(FramSpiError::Cs),
            _ => Err(FramSpiError::InvalidChip),
        }
    }

    pub(crate) fn deselect_chip(
        &mut self,
        chip: usize,
    ) -> Result<(), FramSpiError<embassy_stm32::spi::Error, CS0::Error>> {
        match chip {
            0 => self.cs0.set_high().map_err(FramSpiError::Cs),
            1 => self.cs1.set_high().map_err(FramSpiError::Cs),
            2 => self.cs2.set_high().map_err(FramSpiError::Cs),
            3 => self.cs3.set_high().map_err(FramSpiError::Cs),
            _ => Err(FramSpiError::InvalidChip),
        }
    }
}
