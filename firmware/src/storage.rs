pub const BLOCK_SIZE: usize = 512;

pub trait BlockDevice {
    fn block_count(&self) -> u32;
}

pub struct Storage;

impl Storage {
    pub const fn new() -> Self {
        Self
    }

    pub fn poll(&mut self) {
        // Placeholder for SPI/FeRAM state machine.
    }
}

impl BlockDevice for Storage {
    fn block_count(&self) -> u32 {
        // 1 MiB / 512-byte blocks.
        2048
    }
}
