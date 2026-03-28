use crate::storage::{BlockDevice, BLOCK_SIZE};

pub struct MscStack {
    _max_packet_size: u16,
}

impl MscStack {
    pub const fn new() -> Self {
        Self {
            _max_packet_size: 64,
        }
    }

    pub fn poll<D: BlockDevice>(&mut self, storage: &mut D) {
        // Placeholder for USB MSC BOT state machine.
        let _ = BLOCK_SIZE;
        let _ = storage.block_count();
    }
}
