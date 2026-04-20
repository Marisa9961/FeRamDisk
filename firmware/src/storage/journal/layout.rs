#![allow(dead_code)]

pub const JOURNAL_RESERVED_BLOCKS: u32 = 2;

pub(super) const JOURNAL_STATE_CLEAN: u8 = 0x00;
pub(super) const JOURNAL_STATE_COMMITTED: u8 = 0xA5;
pub(super) const JOURNAL_MAGIC: [u8; 3] = *b"JNL";
pub(super) const JOURNAL_HEADER_STATE_OFFSET: usize = 0;
pub(super) const JOURNAL_HEADER_MAGIC_OFFSET: usize = 1;
pub(super) const JOURNAL_HEADER_TARGET_LBA_OFFSET: usize = 4;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(super) struct LbaRange {
    pub(super) start: u32,
    pub(super) end_exclusive: u32,
}

impl LbaRange {
    pub(super) const fn empty() -> Self {
        Self {
            start: 0,
            end_exclusive: 0,
        }
    }

    pub(super) fn contains(&self, lba: u32) -> bool {
        lba >= self.start && lba < self.end_exclusive
    }
}

pub const fn visible_block_count_from_physical(physical_blocks: u32) -> u32 {
    physical_blocks.saturating_sub(JOURNAL_RESERVED_BLOCKS)
}
