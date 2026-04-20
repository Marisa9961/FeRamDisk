pub mod layout;
pub mod metadata;

pub use layout::{visible_block_count_from_physical, JOURNAL_RESERVED_BLOCKS};
pub use metadata::MetadataJournalStorage;
