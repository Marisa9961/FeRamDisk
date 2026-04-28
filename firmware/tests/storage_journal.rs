#![cfg(not(feature = "hardware"))]

mod common;

use common::{run_async, test_lock};
use feramdisk_firmware::backend::simulated::{BackendOp, RamBackend, SharedRamBackend, StorageError, BLOCK_SIZE};
use feramdisk_firmware::storage::{BlockStorage, MetadataJournalStorage, JOURNAL_RESERVED_BLOCKS};

const JOURNAL_STATE_CLEAN: u8 = 0x00;
const JOURNAL_STATE_COMMITTED: u8 = 0xA5;
const JOURNAL_MAGIC: [u8; 3] = *b"JNL";

fn logical_to_physical(lba: u32) -> u32 {
    JOURNAL_RESERVED_BLOCKS + lba
}

fn build_mbr(partition_start: u32, partition_blocks: u32) -> [u8; BLOCK_SIZE] {
    let mut mbr = [0u8; BLOCK_SIZE];
    let entry = &mut mbr[446..462];
    entry[4] = 0x01;
    entry[8..12].copy_from_slice(&partition_start.to_le_bytes());
    entry[12..16].copy_from_slice(&partition_blocks.to_le_bytes());
    mbr[510] = 0x55;
    mbr[511] = 0xAA;
    mbr
}

fn build_boot_sector(
    partition_blocks: u32,
    bytes_per_sector: u16,
    reserved_sectors: u16,
    fat_count: u8,
    root_entries: u16,
    fat_sectors: u16,
) -> [u8; BLOCK_SIZE] {
    let mut boot = [0u8; BLOCK_SIZE];
    boot[11..13].copy_from_slice(&bytes_per_sector.to_le_bytes());
    boot[14..16].copy_from_slice(&reserved_sectors.to_le_bytes());
    boot[16] = fat_count;
    boot[17..19].copy_from_slice(&root_entries.to_le_bytes());
    boot[19..21].copy_from_slice(&(partition_blocks as u16).to_le_bytes());
    boot[22..24].copy_from_slice(&fat_sectors.to_le_bytes());
    boot[510] = 0x55;
    boot[511] = 0xAA;
    boot
}

fn install_valid_fat12_layout(backend: &mut RamBackend, logical_blocks: u32) {
    let partition_start = 1u32;
    let partition_blocks = logical_blocks.saturating_sub(partition_start);
    backend.set_physical_block(logical_to_physical(0), build_mbr(partition_start, partition_blocks));
    backend.set_physical_block(
        logical_to_physical(partition_start),
        build_boot_sector(partition_blocks, BLOCK_SIZE as u16, 1, 2, 32, 1),
    );
}

fn write_journal_header(backend: &mut RamBackend, state: u8, target_lba: u32, shadow_crc32: u32) {
    let mut header = [0u8; BLOCK_SIZE];
    header[0] = state;
    header[1..4].copy_from_slice(&JOURNAL_MAGIC);
    header[4..8].copy_from_slice(&target_lba.to_le_bytes());
    header[8..12].copy_from_slice(&shadow_crc32.to_le_bytes());
    let header_crc = crc32(&header[..12]);
    header[12..16].copy_from_slice(&header_crc.to_le_bytes());
    backend.set_physical_block(0, header);
}

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for byte in data {
        crc ^= *byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg() & 0xEDB8_8320;
            crc = (crc >> 1) ^ mask;
        }
    }
    !crc
}

fn op_index(ops: &[BackendOp], predicate: impl Fn(&BackendOp) -> bool) -> usize {
    ops.iter()
        .position(predicate)
        .expect("expected operation not found")
}

#[test]
// Verify blank media initialization writes a clean journal header with magic.
fn init_blank_media_creates_clean_header() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(16);
    let mut storage = MetadataJournalStorage::new(backend.clone());

    run_async(async {
        storage.initialize().await.expect("initialize failed");
    });

    let state = backend.inner();
    let state = state.lock().expect("backend poisoned");
    let header = state.bytes_at(0, 8);

    assert_eq!(header[0], JOURNAL_STATE_CLEAN);
    assert_eq!(&header[1..4], JOURNAL_MAGIC.as_slice());
}

#[test]
// Verify unknown journal state is normalized to clean during initialize.
fn recovery_with_unknown_journal_state_initializes_clean() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(32);
    {
        let state = backend.inner();
        let mut state = state.lock().expect("backend poisoned");
        let logical_blocks = 32 - JOURNAL_RESERVED_BLOCKS;
        install_valid_fat12_layout(&mut state, logical_blocks);
        let mut header = [0u8; BLOCK_SIZE];
        header[0] = 0x42;
        header[1..4].copy_from_slice(&JOURNAL_MAGIC);
        state.set_physical_block(0, header);
    }

    let mut storage = MetadataJournalStorage::new(backend.clone());
    run_async(async { storage.initialize().await.expect("init should succeed") });

    let state = backend.inner();
    let state = state.lock().unwrap();
    let header = state.bytes_at(0, 16);
    assert_eq!(header[0], JOURNAL_STATE_CLEAN);
    assert_eq!(u32::from_le_bytes([header[12], header[13], header[14], header[15]]), crc32(&header[..12]));
}

#[test]
// Verify zero physical blocks reports NotReady during initialize.
fn init_with_zero_physical_blocks_returns_not_ready() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(0);
    let mut storage = MetadataJournalStorage::new(backend);

    let err = run_async(async { storage.initialize().await.expect_err("expected NotReady") });
    assert_eq!(err, StorageError::NotReady);
}

#[test]
// Verify FAT12 layout detection marks metadata LBAs as protected and uses journal sequence.
fn fat12_layout_detection_marks_protected_region() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(32);
    {
        let state = backend.inner();
        let mut state = state.lock().expect("backend poisoned");
        let logical_blocks = 32 - JOURNAL_RESERVED_BLOCKS;
        install_valid_fat12_layout(&mut state, logical_blocks);
    }

    let mut storage = MetadataJournalStorage::new(backend.clone());
    run_async(async { storage.initialize().await.expect("initialize failed") });

    {
        let state = backend.inner();
        state.lock().expect("backend poisoned").clear_operations();
    }

    let block = [0x5Au8; BLOCK_SIZE];
    run_async(async {
        storage
            .write_block(2, &block)
            .await
            .expect("journaled write failed")
    });

    let ops = backend.inner();
    let ops = ops.lock().expect("backend poisoned");

    let writes = ops.operations();
    let idx_clean_header = op_index(writes, |op| matches!(op, BackendOp::WriteBytes { address: 0, data } if data.first() == Some(&JOURNAL_STATE_CLEAN)));
    let idx_clean_header_crc = op_index(writes, |op| matches!(op, BackendOp::WriteBytes { address: 12, data } if data.len() == 4));
    let idx_shadow = op_index(writes, |op| matches!(op, BackendOp::WritePhysicalBlock { block_index: 1, .. }));
    let idx_commit_header = op_index(writes, |op| matches!(op, BackendOp::WriteBytes { address: 0, data } if data.first() == Some(&JOURNAL_STATE_COMMITTED)));
    let idx_target_write = op_index(
        writes,
        |op| matches!(op, BackendOp::WritePhysicalBlock { block_index, .. } if *block_index == logical_to_physical(2)),
    );
    let idx_final_clean_header = writes
        .iter()
        .enumerate()
        .rfind(|(_, op)| matches!(op, BackendOp::WriteBytes { address: 0, data } if data.first() == Some(&JOURNAL_STATE_CLEAN)))
        .map(|(idx, _)| idx)
        .expect("final clean state write not found");

    assert!(idx_clean_header < idx_clean_header_crc);
    assert!(idx_clean_header_crc < idx_shadow);
    assert!(idx_shadow < idx_commit_header);
    assert!(idx_commit_header < idx_target_write);
    assert!(idx_target_write < idx_final_clean_header);
}

#[test]
// Verify unknown volume layout bypasses journaling and leaves metadata writes untouched.
fn no_valid_partition_leaves_protected_region_empty() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(16);
    let mut storage = MetadataJournalStorage::new(backend.clone());

    run_async(async { storage.initialize().await.expect("initialize failed") });
    {
        let state = backend.inner();
        state.lock().expect("backend poisoned").clear_operations();
    }

    let block = [0x33u8; BLOCK_SIZE];
    run_async(async {
        storage
            .write_block(2, &block)
            .await
                .expect("passthrough write failed")
    });

    let state = backend.inner();
    let state = state.lock().expect("backend poisoned");
    let ops = state.operations();

    assert!(ops.iter().any(|op| matches!(op, BackendOp::WritePhysicalBlock { block_index, .. } if *block_index == logical_to_physical(2))));
    assert!(!ops.iter().any(|op| matches!(op, BackendOp::WriteBytes { address: 0, .. })));
}

#[test]
// Verify invalid boot-sector signature disables metadata protection.
fn damaged_boot_sector_disables_protected_region() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(16);
    {
        let state = backend.inner();
        let mut state = state.lock().expect("backend poisoned");
        let logical_blocks = 16 - JOURNAL_RESERVED_BLOCKS;
        state.set_physical_block(logical_to_physical(0), build_mbr(1, logical_blocks - 1));

        let mut boot = build_boot_sector(logical_blocks - 1, BLOCK_SIZE as u16, 1, 2, 32, 1);
        boot[510] = 0;
        boot[511] = 0;
        state.set_physical_block(logical_to_physical(1), boot);
    }

    let mut storage = MetadataJournalStorage::new(backend.clone());
    run_async(async { storage.initialize().await.expect("initialize failed") });

    {
        let state = backend.inner();
        state.lock().expect("backend poisoned").clear_operations();
    }

    let block = [0x21u8; BLOCK_SIZE];
    run_async(async { storage.write_block(2, &block).await.expect("write failed") });

    let state = backend.inner();
    let state = state.lock().expect("backend poisoned");
    let ops = state.operations();
    assert!(!ops
        .iter()
        .any(|op| matches!(op, BackendOp::WriteBytes { address: 0, .. })));
}

#[test]
// Verify non-512-byte sectors disable metadata protection.
fn non_512_byte_boot_sector_disables_protected_region() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(16);
    {
        let state = backend.inner();
        let mut state = state.lock().expect("backend poisoned");
        let logical_blocks = 16 - JOURNAL_RESERVED_BLOCKS;
        state.set_physical_block(logical_to_physical(0), build_mbr(1, logical_blocks - 1));
        state.set_physical_block(logical_to_physical(1), build_boot_sector(logical_blocks - 1, 1024, 1, 2, 32, 1));
    }

    let mut storage = MetadataJournalStorage::new(backend.clone());
    run_async(async { storage.initialize().await.expect("initialize failed") });

    {
        let state = backend.inner();
        state.lock().expect("backend poisoned").clear_operations();
    }

    let block = [0x44u8; BLOCK_SIZE];
    run_async(async { storage.write_block(2, &block).await.expect("write failed") });

    let state = backend.inner();
    let state = state.lock().expect("backend poisoned");
    let ops = state.operations();
    assert!(!ops
        .iter()
        .any(|op| matches!(op, BackendOp::WriteBytes { address: 0, .. })));
}

#[test]
// Verify COMMITTED journal state replays shadow block to protected target on initialize.
fn recovery_replays_shadow_after_committed_before_target_write() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(32);
    {
        let state = backend.inner();
        let mut state = state.lock().expect("backend poisoned");
        let logical_blocks = 32 - JOURNAL_RESERVED_BLOCKS;
        install_valid_fat12_layout(&mut state, logical_blocks);
        let mut shadow = [0xA7u8; BLOCK_SIZE];
        shadow[0] = 0x5C;
        write_journal_header(&mut state, JOURNAL_STATE_COMMITTED, 2, crc32(&shadow));

        state.set_physical_block(1, shadow);

        let mut target = [0x00u8; BLOCK_SIZE];
        target[0] = 0xEE;
        state.set_physical_block(logical_to_physical(2), target);
    }

    let mut storage = MetadataJournalStorage::new(backend.clone());
    run_async(async { storage.initialize().await.expect("initialize failed") });

    let state = backend.inner();
    let state = state.lock().expect("backend poisoned");
    assert_eq!(state.bytes_at(0, 1)[0], JOURNAL_STATE_CLEAN);
    assert_eq!(state.physical_block(logical_to_physical(2)), state.physical_block(1));
}

#[test]
// Verify COMMITTED state is cleared even when target already has shadow data.
fn recovery_clears_state_after_target_already_written() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(32);
    {
        let state = backend.inner();
        let mut state = state.lock().expect("backend poisoned");
        let logical_blocks = 32 - JOURNAL_RESERVED_BLOCKS;
        install_valid_fat12_layout(&mut state, logical_blocks);
        let shadow = [0x7Bu8; BLOCK_SIZE];
        write_journal_header(&mut state, JOURNAL_STATE_COMMITTED, 2, crc32(&shadow));

        state.set_physical_block(1, shadow);
        state.set_physical_block(logical_to_physical(2), shadow);
    }

    let mut storage = MetadataJournalStorage::new(backend.clone());
    run_async(async { storage.initialize().await.expect("initialize failed") });

    let state = backend.inner();
    let state = state.lock().expect("backend poisoned");
    assert_eq!(state.bytes_at(0, 1)[0], JOURNAL_STATE_CLEAN);
    assert_eq!(state.physical_block(logical_to_physical(2)), state.physical_block(1));
}

#[test]
// Verify out-of-range committed target is ignored while state still clears.
fn recovery_ignores_out_of_capacity_target_lba() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(12);
    {
        let state = backend.inner();
        let mut state = state.lock().expect("backend poisoned");
        let logical_blocks = 12 - JOURNAL_RESERVED_BLOCKS;
        install_valid_fat12_layout(&mut state, logical_blocks);
        write_journal_header(&mut state, JOURNAL_STATE_COMMITTED, 99, 0);

        let shadow = [0xC3u8; BLOCK_SIZE];
        state.set_physical_block(1, shadow);
    }

    let mut storage = MetadataJournalStorage::new(backend.clone());
    run_async(async { storage.initialize().await.expect("initialize failed") });

    let state = backend.inner();
    let state = state.lock().expect("backend poisoned");
    assert_eq!(state.bytes_at(0, 1)[0], JOURNAL_STATE_CLEAN);
}

#[test]
// Verify read errors during replay are propagated from initialize.
fn recovery_propagates_shadow_read_error() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(20);
    {
        let state = backend.inner();
        let mut state = state.lock().expect("backend poisoned");
        let logical_blocks = 20 - JOURNAL_RESERVED_BLOCKS;
        install_valid_fat12_layout(&mut state, logical_blocks);
        write_journal_header(&mut state, JOURNAL_STATE_COMMITTED, 2, 0);
        state.inject_read_block_error_at(1, StorageError::HardwareError);
    }

    let mut storage = MetadataJournalStorage::new(backend);
    let err = run_async(async { storage.initialize().await.expect_err("expected HardwareError") });
    assert_eq!(err, StorageError::HardwareError);
}

#[test]
// Verify write errors during replay are propagated from initialize.
fn recovery_propagates_target_write_error() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(20);
    {
        let state = backend.inner();
        let mut state = state.lock().expect("backend poisoned");
        let logical_blocks = 20 - JOURNAL_RESERVED_BLOCKS;
        install_valid_fat12_layout(&mut state, logical_blocks);
        let shadow = [0u8; BLOCK_SIZE];
        write_journal_header(&mut state, JOURNAL_STATE_COMMITTED, 2, crc32(&shadow));
        state.inject_write_block_error_at(logical_to_physical(2), StorageError::MediumError);
    }

    let mut storage = MetadataJournalStorage::new(backend);
    let err = run_async(async { storage.initialize().await.expect_err("expected MediumError") });
    assert_eq!(err, StorageError::MediumError);
}

#[test]
// Verify CRC mismatch in committed shadow block is discarded and initialization continues.
fn recovery_rejects_corrupted_shadow_crc() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(24);
    {
        let state = backend.inner();
        let mut state = state.lock().expect("backend poisoned");
        let logical_blocks = 24 - JOURNAL_RESERVED_BLOCKS;
        install_valid_fat12_layout(&mut state, logical_blocks);

        let shadow = [0x42u8; BLOCK_SIZE];
        write_journal_header(&mut state, JOURNAL_STATE_COMMITTED, 2, 0xDEAD_BEEF);
        state.set_physical_block(1, shadow);
        state.set_physical_block(logical_to_physical(2), [0x11u8; BLOCK_SIZE]);
    }

    let mut storage = MetadataJournalStorage::new(backend.clone());
    run_async(async { storage.initialize().await.expect("initialize should continue") });

    let state = backend.inner();
    let state = state.lock().expect("backend poisoned");
    assert_eq!(state.bytes_at(0, 1)[0], JOURNAL_STATE_CLEAN);
    assert_eq!(state.physical_block(logical_to_physical(2)), [0x11u8; BLOCK_SIZE]);
}

#[test]
// Verify journal header CRC mismatch resets journal to clean defaults.
fn recovery_rewrites_header_on_header_crc_mismatch() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(24);
    {
        let state = backend.inner();
        let mut state = state.lock().expect("backend poisoned");
        let logical_blocks = 24 - JOURNAL_RESERVED_BLOCKS;
        install_valid_fat12_layout(&mut state, logical_blocks);

        let mut header = [0u8; BLOCK_SIZE];
        header[0] = JOURNAL_STATE_COMMITTED;
        header[1..4].copy_from_slice(&JOURNAL_MAGIC);
        header[4..8].copy_from_slice(&2u32.to_le_bytes());
        header[8..12].copy_from_slice(&0x1234_5678u32.to_le_bytes());
        header[12..16].copy_from_slice(&0x0000_0000u32.to_le_bytes());
        state.set_physical_block(0, header);
    }

    let mut storage = MetadataJournalStorage::new(backend.clone());
    run_async(async { storage.initialize().await.expect("initialize should continue") });

    let state = backend.inner();
    let state = state.lock().expect("backend poisoned");
    let header = state.physical_block(0);
    assert_eq!(header[0], JOURNAL_STATE_CLEAN);
    assert_eq!(&header[1..4], JOURNAL_MAGIC.as_slice());
    assert_eq!(u32::from_le_bytes([header[12], header[13], header[14], header[15]]), crc32(&header[..12]));
}
#[test]
// Verify shadow block write failures during journaled writes are propagated.
fn journal_write_propagates_shadow_write_error() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(32);
    {
        let state = backend.inner();
        let mut state = state.lock().expect("backend poisoned");
        let logical_blocks = 32 - JOURNAL_RESERVED_BLOCKS;
        install_valid_fat12_layout(&mut state, logical_blocks);
        state.inject_write_block_error_at(1, StorageError::HardwareError);
    }

    let mut storage = MetadataJournalStorage::new(backend.clone());
    run_async(async { storage.initialize().await.expect("init should be ok") });

    let result = run_async(async { storage.write_block(2, &[0xCC; BLOCK_SIZE]).await });
    assert_eq!(result, Err(StorageError::HardwareError));
}

#[test]
// Verify repeated initialize calls remain safe and keep the journal clean.
fn double_initialize_should_be_idempotent() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(32);
    let mut storage = MetadataJournalStorage::new(backend.clone());

    run_async(async { storage.initialize().await.expect("first init") });
    run_async(async { storage.initialize().await.expect("second init") });

    let state = backend.inner();
    let state = state.lock().unwrap();
    assert_eq!(state.bytes_at(0, 1)[0], JOURNAL_STATE_CLEAN);
}

#[test]
// Verify read/write before initialize returns NotReady.
fn uninitialized_access_returns_not_ready() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(16);
    let mut storage = MetadataJournalStorage::new(backend);

    let mut read_out = [0u8; BLOCK_SIZE];
    let read_err = run_async(async { storage.read_block(0, &mut read_out).await.expect_err("expected NotReady") });
    assert_eq!(read_err, StorageError::NotReady);

    let write_err =
        run_async(async { storage.write_block(0, &[0u8; BLOCK_SIZE]).await.expect_err("expected NotReady") });
    assert_eq!(write_err, StorageError::NotReady);
}

#[test]
// Verify write-protected backend surfaces WriteProtect on journal initialization writes.
fn write_protected_backend_returns_write_protect() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(16);
    {
        let state = backend.inner();
        state
            .lock()
            .expect("backend poisoned")
            .set_write_protected(true);
    }

    let mut storage = MetadataJournalStorage::new(backend);
    let err = run_async(async { storage.initialize().await.expect_err("expected WriteProtect") });
    assert_eq!(err, StorageError::WriteProtect);
}

#[test]
// Verify storage backend errors propagate during initialize reads.
fn initialize_propagates_backend_errors() {
    let _guard = test_lock();

    let backend = SharedRamBackend::new(16);
    {
        let state = backend.inner();
        state
            .lock()
            .expect("backend poisoned")
            .inject_next_read_bytes_error(StorageError::HardwareError);
    }

    let mut storage = MetadataJournalStorage::new(backend);
    let err = run_async(async { storage.initialize().await.expect_err("expected HardwareError") });
    assert_eq!(err, StorageError::HardwareError);
}
