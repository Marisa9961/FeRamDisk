#![cfg(not(feature = "hardware"))]

mod common;

use common::mock_endpoint::MockEndpoint;
use common::{run_async, test_lock};
use feramdisk_firmware::backend::simulated::{
    CommandOutcome, Cbw, RamStorage, SenseData, StallAfterCsw, StorageError, BLOCK_SIZE,
    CSW_STATUS_FAILED, CSW_STATUS_PASSED, SCSI_INQUIRY, SCSI_MODE_SENSE_10,
    SCSI_MODE_SENSE_6, SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL, SCSI_READ_10,
    SCSI_READ_CAPACITY_10, SCSI_REQUEST_SENSE, SCSI_START_STOP_UNIT,
    SCSI_SYNCHRONIZE_CACHE_10, SCSI_TEST_UNIT_READY, SCSI_VERIFY_10,
    SCSI_WRITE_10, USB_PACKET_SIZE, execute_command,
};

fn build_cbw(opcode: u8, data_len: u32, flags: u8, command_len: u8, fill: impl FnOnce(&mut [u8; 16])) -> Cbw {
    let mut command = [0u8; 16];
    command[0] = opcode;
    fill(&mut command);
    Cbw::new(0x1122_3344, data_len, flags, 0, command_len, command)
}

fn read10_cbw(lba: u32, blocks: u16, expected_len: u32) -> Cbw {
    build_cbw(SCSI_READ_10, expected_len, 0x80, 10, |cmd| {
        cmd[2..6].copy_from_slice(&lba.to_be_bytes());
        cmd[7..9].copy_from_slice(&blocks.to_be_bytes());
    })
}

fn write10_cbw(lba: u32, blocks: u16, expected_len: u32) -> Cbw {
    build_cbw(SCSI_WRITE_10, expected_len, 0x00, 10, |cmd| {
        cmd[2..6].copy_from_slice(&lba.to_be_bytes());
        cmd[7..9].copy_from_slice(&blocks.to_be_bytes());
    })
}

fn flatten_packets(packets: &[Vec<u8>]) -> Vec<u8> {
    packets.iter().flat_map(|packet| packet.clone()).collect()
}

fn run_cmd(
    storage: &mut RamStorage,
    out_ep: &mut MockEndpoint,
    in_ep: &mut MockEndpoint,
    sense: &mut SenseData,
    prevent_medium_removal: &mut bool,
    cbw: Cbw,
) -> CommandOutcome {
    run_async(async {
        execute_command(storage, out_ep, in_ep, sense, prevent_medium_removal, cbw)
            .await
            .expect("endpoint error")
    })
}

#[test]
// Verify INQUIRY returns the standard 36-byte response payload.
fn inquiry_returns_standard_response() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(32);
    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let cbw = build_cbw(SCSI_INQUIRY, 36, 0x80, 6, |cmd| {
        cmd[4] = 36;
    });

    let outcome = run_cmd(&mut storage, &mut out_ep, &mut in_ep, &mut sense, &mut prevent, cbw);
    let payload = flatten_packets(in_ep.writes());

    assert_eq!(outcome.csw.status, CSW_STATUS_PASSED);
    assert_eq!(payload.len(), 36);
    assert_eq!(&payload[8..16], b"FeRam   ");
}

#[test]
// Verify INQUIRY with EVPD page_code!=0 reports CHECK CONDITION with field pointer 2.
fn inquiry_evpd_invalid_page_sets_illegal_field_sense() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(8);
    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let cbw = build_cbw(SCSI_INQUIRY, 36, 0x80, 6, |cmd| {
        cmd[1] = 0x01;
        cmd[2] = 0x12;
        cmd[4] = 36;
    });

    let outcome = run_cmd(&mut storage, &mut out_ep, &mut in_ep, &mut sense, &mut prevent, cbw);
    let resp = sense.to_response();

    assert_eq!(outcome.csw.status, CSW_STATUS_FAILED);
    assert_eq!(resp[2] & 0x0F, 0x05);
    assert_eq!(resp[15], 0xC0);
    assert_eq!(resp[16], 0x00);
    assert_eq!(resp[17], 0x02);
}

#[test]
// Verify READ CAPACITY (10) returns max LBA and block size.
fn read_capacity_10_returns_capacity_payload() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(128);
    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let cbw = build_cbw(SCSI_READ_CAPACITY_10, 8, 0x80, 10, |_| {});
    let outcome = run_cmd(&mut storage, &mut out_ep, &mut in_ep, &mut sense, &mut prevent, cbw);
    let payload = flatten_packets(in_ep.writes());

    assert_eq!(outcome.csw.status, CSW_STATUS_PASSED);
    assert_eq!(payload.len(), 8);
    assert_eq!(u32::from_be_bytes([payload[0], payload[1], payload[2], payload[3]]), 127);
    assert_eq!(u32::from_be_bytes([payload[4], payload[5], payload[6], payload[7]]), BLOCK_SIZE as u32);
}

#[test]
// Verify READ(10) single-block success streams block data and reports PASSED.
fn read10_single_block_success() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(16);
    let mut block = [0u8; BLOCK_SIZE];
    for (i, b) in block.iter_mut().enumerate() {
        *b = (i & 0xFF) as u8;
    }
    storage.set_block(3, block);

    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        read10_cbw(3, 1, BLOCK_SIZE as u32),
    );
    let payload = flatten_packets(in_ep.writes());

    assert_eq!(outcome.csw.status, CSW_STATUS_PASSED);
    assert_eq!(outcome.csw.residue, 0);
    assert_eq!(payload, block.to_vec());
}

#[test]
// Verify READ(10) multi-block success concatenates all requested blocks.
fn read10_multi_block_success() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(16);
    storage.set_block(1, [0x11; BLOCK_SIZE]);
    storage.set_block(2, [0x22; BLOCK_SIZE]);

    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        read10_cbw(1, 2, (2 * BLOCK_SIZE) as u32),
    );

    let payload = flatten_packets(in_ep.writes());
    assert_eq!(outcome.csw.status, CSW_STATUS_PASSED);
    assert_eq!(payload.len(), 2 * BLOCK_SIZE);
    assert_eq!(&payload[..BLOCK_SIZE], &[0x11; BLOCK_SIZE]);
    assert_eq!(&payload[BLOCK_SIZE..], &[0x22; BLOCK_SIZE]);
}

#[test]
// Verify READ(10) out-of-range LBA returns CHECK CONDITION with valid information field.
fn read10_out_of_range_sets_lba_sense() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(4);
    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let bad_lba = 4;
    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        read10_cbw(bad_lba, 1, BLOCK_SIZE as u32),
    );

    let resp = sense.to_response();
    assert_eq!(outcome.csw.status, CSW_STATUS_FAILED);
    assert_ne!(resp[0] & 0x80, 0);
    assert_eq!(u32::from_be_bytes([resp[3], resp[4], resp[5], resp[6]]), bad_lba);
}

#[test]
// Verify READ(10) with transfer length 0 succeeds without data phase.
fn read10_zero_blocks_succeeds_without_data() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(8);
    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        read10_cbw(0, 0, 0),
    );

    assert_eq!(outcome.csw.status, CSW_STATUS_PASSED);
    assert!(in_ep.writes().is_empty());
}

#[test]
// Verify READ(10) backend failures map sense and request IN stall after CSW.
fn read10_storage_error_maps_sense_and_stalls_in() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(8);
    storage.inject_read_error_at(0, StorageError::MediumError);

    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        read10_cbw(0, 1, BLOCK_SIZE as u32),
    );

    let resp = sense.to_response();
    assert_eq!(outcome.csw.status, CSW_STATUS_FAILED);
    assert_eq!(outcome.stall_after_csw, StallAfterCsw::In);
    assert_eq!(resp[2] & 0x0F, 0x03);
}

#[test]
// Verify READ(10) sends ZLP when host expects more and payload length is packet aligned.
fn read10_sends_zlp_on_short_aligned_in_transfer() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(8);
    storage.set_block(0, [0xAB; BLOCK_SIZE]);

    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        read10_cbw(0, 1, (BLOCK_SIZE + USB_PACKET_SIZE) as u32),
    );

    assert_eq!(outcome.csw.status, CSW_STATUS_PASSED);
    assert_eq!(in_ep.writes().last().cloned().unwrap_or_default(), Vec::<u8>::new());
}

fn queue_write_payload(out_ep: &mut MockEndpoint, payload: &[u8]) {
    for chunk in payload.chunks(USB_PACKET_SIZE) {
        out_ep.queue_read_packet(chunk.to_vec());
    }
}

#[test]
// Verify WRITE(10) single-block success stores host data and reports PASSED.
fn write10_single_block_success() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(16);
    let mut out_ep = MockEndpoint::new();

    let payload = vec![0x3Cu8; BLOCK_SIZE];
    queue_write_payload(&mut out_ep, &payload);

    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        write10_cbw(4, 1, BLOCK_SIZE as u32),
    );

    assert_eq!(outcome.csw.status, CSW_STATUS_PASSED);
    assert_eq!(storage.block(4), [0x3Cu8; BLOCK_SIZE]);
}

#[test]
// Verify WRITE(10) multi-block success writes both blocks in order.
fn write10_multi_block_success() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(16);
    let mut out_ep = MockEndpoint::new();

    let mut payload = vec![0x10u8; BLOCK_SIZE];
    payload.extend(vec![0x20u8; BLOCK_SIZE]);
    queue_write_payload(&mut out_ep, &payload);

    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        write10_cbw(2, 2, (2 * BLOCK_SIZE) as u32),
    );

    assert_eq!(outcome.csw.status, CSW_STATUS_PASSED);
    assert_eq!(storage.block(2), [0x10u8; BLOCK_SIZE]);
    assert_eq!(storage.block(3), [0x20u8; BLOCK_SIZE]);
}

#[test]
// Verify WRITE(10) short packet mismatch reports CHECK CONDITION with ILI/SKSV metadata.
fn write10_short_packet_reports_mismatch() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(8);
    let mut out_ep = MockEndpoint::new();

    for _ in 0..7 {
        out_ep.queue_read_packet(vec![0x77; USB_PACKET_SIZE]);
    }
    out_ep.queue_read_packet(vec![0x88; 32]);

    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        write10_cbw(0, 1, BLOCK_SIZE as u32),
    );

    let resp = sense.to_response();
    assert_eq!(outcome.csw.status, CSW_STATUS_FAILED);
    assert_ne!(resp[2] & 0x20, 0);
    assert_eq!(resp[15], 0xC0);
    assert_eq!(resp[17], 0x07);
}

#[test]
// Verify WRITE(10) overflow input is drained and reported as length mismatch.
fn write10_overflow_data_reports_mismatch() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(8);
    let before = storage.block(0);

    let mut out_ep = MockEndpoint::new();
    for _ in 0..8 {
        out_ep.queue_read_packet(vec![0x55; USB_PACKET_SIZE]);
    }
    out_ep.queue_read_packet(vec![0x66; 8]);

    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        write10_cbw(0, 1, 500),
    );

    assert_eq!(outcome.csw.status, CSW_STATUS_FAILED);
    assert_eq!(storage.block(0), before);
}

#[test]
// Verify WRITE(10) with early ZLP then timeout is treated as transfer mismatch.
fn write10_early_zlp_is_mismatch() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(8);
    let mut out_ep = MockEndpoint::new();
    out_ep.queue_read_packet(vec![0xAA; USB_PACKET_SIZE]);
    out_ep.queue_read_packet(Vec::<u8>::new());

    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        write10_cbw(0, 1, BLOCK_SIZE as u32),
    );

    assert_eq!(outcome.csw.status, CSW_STATUS_FAILED);
}

#[test]
// Verify WRITE(10) on write-protected media returns DATA PROTECT sense.
fn write10_write_protect_returns_data_protect_sense() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(8);
    storage.set_write_protected(true);

    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        write10_cbw(0, 1, BLOCK_SIZE as u32),
    );

    let resp = sense.to_response();
    assert_eq!(outcome.csw.status, CSW_STATUS_FAILED);
    assert_eq!(resp[2] & 0x0F, 0x07);
}

#[test]
// Verify WRITE(10) backend error maps to hardware sense class.
fn write10_storage_error_maps_sense() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(8);
    storage.inject_write_error_at(0, StorageError::HardwareError);

    let mut out_ep = MockEndpoint::new();
    queue_write_payload(&mut out_ep, &[0x01; BLOCK_SIZE]);

    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        write10_cbw(0, 1, BLOCK_SIZE as u32),
    );

    let resp = sense.to_response();
    assert_eq!(outcome.csw.status, CSW_STATUS_FAILED);
    assert_eq!(resp[2] & 0x0F, 0x04);
}

#[test]
// Verify REQUEST SENSE returns latched sense data and clears it after read.
fn request_sense_returns_and_clears_sense() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(4);
    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let bad_cbw = build_cbw(0xFE, 0, 0x00, 6, |_| {});
    let bad_outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        bad_cbw,
    );
    assert_eq!(bad_outcome.csw.status, CSW_STATUS_FAILED);

    let req_sense = build_cbw(SCSI_REQUEST_SENSE, 18, 0x80, 6, |cmd| {
        cmd[4] = 18;
    });

    let first = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        req_sense,
    );
    assert_eq!(first.csw.status, CSW_STATUS_PASSED);
    let first_resp = flatten_packets(in_ep.writes());
    assert_eq!(first_resp[12], 0x20);

    in_ep.take_writes();
    let second = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        req_sense,
    );
    assert_eq!(second.csw.status, CSW_STATUS_PASSED);
    let second_resp = flatten_packets(in_ep.writes());
    assert_eq!(second_resp[2] & 0x0F, 0x00);
}

#[test]
// Verify REQUEST SENSE with DESC bit set returns ILLEGAL FIELD IN CDB.
fn request_sense_desc_bit_returns_illegal_field() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(4);
    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let cbw = build_cbw(SCSI_REQUEST_SENSE, 18, 0x80, 6, |cmd| {
        cmd[1] = 0x01;
        cmd[4] = 18;
    });

    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        cbw,
    );

    let resp = sense.to_response();
    assert_eq!(outcome.csw.status, CSW_STATUS_FAILED);
    assert_eq!(resp[15], 0xC0);
    assert_eq!(resp[17], 0x01);
}

#[test]
// Verify TEST UNIT READY returns FAILED when medium is not ready.
fn test_unit_ready_not_ready_fails() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(2);
    storage.set_ready(false);

    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let cbw = build_cbw(SCSI_TEST_UNIT_READY, 0, 0x00, 6, |_| {});
    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        cbw,
    );

    assert_eq!(outcome.csw.status, CSW_STATUS_FAILED);
}

#[test]
// Verify MODE SENSE (6/10) unsupported pages return ILLEGAL FIELD in CDB.
fn mode_sense_unsupported_page_returns_illegal_field() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(2);
    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let mode6 = build_cbw(SCSI_MODE_SENSE_6, 24, 0x80, 6, |cmd| {
        cmd[2] = 0x01;
        cmd[4] = 24;
    });
    let out6 = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        mode6,
    );
    assert_eq!(out6.csw.status, CSW_STATUS_FAILED);

    let mode10 = build_cbw(SCSI_MODE_SENSE_10, 28, 0x80, 10, |cmd| {
        cmd[2] = 0x01;
        cmd[7..9].copy_from_slice(&(28u16).to_be_bytes());
    });
    let out10 = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        mode10,
    );
    assert_eq!(out10.csw.status, CSW_STATUS_FAILED);
}

#[test]
// Verify PREVENT ALLOW MEDIUM REMOVAL toggles the prevent flag.
fn prevent_allow_medium_removal_updates_flag() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(2);
    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let set_prevent = build_cbw(SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL, 0, 0x00, 6, |cmd| {
        cmd[4] = 0x01;
    });
    let clear_prevent = build_cbw(SCSI_PREVENT_ALLOW_MEDIUM_REMOVAL, 0, 0x00, 6, |cmd| {
        cmd[4] = 0x00;
    });

    let first = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        set_prevent,
    );
    assert_eq!(first.csw.status, CSW_STATUS_PASSED);
    assert!(prevent);

    let second = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        clear_prevent,
    );
    assert_eq!(second.csw.status, CSW_STATUS_PASSED);
    assert!(!prevent);
}

#[test]
// Verify SYNCHRONIZE CACHE, VERIFY and START STOP UNIT all return PASSED directly.
fn passthrough_commands_return_success() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(2);
    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    for opcode in [SCSI_SYNCHRONIZE_CACHE_10, SCSI_VERIFY_10, SCSI_START_STOP_UNIT] {
        let cbw = build_cbw(opcode, 0, 0x00, 10, |_| {});
        let outcome = run_cmd(
            &mut storage,
            &mut out_ep,
            &mut in_ep,
            &mut sense,
            &mut prevent,
            cbw,
        );
        assert_eq!(outcome.csw.status, CSW_STATUS_PASSED);
    }
}

#[test]
// Verify unknown opcodes map to INVALID COMMAND OPCODE sense.
fn unknown_opcode_sets_invalid_command_sense() {
    let _guard = test_lock();

    let mut storage = RamStorage::new(2);
    let mut out_ep = MockEndpoint::new();
    let mut in_ep = MockEndpoint::new();
    let mut sense = SenseData::good();
    let mut prevent = false;

    let cbw = build_cbw(0xFF, 0, 0x00, 6, |_| {});
    let outcome = run_cmd(
        &mut storage,
        &mut out_ep,
        &mut in_ep,
        &mut sense,
        &mut prevent,
        cbw,
    );

    let resp = sense.to_response();
    assert_eq!(outcome.csw.status, CSW_STATUS_FAILED);
    assert_eq!(resp[12], 0x20);
}
