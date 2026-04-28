#![cfg(not(feature = "hardware"))]

mod common;

use common::test_harness::BotTestHarness;
use common::test_lock;

const CBW_SIGNATURE: u32 = 0x4342_5355;
const CSW_SIGNATURE: u32 = 0x5342_5355;
const CSW_STATUS_PASSED: u8 = 0;
const CSW_STATUS_PHASE_ERROR: u8 = 2;
const BOT_ACTION_STALL_IN: u8 = 1 << 0;
const BOT_ACTION_STALL_OUT: u8 = 1 << 1;

fn build_cbw_packet(
    tag: u32,
    data_len: u32,
    flags: u8,
    lun: u8,
    command_len: u8,
    command: [u8; 16],
) -> Vec<u8> {
    let mut cbw = [0u8; 31];
    cbw[0..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
    cbw[4..8].copy_from_slice(&tag.to_le_bytes());
    cbw[8..12].copy_from_slice(&data_len.to_le_bytes());
    cbw[12] = flags;
    cbw[13] = lun;
    cbw[14] = command_len;
    cbw[15..31].copy_from_slice(&command);
    cbw.to_vec()
}

fn cbw_test_unit_ready(tag: u32) -> Vec<u8> {
    let mut cdb = [0u8; 16];
    cdb[0] = 0x00;
    build_cbw_packet(tag, 0, 0x00, 0, 6, cdb)
}

fn cbw_write_10(tag: u32, lba: u32, blocks: u16, expected_len: u32) -> Vec<u8> {
    let mut cdb = [0u8; 16];
    cdb[0] = 0x2A;
    cdb[2..6].copy_from_slice(&lba.to_be_bytes());
    cdb[7..9].copy_from_slice(&blocks.to_be_bytes());
    build_cbw_packet(tag, expected_len, 0x00, 0, 10, cdb)
}

fn cbw_read_10(tag: u32, lba: u32, blocks: u16, expected_len: u32) -> Vec<u8> {
    let mut cdb = [0u8; 16];
    cdb[0] = 0x28;
    cdb[2..6].copy_from_slice(&lba.to_be_bytes());
    cdb[7..9].copy_from_slice(&blocks.to_be_bytes());
    build_cbw_packet(tag, expected_len, 0x80, 0, 10, cdb)
}

fn parse_csw(packet: &[u8]) -> Option<(u32, u32, u8)> {
    if packet.len() != 13 {
        return None;
    }

    let signature = u32::from_le_bytes([packet[0], packet[1], packet[2], packet[3]]);
    if signature != CSW_SIGNATURE {
        return None;
    }

    let tag = u32::from_le_bytes([packet[4], packet[5], packet[6], packet[7]]);
    let residue = u32::from_le_bytes([packet[8], packet[9], packet[10], packet[11]]);
    let status = packet[12];
    Some((tag, residue, status))
}

fn collect_csws(harness: &BotTestHarness) -> Vec<(u32, u32, u8)> {
    harness
        .in_packets()
        .into_iter()
        .filter_map(|packet| parse_csw(&packet))
        .collect()
}

#[test]
// Verify a valid CBW command sequence produces a PASSED CSW.
fn bot_valid_cbw_produces_csw() {
    let _guard = test_lock();

    let harness = BotTestHarness::new(16);
    harness.queue_out_packet(cbw_test_unit_ready(0xA1A2_A3A4));

    harness.run_for(40);

    let csws = collect_csws(&harness);
    assert!(csws
        .iter()
        .any(|(tag, _, status)| *tag == 0xA1A2_A3A4 && *status == CSW_STATUS_PASSED));
}

#[test]
// Verify invalid CBW signature yields Phase Error CSW and stalls both bulk endpoints.
fn bot_invalid_cbw_signature_stalls_both_endpoints() {
    let _guard = test_lock();

    let harness = BotTestHarness::new(8);
    let mut bad = cbw_test_unit_ready(0x0102_0304);
    bad[0..4].copy_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
    harness.queue_out_packet(bad);

    harness.run_for(40);

    let csws = collect_csws(&harness);
    assert!(csws
        .iter()
        .any(|(tag, _, status)| *tag == 0x0102_0304 && *status == CSW_STATUS_PHASE_ERROR));

    let actions = harness.take_bus_actions();
    assert_eq!(actions & (BOT_ACTION_STALL_IN | BOT_ACTION_STALL_OUT), BOT_ACTION_STALL_IN | BOT_ACTION_STALL_OUT);
}

#[test]
// Verify invalid CBW length yields Phase Error CSW and dual-endpoint stall request.
fn bot_invalid_cbw_length_stalls_both_endpoints() {
    let _guard = test_lock();

    let harness = BotTestHarness::new(8);
    let mut truncated = cbw_test_unit_ready(0x1111_2222);
    truncated.pop();
    harness.queue_out_packet(truncated);

    harness.run_for(40);

    let csws = collect_csws(&harness);
    assert!(csws
        .iter()
        .any(|(tag, _, status)| *tag == 0x1111_2222 && *status == CSW_STATUS_PHASE_ERROR));

    let actions = harness.take_bus_actions();
    assert_eq!(actions & (BOT_ACTION_STALL_IN | BOT_ACTION_STALL_OUT), BOT_ACTION_STALL_IN | BOT_ACTION_STALL_OUT);
}

#[test]
// Verify oversized CBW packets are accepted and extra bytes are ignored.
fn bot_oversized_cbw_is_accepted() {
    let _guard = test_lock();

    let harness = BotTestHarness::new(16);
    let mut oversized = cbw_test_unit_ready(0xABCD_0001);
    oversized.push(0xFF);
    harness.queue_out_packet(oversized);

    harness.run_for(40);

    let csws = collect_csws(&harness);
    assert!(csws.iter().any(|(tag, _, status)| *tag == 0xABCD_0001 && *status == CSW_STATUS_PASSED));
}

#[test]
// Verify invalid LUN in CBW yields Phase Error CSW and dual-endpoint stall request.
fn bot_invalid_lun_stalls_both_endpoints() {
    let _guard = test_lock();

    let harness = BotTestHarness::new(8);
    let mut cdb = [0u8; 16];
    cdb[0] = 0x00;
    harness.queue_out_packet(build_cbw_packet(0x2222_3333, 0, 0x00, 1, 6, cdb));

    harness.run_for(40);

    let csws = collect_csws(&harness);
    assert!(csws
        .iter()
        .any(|(tag, _, status)| *tag == 0x2222_3333 && *status == CSW_STATUS_PHASE_ERROR));

    let actions = harness.take_bus_actions();
    assert_eq!(actions & (BOT_ACTION_STALL_IN | BOT_ACTION_STALL_OUT), BOT_ACTION_STALL_IN | BOT_ACTION_STALL_OUT);
}

#[test]
// Verify a bulk reset injected during command execution resets BOT flow to wait for next CBW.
fn bot_bulk_reset_during_execution_recovers_to_next_command() {
    let _guard = test_lock();

    let harness = BotTestHarness::new(32);
    harness.queue_out_packet(cbw_write_10(0xAAAA_0001, 0, 1, 512));
    harness.queue_out_packet(vec![0x5A; 64]);

    // Inject reset deterministically during command execution:
    // CBW read is count=1, first WRITE(10) data packet read is count=2.
    harness
        .out_ep
        .signal_reset_on_read(2, harness.bot_control.clone());

    harness.run_for(1700);

    // First command should not emit CSW because reset is consumed before SendingCsw.
    let csws_after_reset = collect_csws(&harness);
    assert!(!csws_after_reset.iter().any(|(tag, _, _)| *tag == 0xAAAA_0001));

    // After reset, BOT returns to WaitingForCbw and processes the next command normally.
    harness.queue_out_packet(cbw_test_unit_ready(0xBBBB_0002));
    harness.run_for(40);

    let csws = collect_csws(&harness);
    assert!(csws
        .iter()
        .any(|(tag, _, status)| *tag == 0xBBBB_0002 && *status == CSW_STATUS_PASSED));
}

#[test]
// Verify command failure causes endpoint stall request according to BOT rules.
fn bot_command_failure_requests_stall() {
    let _guard = test_lock();

    let harness = BotTestHarness::new(4);
    harness.queue_out_packet(cbw_read_10(0x3333_4444, 99, 1, 512));

    harness.run_for(60);

    let actions = harness.take_bus_actions();
    assert_ne!(actions & BOT_ACTION_STALL_IN, 0);
}

#[test]
// Verify write failures request a bulk-OUT stall after the CSW.
fn bot_write_failure_requests_out_stall() {
    let _guard = test_lock();

    let harness = BotTestHarness::new(4);
    harness.storage_inner().lock().unwrap().set_write_protected(true);
    harness.queue_out_packet(cbw_write_10(0x5555_6666, 0, 1, 512));
    harness.queue_out_packet(vec![0x00; 64]);

    harness.run_for(60);

    let actions = harness.take_bus_actions();
    assert_ne!(actions & BOT_ACTION_STALL_OUT, 0);
}
