#![allow(dead_code)]

use crate::feram::BLOCK_SIZE;
use crate::usb::constants::MODE_PAGE_CACHING;

pub(crate) fn build_inquiry_response() -> [u8; 36] {
    let mut response = [0u8; 36];
    response[0] = 0x00;
    response[1] = 0x80;
    response[2] = 0x06;
    response[3] = 0x02;
    response[4] = 31;
    response[8..16].copy_from_slice(b"FeRam   ");
    response[16..32].copy_from_slice(b"FeRamDisk       ");
    response[32..36].copy_from_slice(b"1.00");
    response
}

pub(crate) fn build_read_capacity_10_response(block_count: u32) -> [u8; 8] {
    let mut response = [0u8; 8];
    let last_block = block_count.saturating_sub(1);
    response[0..4].copy_from_slice(&last_block.to_be_bytes());
    response[4..8].copy_from_slice(&(BLOCK_SIZE as u32).to_be_bytes());
    response
}

pub(crate) fn build_read_format_capacities_response(block_count: u32) -> [u8; 12] {
    let mut response = [0u8; 12];
    response[0] = 0x00;
    response[1] = 0x00;
    response[2] = 0x00;
    response[3] = 8;
    response[4..8].copy_from_slice(&block_count.to_be_bytes());
    response[8] = 0x02;
    response[9..12].copy_from_slice(&[
        ((BLOCK_SIZE as u32 >> 16) & 0xFF) as u8,
        ((BLOCK_SIZE as u32 >> 8) & 0xFF) as u8,
        (BLOCK_SIZE as u32 & 0xFF) as u8,
    ]);
    response
}

pub(crate) fn mode_page_supported(page_code: u8, subpage: u8) -> bool {
    // MODE SENSE page code 0x3F means "return all supported pages".
    // This device currently supports only Caching Page (08h), so 0x3F returns
    // the same single page payload as an explicit 08h request.
    (page_code == MODE_PAGE_CACHING || page_code == 0x3F) && (subpage == 0x00 || subpage == 0xFF)
}

pub(crate) fn build_caching_page() -> [u8; 20] {
    let mut page = [0u8; 20];
    page[0] = MODE_PAGE_CACHING;
    page[1] = 0x12;

    let write_cache_enabled = false;
    let read_cache_disabled = true;
    page[2] = (if write_cache_enabled { 0x04 } else { 0x00 }) | if read_cache_disabled { 0x01 } else { 0x00 };

    page
}

pub(crate) fn build_mode_sense_6_response(write_protected: bool) -> [u8; 24] {
    let mut response = [0u8; 24];
    let caching_page = build_caching_page();

    response[0] = (response.len() - 1) as u8;
    response[1] = 0x00;
    response[2] = if write_protected { 0x80 } else { 0x00 };
    response[3] = 0x00;
    response[4..].copy_from_slice(&caching_page);

    response
}

pub(crate) fn build_mode_sense_10_response(write_protected: bool) -> [u8; 28] {
    let mut response = [0u8; 28];
    let caching_page = build_caching_page();

    let mode_data_length = (response.len() - 2) as u16;
    response[0..2].copy_from_slice(&mode_data_length.to_be_bytes());
    response[2] = 0x00;
    response[3] = if write_protected { 0x80 } else { 0x00 };
    response[4] = 0x00;
    response[5] = 0x00;
    response[6] = 0x00;
    response[7] = 0x00;
    response[8..].copy_from_slice(&caching_page);

    response
}
