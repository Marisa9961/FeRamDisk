#![allow(dead_code)]

use std::collections::VecDeque;

use embassy_usb_driver::EndpointError;
use feramdisk_firmware::backend::simulated::DataEndpoint;

#[derive(Default, Debug)]
pub struct MockEndpoint {
    read_packets: VecDeque<Vec<u8>>,
    writes: Vec<Vec<u8>>,
    fail_next_read: Option<EndpointError>,
    fail_next_write: Option<EndpointError>,
    disabled_when_empty: bool,
}

impl MockEndpoint {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn disabled_when_empty(mut self, enabled: bool) -> Self {
        self.disabled_when_empty = enabled;
        self
    }

    pub fn queue_read_packet(&mut self, packet: impl Into<Vec<u8>>) {
        self.read_packets.push_back(packet.into());
    }

    pub fn queue_read_packets<I, P>(&mut self, packets: I)
    where
        I: IntoIterator<Item = P>,
        P: Into<Vec<u8>>,
    {
        for packet in packets {
            self.queue_read_packet(packet);
        }
    }

    pub fn set_fail_next_read(&mut self, error: EndpointError) {
        self.fail_next_read = Some(error);
    }

    pub fn set_fail_next_write(&mut self, error: EndpointError) {
        self.fail_next_write = Some(error);
    }

    pub fn writes(&self) -> &[Vec<u8>] {
        &self.writes
    }

    pub fn take_writes(&mut self) -> Vec<Vec<u8>> {
        core::mem::take(&mut self.writes)
    }
}

impl DataEndpoint for MockEndpoint {
    async fn endpoint_read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        if let Some(error) = self.fail_next_read.take() {
            return Err(error);
        }

        if let Some(packet) = self.read_packets.pop_front() {
            if packet.len() > buf.len() {
                return Err(EndpointError::BufferOverflow);
            }

            buf[..packet.len()].copy_from_slice(&packet);
            return Ok(packet.len());
        }

        if self.disabled_when_empty {
            return Err(EndpointError::Disabled);
        }

        core::future::pending::<Result<usize, EndpointError>>().await
    }

    async fn endpoint_write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        if let Some(error) = self.fail_next_write.take() {
            return Err(error);
        }

        self.writes.push(buf.to_vec());
        Ok(())
    }
}
