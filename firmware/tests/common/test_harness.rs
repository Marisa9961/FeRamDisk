#![allow(dead_code)]

use std::collections::VecDeque;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use embassy_time::{with_timeout, Duration};
use embassy_usb_driver::{
    Endpoint, EndpointAddress, EndpointError, EndpointIn, EndpointInfo, EndpointOut,
    EndpointType,
};
use feramdisk_firmware::backend::simulated::{RamStorage, SharedRamStorage};
use feramdisk_firmware::usb::core::{self, BotControl};

#[derive(Clone)]
pub struct BotOutEndpoint {
    info: EndpointInfo,
    packets: Arc<Mutex<VecDeque<Vec<u8>>>>,
    read_count: Arc<AtomicUsize>,
    reset_hook: Arc<Mutex<Option<ResetHook>>>,
}

#[derive(Clone)]
struct ResetHook {
    trigger_read_count: usize,
    bot_control: Arc<BotControl>,
}

impl BotOutEndpoint {
    pub fn new() -> Self {
        Self {
            info: EndpointInfo {
                addr: EndpointAddress::from(0x01),
                ep_type: EndpointType::Bulk,
                max_packet_size: 64,
                interval_ms: 0,
            },
            packets: Arc::new(Mutex::new(VecDeque::new())),
            read_count: Arc::new(AtomicUsize::new(0)),
            reset_hook: Arc::new(Mutex::new(None)),
        }
    }

    pub fn queue_packet(&self, packet: impl Into<Vec<u8>>) {
        self.packets
            .lock()
            .expect("bot out queue poisoned")
            .push_back(packet.into());
    }

    pub fn read_count(&self) -> usize {
        self.read_count.load(Ordering::Acquire)
    }

    pub fn signal_reset_on_read(&self, trigger_read_count: usize, bot_control: Arc<BotControl>) {
        *self.reset_hook.lock().expect("reset hook poisoned") = Some(ResetHook {
            trigger_read_count,
            bot_control,
        });
    }
}

impl Endpoint for BotOutEndpoint {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {}
}

impl EndpointOut for BotOutEndpoint {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let read_count = self.read_count.fetch_add(1, Ordering::AcqRel) + 1;

        let hook = self.reset_hook.lock().expect("reset hook poisoned").clone();
        if let Some(hook) = hook {
            if read_count >= hook.trigger_read_count {
                hook.bot_control.signal_bulk_reset();
                *self.reset_hook.lock().expect("reset hook poisoned") = None;
            }
        }

        if let Some(packet) = self
            .packets
            .lock()
            .expect("bot out queue poisoned")
            .pop_front()
        {
            if packet.len() > buf.len() {
                return Err(EndpointError::BufferOverflow);
            }

            buf[..packet.len()].copy_from_slice(&packet);
            return Ok(packet.len());
        }

        ::core::future::pending::<Result<usize, EndpointError>>().await
    }
}

#[derive(Clone)]
pub struct BotInEndpoint {
    info: EndpointInfo,
    packets: Arc<Mutex<Vec<Vec<u8>>>>,
}

impl BotInEndpoint {
    pub fn new() -> Self {
        Self {
            info: EndpointInfo {
                addr: EndpointAddress::from(0x81),
                ep_type: EndpointType::Bulk,
                max_packet_size: 64,
                interval_ms: 0,
            },
            packets: Arc::new(Mutex::new(Vec::new())),
        }
    }

    pub fn packets(&self) -> Vec<Vec<u8>> {
        self.packets
            .lock()
            .expect("bot in queue poisoned")
            .clone()
    }
}

impl Endpoint for BotInEndpoint {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {}
}

impl EndpointIn for BotInEndpoint {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        self.packets
            .lock()
            .expect("bot in queue poisoned")
            .push(buf.to_vec());
        Ok(())
    }
}

pub struct BotTestHarness {
    pub bot_control: Arc<BotControl>,
    pub out_ep: BotOutEndpoint,
    pub in_ep: BotInEndpoint,
    pub storage: SharedRamStorage,
}

impl BotTestHarness {
    pub fn new(block_count: u32) -> Self {
        Self {
            bot_control: Arc::new(BotControl::new()),
            out_ep: BotOutEndpoint::new(),
            in_ep: BotInEndpoint::new(),
            storage: SharedRamStorage::new(block_count),
        }
    }

    pub fn queue_out_packet(&self, packet: impl Into<Vec<u8>>) {
        self.out_ep.queue_packet(packet);
    }

    pub fn run_for(&self, duration_ms: u64) {
        let out_ep = self.out_ep.clone();
        let in_ep = self.in_ep.clone();
        let storage = self.storage.clone();

        let _ = super::run_async(async {
            let _ = with_timeout(
                Duration::from_millis(duration_ms),
                core::run(out_ep, in_ep, storage, self.bot_control.as_ref()),
            )
            .await;
        });
    }

    pub fn in_packets(&self) -> Vec<Vec<u8>> {
        self.in_ep.packets()
    }

    pub fn take_bus_actions(&self) -> u8 {
        self.bot_control.take_bus_actions()
    }

    pub fn storage_inner(&self) -> Arc<Mutex<RamStorage>> {
        self.storage.inner()
    }
}
