#![allow(dead_code)]

use core::cmp::min;

use crate::usb::constants::{BOT_ACTION_STALL_IN, BOT_ACTION_STALL_OUT};
use crate::usb::core as msc;
use crate::usb::storage::BlockStorage;
use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_time::Timer;
use embassy_usb_driver::{self as driver, Endpoint, EndpointAddress, EndpointType, Event};

const USB_VID: u16 = 0x1209;
const USB_PID: u16 = 0x0001;
const CONTROL_MAX_PACKET_SIZE: u16 = 64;
const BULK_MAX_PACKET_SIZE: u16 = 64;
const CONFIGURATION_VALUE: u8 = 1;
const INTERFACE_NUMBER: u8 = 0;
const MANUFACTURER: &str = "FeRam";
const PRODUCT: &str = "FeRamDisk";
const SERIAL: &str = "0001";

#[derive(Copy, Clone)]
struct SetupPacket {
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
}

impl SetupPacket {
    fn parse(bytes: [u8; 8]) -> Self {
        Self {
            request_type: bytes[0],
            request: bytes[1],
            value: u16::from_le_bytes([bytes[2], bytes[3]]),
            index: u16::from_le_bytes([bytes[4], bytes[5]]),
            length: u16::from_le_bytes([bytes[6], bytes[7]]),
        }
    }

    fn recipient(&self) -> u8 {
        self.request_type & 0x1f
    }

    fn request_kind(&self) -> u8 {
        self.request_type & 0x60
    }

    fn is_in(&self) -> bool {
        self.request_type & 0x80 != 0
    }
}

pub async fn run<'d, D, S>(mut driver: D, storage: S)
where
    D: driver::Driver<'d>,
    D::Bus: driver::Bus,
    D::ControlPipe: driver::ControlPipe,
    S: BlockStorage,
{
    let bulk_out = driver
        .alloc_endpoint_out(EndpointType::Bulk, None, BULK_MAX_PACKET_SIZE, 0)
        .expect("bulk OUT endpoint allocation failed");
    let bulk_in = driver
        .alloc_endpoint_in(EndpointType::Bulk, None, BULK_MAX_PACKET_SIZE, 0)
        .expect("bulk IN endpoint allocation failed");

    let bulk_out_addr = bulk_out.info().addr;
    let bulk_in_addr = bulk_in.info().addr;
    let descriptors = UsbDescriptors::new(bulk_out_addr, bulk_in_addr);

    let (bus, control) = driver.start(CONTROL_MAX_PACKET_SIZE);
    let bot_control = msc::BotControl::new();

    join(
        control_task(bus, control, descriptors, bulk_out_addr, bulk_in_addr, &bot_control),
        msc::run(bulk_out, bulk_in, storage, &bot_control),
    )
    .await;
}

async fn control_task<B, C>(
    mut bus: B,
    mut control: C,
    descriptors: UsbDescriptors,
    bulk_out_addr: EndpointAddress,
    bulk_in_addr: EndpointAddress,
    bot_control: &msc::BotControl,
) where
    B: driver::Bus,
    C: driver::ControlPipe,
{
    let mut configuration: u8 = 0;

    loop {
        apply_pending_bot_actions(&mut bus, bulk_out_addr, bulk_in_addr, bot_control);

        match select(select(bus.poll(), control.setup()), Timer::after_millis(1)).await {
            Either::First(Either::First(event)) => match event {
                Event::Reset => {
                    configuration = 0;
                    bus.endpoint_set_enabled(bulk_out_addr, false);
                    bus.endpoint_set_enabled(bulk_in_addr, false);
                    bus.endpoint_set_stalled(bulk_out_addr, false);
                    bus.endpoint_set_stalled(bulk_in_addr, false);
                    bot_control.signal_bulk_reset();
                }
                Event::Suspend | Event::Resume | Event::PowerDetected | Event::PowerRemoved => {}
            },
            Either::First(Either::Second(setup)) => {
                handle_setup(
                    &mut bus,
                    &mut control,
                    &descriptors,
                    SetupPacket::parse(setup),
                    &mut configuration,
                    bulk_out_addr,
                    bulk_in_addr,
                    bot_control,
                )
                .await;
            }
            Either::Second(_) => {}
        }

        apply_pending_bot_actions(&mut bus, bulk_out_addr, bulk_in_addr, bot_control);
    }
}

fn apply_pending_bot_actions<B>(
    bus: &mut B,
    bulk_out_addr: EndpointAddress,
    bulk_in_addr: EndpointAddress,
    bot_control: &msc::BotControl,
) where
    B: driver::Bus,
{
    let actions = bot_control.take_bus_actions();

    if actions & BOT_ACTION_STALL_OUT != 0 {
        bus.endpoint_set_stalled(bulk_out_addr, true);
    }

    if actions & BOT_ACTION_STALL_IN != 0 {
        bus.endpoint_set_stalled(bulk_in_addr, true);
    }
}

fn reset_bulk_endpoints<B>(
    bus: &mut B,
    configuration: u8,
    bulk_out_addr: EndpointAddress,
    bulk_in_addr: EndpointAddress,
) where
    B: driver::Bus,
{
    let enabled = configuration == CONFIGURATION_VALUE;

    // BOT Reset requires clearing endpoint halt and re-synchronizing data toggle.
    // The generic embassy-usb-driver Bus trait has no explicit "reset toggle"
    // primitive, so we perform the most portable sequence: clear STALL +
    // disable/enable endpoints.
    //
    // Risk: if a backend does not reset data PID on this sequence internally,
    // host/device may still see PID mismatch until the next bus reset.
    bus.endpoint_set_stalled(bulk_out_addr, false);
    bus.endpoint_set_stalled(bulk_in_addr, false);

    bus.endpoint_set_enabled(bulk_out_addr, false);
    bus.endpoint_set_enabled(bulk_in_addr, false);

    if enabled {
        bus.endpoint_set_enabled(bulk_out_addr, true);
        bus.endpoint_set_enabled(bulk_in_addr, true);
    }
}

async fn handle_setup<B, C>(
    bus: &mut B,
    control: &mut C,
    descriptors: &UsbDescriptors,
    setup: SetupPacket,
    configuration: &mut u8,
    bulk_out_addr: EndpointAddress,
    bulk_in_addr: EndpointAddress,
    bot_control: &msc::BotControl,
) where
    B: driver::Bus,
    C: driver::ControlPipe,
{
    match setup.request_kind() {
        0x00 => handle_standard_request(
            bus,
            control,
            descriptors,
            setup,
            configuration,
            bulk_out_addr,
            bulk_in_addr,
        )
        .await,
        0x20 => {
            handle_class_request(
                bus,
                control,
                setup,
                *configuration,
                bulk_out_addr,
                bulk_in_addr,
                bot_control,
            )
            .await
        }
        _ => control.reject().await,
    }
}

async fn handle_standard_request<B, C>(
    bus: &mut B,
    control: &mut C,
    descriptors: &UsbDescriptors,
    setup: SetupPacket,
    configuration: &mut u8,
    bulk_out_addr: EndpointAddress,
    bulk_in_addr: EndpointAddress,
) where
    B: driver::Bus,
    C: driver::ControlPipe,
{
    match setup.request {
        0x06 if setup.is_in() => {
            let descriptor_type = (setup.value >> 8) as u8;
            let descriptor_index = setup.value as u8;
            match descriptor_type {
                0x01 => {
                    let descriptor = descriptors.device_descriptor();
                    send_control_in(control, &descriptor, setup.length).await;
                }
                0x02 => {
                    let descriptor = descriptors.configuration_descriptor();
                    send_control_in(control, &descriptor, setup.length).await;
                }
                0x03 => {
                    if let Some((descriptor, descriptor_length)) = descriptors.string_descriptor(descriptor_index) {
                        send_control_in(control, &descriptor[..descriptor_length], setup.length).await;
                    } else {
                        control.reject().await;
                    }
                }
                _ => control.reject().await,
            }
        }
        0x05 => {
            if setup.value <= 127 {
                control.accept_set_address(setup.value as u8).await;
            } else {
                control.reject().await;
            }
        }
        0x09 => {
            if setup.index == 0 && (setup.value == 0 || setup.value == 1) {
                *configuration = setup.value as u8;
                let enabled = *configuration == CONFIGURATION_VALUE;
                bus.endpoint_set_enabled(bulk_out_addr, enabled);
                bus.endpoint_set_enabled(bulk_in_addr, enabled);
                if !enabled {
                    bus.endpoint_set_stalled(bulk_out_addr, false);
                    bus.endpoint_set_stalled(bulk_in_addr, false);
                }
                control.accept().await;
            } else {
                control.reject().await;
            }
        }
        0x08 => {
            if setup.length >= 1 {
                let value = [*configuration];
                control.data_in(&value, true, true).await.ok();
            } else {
                control.reject().await;
            }
        }
        0x00 => {
            let mut status = [0u8; 2];
            match setup.recipient() {
                0x00 => {}
                0x01 => {}
                0x02 => {
                    let endpoint = EndpointAddress::from(setup.index as u8);
                    status[0] = bus.endpoint_is_stalled(endpoint) as u8;
                }
                _ => {
                    control.reject().await;
                    return;
                }
            }
            control.data_in(&status, true, true).await.ok();
        }
        0x01 => {
            if setup.recipient() == 0x02 {
                let endpoint = EndpointAddress::from(setup.index as u8);
                bus.endpoint_set_stalled(endpoint, false);
                control.accept().await;
            } else {
                control.reject().await;
            }
        }
        0x03 => {
            if setup.recipient() == 0x02 {
                let endpoint = EndpointAddress::from(setup.index as u8);
                bus.endpoint_set_stalled(endpoint, true);
                control.accept().await;
            } else {
                control.reject().await;
            }
        }
        0x0A => {
            if setup.recipient() == 0x01 && setup.index == INTERFACE_NUMBER as u16 && setup.length >= 1 {
                let value = [0u8];
                control.data_in(&value, true, true).await.ok();
            } else {
                control.reject().await;
            }
        }
        0x0B => {
            if setup.recipient() == 0x01 && setup.index == INTERFACE_NUMBER as u16 && setup.value == 0 {
                control.accept().await;
            } else {
                control.reject().await;
            }
        }
        _ => control.reject().await,
    }
}

async fn handle_class_request<B, C>(
    bus: &mut B,
    control: &mut C,
    setup: SetupPacket,
    configuration: u8,
    bulk_out_addr: EndpointAddress,
    bulk_in_addr: EndpointAddress,
    bot_control: &msc::BotControl,
)
where
    B: driver::Bus,
    C: driver::ControlPipe,
{
    match setup.request {
        0xFE if setup.request_type == 0xA1 && setup.index == INTERFACE_NUMBER as u16 && setup.length >= 1 => {
            let value = [0u8];
            control.data_in(&value, true, true).await.ok();
        }
        0xFF
            if setup.request_type == 0x21
                && setup.index == INTERFACE_NUMBER as u16
                && setup.value == 0
                && setup.length == 0 =>
        {
            control.accept().await;
            bus.endpoint_set_stalled(bulk_out_addr, false);
            bus.endpoint_set_stalled(bulk_in_addr, false);
            reset_bulk_endpoints(bus, configuration, bulk_out_addr, bulk_in_addr);
            bot_control.signal_bulk_reset();
        }
        _ => control.reject().await,
    }
}

async fn send_control_in<C>(control: &mut C, data: &[u8], requested_length: u16)
where
    C: driver::ControlPipe,
{
    let length = min(data.len(), requested_length as usize);
    if length == 0 {
        control.reject().await;
    } else {
        control.data_in(&data[..length], true, true).await.ok();
    }
}

struct UsbDescriptors {
    bulk_out: u8,
    bulk_in: u8,
}

impl UsbDescriptors {
    fn new(bulk_out: EndpointAddress, bulk_in: EndpointAddress) -> Self {
        Self {
            bulk_out: bulk_out.into(),
            bulk_in: bulk_in.into(),
        }
    }

    fn device_descriptor(&self) -> [u8; 18] {
        [
            18,
            0x01,
            0x00,
            0x02,
            0x00,
            0x00,
            0x00,
            0x40,
            (USB_VID & 0xFF) as u8,
            (USB_VID >> 8) as u8,
            (USB_PID & 0xFF) as u8,
            (USB_PID >> 8) as u8,
            0x00,
            0x01,
            0x01,
            0x02,
            0x03,
            0x01,
        ]
    }

    fn configuration_descriptor(&self) -> [u8; 32] {
        let total_length: u16 = 32;
        [
            9,
            0x02,
            (total_length & 0xFF) as u8,
            (total_length >> 8) as u8,
            0x01,
            0x01,
            0x00,
            0x80,
            0x32,
            9,
            0x04,
            INTERFACE_NUMBER,
            0x00,
            0x02,
            0x08,
            0x06,
            0x50,
            0x00,
            7,
            0x05,
            self.bulk_out,
            0x02,
            0x40,
            0x00,
            0x00,
            7,
            0x05,
            self.bulk_in,
            0x02,
            0x40,
            0x00,
            0x00,
        ]
    }

    fn string_descriptor(&self, index: u8) -> Option<([u8; 64], usize)> {
        match index {
            0 => {
                let mut descriptor = [0u8; 64];
                descriptor[0] = 0x04;
                descriptor[1] = 0x03;
                descriptor[2] = 0x09;
                descriptor[3] = 0x04;
                Some((descriptor, 4))
            }
            1 => Some(build_string_descriptor(MANUFACTURER)),
            2 => Some(build_string_descriptor(PRODUCT)),
            3 => Some(build_string_descriptor(SERIAL)),
            _ => None,
        }
    }
}

fn build_string_descriptor(text: &str) -> ([u8; 64], usize) {
    let mut descriptor = [0u8; 64];
    descriptor[1] = 0x03;

    let mut index = 2usize;
    for unit in text.encode_utf16() {
        if index + 2 > descriptor.len() {
            break;
        }
        let bytes = unit.to_le_bytes();
        descriptor[index] = bytes[0];
        descriptor[index + 1] = bytes[1];
        index += 2;
    }

    descriptor[0] = index as u8;
    (descriptor, index)
}
