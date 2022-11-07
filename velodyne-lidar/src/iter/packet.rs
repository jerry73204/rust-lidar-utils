use crate::Packet;
use anyhow::Result;
use pcap::{Capture, Device};
use std::{iter, path::Path};

const UDP_HEADER_SIZE: usize = 42;

pub fn from_capture<A>(
    mut capture: Capture<A>,
) -> Result<impl Iterator<Item = Result<Packet, pcap::Error>> + Send, pcap::Error>
where
    A: pcap::Activated,
{
    capture.filter("udp", true)?;
    let iter = iter::from_fn(move || {
        Some(loop {
            let packet = match capture.next_packet() {
                Ok(packet) => packet,
                Err(pcap::Error::NoMorePackets) => return None,
                Err(err) => break Err(err),
            };
            let Some(slice) =  packet.data.get(UDP_HEADER_SIZE..) else { continue };
            let Ok(packet) = Packet::from_slice(slice) else { continue };
            break Ok(packet);
        })
    });
    Ok(iter)
}

pub fn from_file<P>(
    path: P,
) -> Result<impl Iterator<Item = Result<Packet, pcap::Error>> + Send, pcap::Error>
where
    P: AsRef<Path>,
{
    let capture: Capture<pcap::Offline> = Capture::from_file(path)?;
    from_capture(capture)
}

pub fn from_device<D>(
    device: D,
) -> Result<impl Iterator<Item = Result<Packet, pcap::Error>> + Send, pcap::Error>
where
    D: Into<Device>,
{
    let capture: Capture<pcap::Inactive> = Capture::from_device(device)?;
    let capture = capture.open()?;
    from_capture(capture)
}
