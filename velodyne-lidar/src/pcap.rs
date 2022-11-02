use crate::{common::*, packet::Packet};
use pcap::Capture;

const UDP_HEADER_SIZE: usize = 42;

pub struct PcapFileReader {
    capture: Capture<pcap::Offline>,
}

impl PcapFileReader {
    pub fn open<P>(file: P) -> Result<Self>
    where
        P: AsRef<Path>,
    {
        let mut capture = Capture::from_file(file)?;
        capture.filter("udp", true)?;
        Ok(Self { capture })
    }
}

impl Iterator for PcapFileReader {
    type Item = Packet;

    fn next(&mut self) -> Option<Self::Item> {
        Some(loop {
            let packet = self.capture.next().ok()?;
            let slice = &packet.data[UDP_HEADER_SIZE..];

            if let Ok(packet) = Packet::from_slice(slice) {
                break packet;
            }
        })
    }
}
