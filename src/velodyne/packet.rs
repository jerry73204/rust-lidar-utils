use super::consts::{AZIMUTH_COUNT_PER_REV, CHANNEL_PER_FIRING, FIRING_PER_PACKET};

use failure::{ensure, Fallible};
#[cfg(feature = "enable-pcap")]
use pcap::Packet as PcapPacket;
use std::mem::size_of;

#[repr(u16)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BlockIdentifier {
    Block0To31 = 0xeeff,
    Block32To63 = 0xddff,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ReturnMode {
    Strongest = 0x37,
    LastReturn = 0x38,
    DualReturn = 0x39,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProductID {
    HDL32E = 0x21,
    VLP16 = 0x22,
    PuckLite = 0x23,
    PuckHiRes = 0x24,
    VLP32C = 0x28,
    Velarray = 0x31,
    VLS128 = 0xa1,
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaserReturn {
    /// The raw distance of laser return. The distance in meter is the raw distance times 0.002.
    pub distance: u16,
    /// The intensity of laser return.
    pub intensity: u8,
}

impl LaserReturn {
    /// Compute distance in meters from sensor data.
    pub fn meter_distance(&self) -> f64 {
        self.distance as f64 * 0.002
    }

    /// Compute distance in millimetres from sensor data.
    pub fn mm_distance(&self) -> f64 {
        self.distance as f64 * 2.0
    }
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Firing {
    /// Valid if either 0xeeff or 0xddff, corresponding to range from 0 to 31, or range from 32 to 63.
    pub block_identifier: BlockIdentifier,
    /// Encoder count of rotation motor ranging from 0 to 36000 (inclusive).
    pub azimuth_count: u16,
    /// Array of laser returns.
    pub sequence_former: [LaserReturn; CHANNEL_PER_FIRING],
    /// Array of laser returns.
    pub sequence_latter: [LaserReturn; CHANNEL_PER_FIRING],
}

impl Firing {
    /// Compute azimuth angle in radian from encoder ticks.
    pub fn azimuth_angle(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.azimuth_count as f64 / (AZIMUTH_COUNT_PER_REV - 1) as f64
    }
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Packet {
    /// Sensor data.
    pub firings: [Firing; FIRING_PER_PACKET],
    /// Timestamp in microseconds.
    pub timestamp: u32,
    /// Indicates single return mode or dual return mode.
    pub return_mode: ReturnMode,
    /// Sensor model.
    pub product_id: ProductID,
}

impl Packet {
    /// Construct packet from [pcap::Packet](pcap::Packet).
    #[cfg(feature = "enable-pcap")]
    pub fn from_pcap(packet: &PcapPacket) -> Fallible<Packet> {
        let packet_header_size = 42;

        ensure!(
            packet.header.len as usize - packet_header_size == size_of::<Packet>(),
            "Input pcap packet is not a valid Velodyne Lidar packet",
        );

        let mut buffer = Box::new([0u8; size_of::<Packet>()]);
        buffer.copy_from_slice(&packet.data[packet_header_size..]);
        Ok(Self::from_buffer(*buffer))
    }

    /// Construct packet from binary buffer.
    pub fn from_buffer(buffer: [u8; size_of::<Packet>()]) -> Packet {
        unsafe { std::mem::transmute::<_, Packet>(buffer) }
    }

    /// Construct packet from slice of bytes. Fail if the slice size is not correct.
    pub fn from_slice<'a>(buffer: &'a [u8]) -> Fallible<&'a Packet> {
        ensure!(
            buffer.len() == size_of::<Packet>(),
            "Requre the slice length to be {}, but get {}",
            size_of::<Packet>(),
            buffer.len(),
        );
        let packet = unsafe { &*(buffer.as_ptr() as *const Packet) };
        Ok(packet)
    }
}
