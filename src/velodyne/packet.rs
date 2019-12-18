//! Provides `C-packed` structs for Velodyne data packets.

use super::consts::{AZIMUTH_COUNT_PER_REV, BLOCKS_PER_PACKET, CHANNELS_PER_BLOCK};

use chrono::NaiveDateTime;
use failure::{ensure, Fallible};
#[cfg(feature = "enable-pcap")]
use pcap::Packet as PcapPacket;
use std::mem::size_of;

/// Represents the block index in range from 0 to 31, or from 32 to 63.
#[repr(u16)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockIdentifier {
    Block0To31 = 0xeeff,
    Block32To63 = 0xddff,
}

/// Represents the way the sensor measures the laser signal.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReturnMode {
    StrongestReturn = 0x37,
    LastReturn = 0x38,
    DualReturn = 0x39,
}

/// Represents the hardware model.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProductID {
    HDL32E = 0x21,
    VLP16 = 0x22,
    PuckLite = 0x23,
    PuckHiRes = 0x24,
    VLP32C = 0x28,
    Velarray = 0x31,
    VLS128 = 0xa1,
}

/// Represents a point of measurement.
#[repr(C, packed)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Channel {
    /// The raw distance of laser return.
    pub distance: u16,
    /// The intensity of laser return.
    pub intensity: u8,
}

impl Channel {
    /// Compute distance in meters by raw distance times 0.002.
    pub fn meter_distance(&self) -> f64 {
        self.distance as f64 * 0.002
    }

    /// Compute distance in millimetres by raw distance times 2.
    pub fn mm_distance(&self) -> u32 {
        self.distance as u32 * 2
    }

    pub fn uom_distance(&self) -> uom::si::u32::Length {
        uom::si::u32::Length::new::<uom::si::length::millimeter>(self.mm_distance())
    }
}

/// Represents a sequence of measurements with meta data.
#[repr(C, packed)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Block {
    /// Represents the block that the firing belongs to.
    pub block_identifier: BlockIdentifier,
    /// Encoder count of rotation motor ranging from 0 to 36000 (inclusive).
    pub azimuth_count: u16,
    /// Array of channels.
    pub channels: [Channel; CHANNELS_PER_BLOCK],
}

impl Block {
    /// Compute azimuth angle in radian from encoder ticks.
    pub fn azimuth_angle(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.azimuth_count as f64 / (AZIMUTH_COUNT_PER_REV - 1) as f64
    }

    pub fn uom_azimuth_angle(&self) -> uom::si::f64::Angle {
        uom::si::f64::Angle::new::<uom::si::angle::radian>(self.azimuth_angle())
    }
}

/// Represents the data packet from Velodyne sensor.
#[repr(C, packed)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Packet {
    /// Sensor data.
    pub blocks: [Block; BLOCKS_PER_PACKET],
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

    /// Construct [NaiveDateTime](chrono::NaiveDateTime) from packet timestamp.
    pub fn datetime(&self) -> NaiveDateTime {
        let secs = self.timestamp / 1_000_000;
        let nsecs = (self.timestamp % 1_000_000) * 1000;
        NaiveDateTime::from_timestamp(secs as i64, nsecs as u32)
    }

    pub fn uom_time(&self) -> uom::si::u32::Time {
        uom::si::u32::Time::new::<uom::si::time::microsecond>(self.timestamp)
    }
}
