use super::{COLUMNS_PER_PACKET, ENCODER_TICKS_PER_REV, PIXELS_PER_COLUMN};
use chrono::NaiveDateTime;
use failure::Fallible;
#[cfg(feature = "enable-pcap")]
use pcap::Packet as PcapPacket;
use std::{
    fmt::{Debug, Formatter, Result as FormatResult},
    mem::size_of,
};

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct Pixel {
    /// The least significant 20 bits form distance in millimeters.
    pub raw_range: u32,
    pub reflectivity: u16,
    pub signal_photons: u16,
    pub noise_photons: u16,
    _pad: u16,
}

impl Pixel {
    /// Extract distance in millimeters from raw_range field.
    pub fn range(&self) -> u32 {
        self.raw_range & 0x000fffff
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct Column {
    /// Unix timestamp.
    pub timestamp: u64,
    /// The column index.
    pub measurement_id: u16,
    /// The frame index.
    pub frame_id: u16,
    /// Encoder count of rotation motor ranging from 0 to 90111 (inclusive).
    pub encoder_ticks: u32,
    /// Array of pixels.
    pub pixels: [Pixel; PIXELS_PER_COLUMN],
    /// Packet validility mark. True if value is 0xffffffff.
    pub raw_valid: u32,
}

impl Column {
    /// Construct [NaiveDateTime](chrono::NaiveDateTime) object from column timestamp.
    pub fn datetime(&self) -> NaiveDateTime {
        let secs = self.timestamp / 1_000_000_000;
        let nsecs = self.timestamp % 1_000_000_000;
        NaiveDateTime::from_timestamp(secs as i64, nsecs as u32)
    }

    /// Compute azimuth angle in radian from encoder ticks.
    pub fn azimuth_angle(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.encoder_ticks as f64 / ENCODER_TICKS_PER_REV as f64
    }

    /// Return if this packet is marked valid.
    pub fn valid(&self) -> bool {
        self.raw_valid == 0xffffffff
    }
}

impl Debug for Column {
    fn fmt(&self, formatter: &mut Formatter) -> FormatResult {
        let timestamp = self.timestamp;
        let measurement_id = self.measurement_id;
        let frame_id = self.frame_id;
        let encoder_ticks = self.encoder_ticks;
        let raw_valid = self.raw_valid;

        write!(
            formatter,
            "Column {{ \
             timestamp: {}, \
             measurement_id: {}, \
             frame_id: {}, \
             encoder_ticks: {}, \
             pixels: {:?}, \
             raw_valid: 0x{:x} \
             }}",
            timestamp, measurement_id, frame_id, encoder_ticks, &self.pixels as &[_], raw_valid
        )
    }
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct Packet {
    pub columns: [Column; COLUMNS_PER_PACKET],
}

impl Packet {
    /// Construct packet from [pcap's Packet](pcap::Packet).
    #[cfg(feature = "enable-pcap")]
    pub fn from_pcap(packet: &PcapPacket) -> Fallible<Packet> {
        let packet_header_size = 42;

        ensure!(
            packet.header.len as usize - packet_header_size == size_of::<Packet>(),
            "Input pcap packet is not a valid Ouster Lidar packet",
        );

        let mut buffer = Box::new([0u8; size_of::<Packet>()]);
        buffer.copy_from_slice(&packet.data[packet_header_size..]);
        Ok(Self::from_buffer(*buffer))
    }

    /// Construct packet from binary buffer.
    pub fn from_buffer(buffer: [u8; size_of::<Packet>()]) -> Packet {
        unsafe { std::mem::transmute::<_, Packet>(buffer) }
    }

    /// Construct packet from slice of bytes. Error if the slice size is not correct.
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
