//! Provides a set of _C-packed_ structs for Ouster packets.

use super::consts::{COLUMNS_PER_PACKET, ENCODER_TICKS_PER_REV, PIXELS_PER_COLUMN};
use crate::common::*;

/// Represents a point of signal measurement.
#[repr(C, packed)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pixel {
    /// The least significant 20 bits form distance in millimeters.
    pub raw_distance: u32,
    pub reflectivity: u16,
    pub signal_photons: u16,
    pub noise_photons: u16,
    _pad: u16,
}

impl Pixel {
    /// Extract distance in millimeters from raw_distance field.
    pub fn distance_millimeter(&self) -> u32 {
        self.raw_distance & 0x000fffff
    }

    pub fn distance(&self) -> Length {
        Length::new::<millimeter>(self.distance_millimeter() as f64)
    }
}

/// Represents a list of [Pixel]s along with meta data.
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct Column {
    /// Unix timestamp in nanoseconds.
    pub timestamp: u64,
    /// The column index.
    pub measurement_id: u16,
    /// The frame index.
    pub frame_id: u16,
    /// Clockwise encoder count of rotation motor ranging from 0 to [ENCODER_TICKS_PER_REV] (exclusive).
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

    pub fn time(&self) -> Time {
        Time::new::<nanosecond>(self.timestamp as f64)
    }

    /// Compute azimuth angle in degrees from encoder ticks.
    pub fn azimuth_angle_degrees(&self) -> f64 {
        360.0 * self.encoder_ticks as f64 / ENCODER_TICKS_PER_REV as f64
    }

    /// Compute azimuth angle in radians from encoder ticks.
    pub fn azimuth_angle_radians(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.encoder_ticks as f64 / ENCODER_TICKS_PER_REV as f64
    }

    pub fn azimuth_angle(&self) -> Angle {
        Angle::new::<radian>(self.azimuth_angle_radians())
    }

    /// Return if this packet is marked valid.
    pub fn valid(&self) -> bool {
        self.raw_valid == 0xffffffff
    }
}

impl PartialEq for Column {
    fn eq(&self, other: &Column) -> bool {
        let Self {
            timestamp: timestamp_lhs,
            measurement_id: measurement_id_lhs,
            frame_id: frame_id_lhs,
            encoder_ticks: encoder_ticks_lhs,
            raw_valid: raw_valid_lhs,
            pixels: pixels_lhs,
        } = *self;

        let Self {
            timestamp: timestamp_rhs,
            measurement_id: measurement_id_rhs,
            frame_id: frame_id_rhs,
            encoder_ticks: encoder_ticks_rhs,
            raw_valid: raw_valid_rhs,
            pixels: pixels_rhs,
        } = *other;

        timestamp_lhs == timestamp_rhs
            && measurement_id_lhs == measurement_id_rhs
            && frame_id_lhs == frame_id_rhs
            && encoder_ticks_lhs == encoder_ticks_rhs
            && raw_valid_lhs == raw_valid_rhs
            && pixels_lhs
                .iter()
                .zip(pixels_rhs.iter())
                .all(|(lval, rval)| lval == rval)
    }
}

impl Eq for Column {}

impl Debug for Column {
    fn fmt(&self, formatter: &mut Formatter) -> fmt::Result {
        let Self {
            timestamp,
            measurement_id,
            frame_id,
            encoder_ticks,
            raw_valid,
            pixels,
        } = *self;

        formatter
            .debug_struct(std::any::type_name::<Self>())
            .field("timestamp", &timestamp)
            .field("measurement_id", &measurement_id)
            .field("frame_id", &frame_id)
            .field("encoder_ticks", &encoder_ticks)
            .field("raw_valid", &raw_valid)
            .field("pixels", &pixels)
            .finish()
    }
}

/// Represents a data packet from Ouster sensor.
#[repr(C, packed)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Packet {
    pub columns: [Column; COLUMNS_PER_PACKET],
}

impl Packet {
    /// Construct packet from [pcap's Packet](pcap::Packet).
    #[cfg(feature = "pcap")]
    pub fn from_pcap(packet: &PcapPacket) -> Result<Packet> {
        let packet_header_size = 42;

        ensure!(
            packet.header.len as usize - packet_header_size == mem::size_of::<Packet>(),
            "Input pcap packet is not a valid Ouster Lidar packet",
        );

        let mut buffer = Box::new([0u8; mem::size_of::<Packet>()]);
        buffer.copy_from_slice(&packet.data[packet_header_size..]);
        Ok(Self::from_buffer(*buffer))
    }

    /// Construct packet from binary buffer.
    pub fn from_buffer(buffer: [u8; mem::size_of::<Packet>()]) -> Packet {
        unsafe { std::mem::transmute::<_, Packet>(buffer) }
    }

    /// Construct packet from slice of bytes. Error if the slice size is not correct.
    pub fn from_slice<'a>(buffer: &'a [u8]) -> Result<&'a Packet> {
        ensure!(
            buffer.len() == mem::size_of::<Packet>(),
            "Requre the slice length to be {}, but get {}",
            mem::size_of::<Packet>(),
            buffer.len(),
        );
        let packet = unsafe { &*(buffer.as_ptr() as *const Packet) };
        Ok(packet)
    }
}

impl AsRef<Packet> for Packet {
    fn as_ref(&self) -> &Packet {
        &self
    }
}
