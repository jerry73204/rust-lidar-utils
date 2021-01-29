//! Provides `C-packed` structs for Velodyne data packets.

use super::consts::{AZIMUTH_COUNT_PER_REV, BLOCKS_PER_PACKET, CHANNELS_PER_BLOCK};

use crate::common::*;

pub use data_packet::*;

mod data_packet {
    use super::*;

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
        pub fn azimuth_angle_radian(&self) -> f64 {
            2.0 * std::f64::consts::PI * self.azimuth_count as f64
                / (AZIMUTH_COUNT_PER_REV - 1) as f64
        }

        pub fn azimuth_angle_degree(&self) -> f64 {
            360.0 * self.azimuth_count as f64 / (AZIMUTH_COUNT_PER_REV - 1) as f64
        }

        pub fn azimuth_angle(&self) -> uom::si::f64::Angle {
            uom::si::f64::Angle::new::<uom::si::angle::radian>(self.azimuth_angle_radian())
        }
    }

    /// Represents the data packet from Velodyne sensor.
    #[repr(C, packed)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct DataPacket {
        /// Sensor data.
        pub blocks: [Block; BLOCKS_PER_PACKET],
        /// Timestamp in microseconds.
        pub timestamp: u32,
        /// Indicates single return mode or dual return mode.
        pub return_mode: ReturnMode,
        /// Sensor model.
        pub product_id: ProductID,
    }

    impl DataPacket {
        /// Construct packet from [pcap::Packet](pcap::Packet).
        #[cfg(feature = "pcap")]
        pub fn from_pcap(packet: &PcapPacket) -> Result<DataPacket> {
            let packet_header_size = 42;

            ensure!(
                packet.header.len as usize - packet_header_size == mem::size_of::<Self>(),
                "Input pcap packet is not a valid Velodyne Lidar packet",
            );

            let mut buffer = Box::new([0u8; mem::size_of::<Self>()]);
            buffer.copy_from_slice(&packet.data[packet_header_size..]);
            Ok(Self::from_buffer(*buffer))
        }

        /// Construct packet from binary buffer.
        pub fn from_buffer(buffer: [u8; mem::size_of::<Self>()]) -> Self {
            unsafe { mem::transmute::<_, Self>(buffer) }
        }

        /// Construct packet from slice of bytes. Fail if the slice size is not correct.
        pub fn from_slice<'a>(buffer: &'a [u8]) -> Result<&'a Self> {
            ensure!(
                buffer.len() == mem::size_of::<Self>(),
                "Requre the slice length to be {}, but get {}",
                mem::size_of::<Self>(),
                buffer.len(),
            );
            let packet = unsafe { &*(buffer.as_ptr() as *const Self) };
            Ok(packet)
        }

        /// Construct [NaiveDateTime](chrono::NaiveDateTime) from packet timestamp.
        pub fn datetime(&self) -> NaiveDateTime {
            let secs = self.timestamp / 1_000_000;
            let nsecs = (self.timestamp % 1_000_000) * 1000;
            NaiveDateTime::from_timestamp(secs as i64, nsecs as u32)
        }

        pub fn time(&self) -> Time {
            Time::new::<microsecond>(self.timestamp as f64)
        }
    }
}

mod position_packet {
    use super::*;

    #[repr(C, packed)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct PositionPacket {
        pub reserved_head: [u8; 187],
        pub top_board_temperature: u8,
        pub bottom_board_temperature: u8,
        pub last_adc_calibration_temperature: u8,
        pub last_adc_calibration_temperature_change: u16,
        pub seconds_since_last_adc_calibration: u32,
        pub last_adc_calibration_reason: LastAdcCalibrationReason,
        pub adc_calibration_bitmask: u8,
        pub toh: u32,
        pub pps_status: PpsStatus,
        pub thermal_status: ThermalStatus,
        pub last_shutdown_temperature: u8,
        pub temperature_of_unit_at_power_up: u8,
        pub nmea: [u8; 128],
        pub reserved_tail: [u8; 178],
    }

    impl PositionPacket {
        /// Construct packet from [pcap::Packet](pcap::Packet).
        #[cfg(feature = "pcap")]
        pub fn from_pcap(packet: &pcap::Packet) -> Result<Self> {
            let packet_header_size = 42;

            ensure!(
                packet.header.len as usize - packet_header_size == mem::size_of::<Self>(),
                "Input pcap packet is not a valid Velodyne Lidar packet",
            );

            let mut buffer = Box::new([0u8; mem::size_of::<Self>()]);
            buffer.copy_from_slice(&packet.data[packet_header_size..]);
            Ok(Self::from_buffer(*buffer))
        }

        /// Construct packet from binary buffer.
        pub fn from_buffer(buffer: [u8; mem::size_of::<Self>()]) -> Self {
            unsafe { mem::transmute::<_, Self>(buffer) }
        }

        /// Construct packet from slice of bytes. Fail if the slice size is not correct.
        pub fn from_slice<'a>(buffer: &'a [u8]) -> Result<&'a Self> {
            ensure!(
                buffer.len() == mem::size_of::<Self>(),
                "Requre the slice length to be {}, but get {}",
                mem::size_of::<Self>(),
                buffer.len(),
            );
            let packet = unsafe { &*(buffer.as_ptr() as *const Self) };
            Ok(packet)
        }

        pub fn calibration_in_progress(&self) -> bool {
            self.adc_calibration_bitmask & 0b0001 != 0
        }

        pub fn meet_delta_temperature(&self) -> bool {
            self.adc_calibration_bitmask & 0b0010 != 0
        }

        pub fn meet_periodic_elapsed_time_limit(&self) -> bool {
            self.adc_calibration_bitmask & 0b0100 != 0
        }
    }

    #[repr(u8)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum LastAdcCalibrationReason {
        NoCalibration = 0,
        PowerOn = 1,
        Manual = 2,
        DeltaTemperature = 3,
        Periodic = 4,
    }

    #[repr(u8)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum PpsStatus {
        Abscent = 0,
        Synchronizing = 1,
        Locked = 2,
        Error = 3,
    }

    #[repr(u8)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum ThermalStatus {
        Ok = 0,
        ThermalShutdown = 1,
    }
}
