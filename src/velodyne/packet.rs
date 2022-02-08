//! Provides `C-packed` structs for Velodyne data packets.

use super::consts::{AZIMUTH_COUNT_PER_REV, BLOCKS_PER_PACKET, CHANNELS_PER_BLOCK, FIRING_PERIOD};
use crate::{common::*, utils::AngleExt as _};
use std::f64::consts::PI;

pub use data_packet::*;
mod data_packet {
    use super::*;

    /// Represents the block index in range from 0 to 31, or from 32 to 63.
    #[repr(u16)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum BlockIdentifier {
        Block0To31 = 0xeeff,
        Block32To63 = 0xddff,
    }

    /// Represents the way the sensor measures the laser signal.
    #[repr(u8)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum ReturnMode {
        StrongestReturn = 0x37,
        LastReturn = 0x38,
        DualReturn = 0x39,
    }

    /// Represents the hardware model.
    #[repr(u8)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct Channel {
        /// The raw distance of laser return.
        pub distance: u16,
        /// The intensity of laser return.
        pub intensity: u8,
    }

    /// Represents a sequence of measurements with meta data.
    #[repr(C, packed)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct Block {
        /// Represents the block that the firing belongs to.
        pub block_identifier: BlockIdentifier,
        /// Encoder count of rotation motor ranging from 0 to 36000 (inclusive).
        pub azimuth_count: u16,
        /// Array of channels.
        pub channels: [Channel; CHANNELS_PER_BLOCK],
    }

    impl Block {
        pub fn azimuth_radians(&self) -> f64 {
            2.0 * PI * self.azimuth_count as f64 / (AZIMUTH_COUNT_PER_REV - 1) as f64
        }

        pub fn azimuth_degrees(&self) -> f64 {
            360.0 * self.azimuth_count as f64 / (AZIMUTH_COUNT_PER_REV - 1) as f64
        }

        pub fn azimuth(&self) -> Angle {
            Angle::from_radians(self.azimuth_radians())
        }
    }

    /// Represents the data packet from Velodyne sensor.
    #[repr(C, packed)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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
        pub fn from_pcap(packet: &pcap::Packet) -> Result<Self> {
            let packet_header_size = 42;

            let body_size = packet.header.len as usize - packet_header_size;
            ensure!(
                body_size == mem::size_of::<Self>(),
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
        pub fn from_slice(buffer: &[u8]) -> Result<&Self> {
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

        pub fn time(&self) -> Duration {
            Duration::from_micros(self.timestamp as u64)
        }

        pub fn single_16_firings(
            &self,
        ) -> Result<impl Iterator<Item = SingleFiring16<'_>> + Clone> {
            use ProductID as P;
            use ReturnMode as R;

            ensure!(
                [R::StrongestReturn, R::LastReturn].contains(&self.return_mode),
                "expect strongest or last return mode, but get dual mode"
            );
            ensure!([P::VLP16, P::PuckLite, P::PuckHiRes].contains(&self.product_id));

            let block_period = FIRING_PERIOD.mul_f64(2.0);
            let times = iter::successors(Some(self.time()), move |prev| Some(*prev + block_period));
            let firing_azimuths = {
                let block_azimuths: Vec<_> =
                    self.blocks.iter().map(|block| block.azimuth()).collect();
                let block_azimuth_diffs: Vec<_> = block_azimuths
                    .iter()
                    .cloned()
                    .tuple_windows()
                    .map(|(curr, next)| (next - curr).wrap_to_2pi())
                    .collect();
                let last_block_azimuth_diff = *block_azimuth_diffs.last().unwrap();

                izip!(
                    block_azimuths,
                    chain!(block_azimuth_diffs, [last_block_azimuth_diff])
                )
                .map(|(block_azimuth, block_azimuth_diff)| {
                    let mid_azimuth = block_azimuth + block_azimuth_diff / 2.0;
                    let last_azimuth = block_azimuth + block_azimuth_diff;
                    [block_azimuth..mid_azimuth, mid_azimuth..last_azimuth]
                })
            };

            let iter = izip!(times, firing_azimuths, &self.blocks).flat_map(
                move |(block_time, [former_azimuth, latter_azimuth], block)| {
                    let former_time = block_time;
                    let latter_time = former_time + FIRING_PERIOD;

                    let (former_channels, latter_channels) = block.channels.split_at(16);

                    let former = SingleFiring16 {
                        time: former_time,
                        azimuth_range: former_azimuth,
                        block,
                        channels: former_channels
                            .try_into()
                            .unwrap_or_else(|_| unreachable!()),
                    };
                    let latter = SingleFiring16 {
                        time: latter_time,
                        azimuth_range: latter_azimuth,
                        block,
                        channels: latter_channels
                            .try_into()
                            .unwrap_or_else(|_| unreachable!()),
                    };

                    [former, latter]
                },
            );

            Ok(iter)
        }

        pub fn dual_16_firings(&self) -> Result<impl Iterator<Item = DualFiring16<'_>> + Clone> {
            use ProductID as P;
            use ReturnMode as R;

            ensure!(
                self.return_mode == R::DualReturn,
                "expect dual mode, but get {:?}",
                self.return_mode
            );
            ensure!([P::VLP16, P::PuckLite, P::PuckHiRes].contains(&self.product_id));

            let block_period = FIRING_PERIOD.mul_f64(2.0);
            let times = iter::successors(Some(self.time()), move |prev| Some(*prev + block_period));
            let firing_azimuths = {
                let block_azimuths: Vec<_> = self
                    .blocks
                    .iter()
                    .step_by(2)
                    .map(|block| block.azimuth())
                    .collect();
                let block_azimuth_diffs: Vec<_> = block_azimuths
                    .iter()
                    .cloned()
                    .tuple_windows()
                    .map(|(curr, next)| (next - curr).wrap_to_2pi())
                    .collect();
                let last_block_azimuth_diff = *block_azimuth_diffs.last().unwrap();

                izip!(
                    block_azimuths,
                    chain!(block_azimuth_diffs, [last_block_azimuth_diff])
                )
                .map(|(block_azimuth, block_azimuth_diff)| {
                    let mid_azimuth = block_azimuth + block_azimuth_diff / 2.0;
                    let last_azimuth = block_azimuth + block_azimuth_diff;
                    [block_azimuth..mid_azimuth, mid_azimuth..last_azimuth]
                })
            };

            let firings = izip!(times, firing_azimuths, self.blocks.chunks(2)).flat_map(
                |(block_time, [former_azimuth, latter_azimuth], block_pair)| {
                    let [block_strongest, block_last] = match block_pair {
                        [first, second] => [first, second],
                        _ => unreachable!(),
                    };

                    let former_time = block_time;
                    let latter_time = former_time + FIRING_PERIOD;

                    let (former_strongest, latter_strongest) =
                        block_strongest.channels.split_at(16);
                    let (former_last, latter_last) = block_last.channels.split_at(16);

                    [
                        DualFiring16 {
                            time: former_time,
                            azimuth_range: former_azimuth,
                            block_strongest,
                            block_last,
                            channels_strongest: former_strongest
                                .try_into()
                                .unwrap_or_else(|_| unreachable!()),
                            channels_last: former_last
                                .try_into()
                                .unwrap_or_else(|_| unreachable!()),
                        },
                        DualFiring16 {
                            time: latter_time,
                            azimuth_range: latter_azimuth,
                            block_strongest,
                            block_last,
                            channels_strongest: latter_strongest
                                .try_into()
                                .unwrap_or_else(|_| unreachable!()),
                            channels_last: latter_last
                                .try_into()
                                .unwrap_or_else(|_| unreachable!()),
                        },
                    ]
                },
            );

            Ok(firings)
        }

        pub fn single_32_firings(
            &self,
        ) -> Result<impl Iterator<Item = SingleFiring32<'_>> + Clone> {
            use ProductID as P;
            use ReturnMode as R;

            ensure!(
                [R::StrongestReturn, R::LastReturn].contains(&self.return_mode),
                "expect strongest or last return mode, but get dual mode"
            );
            ensure!([P::HDL32E, P::VLP32C].contains(&self.product_id));

            let times =
                iter::successors(Some(self.time()), move |prev| Some(*prev + FIRING_PERIOD));
            let azimuths = {
                let block_azimuths: Vec<_> =
                    self.blocks.iter().map(|block| block.azimuth()).collect();
                let block_azimuth_diffs: Vec<_> = block_azimuths
                    .iter()
                    .cloned()
                    .tuple_windows()
                    .map(|(curr, next)| (next - curr).wrap_to_2pi())
                    .collect();
                let last_block_azimuth_diff = *block_azimuth_diffs.last().unwrap();

                izip!(
                    block_azimuths,
                    chain!(block_azimuth_diffs, [last_block_azimuth_diff])
                )
                .map(|(former_azimuth, azimuth_diff)| {
                    let latter_azimuth = former_azimuth + azimuth_diff;
                    former_azimuth..latter_azimuth
                })
            };

            let iter = izip!(times, azimuths, &self.blocks).map(
                move |(block_time, azimuth_range, block)| {
                    let former_time = block_time;
                    let latter_time = former_time + FIRING_PERIOD;

                    SingleFiring32 {
                        time: latter_time,
                        azimuth_range,
                        block,
                        channels: &block.channels,
                    }
                },
            );

            Ok(iter)
        }

        pub fn dual_32_firings(&self) -> Result<impl Iterator<Item = DualFiring32<'_>> + Clone> {
            use ProductID as P;
            use ReturnMode as R;

            ensure!(
                self.return_mode == R::DualReturn,
                "expect dual mode, but get {:?}",
                self.return_mode
            );
            ensure!([P::HDL32E, P::VLP32C].contains(&self.product_id));

            let times =
                iter::successors(Some(self.time()), move |prev| Some(*prev + FIRING_PERIOD));
            let azimuths = {
                let azimuths: Vec<_> = self
                    .blocks
                    .iter()
                    .step_by(2)
                    .map(|block| block.azimuth())
                    .collect();
                let azimuth_diffs: Vec<_> = azimuths
                    .iter()
                    .cloned()
                    .tuple_windows()
                    .map(|(curr, next)| (next - curr).wrap_to_2pi())
                    .collect();
                let last_azimuth_diff = *azimuth_diffs.last().unwrap();

                izip!(azimuths, chain!(azimuth_diffs, [last_azimuth_diff])).map(
                    |(former_azimuth, azimuth_diff)| {
                        let latter_azimuth = former_azimuth + azimuth_diff;
                        former_azimuth..latter_azimuth
                    },
                )
            };

            let iter = izip!(times, azimuths, self.blocks.chunks(2)).map(
                move |(block_time, azimuth_range, chunk)| {
                    let [block_strongest, block_last] = match chunk {
                        [first, second] => [first, second],
                        _ => unreachable!(),
                    };

                    DualFiring32 {
                        time: block_time,
                        azimuth_range,
                        block_strongest,
                        block_last,
                        channels_strongest: &block_strongest.channels,
                        channels_last: &block_last.channels,
                    }
                },
            );

            Ok(iter)
        }
    }
}

pub use position_packet::*;
mod position_packet {
    use super::*;

    #[repr(C, packed)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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

            let body_size = packet.header.len as usize - packet_header_size;
            ensure!(
                body_size == mem::size_of::<Self>(),
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
        pub fn from_slice(buffer: &[u8]) -> Result<&Self> {
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
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum LastAdcCalibrationReason {
        NoCalibration = 0,
        PowerOn = 1,
        Manual = 2,
        DeltaTemperature = 3,
        Periodic = 4,
    }

    #[repr(u8)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum PpsStatus {
        Abscent = 0,
        Synchronizing = 1,
        Locked = 2,
        Error = 3,
    }

    #[repr(u8)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum ThermalStatus {
        Ok = 0,
        ThermalShutdown = 1,
    }
}

pub(crate) use firing::*;
mod firing {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct SingleFiring16<'a> {
        pub time: Duration,
        pub azimuth_range: Range<Angle>,
        pub block: &'a Block,
        pub channels: &'a [Channel; 16],
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct DualFiring16<'a> {
        pub time: Duration,
        pub azimuth_range: Range<Angle>,
        pub block_strongest: &'a Block,
        pub block_last: &'a Block,
        pub channels_strongest: &'a [Channel; 16],
        pub channels_last: &'a [Channel; 16],
    }

    impl<'a> DualFiring16<'a> {
        pub fn strongest_part(&self) -> SingleFiring16<'a> {
            let Self {
                time,
                ref azimuth_range,
                block_strongest: block,
                channels_strongest: channels,
                ..
            } = *self;

            SingleFiring16 {
                time,
                azimuth_range: azimuth_range.clone(),
                block,
                channels,
            }
        }

        pub fn last_part(&self) -> SingleFiring16<'a> {
            let Self {
                time,
                ref azimuth_range,
                block_last: block,
                channels_last: channels,
                ..
            } = *self;

            SingleFiring16 {
                time,
                azimuth_range: azimuth_range.clone(),
                block,
                channels,
            }
        }
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct SingleFiring32<'a> {
        pub time: Duration,
        pub azimuth_range: Range<Angle>,
        pub block: &'a Block,
        pub channels: &'a [Channel; 32],
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct DualFiring32<'a> {
        pub time: Duration,
        pub azimuth_range: Range<Angle>,
        pub block_strongest: &'a Block,
        pub block_last: &'a Block,
        pub channels_strongest: &'a [Channel; 32],
        pub channels_last: &'a [Channel; 32],
    }

    impl<'a> DualFiring32<'a> {
        pub fn strongest_part(&self) -> SingleFiring32<'a> {
            let Self {
                time,
                ref azimuth_range,
                block_strongest: block,
                channels_strongest: channels,
                ..
            } = *self;

            SingleFiring32 {
                time,
                azimuth_range: azimuth_range.clone(),
                block,
                channels,
            }
        }

        pub fn last_part(&self) -> SingleFiring32<'a> {
            let Self {
                time,
                ref azimuth_range,
                block_last: block,
                channels_last: channels,
                ..
            } = *self;

            SingleFiring32 {
                time,
                azimuth_range: azimuth_range.clone(),
                block,
                channels,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn velodyne_packet_size_test() {
        assert_eq!(mem::size_of::<DataPacket>(), 1206);
        assert_eq!(mem::size_of::<PositionPacket>(), 512);
    }
}
