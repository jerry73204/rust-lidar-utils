use super::context::{
    DualReturn16ChannelContext, DualReturn32ChannelContext, SingleReturn16ChannelContext,
    SingleReturn32ChannelContext, ToConverterContext,
};
use crate::{
    common::{compute_interpolation_ratio, spherical_to_xyz},
    velodyne::{
        config::{Config16Channel, Config32Channel, VelodyneConfig},
        consts::{CHANNEL_PERIOD, FIRING_PERIOD},
        marker::{DualReturn, LastReturn, ReturnTypeMarker, StrongestReturn},
        packet::{Block, Channel, Packet, ReturnMode},
    },
};
use itertools::izip;
use uom::si::{
    angle::radian,
    f64::{Angle as F64Angle, Length as F64Length, Ratio as F64Ratio, Time as F64Time},
    length::millimeter,
    time::microsecond,
    u32::Length as U32Length,
};

pub struct PointCloudConverter<Config>
where
    Config: VelodyneConfig + ToConverterContext,
{
    context: <Config as ToConverterContext>::Context,
}

fn convert_single_return_16_channel<P>(
    context: &mut SingleReturn16ChannelContext,
    packet: P,
) -> Vec<SingleReturnPoint>
where
    P: AsRef<Packet>,
{
    let packet = packet.as_ref();

    // consts
    let channel_period = F64Time::new::<microsecond>(CHANNEL_PERIOD);
    let firing_period = F64Time::new::<microsecond>(FIRING_PERIOD);
    let block_period = firing_period * 2.0;
    let packet_timestamp =
        F64Time::new::<microsecond>(packet.uom_time().get::<microsecond>() as f64);

    // update last seen block
    let prev_last_block = {
        let new_timestamp = packet_timestamp + block_period * (packet.blocks.len() - 1) as f64;
        let new_block = packet.blocks.last().unwrap().clone();
        context.last_block.replace((new_timestamp, new_block))
    };

    let packet_blocks_iter = packet.blocks.iter().enumerate().map(|(idx, block)| {
        let block_timestamp = packet_timestamp + block_period * idx as f64;
        (block_timestamp, block)
    });

    let mut blocks_iter = prev_last_block
        .iter()
        .map(|(block_timestamp, block)| (*block_timestamp, block))
        .chain(packet_blocks_iter);
    let first_block = blocks_iter.next().unwrap();
    let points = blocks_iter
        .scan(
            first_block,
            |prev_ctx: &mut (F64Time, &Block), curr_ctx: (F64Time, &Block)| {
                let (curr_timestamp, curr_block) = curr_ctx;
                let (prev_timestamp, prev_block) = prev_ctx;
                let firings = {
                    let mid_timestamp = *prev_timestamp + firing_period;

                    let ratio: F64Ratio =
                        compute_interpolation_ratio(*prev_timestamp, mid_timestamp, curr_timestamp);

                    let prev_azimuth_angle = prev_block.uom_azimuth_angle();
                    let curr_azimuth_angle = {
                        let curr_angle = curr_block.uom_azimuth_angle();
                        // fix roll-over case
                        if curr_angle < prev_azimuth_angle {
                            curr_angle + F64Angle::new::<radian>(std::f64::consts::PI * 2.0)
                        } else {
                            curr_angle
                        }
                    };
                    let mid_azimuth_angle: F64Angle = prev_azimuth_angle
                        + F64Angle::from((curr_azimuth_angle - prev_azimuth_angle) * ratio);

                    let former_firing = FiringInfo {
                        lower_timestamp: *prev_timestamp,
                        upper_timestamp: mid_timestamp,
                        lower_azimuth_angle: prev_azimuth_angle,
                        upper_azimuth_angle: mid_azimuth_angle,
                        firing: &prev_block.channels[0..16],
                    };

                    let latter_firing = FiringInfo {
                        lower_timestamp: mid_timestamp,
                        upper_timestamp: curr_timestamp,
                        lower_azimuth_angle: mid_azimuth_angle,
                        upper_azimuth_angle: curr_azimuth_angle,
                        firing: &prev_block.channels[16..32],
                    };

                    Some(vec![former_firing, latter_firing])
                };

                *prev_ctx = (curr_timestamp, curr_block);
                firings
            },
        )
        .flat_map(|firings| firings)
        .flat_map(|firing_info| {
            let FiringInfo {
                lower_timestamp,
                upper_timestamp,
                lower_azimuth_angle,
                upper_azimuth_angle,
                firing,
            } = firing_info;

            debug_assert_eq!(firing.len(), 16);
            debug_assert!(lower_azimuth_angle <= upper_azimuth_angle);

            izip!(
                firing.iter(),
                context.altitude_angles.iter(),
                context.vertical_corrections.iter(),
            )
            .enumerate()
            .map(
                move |(channel_idx, (channel, altitude_angle, vertical_correction))| {
                    let timestamp = lower_timestamp + channel_period * channel_idx as f64;
                    let ratio: F64Ratio =
                        compute_interpolation_ratio(lower_timestamp, timestamp, upper_timestamp);

                    // clockwise angle with origin points to front of sensor
                    let sensor_azimuth_angle = lower_azimuth_angle
                        + F64Angle::from((upper_azimuth_angle - lower_azimuth_angle) * ratio);

                    // counter-clockwise angle with origin points to right hand side of sensor
                    let spherical_azimuth_angle =
                        F64Angle::new::<radian>(std::f64::consts::FRAC_PI_2) - sensor_azimuth_angle;

                    let u32_distance = channel.uom_distance();
                    let f64_distance =
                        F64Length::new::<millimeter>(u32_distance.get::<millimeter>() as f64);

                    let [x, y, mut z] =
                        spherical_to_xyz(f64_distance, spherical_azimuth_angle, *altitude_angle);
                    z += *vertical_correction;

                    SingleReturnPoint {
                        timestamp,
                        distance: u32_distance,
                        itensity: channel.intensity,
                        azimuth_angle: sensor_azimuth_angle,
                        point: [x, y, z],
                    }
                },
            )
        })
        .collect::<Vec<_>>();

    points
}

fn convert_dual_return_16_channel<P>(
    context: &mut DualReturn16ChannelContext,
    packet: P,
) -> Vec<DualReturnPoint>
where
    P: AsRef<Packet>,
{
    unimplemented!();
}

fn convert_single_return_32_channel<P>(
    context: &mut SingleReturn32ChannelContext,
    packet: P,
) -> Vec<SingleReturnPoint>
where
    P: AsRef<Packet>,
{
    let packet = packet.as_ref();

    // consts
    let channel_period = F64Time::new::<microsecond>(CHANNEL_PERIOD);
    let firing_period = F64Time::new::<microsecond>(FIRING_PERIOD);
    let block_period = firing_period;
    let packet_timestamp =
        F64Time::new::<microsecond>(packet.uom_time().get::<microsecond>() as f64);

    let packet_blocks_iter = packet.blocks.iter().enumerate().map(|(idx, block)| {
        let block_timestamp = packet_timestamp + block_period * idx as f64;
        (block_timestamp, block)
    });

    let prev_last_block = {
        let new_timestamp = packet_timestamp + block_period * (packet.blocks.len() - 1) as f64;
        let new_block = packet.blocks.last().unwrap().clone();
        context.last_block.replace((new_timestamp, new_block))
    };

    let mut blocks_iter = prev_last_block
        .iter()
        .map(|(block_timestamp, block)| (*block_timestamp, block))
        .chain(packet_blocks_iter);
    let first_block = blocks_iter.next().unwrap();

    let points = blocks_iter
        .scan(
            first_block,
            |prev_ctx: &mut (F64Time, &Block), curr_ctx: (F64Time, &Block)| {
                let (curr_timestamp, curr_block) = curr_ctx;
                let (prev_timestamp, prev_block) = prev_ctx;

                let prev_azimuth_angle = prev_block.uom_azimuth_angle();
                let curr_azimuth_angle = {
                    let curr_angle = curr_block.uom_azimuth_angle();
                    // fix roll-over case
                    if curr_angle < prev_azimuth_angle {
                        curr_angle + F64Angle::new::<radian>(std::f64::consts::PI * 2.0)
                    } else {
                        curr_angle
                    }
                };

                let firing_info = FiringInfo {
                    lower_timestamp: *prev_timestamp,
                    upper_timestamp: curr_timestamp,
                    lower_azimuth_angle: prev_azimuth_angle,
                    upper_azimuth_angle: curr_azimuth_angle,
                    firing: &prev_block.channels,
                };

                *prev_ctx = (curr_timestamp, curr_block);
                Some(firing_info)
            },
        )
        .flat_map(|firing_info| {
            let FiringInfo {
                lower_timestamp,
                upper_timestamp,
                lower_azimuth_angle,
                upper_azimuth_angle,
                firing,
            } = firing_info;

            debug_assert_eq!(firing.len(), 32);

            izip!(
                firing.iter(),
                context.altitude_angles.iter(),
                context.vertical_corrections.iter(),
            )
            .enumerate()
            .map(
                move |(channel_idx, (channel, altitude_angle, vertical_correction))| {
                    let timestamp = lower_timestamp + channel_period * (channel_idx / 2) as f64;
                    let ratio: F64Ratio =
                        compute_interpolation_ratio(lower_timestamp, timestamp, upper_timestamp);

                    // clockwise angle with origin points to front of sensor
                    let sensor_azimuth_angle = lower_azimuth_angle
                        + F64Angle::from((upper_azimuth_angle - lower_azimuth_angle) * ratio);

                    // counter-clockwise angle with origin points to right hand side of sensor
                    let spherical_azimuth_angle =
                        F64Angle::new::<radian>(std::f64::consts::FRAC_PI_2) - sensor_azimuth_angle;

                    let u32_distance = channel.uom_distance();
                    let f64_distance =
                        F64Length::new::<millimeter>(u32_distance.get::<millimeter>() as f64);

                    let [x, y, mut z] =
                        spherical_to_xyz(f64_distance, spherical_azimuth_angle, *altitude_angle);
                    z += *vertical_correction;

                    SingleReturnPoint {
                        timestamp,
                        distance: u32_distance,
                        itensity: channel.intensity,
                        azimuth_angle: sensor_azimuth_angle,
                        point: [x, y, z],
                    }
                },
            )
        })
        .collect::<Vec<_>>();

    points
}

fn convert_dual_return_32_channel<P>(
    context: &mut DualReturn32ChannelContext,
    packet: P,
) -> Vec<DualReturnPoint> {
    unimplemented!();
}

impl PointCloudConverter<Config16Channel<StrongestReturn>> {
    pub fn convert<P>(&mut self, packet: P) -> Vec<SingleReturnPoint>
    where
        P: AsRef<Packet>,
    {
        convert_single_return_16_channel(&mut self.context, packet)
    }
}

impl PointCloudConverter<Config16Channel<LastReturn>> {
    pub fn convert<P>(&mut self, packet: P) -> Vec<SingleReturnPoint>
    where
        P: AsRef<Packet>,
    {
        convert_single_return_16_channel(&mut self.context, packet)
    }
}

impl PointCloudConverter<Config16Channel<DualReturn>> {
    pub fn convert<P>(&mut self, packet: P) -> Vec<DualReturnPoint>
    where
        P: AsRef<Packet>,
    {
        convert_dual_return_16_channel(&mut self.context, packet)
    }
}

impl PointCloudConverter<Config32Channel<StrongestReturn>> {
    pub fn convert<P>(&mut self, packet: P) -> Vec<SingleReturnPoint>
    where
        P: AsRef<Packet>,
    {
        convert_single_return_32_channel(&mut self.context, packet)
    }
}

impl PointCloudConverter<Config32Channel<LastReturn>> {
    pub fn convert<P>(&mut self, packet: P) -> Vec<SingleReturnPoint>
    where
        P: AsRef<Packet>,
    {
        convert_single_return_32_channel(&mut self.context, packet)
    }
}

impl PointCloudConverter<Config32Channel<DualReturn>> {
    pub fn convert<P>(&mut self, packet: P) -> Vec<DualReturnPoint>
    where
        P: AsRef<Packet>,
    {
        convert_dual_return_32_channel(&mut self.context, packet)
    }
}

// fn interpoate<T, S>(first: T, last: T, ratio: S) -> T
// where
//     T: PartialOrd + Add<T, Output = T> + Sub<T, Output = T> + Mul<S, Output = T>,
// {
//     debug_assert!(first <= last);
//     first + (last - first) * ratio
// }

struct FiringInfo<'a> {
    lower_timestamp: F64Time,
    upper_timestamp: F64Time,
    lower_azimuth_angle: F64Angle,
    upper_azimuth_angle: F64Angle,
    firing: &'a [Channel],
}

pub struct SingleReturnPoint {
    timestamp: F64Time,
    distance: U32Length,
    itensity: u8,
    azimuth_angle: F64Angle,
    point: [F64Length; 3],
}

pub struct DualReturnPoint {
    timestamp: F64Time,
    distance: U32Length,
    itensity: u8,
    azimuth_angle: F64Angle,
    strongest_return_point: [F64Length; 3],
    last_return_point: [F64Length; 3],
}
