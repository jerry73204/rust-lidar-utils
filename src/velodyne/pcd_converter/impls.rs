use super::{
    context::{DualReturnContext, SingleReturnContext},
    data::{DualReturnPoint, PointData, SingleReturnPoint},
};
use crate::velodyne::{
    config::LaserParameter,
    consts::{CHANNEL_PERIOD, FIRING_PERIOD},
    marker::ReturnTypeMarker,
    packet::{Block, Channel, Packet, ReturnMode},
};
use generic_array::GenericArray;
use itertools::izip;
use std::borrow::Borrow;
use typenum::{U16, U32};
use uom::si::{
    angle::radian,
    f64::{Angle as F64Angle, Length as F64Length, Ratio as F64Ratio, Time as F64Time},
    time::microsecond,
};

#[derive(Debug, Clone)]
struct FiringInfo<'a> {
    lower_timestamp: F64Time,
    lower_azimuth_angle: F64Angle,
    upper_azimuth_angle: F64Angle,
    firing: &'a [Channel],
}

pub(crate) fn convert_single_return_16_channel<PacketType, ReturnType>(
    context: &mut SingleReturnContext<U16, ReturnType>,
    packet: PacketType,
) -> Vec<SingleReturnPoint>
where
    PacketType: AsRef<Packet>,
    ReturnType: ReturnTypeMarker,
{
    let packet = packet.as_ref();
    debug_assert!(
        [ReturnMode::StrongestReturn, ReturnMode::LastReturn].contains(&packet.return_mode)
    );

    // consts
    let firing_period = F64Time::new::<microsecond>(FIRING_PERIOD);
    let block_period = firing_period * 2.0;
    let packet_timestamp = F64Time::new::<microsecond>(packet.time().get::<microsecond>() as f64);

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
    let points = convert_to_points_16_channel(
        &context.lasers,
        context.distance_resolution,
        &mut blocks_iter,
    );
    points
}

pub(crate) fn convert_dual_return_16_channel<PacketType, ReturnType>(
    context: &mut DualReturnContext<U16, ReturnType>,
    packet: PacketType,
) -> Vec<DualReturnPoint>
where
    PacketType: AsRef<Packet>,
    ReturnType: ReturnTypeMarker,
{
    let packet = packet.as_ref();
    debug_assert_eq!(packet.return_mode, ReturnMode::DualReturn);

    // consts
    let firing_period = F64Time::new::<microsecond>(FIRING_PERIOD);
    let block_period = firing_period * 2.0;
    let packet_timestamp = F64Time::new::<microsecond>(packet.time().get::<microsecond>() as f64);

    // update last blocks
    let (prev_strongest_block, prev_last_block) = {
        let (last_strongest_block, last_last_block) =
            match packet.blocks[(packet.blocks.len() - 2)..] {
                [strongest_block, last_block] => (strongest_block, last_block),
                _ => unreachable!(),
            };
        let last_timestamp = packet_timestamp + block_period * (packet.blocks.len() / 2 - 1) as f64;

        match context
            .last_block
            .replace((last_timestamp, last_strongest_block, last_last_block))
        {
            Some((prev_last_timestamp, prev_strongest_block, prev_last_block)) => (
                Some((prev_last_timestamp, prev_strongest_block)),
                Some((prev_last_timestamp, prev_last_block)),
            ),
            None => (None, None),
        }
    };

    let mut strongest_blocks_iter = {
        let packet_strongest_blocks_iter =
            packet
                .blocks
                .iter()
                .step_by(2)
                .enumerate()
                .map(|(idx, block)| {
                    let block_timestamp = packet_timestamp + block_period * idx as f64;
                    (block_timestamp, block)
                });

        prev_strongest_block
            .iter()
            .map(|(timestamp, block)| (*timestamp, block))
            .chain(packet_strongest_blocks_iter)
    };

    let mut last_blocks_iter = {
        let packet_last_blocks_iter =
            packet
                .blocks
                .iter()
                .step_by(2)
                .enumerate()
                .map(|(idx, block)| {
                    let block_timestamp = packet_timestamp + block_period * idx as f64;
                    (block_timestamp, block)
                });

        prev_last_block
            .iter()
            .map(|(timestamp, block)| (*timestamp, block))
            .chain(packet_last_blocks_iter)
    };

    let strongest_points = convert_to_points_16_channel(
        &context.lasers,
        context.distance_resolution,
        &mut strongest_blocks_iter,
    );
    let last_points = convert_to_points_16_channel(
        &context.lasers,
        context.distance_resolution,
        &mut last_blocks_iter,
    );

    debug_assert_eq!(
        strongest_points.len(),
        packet.blocks.len() / 2 * packet.blocks[0].channels.len()
    );
    debug_assert_eq!(
        last_points.len(),
        packet.blocks.len() / 2 * packet.blocks[0].channels.len()
    );

    let points = strongest_points
        .into_iter()
        .zip(last_points.into_iter())
        .map(|(strongest_return_point, last_return_point)| {
            DualReturnPoint::try_from_pair(strongest_return_point, last_return_point).unwrap()
        })
        .collect::<Vec<_>>();
    points
}

pub(crate) fn convert_single_return_32_channel<PacketType, ReturnType>(
    context: &mut SingleReturnContext<U32, ReturnType>,
    packet: PacketType,
) -> Vec<SingleReturnPoint>
where
    PacketType: AsRef<Packet>,
    ReturnType: ReturnTypeMarker,
{
    let packet = packet.as_ref();
    debug_assert!(
        [ReturnMode::StrongestReturn, ReturnMode::LastReturn].contains(&packet.return_mode)
    );

    // consts
    let firing_period = F64Time::new::<microsecond>(FIRING_PERIOD);
    let block_period = firing_period;
    let packet_timestamp = F64Time::new::<microsecond>(packet.time().get::<microsecond>() as f64);

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
    let points = convert_to_points_32_channel(
        &context.lasers,
        context.distance_resolution,
        &mut blocks_iter,
    );
    points
}

pub(crate) fn convert_dual_return_32_channel<PacketType, ReturnType>(
    context: &mut DualReturnContext<U32, ReturnType>,
    packet: PacketType,
) -> Vec<DualReturnPoint>
where
    PacketType: AsRef<Packet>,
    ReturnType: ReturnTypeMarker,
{
    let packet = packet.as_ref();
    debug_assert_eq!(packet.return_mode, ReturnMode::DualReturn);

    // consts
    let firing_period = F64Time::new::<microsecond>(FIRING_PERIOD);
    let block_period = firing_period;
    let packet_timestamp = F64Time::new::<microsecond>(packet.time().get::<microsecond>() as f64);

    // update last blocks
    let (prev_strongest_block, prev_last_block) = {
        let (last_strongest_block, last_last_block) =
            match packet.blocks[(packet.blocks.len() - 2)..] {
                [strongest_block, last_block] => (strongest_block, last_block),
                _ => unreachable!(),
            };
        let last_timestamp = packet_timestamp + block_period * (packet.blocks.len() / 2 - 1) as f64;

        match context
            .last_block
            .replace((last_timestamp, last_strongest_block, last_last_block))
        {
            Some((prev_last_timestamp, prev_strongest_block, prev_last_block)) => (
                Some((prev_last_timestamp, prev_strongest_block)),
                Some((prev_last_timestamp, prev_last_block)),
            ),
            None => (None, None),
        }
    };

    let mut strongest_blocks_iter = {
        let packet_strongest_blocks_iter =
            packet
                .blocks
                .iter()
                .step_by(2)
                .enumerate()
                .map(|(idx, block)| {
                    let block_timestamp = packet_timestamp + block_period * idx as f64;
                    (block_timestamp, block)
                });

        prev_strongest_block
            .iter()
            .map(|(timestamp, block)| (*timestamp, block))
            .chain(packet_strongest_blocks_iter)
    };

    let mut last_blocks_iter = {
        let packet_last_blocks_iter =
            packet
                .blocks
                .iter()
                .step_by(2)
                .enumerate()
                .map(|(idx, block)| {
                    let block_timestamp = packet_timestamp + block_period * idx as f64;
                    (block_timestamp, block)
                });

        prev_last_block
            .iter()
            .map(|(timestamp, block)| (*timestamp, block))
            .chain(packet_last_blocks_iter)
    };

    let strongest_points = convert_to_points_32_channel(
        &context.lasers,
        context.distance_resolution,
        &mut strongest_blocks_iter,
    );
    let last_points = convert_to_points_32_channel(
        &context.lasers,
        context.distance_resolution,
        &mut last_blocks_iter,
    );

    debug_assert_eq!(
        strongest_points.len(),
        packet.blocks.len() / 2 * packet.blocks[0].channels.len()
    );
    debug_assert_eq!(
        last_points.len(),
        packet.blocks.len() / 2 * packet.blocks[0].channels.len()
    );

    let points = strongest_points
        .into_iter()
        .zip(last_points.into_iter())
        .map(|(strongest_return_point, last_return_point)| {
            DualReturnPoint::try_from_pair(strongest_return_point, last_return_point).unwrap()
        })
        .collect::<Vec<_>>();
    points
}

pub(crate) fn convert_to_points_16_channel<'a, A, I>(
    lasers: A,
    distance_resolution: F64Length,
    iter: &mut I,
) -> Vec<SingleReturnPoint>
where
    A: Borrow<GenericArray<LaserParameter, U16>>,
    I: Iterator<Item = (F64Time, &'a Block)>,
{
    let channel_period = F64Time::new::<microsecond>(CHANNEL_PERIOD);
    let firing_period = F64Time::new::<microsecond>(FIRING_PERIOD);

    let first_item = iter.next().unwrap();
    iter.scan(first_item, |prev_pair, (curr_timestamp, curr_block)| {
        let (prev_timestamp, prev_block) = *prev_pair;
        *prev_pair = (curr_timestamp, curr_block);

        let mid_timestamp = prev_timestamp + firing_period;

        let prev_azimuth_angle = prev_block.azimuth_angle();
        let curr_azimuth_angle = {
            // fix roll-over case
            let curr_angle = curr_block.azimuth_angle();
            if curr_angle < prev_azimuth_angle {
                curr_angle + F64Angle::new::<radian>(std::f64::consts::PI * 2.0)
            } else {
                curr_angle
            }
        };
        let mid_azimuth_angle: F64Angle = (prev_azimuth_angle + curr_azimuth_angle) / 2.0;

        let former_firing = FiringInfo {
            lower_timestamp: prev_timestamp,
            lower_azimuth_angle: prev_azimuth_angle,
            upper_azimuth_angle: mid_azimuth_angle,
            firing: &prev_block.channels[0..16],
        };

        let latter_firing = FiringInfo {
            lower_timestamp: mid_timestamp,
            lower_azimuth_angle: mid_azimuth_angle,
            upper_azimuth_angle: curr_azimuth_angle,
            firing: &prev_block.channels[16..32],
        };

        Some(vec![former_firing, latter_firing])
    })
    .flat_map(|firings| firings)
    .flat_map(|firing_info| {
        let FiringInfo {
            lower_timestamp,
            lower_azimuth_angle,
            upper_azimuth_angle,
            firing,
        } = firing_info;

        debug_assert_eq!(firing.len(), 16);
        debug_assert!(lower_azimuth_angle <= upper_azimuth_angle);

        izip!(firing.iter(), lasers.borrow().iter(), 0..)
            .enumerate()
            .map(move |(channel_idx, (channel, laser_params, laser_id))| {
                let timestamp = lower_timestamp + channel_period * channel_idx as f64;
                let ratio = channel_period * channel_idx as f64 / firing_period;
                let LaserParameter {
                    elevation_angle,
                    azimuth_offset,
                    vertical_offset,
                    horizontal_offset,
                } = laser_params;
                let altitude_angle =
                    F64Angle::new::<radian>(std::f64::consts::FRAC_PI_2) - *elevation_angle;

                // clockwise angle with origin points to front of sensor
                let original_azimuth_angle = {
                    let mut azimuth = lower_azimuth_angle
                        + F64Angle::from((upper_azimuth_angle - lower_azimuth_angle) * ratio)
                        + *azimuth_offset;
                    if azimuth >= F64Angle::new::<radian>(std::f64::consts::PI * 2.0) {
                        azimuth -= F64Angle::new::<radian>(std::f64::consts::PI * 2.0);
                    }
                    azimuth
                };
                let corrected_azimuth_angle = {
                    let mut azimuth = original_azimuth_angle + *azimuth_offset;
                    if azimuth >= F64Angle::new::<radian>(std::f64::consts::PI * 2.0) {
                        azimuth -= F64Angle::new::<radian>(std::f64::consts::PI * 2.0);
                    }
                    azimuth
                };
                let distance = distance_resolution * channel.distance as f64;
                let position = compute_position(
                    distance,
                    *elevation_angle,
                    corrected_azimuth_angle,
                    *vertical_offset,
                    *horizontal_offset,
                );

                SingleReturnPoint {
                    laser_id,
                    timestamp,
                    original_azimuth_angle,
                    corrected_azimuth_angle,
                    data: PointData {
                        distance,
                        intensity: channel.intensity,
                        position,
                    },
                }
            })
    })
    .collect::<Vec<_>>()
}

pub(crate) fn convert_to_points_32_channel<'a, I, A>(
    lasers: A,
    distance_resolution: F64Length,
    iter: &mut I,
) -> Vec<SingleReturnPoint>
where
    A: Borrow<GenericArray<LaserParameter, U32>>,
    I: Iterator<Item = (F64Time, &'a Block)>,
{
    let channel_period = F64Time::new::<microsecond>(CHANNEL_PERIOD);
    let firing_period = F64Time::new::<microsecond>(FIRING_PERIOD);

    let first_item = iter.next().unwrap();
    iter.scan(first_item, |prev_pair, (curr_timestamp, curr_block)| {
        let (prev_timestamp, prev_block) = *prev_pair;
        *prev_pair = (curr_timestamp, curr_block);

        let prev_azimuth_angle = prev_block.azimuth_angle();
        let curr_azimuth_angle = {
            let curr_angle = curr_block.azimuth_angle();
            // fix roll-over case
            if curr_angle < prev_azimuth_angle {
                curr_angle + F64Angle::new::<radian>(std::f64::consts::PI * 2.0)
            } else {
                curr_angle
            }
        };

        let firing_info = FiringInfo {
            lower_timestamp: prev_timestamp,
            lower_azimuth_angle: prev_azimuth_angle,
            upper_azimuth_angle: curr_azimuth_angle,
            firing: &prev_block.channels,
        };
        Some(firing_info)
    })
    .flat_map(|firing_info| {
        let FiringInfo {
            lower_timestamp,
            lower_azimuth_angle,
            upper_azimuth_angle,
            firing,
        } = firing_info;

        debug_assert_eq!(firing.len(), 32);

        izip!(firing.iter(), lasers.borrow().iter(), 0..)
            .enumerate()
            .map(move |(channel_idx, (channel, laser_params, laser_id))| {
                let timestamp = lower_timestamp + channel_period * (channel_idx / 2) as f64;
                let ratio: F64Ratio = channel_period * (channel_idx / 2) as f64 / firing_period;
                let LaserParameter {
                    elevation_angle,
                    azimuth_offset,
                    vertical_offset,
                    horizontal_offset,
                } = laser_params;

                // clockwise angle with origin points to front of sensor
                let original_azimuth_angle = {
                    let mut azimuth = lower_azimuth_angle
                        + F64Angle::from((upper_azimuth_angle - lower_azimuth_angle) * ratio);
                    if azimuth >= F64Angle::new::<radian>(std::f64::consts::PI * 2.0) {
                        azimuth -= F64Angle::new::<radian>(std::f64::consts::PI * 2.0);
                    }
                    azimuth
                };
                let corrected_azimuth_angle = {
                    let mut azimuth = original_azimuth_angle + *azimuth_offset;
                    if azimuth >= F64Angle::new::<radian>(std::f64::consts::PI * 2.0) {
                        azimuth -= F64Angle::new::<radian>(std::f64::consts::PI * 2.0);
                    }
                    azimuth
                };
                let distance = distance_resolution * channel.distance as f64;
                let position = compute_position(
                    distance,
                    *elevation_angle,
                    corrected_azimuth_angle,
                    *vertical_offset,
                    *horizontal_offset,
                );

                SingleReturnPoint {
                    laser_id,
                    timestamp,
                    original_azimuth_angle,
                    corrected_azimuth_angle,
                    data: PointData {
                        distance,
                        intensity: channel.intensity,
                        position,
                    },
                }
            })
    })
    .collect::<Vec<_>>()
}

fn compute_position(
    distance: F64Length,
    elevation_angle: F64Angle,
    azimuth_angle: F64Angle,
    vertical_offset: F64Length,
    horizontal_offset: F64Length,
) -> [F64Length; 3] {
    // The origin of elevaion_angle lies on xy plane.
    // The azimuth angle starts from y-axis, rotates clockwise.

    let distance_plane = distance * elevation_angle.cos() - vertical_offset * elevation_angle.sin();
    let x = distance_plane * azimuth_angle.sin() - horizontal_offset * azimuth_angle.cos();
    let y = distance_plane * azimuth_angle.cos() + horizontal_offset * azimuth_angle.sin();
    let z = distance * elevation_angle.sin() + vertical_offset * elevation_angle.cos();
    [x, y, z]
}
