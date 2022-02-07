use std::f64::consts::PI;

use crate::{
    common::*,
    utils::{AngleExt as _, DurationExt as _},
    velodyne::{
        config::LaserParameter,
        consts::{self, CHANNEL_PERIOD, FIRING_PERIOD},
        packet::{Block, Channel, DataPacket, ReturnMode},
        point::{DualReturnPoint, LidarFrameEntry, PointData, SingleReturnPoint},
    },
};

#[derive(Debug, Clone)]
struct FiringInfo<'a> {
    lower_timestamp: Duration,
    lower_azimuth_angle: Angle,
    upper_azimuth_angle: Angle,
    firing: &'a [Channel],
}

pub(crate) fn convert_single_return_16_channel(
    lasers: &[LaserParameter; 16],
    distance_resolution: Length,
    last_block: &mut Option<(Duration, Block)>,
    packet: &DataPacket,
) -> Vec<SingleReturnPoint> {
    debug_assert!(
        [ReturnMode::StrongestReturn, ReturnMode::LastReturn].contains(&packet.return_mode)
    );

    // consts
    let block_period = FIRING_PERIOD.mul_f64(2.0);
    let packet_timestamp = packet.time();

    // update last seen block
    let prev_last_block = {
        let new_timestamp =
            packet_timestamp + block_period.mul_f64((packet.blocks.len() - 1) as f64);
        let new_block = *packet.blocks.last().unwrap();
        last_block.replace((new_timestamp, new_block))
    };

    let packet_blocks_iter = packet.blocks.iter().enumerate().map(|(idx, block)| {
        let block_timestamp = packet_timestamp + block_period.mul_f64(idx as f64);
        (block_timestamp, block)
    });

    let mut blocks_iter = prev_last_block
        .iter()
        .map(|(block_timestamp, block)| (*block_timestamp, block))
        .chain(packet_blocks_iter);

    //set lidar beam channel index
    let mut channel_vec =
        convert_to_points_16_channel(lasers, distance_resolution, &mut blocks_iter);

    // get index  array
    let corr_deg_index = consts::vlp_16::ELEVAION_INDEX;

    // set channel_index
    for i in 0..channel_vec.len() {
        // set channel index
        channel_vec[i].lidar_frame_entry.row_idx = corr_deg_index[i % 16];
    }

    channel_vec
}

pub(crate) fn convert_dual_return_16_channel(
    lasers: &[LaserParameter; 16],
    distance_resolution: Length,
    last_block: &mut Option<(Duration, Block, Block)>,
    packet: &DataPacket,
) -> Vec<DualReturnPoint> {
    debug_assert_eq!(packet.return_mode, ReturnMode::DualReturn);

    // consts
    let block_period = FIRING_PERIOD.mul_f64(2.0);
    let packet_timestamp = packet.time();

    // update last blocks
    let (prev_strongest_block, prev_last_block) = {
        let (last_strongest_block, last_last_block) =
            match packet.blocks[(packet.blocks.len() - 2)..] {
                [strongest_block, last_block] => (strongest_block, last_block),
                _ => unreachable!(),
            };
        let last_timestamp =
            packet_timestamp + block_period.mul_f64((packet.blocks.len() / 2 - 1) as f64);

        match last_block.replace((last_timestamp, last_strongest_block, last_last_block)) {
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
                    let block_timestamp = packet_timestamp + block_period.mul_f64(idx as f64);
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
                    let block_timestamp = packet_timestamp + block_period.mul_f64(idx as f64);
                    (block_timestamp, block)
                });

        prev_last_block
            .iter()
            .map(|(timestamp, block)| (*timestamp, block))
            .chain(packet_last_blocks_iter)
    };

    // get index  array
    let corr_deg_index = consts::vlp_16::ELEVAION_INDEX;

    let mut strongest_points =
        convert_to_points_16_channel(lasers, distance_resolution, &mut strongest_blocks_iter);

    // set channel_index
    for i in 0..strongest_points.len() {
        //set channel index
        strongest_points[i].lidar_frame_entry.row_idx = corr_deg_index[i % 16];
    }

    let mut last_points =
        convert_to_points_16_channel(lasers, distance_resolution, &mut last_blocks_iter);

    // set channel_index
    for i in 0..last_points.len() {
        //set channel index
        last_points[i].lidar_frame_entry.row_idx = corr_deg_index[i % 16];
    }

    debug_assert_eq!(
        strongest_points.len(),
        packet.blocks.len() / 2 * packet.blocks[0].channels.len()
    );
    debug_assert_eq!(
        last_points.len(),
        packet.blocks.len() / 2 * packet.blocks[0].channels.len()
    );

    strongest_points
        .into_iter()
        .zip(last_points.into_iter())
        .map(|(strongest_return_point, last_return_point)| {
            DualReturnPoint::try_from_pair(strongest_return_point, last_return_point).unwrap()
        })
        .collect()
}

pub(crate) fn convert_single_return_32_channel(
    lasers: &[LaserParameter; 32],
    distance_resolution: Length,
    last_block: &mut Option<(Duration, Block)>,
    packet: &DataPacket,
) -> Vec<SingleReturnPoint> {
    debug_assert!(
        [ReturnMode::StrongestReturn, ReturnMode::LastReturn].contains(&packet.return_mode)
    );

    // consts
    let block_period = FIRING_PERIOD;
    let packet_timestamp = packet.time();

    let packet_blocks_iter = packet.blocks.iter().enumerate().map(|(idx, block)| {
        let block_timestamp = packet_timestamp + block_period.mul_f64(idx as f64);
        (block_timestamp, block)
    });

    let prev_last_block = {
        let new_timestamp =
            packet_timestamp + block_period.mul_f64((packet.blocks.len() - 1) as f64);
        let new_block = *packet.blocks.last().unwrap();
        last_block.replace((new_timestamp, new_block))
    };

    let mut blocks_iter = prev_last_block
        .iter()
        .map(|(block_timestamp, block)| (*block_timestamp, block))
        .chain(packet_blocks_iter);

    //set lidar beam channel index
    let mut channel_vec =
        convert_to_points_32_channel(lasers, distance_resolution, &mut blocks_iter);

    // get index  array
    let corr_deg_index = consts::vlp_32c::ELEVAION_INDEX;

    // set channel_index
    for i in 0..channel_vec.len() {
        //set channel index
        channel_vec[i].lidar_frame_entry.row_idx = corr_deg_index[i % 32];
    }

    channel_vec
}

pub(crate) fn convert_dual_return_32_channel(
    lasers: &[LaserParameter; 32],
    distance_resolution: Length,
    last_block: &mut Option<(Duration, Block, Block)>,
    packet: &DataPacket,
) -> Vec<DualReturnPoint> {
    debug_assert_eq!(packet.return_mode, ReturnMode::DualReturn);

    // consts
    let block_period = FIRING_PERIOD;
    let packet_timestamp = packet.time();

    // update last blocks
    let (prev_strongest_block, prev_last_block) = {
        let (last_strongest_block, last_last_block) =
            match packet.blocks[(packet.blocks.len() - 2)..] {
                [strongest_block, last_block] => (strongest_block, last_block),
                _ => unreachable!(),
            };
        let last_timestamp =
            packet_timestamp + block_period.mul_f64((packet.blocks.len() / 2 - 1) as f64);

        match last_block.replace((last_timestamp, last_strongest_block, last_last_block)) {
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
                    let block_timestamp = packet_timestamp + block_period.mul_f64(idx as f64);
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
                    let block_timestamp = packet_timestamp + block_period.mul_f64(idx as f64);
                    (block_timestamp, block)
                });

        prev_last_block
            .iter()
            .map(|(timestamp, block)| (*timestamp, block))
            .chain(packet_last_blocks_iter)
    };

    let mut strongest_points =
        convert_to_points_32_channel(lasers, distance_resolution, &mut strongest_blocks_iter);

    // get index  array
    let corr_deg_index = consts::vlp_32c::ELEVAION_INDEX;

    // set channel_index
    for i in 0..strongest_points.len() {
        //set channel index
        strongest_points[i].lidar_frame_entry.row_idx = corr_deg_index[i % 32];
    }

    let mut last_points =
        convert_to_points_32_channel(lasers, distance_resolution, &mut last_blocks_iter);

    // set channel_index
    for i in 0..last_points.len() {
        //set channel index
        last_points[i].lidar_frame_entry.row_idx = corr_deg_index[i % 32];
    }

    debug_assert_eq!(
        strongest_points.len(),
        packet.blocks.len() / 2 * packet.blocks[0].channels.len()
    );
    debug_assert_eq!(
        last_points.len(),
        packet.blocks.len() / 2 * packet.blocks[0].channels.len()
    );

    strongest_points
        .into_iter()
        .zip(last_points.into_iter())
        .map(|(strongest_return_point, last_return_point)| {
            DualReturnPoint::try_from_pair(strongest_return_point, last_return_point).unwrap()
        })
        .collect()
}

pub(crate) fn convert_to_points_16_channel<'a, I>(
    lasers: &[LaserParameter; 16],
    distance_resolution: Length,
    iter: &mut I,
) -> Vec<SingleReturnPoint>
where
    I: Iterator<Item = (Duration, &'a Block)>,
{
    let first_item = iter.next().unwrap();

    iter.scan(first_item, |prev_pair, (curr_timestamp, curr_block)| {
        let (prev_timestamp, prev_block) = *prev_pair;
        *prev_pair = (curr_timestamp, curr_block);

        let mid_timestamp = prev_timestamp + FIRING_PERIOD;

        let prev_azimuth_angle = prev_block.azimuth_angle();
        let curr_azimuth_angle = {
            // fix roll-over case
            let curr_angle = curr_block.azimuth_angle();
            if curr_angle < prev_azimuth_angle {
                curr_angle + Angle::from_radians(PI * 2.0)
            } else {
                curr_angle
            }
        };
        let mid_azimuth_angle: Angle = (prev_azimuth_angle + curr_azimuth_angle) / 2.0;

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
    .flatten()
    .flat_map(|firing_info| {
        let FiringInfo {
            lower_timestamp,
            lower_azimuth_angle,
            upper_azimuth_angle,
            firing,
        } = firing_info;

        debug_assert_eq!(firing.len(), 16);
        debug_assert!(lower_azimuth_angle <= upper_azimuth_angle);

        izip!(firing.iter(), lasers.iter(), 0..).enumerate().map(
            move |(channel_idx, (channel, laser_params, laser_id))| {
                let timestamp = lower_timestamp + CHANNEL_PERIOD.mul_f64(channel_idx as f64);
                let ratio = CHANNEL_PERIOD
                    .mul_f64(channel_idx as f64)
                    .div_duration(FIRING_PERIOD);
                let LaserParameter {
                    elevation_angle,
                    azimuth_offset,
                    vertical_offset,
                    horizontal_offset,
                } = laser_params;

                // clockwise angle with origin points to front of sensor
                let original_azimuth_angle = {
                    let azimuth = lower_azimuth_angle
                        + ((upper_azimuth_angle - lower_azimuth_angle) * ratio)
                        + *azimuth_offset;
                    azimuth.normalize()
                };
                let corrected_azimuth_angle = {
                    let azimuth = original_azimuth_angle + *azimuth_offset;
                    azimuth.normalize()
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
                    lidar_frame_entry: LidarFrameEntry {
                        row_idx: std::usize::MIN,
                        col_idx: std::usize::MIN,
                    },
                }
            },
        )
    })
    .collect()
}

pub(crate) fn convert_to_points_32_channel<'a, I>(
    lasers: &[LaserParameter; 32],
    distance_resolution: Length,
    iter: &mut I,
) -> Vec<SingleReturnPoint>
where
    I: Iterator<Item = (Duration, &'a Block)>,
{
    let first_item = iter.next().unwrap();
    iter.scan(first_item, |prev_pair, (curr_timestamp, curr_block)| {
        let (prev_timestamp, prev_block) = *prev_pair;
        *prev_pair = (curr_timestamp, curr_block);

        let prev_azimuth_angle = prev_block.azimuth_angle();
        let curr_azimuth_angle = {
            let curr_angle = curr_block.azimuth_angle();
            // fix roll-over case
            if curr_angle < prev_azimuth_angle {
                curr_angle + Angle::from_radians(PI * 2.0)
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

        izip!(firing.iter(), lasers.iter(), 0..).enumerate().map(
            move |(channel_idx, (channel, laser_params, laser_id))| {
                let timestamp = lower_timestamp + CHANNEL_PERIOD.mul_f64((channel_idx / 2) as f64);
                let ratio = CHANNEL_PERIOD
                    .mul_f64((channel_idx / 2) as f64)
                    .div_duration(FIRING_PERIOD);
                let LaserParameter {
                    elevation_angle,
                    azimuth_offset,
                    vertical_offset,
                    horizontal_offset,
                } = laser_params;

                // clockwise angle with origin points to front of sensor
                let original_azimuth_angle = {
                    let azimuth =
                        lower_azimuth_angle + ((upper_azimuth_angle - lower_azimuth_angle) * ratio);
                    azimuth.normalize()
                };
                let corrected_azimuth_angle = {
                    let azimuth = original_azimuth_angle + *azimuth_offset;
                    azimuth.normalize()
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
                    lidar_frame_entry: LidarFrameEntry {
                        row_idx: std::usize::MIN,
                        col_idx: std::usize::MIN,
                    },
                }
            },
        )
    })
    .collect::<Vec<_>>()
}

fn compute_position(
    distance: Length,
    elevation_angle: Angle,
    azimuth_angle: Angle,
    vertical_offset: Length,
    horizontal_offset: Length,
) -> [Length; 3] {
    // The origin of elevaion_angle lies on xy plane.
    // The azimuth angle starts from y-axis, rotates clockwise.

    let distance_plane = distance * elevation_angle.cos() - vertical_offset * elevation_angle.sin();
    let x = distance_plane * azimuth_angle.sin() - horizontal_offset * azimuth_angle.cos();
    let y = distance_plane * azimuth_angle.cos() + horizontal_offset * azimuth_angle.sin();
    let z = distance * elevation_angle.sin() + vertical_offset * elevation_angle.cos();
    [x, y, z]
}
