use super::context::{
    DualReturn16ChannelContext, DualReturn32ChannelContext, DynamicContext,
    SingleReturn16ChannelContext, SingleReturn32ChannelContext, ToConverterContext,
};
use crate::{
    common::{compute_interpolation_ratio, spherical_to_xyz},
    velodyne::{
        config::{Config16Channel, Config32Channel, DynamicConfig, VelodyneConfigKind},
        consts::{CHANNEL_PERIOD, FIRING_PERIOD},
        marker::{DualReturn, LastReturn, StrongestReturn},
        packet::{Block, Channel, Packet, ReturnMode},
    },
};
use failure::{ensure, Fallible};
use itertools::izip;
use uom::si::{
    angle::radian,
    f64::{Angle as F64Angle, Length as F64Length, Ratio as F64Ratio, Time as F64Time},
    time::microsecond,
};

/// An _interface_ trait that is implemented by all variants of [PointCloudConverter]
pub trait PointCloudConverterEx<Config>
where
    Config: ToConverterContext,
{
    fn from_config(config: Config) -> PointCloudConverter<Config>;
    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<Config::Point>>
    where
        P: AsRef<Packet>;
}

#[derive(Debug, Clone)]
struct FiringInfo<'a> {
    lower_timestamp: F64Time,
    upper_timestamp: F64Time,
    lower_azimuth_angle: F64Angle,
    upper_azimuth_angle: F64Angle,
    firing: &'a [Channel],
}

/// Point in strongest or last return mode.
#[derive(Debug, Clone)]
pub struct SingleReturnPoint {
    pub timestamp: F64Time,
    pub azimuth_angle: F64Angle,
    pub distance: F64Length,
    pub intensity: u8,
    pub point: [F64Length; 3],
}

impl SingleReturnPoint {
    pub fn to_dynamic(self) -> DynamicPoint {
        DynamicPoint::SingleReturn(self)
    }

    pub fn timestamp(&self) -> F64Time {
        self.timestamp
    }

    pub fn azimuth_angle(&self) -> F64Angle {
        self.azimuth_angle
    }
}

/// Point in dual return mode.
#[derive(Debug, Clone)]
pub struct DualReturnPoint {
    pub strongest_return: SingleReturnPoint,
    pub last_return: SingleReturnPoint,
}

impl DualReturnPoint {
    pub fn to_dynamic(self) -> DynamicPoint {
        DynamicPoint::DualReturn(self)
    }

    pub fn timestamp(&self) -> F64Time {
        assert_eq!(self.strongest_return.timestamp, self.last_return.timestamp);
        self.strongest_return.timestamp
    }

    pub fn azimuth_angle(&self) -> F64Angle {
        assert_eq!(
            self.strongest_return.azimuth_angle,
            self.last_return.azimuth_angle
        );
        self.strongest_return.azimuth_angle
    }
}

/// A point type can be in strongest, last or dual return mode.
#[derive(Debug, Clone)]
pub enum DynamicPoint {
    SingleReturn(SingleReturnPoint),
    DualReturn(DualReturnPoint),
}

impl DynamicPoint {
    pub fn timestamp(&self) -> F64Time {
        use DynamicPoint::*;
        match self {
            SingleReturn(point) => point.timestamp(),
            DualReturn(point) => point.timestamp(),
        }
    }

    pub fn azimuth_angle(&self) -> F64Angle {
        use DynamicPoint::*;
        match self {
            SingleReturn(point) => point.azimuth_angle(),
            DualReturn(point) => point.azimuth_angle(),
        }
    }
}

fn convert_single_return_16_channel<P>(
    context: &mut SingleReturn16ChannelContext,
    packet: P,
) -> Vec<SingleReturnPoint>
where
    P: AsRef<Packet>,
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
        &context.altitude_angles,
        &context.vertical_corrections,
        &mut blocks_iter,
    );
    points
}

fn convert_dual_return_16_channel<P>(
    context: &mut DualReturn16ChannelContext,
    packet: P,
) -> Vec<DualReturnPoint>
where
    P: AsRef<Packet>,
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
        &context.altitude_angles,
        &context.vertical_corrections,
        &mut strongest_blocks_iter,
    );
    let last_points = convert_to_points_16_channel(
        &context.altitude_angles,
        &context.vertical_corrections,
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
        .map(|(strongest_point, last_point)| {
            // TODO: verify time and azimuth angle
            DualReturnPoint {
                strongest_return: strongest_point,
                last_return: last_point,
            }
        })
        .collect::<Vec<_>>();
    points
}

fn convert_single_return_32_channel<P>(
    context: &mut SingleReturn32ChannelContext,
    packet: P,
) -> Vec<SingleReturnPoint>
where
    P: AsRef<Packet>,
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
        &context.altitude_angles,
        &context.vertical_corrections,
        &mut blocks_iter,
    );
    points
}

fn convert_dual_return_32_channel<P>(
    context: &mut DualReturn32ChannelContext,
    packet: P,
) -> Vec<DualReturnPoint>
where
    P: AsRef<Packet>,
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
        &context.altitude_angles,
        &context.vertical_corrections,
        &mut strongest_blocks_iter,
    );
    let last_points = convert_to_points_32_channel(
        &context.altitude_angles,
        &context.vertical_corrections,
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
        .map(|(strongest_point, last_point)| {
            // TODO: verify time and azimuth angle
            DualReturnPoint {
                strongest_return: strongest_point,
                last_return: last_point,
            }
        })
        .collect::<Vec<_>>();
    points
}

fn convert_to_points_16_channel<'a, I>(
    altitude_angles: &[F64Angle; 16],
    vertical_corrections: &[F64Length; 16],
    iter: &mut I,
) -> Vec<SingleReturnPoint>
where
    I: Iterator<Item = (F64Time, &'a Block)>,
{
    let channel_period = F64Time::new::<microsecond>(CHANNEL_PERIOD);
    let firing_period = F64Time::new::<microsecond>(FIRING_PERIOD);

    let first_item = iter.next().unwrap();
    iter.scan(first_item, |prev_pair, (curr_timestamp, curr_block)| {
        let (prev_timestamp, prev_block) = *prev_pair;
        *prev_pair = (curr_timestamp, curr_block);

        let mid_timestamp = prev_timestamp + firing_period;
        let ratio = compute_interpolation_ratio(prev_timestamp, mid_timestamp, curr_timestamp);

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
        let mid_azimuth_angle: F64Angle =
            prev_azimuth_angle + F64Angle::from((curr_azimuth_angle - prev_azimuth_angle) * ratio);

        let former_firing = FiringInfo {
            lower_timestamp: prev_timestamp,
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
    })
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
            altitude_angles.iter(),
            vertical_corrections.iter(),
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

                let distance = channel.distance();

                let [x, y, mut z] =
                    spherical_to_xyz(distance, spherical_azimuth_angle, *altitude_angle);
                z += *vertical_correction;

                SingleReturnPoint {
                    timestamp,
                    distance,
                    intensity: channel.intensity,
                    azimuth_angle: sensor_azimuth_angle,
                    point: [x, y, z],
                }
            },
        )
    })
    .collect::<Vec<_>>()
}

fn convert_to_points_32_channel<'a, I>(
    altitude_angles: &[F64Angle; 32],
    vertical_corrections: &[F64Length; 32],
    iter: &mut I,
) -> Vec<SingleReturnPoint>
where
    I: Iterator<Item = (F64Time, &'a Block)>,
{
    let channel_period = F64Time::new::<microsecond>(CHANNEL_PERIOD);

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
            upper_timestamp: curr_timestamp,
            lower_azimuth_angle: prev_azimuth_angle,
            upper_azimuth_angle: curr_azimuth_angle,
            firing: &prev_block.channels,
        };
        Some(firing_info)
    })
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
            altitude_angles.iter(),
            vertical_corrections.iter(),
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

                let distance = channel.distance();

                let [x, y, mut z] =
                    spherical_to_xyz(distance, spherical_azimuth_angle, *altitude_angle);
                z += *vertical_correction;

                SingleReturnPoint {
                    timestamp,
                    distance,
                    intensity: channel.intensity,
                    azimuth_angle: sensor_azimuth_angle,
                    point: [x, y, z],
                }
            },
        )
    })
    .collect::<Vec<_>>()
}

/// Converts UDP packets from a Velodyne LiDAR to points.
pub struct PointCloudConverter<Config>
where
    Config: VelodyneConfigKind + ToConverterContext,
{
    context: <Config as ToConverterContext>::Context,
}

impl PointCloudConverterEx<Config16Channel<StrongestReturn>>
    for PointCloudConverter<Config16Channel<StrongestReturn>>
{
    fn from_config(config: Config16Channel<StrongestReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<SingleReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!([ReturnMode::StrongestReturn, ReturnMode::LastReturn]
            .contains(&packet.as_ref().return_mode));
        Ok(convert_single_return_16_channel(&mut self.context, packet))
    }
}

impl PointCloudConverterEx<Config16Channel<LastReturn>>
    for PointCloudConverter<Config16Channel<LastReturn>>
{
    fn from_config(config: Config16Channel<LastReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<SingleReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!([ReturnMode::StrongestReturn, ReturnMode::LastReturn]
            .contains(&packet.as_ref().return_mode));
        Ok(convert_single_return_16_channel(&mut self.context, packet))
    }
}

impl PointCloudConverterEx<Config16Channel<DualReturn>>
    for PointCloudConverter<Config16Channel<DualReturn>>
{
    fn from_config(config: Config16Channel<DualReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<DualReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(packet.as_ref().return_mode == ReturnMode::DualReturn);
        Ok(convert_dual_return_16_channel(&mut self.context, packet))
    }
}

impl PointCloudConverterEx<Config32Channel<StrongestReturn>>
    for PointCloudConverter<Config32Channel<StrongestReturn>>
{
    fn from_config(config: Config32Channel<StrongestReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<SingleReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!([ReturnMode::StrongestReturn, ReturnMode::LastReturn]
            .contains(&packet.as_ref().return_mode));
        Ok(convert_single_return_32_channel(&mut self.context, packet))
    }
}

impl PointCloudConverterEx<Config32Channel<LastReturn>>
    for PointCloudConverter<Config32Channel<LastReturn>>
{
    fn from_config(config: Config32Channel<LastReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<SingleReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!([ReturnMode::StrongestReturn, ReturnMode::LastReturn]
            .contains(&packet.as_ref().return_mode));
        Ok(convert_single_return_32_channel(&mut self.context, packet))
    }
}

impl PointCloudConverterEx<Config32Channel<DualReturn>>
    for PointCloudConverter<Config32Channel<DualReturn>>
{
    fn from_config(config: Config32Channel<DualReturn>) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<DualReturnPoint>>
    where
        P: AsRef<Packet>,
    {
        ensure!(packet.as_ref().return_mode == ReturnMode::DualReturn);
        Ok(convert_dual_return_32_channel(&mut self.context, packet))
    }
}

impl PointCloudConverterEx<DynamicConfig> for PointCloudConverter<DynamicConfig> {
    fn from_config(config: DynamicConfig) -> Self {
        Self {
            context: config.into(),
        }
    }

    fn convert<P>(&mut self, packet: P) -> Fallible<Vec<DynamicPoint>>
    where
        P: AsRef<Packet>,
    {
        use DynamicContext::*;

        let points = match &mut self.context {
            StrongestReturn16Channel(context) => convert_single_return_16_channel(context, packet)
                .into_iter()
                .map(|point| point.to_dynamic())
                .collect::<Vec<_>>(),
            LastReturn16Channel(context) => convert_single_return_16_channel(context, packet)
                .into_iter()
                .map(|point| point.to_dynamic())
                .collect::<Vec<_>>(),
            DualReturn16Channel(context) => convert_dual_return_16_channel(context, packet)
                .into_iter()
                .map(|point| point.to_dynamic())
                .collect::<Vec<_>>(),
            StrongestReturn32Channel(context) => convert_single_return_32_channel(context, packet)
                .into_iter()
                .map(|point| point.to_dynamic())
                .collect::<Vec<_>>(),
            LastReturn32Channel(context) => convert_single_return_32_channel(context, packet)
                .into_iter()
                .map(|point| point.to_dynamic())
                .collect::<Vec<_>>(),
            DualReturn32Channel(context) => convert_dual_return_32_channel(context, packet)
                .into_iter()
                .map(|point| point.to_dynamic())
                .collect::<Vec<_>>(),
        };

        Ok(points)
    }
}
