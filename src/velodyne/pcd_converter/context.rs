use super::converter::{DualReturnPoint, DynamicPoint, SingleReturnPoint};
use crate::velodyne::{
    config::{Config16Channel, Config32Channel, DynamicConfig, VelodyneConfigKind},
    marker::{DualReturn, LastReturn, ReturnTypeMarker, StrongestReturn},
    packet::Block,
};
use std::convert::TryFrom;
use uom::si::{
    angle::radian,
    f64::{Angle as F64Angle, Length as F64Length, Time as F64Time},
    length::millimeter,
};

pub trait ConverterContext {}

pub struct SingleReturn16ChannelContext {
    pub altitude_angles: [F64Angle; 16],
    pub vertical_corrections: [F64Length; 16],
    pub last_block: Option<(F64Time, Block)>,
}

impl From<Config16Channel<StrongestReturn>> for SingleReturn16ChannelContext {
    fn from(orig_config: Config16Channel<StrongestReturn>) -> Self {
        let (altitude_angles, vertical_corrections) = convert_16_channel_config(orig_config);

        Self {
            altitude_angles,
            vertical_corrections,
            last_block: None,
        }
    }
}

impl From<Config16Channel<LastReturn>> for SingleReturn16ChannelContext {
    fn from(orig_config: Config16Channel<LastReturn>) -> Self {
        let (altitude_angles, vertical_corrections) = convert_16_channel_config(orig_config);

        Self {
            altitude_angles,
            vertical_corrections,
            last_block: None,
        }
    }
}

impl ConverterContext for SingleReturn16ChannelContext {}

pub struct DualReturn16ChannelContext {
    pub altitude_angles: [F64Angle; 16],
    pub vertical_corrections: [F64Length; 16],
    pub last_block: Option<(F64Time, Block, Block)>,
}

impl From<Config16Channel<DualReturn>> for DualReturn16ChannelContext {
    fn from(orig_config: Config16Channel<DualReturn>) -> Self {
        let (altitude_angles, vertical_corrections) = convert_16_channel_config(orig_config);

        Self {
            altitude_angles,
            vertical_corrections,
            last_block: None,
        }
    }
}

impl ConverterContext for DualReturn16ChannelContext {}

pub struct SingleReturn32ChannelContext {
    pub altitude_angles: [F64Angle; 32],
    pub vertical_corrections: [F64Length; 32],
    pub last_block: Option<(F64Time, Block)>,
}

impl From<Config32Channel<StrongestReturn>> for SingleReturn32ChannelContext {
    fn from(orig_config: Config32Channel<StrongestReturn>) -> Self {
        let (altitude_angles, vertical_corrections) = convert_32_channel_config(orig_config);

        Self {
            altitude_angles,
            vertical_corrections,
            last_block: None,
        }
    }
}

impl From<Config32Channel<LastReturn>> for SingleReturn32ChannelContext {
    fn from(orig_config: Config32Channel<LastReturn>) -> Self {
        let (altitude_angles, vertical_corrections) = convert_32_channel_config(orig_config);

        Self {
            altitude_angles,
            vertical_corrections,
            last_block: None,
        }
    }
}

impl ConverterContext for SingleReturn32ChannelContext {}

pub struct DualReturn32ChannelContext {
    pub altitude_angles: [F64Angle; 32],
    pub vertical_corrections: [F64Length; 32],
    pub last_block: Option<(F64Time, Block, Block)>,
}

impl From<Config32Channel<DualReturn>> for DualReturn32ChannelContext {
    fn from(orig_config: Config32Channel<DualReturn>) -> Self {
        let (altitude_angles, vertical_corrections) = convert_32_channel_config(orig_config);

        Self {
            altitude_angles,
            vertical_corrections,
            last_block: None,
        }
    }
}

impl ConverterContext for DualReturn32ChannelContext {}

pub enum DynamicContext {
    StrongestReturn16Channel(SingleReturn16ChannelContext),
    LastReturn16Channel(SingleReturn16ChannelContext),
    DualReturn16Channel(DualReturn16ChannelContext),
    StrongestReturn32Channel(SingleReturn32ChannelContext),
    LastReturn32Channel(SingleReturn32ChannelContext),
    DualReturn32Channel(DualReturn32ChannelContext),
}

impl ConverterContext for DynamicContext {}

impl From<DynamicConfig> for DynamicContext {
    fn from(orig_config: DynamicConfig) -> Self {
        use DynamicConfig::*;

        match orig_config {
            StrongestReturn16Channel(config) => Self::StrongestReturn16Channel(config.into()),
            LastReturn16Channel(config) => Self::LastReturn16Channel(config.into()),
            DualReturn16Channel(config) => Self::DualReturn16Channel(config.into()),
            StrongestReturn32Channel(config) => Self::StrongestReturn32Channel(config.into()),
            LastReturn32Channel(config) => Self::LastReturn32Channel(config.into()),
            DualReturn32Channel(config) => Self::DualReturn32Channel(config.into()),
        }
    }
}

pub trait ToConverterContext
where
    Self: VelodyneConfigKind,
{
    type Context;
    type Point;
}

impl ToConverterContext for Config16Channel<StrongestReturn> {
    type Context = SingleReturn16ChannelContext;
    type Point = SingleReturnPoint;
}

impl ToConverterContext for Config16Channel<LastReturn> {
    type Context = SingleReturn16ChannelContext;
    type Point = SingleReturnPoint;
}

impl ToConverterContext for Config16Channel<DualReturn> {
    type Context = DualReturn16ChannelContext;
    type Point = DualReturnPoint;
}

impl ToConverterContext for Config32Channel<StrongestReturn> {
    type Context = SingleReturn32ChannelContext;
    type Point = SingleReturnPoint;
}

impl ToConverterContext for Config32Channel<LastReturn> {
    type Context = SingleReturn32ChannelContext;
    type Point = SingleReturnPoint;
}

impl ToConverterContext for Config32Channel<DualReturn> {
    type Context = DualReturn32ChannelContext;
    type Point = DualReturnPoint;
}

impl ToConverterContext for DynamicConfig {
    type Context = DynamicContext;
    type Point = DynamicPoint;
}

fn convert_16_channel_config<ReturnType>(
    orig_config: Config16Channel<ReturnType>,
) -> ([F64Angle; 16], [F64Length; 16])
where
    ReturnType: ReturnTypeMarker,
{
    let vertical_degrees = orig_config.vertical_degrees;
    let vertical_corrections = orig_config.vertical_corrections;

    let angle_vec = vertical_degrees
        .iter()
        .map(|degree| F64Angle::new::<radian>(degree * std::f64::consts::PI / 180.0))
        .collect::<Vec<_>>();
    let altitude_angles = <[F64Angle; 16]>::try_from(angle_vec.as_slice()).unwrap();

    let correction_vec = vertical_corrections
        .iter()
        .map(|correction| F64Length::new::<millimeter>(*correction))
        .collect::<Vec<_>>();
    let vertical_corrections = <[_; 16]>::try_from(correction_vec.as_slice()).unwrap();

    (altitude_angles, vertical_corrections)
}

fn convert_32_channel_config<ReturnType>(
    orig_config: Config32Channel<ReturnType>,
) -> ([F64Angle; 32], [F64Length; 32])
where
    ReturnType: ReturnTypeMarker,
{
    let vertical_degrees = orig_config.vertical_degrees;
    let vertical_corrections = orig_config.vertical_corrections;

    let angle_vec = vertical_degrees
        .iter()
        .map(|degree| F64Angle::new::<radian>(degree * std::f64::consts::PI / 180.0))
        .collect::<Vec<_>>();
    let altitude_angles = <[F64Angle; 32]>::try_from(angle_vec.as_slice()).unwrap();

    let correction_vec = vertical_corrections
        .iter()
        .map(|correction| F64Length::new::<millimeter>(*correction))
        .collect::<Vec<_>>();
    let vertical_corrections = <[_; 32]>::try_from(correction_vec.as_slice()).unwrap();

    (altitude_angles, vertical_corrections)
}
