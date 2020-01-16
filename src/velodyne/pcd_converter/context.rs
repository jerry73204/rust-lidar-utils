//! The module provides context types that are used internally in
//! [PointCloudConverter](crate::velodyne::pcd_converter::PointCloudConverter).

use super::data::{DualReturnPoint, SingleReturnPoint};
use crate::velodyne::{
    config::{Config, LaserParameter},
    marker::{DualReturn, LastReturn, ReturnTypeMarker, StrongestReturn},
    packet::Block,
};
use generic_array::{ArrayLength, GenericArray};
use std::marker::PhantomData;
use uom::si::f64::{Length as F64Length, Time as F64Time};

/// Marker trait for converter contexts.
pub trait ConverterContext {
    type OutputPoint;
}

/// Context for last or strongest return mode.
pub struct SingleReturnContext<Size, ReturnType>
where
    Size: ArrayLength<LaserParameter>,
    ReturnType: ReturnTypeMarker,
{
    pub lasers: GenericArray<LaserParameter, Size>,
    pub distance_resolution: F64Length,
    pub last_block: Option<(F64Time, Block)>,
    _phantom: PhantomData<ReturnType>,
}

impl<Size> From<Config<Size, LastReturn>> for SingleReturnContext<Size, LastReturn>
where
    Size: ArrayLength<LaserParameter>,
{
    fn from(config: Config<Size, LastReturn>) -> Self {
        let Config {
            lasers,
            distance_resolution,
            ..
        } = config;

        Self {
            lasers,
            distance_resolution,
            last_block: None,
            _phantom: PhantomData,
        }
    }
}

impl<Size> From<Config<Size, StrongestReturn>> for SingleReturnContext<Size, StrongestReturn>
where
    Size: ArrayLength<LaserParameter>,
{
    fn from(config: Config<Size, StrongestReturn>) -> Self {
        let Config {
            lasers,
            distance_resolution,
            ..
        } = config;

        Self {
            lasers,
            distance_resolution,
            last_block: None,
            _phantom: PhantomData,
        }
    }
}

impl<Size, ReturnType> ConverterContext for SingleReturnContext<Size, ReturnType>
where
    Size: ArrayLength<LaserParameter>,
    ReturnType: ReturnTypeMarker,
{
    type OutputPoint = SingleReturnPoint;
}

/// Context for dual return mode.
pub struct DualReturnContext<Size, ReturnType>
where
    Size: ArrayLength<LaserParameter>,
    ReturnType: ReturnTypeMarker,
{
    pub lasers: GenericArray<LaserParameter, Size>,
    pub distance_resolution: F64Length,
    pub last_block: Option<(F64Time, Block, Block)>,
    _phantom: PhantomData<ReturnType>,
}

impl<Size> From<Config<Size, DualReturn>> for DualReturnContext<Size, DualReturn>
where
    Size: ArrayLength<LaserParameter>,
{
    fn from(config: Config<Size, DualReturn>) -> Self {
        let Config {
            lasers,
            distance_resolution,
            ..
        } = config;

        Self {
            lasers,
            distance_resolution,
            last_block: None,
            _phantom: PhantomData,
        }
    }
}

impl<Size, ReturnType> ConverterContext for DualReturnContext<Size, ReturnType>
where
    Size: ArrayLength<LaserParameter>,
    ReturnType: ReturnTypeMarker,
{
    type OutputPoint = DualReturnPoint;
}

pub trait ToConverterContext
where
    Self::Context: ConverterContext,
{
    type Context;
}

impl<Size> ToConverterContext for Config<Size, LastReturn>
where
    Size: ArrayLength<LaserParameter>,
{
    type Context = SingleReturnContext<Size, LastReturn>;
}

impl<Size> ToConverterContext for Config<Size, StrongestReturn>
where
    Size: ArrayLength<LaserParameter>,
{
    type Context = SingleReturnContext<Size, StrongestReturn>;
}

impl<Size> ToConverterContext for Config<Size, DualReturn>
where
    Size: ArrayLength<LaserParameter>,
{
    type Context = DualReturnContext<Size, DualReturn>;
}
