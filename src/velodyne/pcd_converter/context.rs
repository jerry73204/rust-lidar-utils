//! The module provides context types that are used internally in
//! [PointCloudConverter](crate::velodyne::pcd_converter::PointCloudConverter).

use super::data::{DualReturnPoint, DynamicReturnPoint, SingleReturnPoint};
use crate::velodyne::{
    config::{Config, LaserParameter},
    marker::{
        DualReturn, DynamicReturn, LastReturn, ModelMarker, ReturnTypeMarker, StrongestReturn,
    },
    packet::Block,
};
use generic_array::GenericArray;
use std::marker::PhantomData;
use uom::si::f64::{Length as F64Length, Time as F64Time};

/// Marker trait for converter contexts.
pub trait ConverterContext {
    type OutputPoint;
}

/// Context for last or strongest return mode.
pub struct SingleReturnContext<Model, ReturnType>
where
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
{
    pub lasers: GenericArray<LaserParameter, Model::ParamSize>,
    pub distance_resolution: F64Length,
    pub last_block: Option<(F64Time, Block)>,
    _phantom: PhantomData<ReturnType>,
}

impl<Model> From<Config<Model, LastReturn>> for SingleReturnContext<Model, LastReturn>
where
    Model: ModelMarker,
{
    fn from(config: Config<Model, LastReturn>) -> Self {
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

impl<Model> From<Config<Model, StrongestReturn>> for SingleReturnContext<Model, StrongestReturn>
where
    Model: ModelMarker,
{
    fn from(config: Config<Model, StrongestReturn>) -> Self {
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

impl<Model, ReturnType> ConverterContext for SingleReturnContext<Model, ReturnType>
where
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
{
    type OutputPoint = SingleReturnPoint;
}

/// Context for dual return mode.
pub struct DualReturnContext<Model, ReturnType>
where
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
{
    pub lasers: GenericArray<LaserParameter, Model::ParamSize>,
    pub distance_resolution: F64Length,
    pub last_block: Option<(F64Time, Block, Block)>,
    _phantom: PhantomData<ReturnType>,
}

impl<Model> From<Config<Model, DualReturn>> for DualReturnContext<Model, DualReturn>
where
    Model: ModelMarker,
{
    fn from(config: Config<Model, DualReturn>) -> Self {
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

impl<Model, ReturnType> ConverterContext for DualReturnContext<Model, ReturnType>
where
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
{
    type OutputPoint = DualReturnPoint;
}

impl<Model> From<Config<Model, DynamicReturn>> for DynamicReturnContext<Model, DynamicReturn>
where
    Model: ModelMarker,
{
    fn from(config: Config<Model, DynamicReturn>) -> Self {
        let Config {
            lasers,
            distance_resolution,
            return_type,
        } = config;

        match return_type {
            DynamicReturn::LastReturn | DynamicReturn::StrongestReturn => {
                DynamicReturnContext::SingleReturn(SingleReturnContext {
                    lasers,
                    distance_resolution,
                    last_block: None,
                    _phantom: PhantomData,
                })
            }
            DynamicReturn::DualReturn => DynamicReturnContext::DualReturn(DualReturnContext {
                lasers,
                distance_resolution,
                last_block: None,
                _phantom: PhantomData,
            }),
        }
    }
}

/// Context for dynamically configured return mode.
pub enum DynamicReturnContext<Model, ReturnType>
where
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
{
    SingleReturn(SingleReturnContext<Model, ReturnType>),
    DualReturn(DualReturnContext<Model, ReturnType>),
}

impl<Model, ReturnType> ConverterContext for DynamicReturnContext<Model, ReturnType>
where
    Model: ModelMarker,
    ReturnType: ReturnTypeMarker,
{
    type OutputPoint = DynamicReturnPoint;
}

pub trait ToConverterContext
where
    Self::Context: ConverterContext,
{
    type Context;
}

impl<Model> ToConverterContext for Config<Model, LastReturn>
where
    Model: ModelMarker,
{
    type Context = SingleReturnContext<Model, LastReturn>;
}

impl<Model> ToConverterContext for Config<Model, StrongestReturn>
where
    Model: ModelMarker,
{
    type Context = SingleReturnContext<Model, StrongestReturn>;
}

impl<Model> ToConverterContext for Config<Model, DualReturn>
where
    Model: ModelMarker,
{
    type Context = DualReturnContext<Model, DualReturn>;
}

impl<Model> ToConverterContext for Config<Model, DynamicReturn>
where
    Model: ModelMarker,
{
    type Context = DynamicReturnContext<Model, DynamicReturn>;
}
