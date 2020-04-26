//! Marker traits and types that are mainly used by config types.

use super::{config::LaserParameter, packet::ReturnMode};
use generic_array::ArrayLength;
use std::fmt::Debug;
use typenum::{U16, U32};

pub trait ReturnTypeMarker
where
    Self: Debug + Clone,
{
}

#[derive(Debug, Clone, Copy)]
pub struct StrongestReturn;

impl ReturnTypeMarker for StrongestReturn {}

#[derive(Debug, Clone, Copy)]
pub struct LastReturn;

impl ReturnTypeMarker for LastReturn {}

#[derive(Debug, Clone, Copy)]
pub struct DualReturn;

impl ReturnTypeMarker for DualReturn {}

#[derive(Debug, Clone, Copy)]
pub enum DynamicReturn {
    LastReturn,
    DualReturn,
    StrongestReturn,
}

impl ReturnTypeMarker for DynamicReturn {}

impl From<ReturnMode> for DynamicReturn {
    fn from(mode: ReturnMode) -> DynamicReturn {
        match mode {
            ReturnMode::LastReturn => DynamicReturn::LastReturn,
            ReturnMode::StrongestReturn => DynamicReturn::StrongestReturn,
            ReturnMode::DualReturn => DynamicReturn::DualReturn,
        }
    }
}

pub trait ModelMarker
where
    Self::ParamSize: ArrayLength<LaserParameter>,
{
    type ParamSize;
}

#[derive(Debug, Clone, Copy)]
pub struct Vlp16;

impl ModelMarker for Vlp16 {
    type ParamSize = U16;
}

#[derive(Debug, Clone, Copy)]
pub struct Vlp32;

impl ModelMarker for Vlp32 {
    type ParamSize = U32;
}
