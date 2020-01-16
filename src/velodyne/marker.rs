//! Marker traits and types that are mainly used by config types.

use std::fmt::Debug;

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
