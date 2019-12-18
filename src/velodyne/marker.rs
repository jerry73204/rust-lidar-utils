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
