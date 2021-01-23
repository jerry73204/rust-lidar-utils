//! Marker traits and types that are mainly used by config types.

use super::{config::LaserParameter, packet::ReturnMode};
use crate::common::*;

pub use model::*;
pub use return_type::*;

mod return_type {
    use super::*;

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
}

mod model {
    pub use super::*;

    pub trait ModelMarker {
        type ParamArray;
    }

    #[derive(Debug, Clone, Copy)]
    pub struct Vlp16;

    impl ModelMarker for Vlp16 {
        type ParamArray = [LaserParameter; 16];
    }

    #[derive(Debug, Clone, Copy)]
    pub struct Vlp32;

    impl ModelMarker for Vlp32 {
        type ParamArray = [LaserParameter; 32];
    }

    #[derive(Debug, Clone, Copy)]
    pub enum DynamicModel {
        Vlp16,
        Vlp32,
    }

    impl ModelMarker for DynamicModel {
        type ParamArray = Vec<LaserParameter>;
    }
}
