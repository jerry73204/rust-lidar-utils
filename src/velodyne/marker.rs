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
        fn into_dynamic(self) -> DynamicReturn;
    }

    #[derive(Debug, Clone, Copy)]
    pub struct StrongestReturn;

    impl ReturnTypeMarker for StrongestReturn {
        fn into_dynamic(self) -> DynamicReturn {
            DynamicReturn::StrongestReturn
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub struct LastReturn;

    impl ReturnTypeMarker for LastReturn {
        fn into_dynamic(self) -> DynamicReturn {
            DynamicReturn::LastReturn
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub struct DualReturn;

    impl ReturnTypeMarker for DualReturn {
        fn into_dynamic(self) -> DynamicReturn {
            DynamicReturn::DualReturn
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub enum DynamicReturn {
        LastReturn,
        DualReturn,
        StrongestReturn,
    }

    impl ReturnTypeMarker for DynamicReturn {
        fn into_dynamic(self) -> DynamicReturn {
            self
        }
    }

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

        fn into_dynamic(self) -> DynamicModel;
        fn to_dynamic_params(params: Self::ParamArray) -> Vec<LaserParameter>;
    }

    #[derive(Debug, Clone, Copy)]
    pub struct Vlp16;

    impl ModelMarker for Vlp16 {
        type ParamArray = [LaserParameter; 16];

        fn into_dynamic(self) -> DynamicModel {
            DynamicModel::Vlp16
        }

        fn to_dynamic_params(params: Self::ParamArray) -> Vec<LaserParameter> {
            params.into()
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub struct Vlp32;

    impl ModelMarker for Vlp32 {
        type ParamArray = [LaserParameter; 32];

        fn into_dynamic(self) -> DynamicModel {
            DynamicModel::Vlp32
        }

        fn to_dynamic_params(params: Self::ParamArray) -> Vec<LaserParameter> {
            params.into()
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub enum DynamicModel {
        Vlp16,
        Vlp32,
    }

    impl ModelMarker for DynamicModel {
        type ParamArray = Vec<LaserParameter>;

        fn into_dynamic(self) -> DynamicModel {
            self
        }

        fn to_dynamic_params(params: Self::ParamArray) -> Vec<LaserParameter> {
            params
        }
    }
}
