//! Defines a set of Velodyne LiDAR configurations.

use super::{
    consts::{
        PUKE_HI_RES_VERTICAL_CORRECTIONS, PUKE_HI_RES_VERTICAL_DEGREES,
        PUKE_LITE_VERTICAL_CORRECTIONS, PUKE_LITE_VERTICAL_DEGREES, VLP_16_VERTICAL_CORRECTIONS,
        VLP_16_VERTICAL_DEGREES,
    },
    marker::{DualReturn, LastReturn, ReturnTypeMarker, StrongestReturn},
    packet::ReturnMode,
};
use failure::{bail, Fallible};
use std::{fmt::Debug, marker::PhantomData};

pub trait VelodyneConfigKind
where
    Self: Debug + Clone,
{
}

#[derive(Debug, Clone)]
pub struct ConfigBuilder {
    params: Option<Parameters>,
    return_mode: Option<ReturnMode>,
}

impl ConfigBuilder {
    pub fn new() -> Self {
        Self {
            params: None,
            return_mode: None,
        }
    }

    pub fn build(self) -> Fallible<DynamicConfig> {
        use DynamicConfig::*;
        use Parameters::*;
        use ReturnMode::*;

        let config = match (self.params, self.return_mode) {
            (Some(Channel16(vertical_degrees, vertical_corrections)), Some(StrongestReturn)) => {
                StrongestReturn16Channel(Config16Channel {
                    vertical_degrees,
                    vertical_corrections,
                    _phantom: PhantomData,
                })
            }
            (Some(Channel16(vertical_degrees, vertical_corrections)), Some(LastReturn)) => {
                LastReturn16Channel(Config16Channel {
                    vertical_degrees,
                    vertical_corrections,
                    _phantom: PhantomData,
                })
            }
            (Some(Channel16(vertical_degrees, vertical_corrections)), Some(DualReturn)) => {
                DualReturn16Channel(Config16Channel {
                    vertical_degrees,
                    vertical_corrections,
                    _phantom: PhantomData,
                })
            }
            (Some(Channel32(vertical_degrees, vertical_corrections)), Some(StrongestReturn)) => {
                StrongestReturn32Channel(Config32Channel {
                    vertical_degrees,
                    vertical_corrections,
                    _phantom: PhantomData,
                })
            }
            (Some(Channel32(vertical_degrees, vertical_corrections)), Some(LastReturn)) => {
                LastReturn32Channel(Config32Channel {
                    vertical_degrees,
                    vertical_corrections,
                    _phantom: PhantomData,
                })
            }
            (Some(Channel32(vertical_degrees, vertical_corrections)), Some(DualReturn)) => {
                DualReturn32Channel(Config32Channel {
                    vertical_degrees,
                    vertical_corrections,
                    _phantom: PhantomData,
                })
            }
            _ => bail!("the builder is not correctly configured"),
        };

        Ok(config)
    }

    pub fn return_mode(mut self, return_mode: ReturnMode) -> Self {
        self.return_mode = Some(return_mode);
        self
    }

    pub fn vlp_16_params(mut self) -> Self {
        self.params = Some(Parameters::Channel16(
            VLP_16_VERTICAL_DEGREES,
            VLP_16_VERTICAL_CORRECTIONS,
        ));
        self
    }

    pub fn puke_lite_params(mut self) -> Self {
        self.params = Some(Parameters::Channel16(
            PUKE_LITE_VERTICAL_DEGREES,
            PUKE_LITE_VERTICAL_CORRECTIONS,
        ));
        self
    }

    pub fn puke_hi_res_params(mut self) -> Self {
        self.params = Some(Parameters::Channel16(
            PUKE_HI_RES_VERTICAL_DEGREES,
            PUKE_HI_RES_VERTICAL_CORRECTIONS,
        ));
        self
    }

    pub fn channel_16_params(
        mut self,
        vertical_degrees: [f64; 16],
        vertical_corrections: [f64; 16],
    ) -> Self {
        self.params = Some(Parameters::Channel16(
            vertical_degrees,
            vertical_corrections,
        ));
        self
    }

    pub fn channel_32_params(
        mut self,
        vertical_degrees: [f64; 32],
        vertical_corrections: [f64; 32],
    ) -> Self {
        self.params = Some(Parameters::Channel32(
            vertical_degrees,
            vertical_corrections,
        ));
        self
    }
}

#[derive(Debug, Clone)]
pub enum Parameters {
    Channel16([f64; 16], [f64; 16]),
    Channel32([f64; 32], [f64; 32]),
}

#[derive(Debug, Clone)]
pub struct Config16Channel<ReturnType>
where
    ReturnType: ReturnTypeMarker,
{
    /// Vertical angles per laser in degrees.
    pub vertical_degrees: [f64; 16],
    /// Vertical correction per laser in millimeters.
    pub vertical_corrections: [f64; 16],
    _phantom: PhantomData<ReturnType>,
}

impl<ReturnType> VelodyneConfigKind for Config16Channel<ReturnType> where
    ReturnType: ReturnTypeMarker
{
}

impl<ReturnType> Config16Channel<ReturnType>
where
    ReturnType: ReturnTypeMarker,
{
    pub fn vlp_16_config() -> Self {
        Self {
            vertical_degrees: VLP_16_VERTICAL_DEGREES,
            vertical_corrections: VLP_16_VERTICAL_CORRECTIONS,
            _phantom: PhantomData,
        }
    }

    pub fn puke_lite_config() -> Self {
        Self {
            vertical_degrees: PUKE_LITE_VERTICAL_DEGREES,
            vertical_corrections: PUKE_LITE_VERTICAL_CORRECTIONS,
            _phantom: PhantomData,
        }
    }

    pub fn puke_hi_res_config() -> Self {
        Self {
            vertical_degrees: PUKE_HI_RES_VERTICAL_DEGREES,
            vertical_corrections: PUKE_HI_RES_VERTICAL_CORRECTIONS,
            _phantom: PhantomData,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Config32Channel<ReturnType>
where
    ReturnType: ReturnTypeMarker,
{
    /// Vertical angles per laser in degrees.
    pub vertical_degrees: [f64; 32],
    /// Vertical correction per laser in millimeters.
    pub vertical_corrections: [f64; 32],
    _phantom: PhantomData<ReturnType>,
}

impl<ReturnType> VelodyneConfigKind for Config32Channel<ReturnType> where
    ReturnType: ReturnTypeMarker
{
}

#[derive(Debug, Clone)]
pub enum DynamicConfig {
    StrongestReturn16Channel(Config16Channel<StrongestReturn>),
    LastReturn16Channel(Config16Channel<LastReturn>),
    DualReturn16Channel(Config16Channel<DualReturn>),
    StrongestReturn32Channel(Config32Channel<StrongestReturn>),
    LastReturn32Channel(Config32Channel<LastReturn>),
    DualReturn32Channel(Config32Channel<DualReturn>),
}

impl VelodyneConfigKind for DynamicConfig {}
