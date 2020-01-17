//! Defines a set of Velodyne LiDAR configurations.

use super::{
    consts::{
        PUCK_HIRES_AZIMUTH_OFFSETS, PUCK_HIRES_ELEVAION_DEGREES, PUCK_HIRES_HORIZONTAL_OFFSETS,
        PUCK_HIRES_VERTICAL_OFFSETS, PUCK_LITE_AZIMUTH_OFFSETS, PUCK_LITE_ELEVAION_DEGREES,
        PUCK_LITE_HORIZONTAL_OFFSETS, PUCK_LITE_VERTICAL_OFFSETS, VLP_16_AZIMUTH_OFFSETS,
        VLP_16_ELEVAION_DEGREES, VLP_16_HORIZONTAL_OFFSETS, VLP_16_VERTICAL_OFFSETS,
        VLP_32C_AZIMUTH_OFFSETS, VLP_32C_ELEVAION_DEGREES, VLP_32C_HORIZONTAL_OFFSETS,
        VLP_32C_VERTICAL_OFFSETS,
    },
    marker::{DualReturn, DynamicReturn, LastReturn, ReturnTypeMarker, StrongestReturn},
    packet::ReturnMode,
};
use failure::{ensure, Fallible};
use generic_array::{ArrayLength, GenericArray};
use itertools::izip;
use serde::{Deserialize, Serialize};
use std::{
    fs::File,
    io::{prelude::*, BufReader},
    path::Path,
};
use typenum::{U16, U32};
use uom::si::{
    angle::degree,
    f64::{Angle as F64Angle, Length as F64Length},
    length::millimeter,
};

/// Config type for Velodyne LiDARs.
#[derive(Debug, Clone)]
pub struct Config<Size, ReturnType>
where
    Size: ArrayLength<LaserParameter>,
    ReturnType: ReturnTypeMarker,
{
    pub lasers: GenericArray<LaserParameter, Size>,
    pub return_type: ReturnType,
    pub distance_resolution: F64Length,
}

#[derive(Debug, Clone)]
pub struct LaserParameter {
    pub elevation_angle: F64Angle,
    pub azimuth_offset: F64Angle,
    pub vertical_offset: F64Length,
    pub horizontal_offset: F64Length,
}

/// Config builder that builds [Config](Config) type.
#[derive(Debug, Clone)]
pub struct ConfigBuilder {}

impl ConfigBuilder {
    fn vlp_16_laser_params() -> GenericArray<LaserParameter, U16> {
        izip!(
            VLP_16_ELEVAION_DEGREES.iter(),
            VLP_16_VERTICAL_OFFSETS.iter(),
            VLP_16_HORIZONTAL_OFFSETS.iter(),
            VLP_16_AZIMUTH_OFFSETS.iter(),
        )
        .map(
            |(elevation_angle, vertical_offset, horizontal_offset, azimuth_offset)| {
                LaserParameter {
                    elevation_angle: F64Angle::new::<degree>(*elevation_angle),
                    vertical_offset: F64Length::new::<millimeter>(*vertical_offset),
                    horizontal_offset: F64Length::new::<millimeter>(*horizontal_offset),
                    azimuth_offset: F64Angle::new::<degree>(*azimuth_offset),
                }
            },
        )
        .collect()
    }

    fn puck_hires_laser_params() -> GenericArray<LaserParameter, U16> {
        izip!(
            PUCK_HIRES_ELEVAION_DEGREES.iter(),
            PUCK_HIRES_VERTICAL_OFFSETS.iter(),
            PUCK_HIRES_HORIZONTAL_OFFSETS.iter(),
            PUCK_HIRES_AZIMUTH_OFFSETS.iter(),
        )
        .map(
            |(elevation_angle, vertical_offset, horizontal_offset, azimuth_offset)| {
                LaserParameter {
                    elevation_angle: F64Angle::new::<degree>(*elevation_angle),
                    vertical_offset: F64Length::new::<millimeter>(*vertical_offset),
                    horizontal_offset: F64Length::new::<millimeter>(*horizontal_offset),
                    azimuth_offset: F64Angle::new::<degree>(*azimuth_offset),
                }
            },
        )
        .collect()
    }

    fn puck_lite_laser_params() -> GenericArray<LaserParameter, U16> {
        izip!(
            PUCK_LITE_ELEVAION_DEGREES.iter(),
            PUCK_LITE_VERTICAL_OFFSETS.iter(),
            PUCK_LITE_HORIZONTAL_OFFSETS.iter(),
            PUCK_LITE_AZIMUTH_OFFSETS.iter(),
        )
        .map(
            |(elevation_angle, vertical_offset, horizontal_offset, azimuth_offset)| {
                LaserParameter {
                    elevation_angle: F64Angle::new::<degree>(*elevation_angle),
                    vertical_offset: F64Length::new::<millimeter>(*vertical_offset),
                    horizontal_offset: F64Length::new::<millimeter>(*horizontal_offset),
                    azimuth_offset: F64Angle::new::<degree>(*azimuth_offset),
                }
            },
        )
        .collect()
    }

    fn vlp_32c_laser_params() -> GenericArray<LaserParameter, U32> {
        izip!(
            VLP_32C_ELEVAION_DEGREES.iter(),
            VLP_32C_VERTICAL_OFFSETS.iter(),
            VLP_32C_HORIZONTAL_OFFSETS.iter(),
            VLP_32C_AZIMUTH_OFFSETS.iter(),
        )
        .map(
            |(elevation_angle, vertical_offset, horizontal_offset, azimuth_offset)| {
                LaserParameter {
                    elevation_angle: F64Angle::new::<degree>(*elevation_angle),
                    vertical_offset: F64Length::new::<millimeter>(*vertical_offset),
                    horizontal_offset: F64Length::new::<millimeter>(*horizontal_offset),
                    azimuth_offset: F64Angle::new::<degree>(*azimuth_offset),
                }
            },
        )
        .collect()
    }

    pub fn vlp_16_last_return() -> Config<U16, LastReturn> {
        Config {
            lasers: Self::vlp_16_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(2.0),
            return_type: LastReturn,
        }
    }

    pub fn vlp_16_strongest_return() -> Config<U16, StrongestReturn> {
        Config {
            lasers: Self::vlp_16_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(2.0),
            return_type: StrongestReturn,
        }
    }

    pub fn vlp_16_dual_return() -> Config<U16, DualReturn> {
        Config {
            lasers: Self::vlp_16_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(2.0),
            return_type: DualReturn,
        }
    }

    pub fn vlp_16_dynamic_return(return_mode: ReturnMode) -> Config<U16, DynamicReturn> {
        Config {
            lasers: Self::vlp_16_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(2.0),
            return_type: DynamicReturn::from(return_mode),
        }
    }

    pub fn puck_hires_last_return() -> Config<U16, LastReturn> {
        Config {
            lasers: Self::puck_hires_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(2.0),
            return_type: LastReturn,
        }
    }

    pub fn puck_hires_strongest_return() -> Config<U16, StrongestReturn> {
        Config {
            lasers: Self::puck_hires_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(2.0),
            return_type: StrongestReturn,
        }
    }

    pub fn puck_hires_dual_return() -> Config<U16, DualReturn> {
        Config {
            lasers: Self::puck_hires_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(2.0),
            return_type: DualReturn,
        }
    }

    pub fn puck_hires_dynamic_return(return_mode: ReturnMode) -> Config<U16, DynamicReturn> {
        Config {
            lasers: Self::puck_hires_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(2.0),
            return_type: DynamicReturn::from(return_mode),
        }
    }

    pub fn puck_lite_last_return() -> Config<U16, LastReturn> {
        Config {
            lasers: Self::puck_lite_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(2.0),
            return_type: LastReturn,
        }
    }

    pub fn puck_lite_strongest_return() -> Config<U16, StrongestReturn> {
        Config {
            lasers: Self::puck_lite_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(2.0),
            return_type: StrongestReturn,
        }
    }

    pub fn puck_lite_dual_return() -> Config<U16, DualReturn> {
        Config {
            lasers: Self::puck_lite_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(2.0),
            return_type: DualReturn,
        }
    }

    pub fn puck_lite_dynamic_return(return_mode: ReturnMode) -> Config<U16, DynamicReturn> {
        Config {
            lasers: Self::puck_lite_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(2.0),
            return_type: DynamicReturn::from(return_mode),
        }
    }

    pub fn vlp_32c_last_return() -> Config<U32, LastReturn> {
        Config {
            lasers: Self::vlp_32c_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(4.0),
            return_type: LastReturn,
        }
    }

    pub fn vlp_32c_strongest_return() -> Config<U32, StrongestReturn> {
        Config {
            lasers: Self::vlp_32c_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(4.0),
            return_type: StrongestReturn,
        }
    }

    pub fn vlp_32c_dual_return() -> Config<U32, DualReturn> {
        Config {
            lasers: Self::vlp_32c_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(4.0),
            return_type: DualReturn,
        }
    }

    pub fn vlp_32c_dynamic_return(return_mode: ReturnMode) -> Config<U32, DynamicReturn> {
        Config {
            lasers: Self::vlp_32c_laser_params(),
            distance_resolution: F64Length::new::<millimeter>(2.0),
            return_type: DynamicReturn::from(return_mode),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParamsConfig {
    lasers: Vec<LaserConfig>,
    num_lasers: usize,
    distance_resolution: f64,
}

impl ParamsConfig {
    pub fn load<P>(path: P) -> Fallible<Self>
    where
        P: AsRef<Path>,
    {
        let mut reader = BufReader::new(File::open(path)?);
        let config = Self::from_reader(&mut reader)?;
        Ok(config)
    }

    pub fn from_reader<R>(reader: &mut R) -> Fallible<Self>
    where
        R: Read,
    {
        let mut text = String::new();
        reader.read_to_string(&mut text)?;
        let config = Self::from_str(&text)?;
        Ok(config)
    }

    pub fn from_str(text: &str) -> Fallible<Self> {
        let config: Self = serde_yaml::from_str(text)?;
        ensure!(config.distance_resolution > 0.0);
        ensure!(config.num_lasers == config.lasers.len());
        ensure!({
            config
                .lasers
                .iter()
                .enumerate()
                .all(|(idx, params)| idx == params.laser_id)
        });
        Ok(config)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaserConfig {
    pub dist_correction: f64,
    pub dist_correction_x: f64,
    pub dist_correction_y: f64,
    pub focal_distance: f64,
    pub focal_slope: f64,
    pub horiz_offset_correction: Option<f64>,
    pub laser_id: usize,
    pub rot_correction: f64,
    pub vert_correction: f64,
    pub vert_offset_correction: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use failure::Fallible;

    #[test]
    fn buildin_config_test() -> Fallible<()> {
        let _: Config<U16, LastReturn> = ConfigBuilder::vlp_16_last_return();
        let _: Config<U16, StrongestReturn> = ConfigBuilder::vlp_16_strongest_return();
        let _: Config<U16, DualReturn> = ConfigBuilder::vlp_16_dual_return();

        let _: Config<U16, LastReturn> = ConfigBuilder::puck_hires_last_return();
        let _: Config<U16, StrongestReturn> = ConfigBuilder::puck_hires_strongest_return();
        let _: Config<U16, DualReturn> = ConfigBuilder::puck_hires_dual_return();

        let _: Config<U16, LastReturn> = ConfigBuilder::puck_lite_last_return();
        let _: Config<U16, StrongestReturn> = ConfigBuilder::puck_lite_strongest_return();
        let _: Config<U16, DualReturn> = ConfigBuilder::puck_lite_dual_return();

        let _: Config<U32, LastReturn> = ConfigBuilder::vlp_32c_last_return();
        let _: Config<U32, StrongestReturn> = ConfigBuilder::vlp_32c_strongest_return();
        let _: Config<U32, DualReturn> = ConfigBuilder::vlp_32c_dual_return();

        Ok(())
    }

    #[test]
    fn load_yaml_params_test() -> Fallible<()> {
        ParamsConfig::from_str(include_str!("params/32db.yaml"))?;
        ParamsConfig::from_str(include_str!("params/64e_s2.1-sztaki.yaml"))?;
        ParamsConfig::from_str(include_str!("params/64e_s3-xiesc.yaml"))?;
        ParamsConfig::from_str(include_str!("params/64e_utexas.yaml"))?;
        ParamsConfig::from_str(include_str!("params/VeloView-VLP-32C.yaml"))?;
        ParamsConfig::from_str(include_str!("params/VLP16db.yaml"))?;
        ParamsConfig::from_str(include_str!("params/VLP16_hires_db.yaml"))?;
        Ok(())
    }
}
