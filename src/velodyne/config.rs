//! Defines a set of Velodyne LiDAR configurations.

use super::{
    consts,
    marker::{
        DualReturn, DynamicModel, DynamicReturn, LastReturn, ModelMarker, ReturnTypeMarker,
        StrongestReturn, Vlp16, Vlp32,
    },
    packet::ReturnMode,
};
use crate::common::*;

pub use config_::*;
pub use param_config::*;
pub use params::*;

mod config_ {

    use super::*;

    /// Config type for Velodyne LiDARs.
    #[derive(Debug, Clone)]
    pub struct Config<Model, ReturnType>
    where
        Model: ModelMarker,
        ReturnType: ReturnTypeMarker,
    {
        pub model: Model,
        pub lasers: Model::ParamArray,
        pub return_type: ReturnType,
        pub distance_resolution: Length,
    }

    #[allow(non_camel_case_types)]
    pub type Vlp16_Strongest_Config = Config<Vlp16, StrongestReturn>;
    #[allow(non_camel_case_types)]
    pub type Vlp16_Last_Config = Config<Vlp16, LastReturn>;
    #[allow(non_camel_case_types)]
    pub type Vlp16_Dual_Config = Config<Vlp16, DualReturn>;
    #[allow(non_camel_case_types)]
    pub type Vlp16_Dynamic_Config = Config<Vlp16, DynamicReturn>;
    #[allow(non_camel_case_types)]
    pub type Vlp32_Strongest_Config = Config<Vlp32, StrongestReturn>;
    #[allow(non_camel_case_types)]
    pub type Vlp32_Last_Config = Config<Vlp32, LastReturn>;
    #[allow(non_camel_case_types)]
    pub type Vlp32_Dual_Config = Config<Vlp32, DualReturn>;
    #[allow(non_camel_case_types)]
    pub type Vlp32_Dynamic_Config = Config<Vlp32, DynamicReturn>;
    #[allow(non_camel_case_types)]
    pub type Dynamic_Config = Config<DynamicModel, DynamicReturn>;

    impl<Model, ReturnType> Config<Model, ReturnType>
    where
        Model: ModelMarker,
        ReturnType: ReturnTypeMarker,
    {
        pub fn into_dyn(self) -> Dynamic_Config {
            let Self {
                model,
                lasers,
                return_type,
                distance_resolution,
            } = self;

            Dynamic_Config {
                model: model.into_dynamic(),
                lasers: Model::to_dynamic_params(lasers),
                return_type: return_type.into_dynamic(),
                distance_resolution,
            }
        }
    }

    impl Vlp16_Last_Config {
        pub fn vlp_16_last_return() -> Self {
            Config {
                model: Vlp16,
                lasers: vlp_16_laser_params(),
                distance_resolution: *consts::vlp_16::DISTANCE_RESOLUTION,
                return_type: LastReturn,
            }
        }

        pub fn puck_hires_last_return() -> Self {
            Config {
                model: Vlp16,
                lasers: puck_hires_laser_params(),
                distance_resolution: *consts::puck_hires::DISTANCE_RESOLUTION,
                return_type: LastReturn,
            }
        }

        pub fn puck_lite_last_return() -> Self {
            Config {
                model: Vlp16,
                lasers: puck_lite_laser_params(),
                distance_resolution: *consts::puck_lite::DISTANCE_RESOLUTION,
                return_type: LastReturn,
            }
        }
    }

    impl Vlp16_Strongest_Config {
        pub fn vlp_16_strongest_return() -> Self {
            Config {
                model: Vlp16,
                lasers: vlp_16_laser_params(),
                distance_resolution: *consts::vlp_16::DISTANCE_RESOLUTION,
                return_type: StrongestReturn,
            }
        }

        pub fn puck_hires_strongest_return() -> Self {
            Config {
                model: Vlp16,
                lasers: puck_hires_laser_params(),
                distance_resolution: *consts::puck_hires::DISTANCE_RESOLUTION,
                return_type: StrongestReturn,
            }
        }

        pub fn puck_lite_strongest_return() -> Self {
            Config {
                model: Vlp16,
                lasers: puck_lite_laser_params(),
                distance_resolution: *consts::puck_lite::DISTANCE_RESOLUTION,
                return_type: StrongestReturn,
            }
        }
    }

    impl Vlp16_Dual_Config {
        pub fn vlp_16_dual_return() -> Self {
            Config {
                model: Vlp16,
                lasers: vlp_16_laser_params(),
                distance_resolution: *consts::vlp_16::DISTANCE_RESOLUTION,
                return_type: DualReturn,
            }
        }

        pub fn puck_hires_dual_return() -> Self {
            Config {
                model: Vlp16,
                lasers: puck_hires_laser_params(),
                distance_resolution: *consts::puck_hires::DISTANCE_RESOLUTION,
                return_type: DualReturn,
            }
        }

        pub fn puck_lite_dual_return() -> Self {
            Config {
                model: Vlp16,
                lasers: puck_lite_laser_params(),
                distance_resolution: *consts::puck_lite::DISTANCE_RESOLUTION,
                return_type: DualReturn,
            }
        }
    }

    impl Vlp16_Dynamic_Config {
        pub fn vlp_16_dynamic_return(return_mode: ReturnMode) -> Self {
            Config {
                model: Vlp16,
                lasers: vlp_16_laser_params(),
                distance_resolution: *consts::vlp_16::DISTANCE_RESOLUTION,
                return_type: DynamicReturn::from(return_mode),
            }
        }

        pub fn puck_hires_dynamic_return(return_mode: ReturnMode) -> Self {
            Config {
                model: Vlp16,
                lasers: puck_hires_laser_params(),
                distance_resolution: *consts::puck_hires::DISTANCE_RESOLUTION,
                return_type: DynamicReturn::from(return_mode),
            }
        }

        pub fn puck_lite_dynamic_return(return_mode: ReturnMode) -> Self {
            Config {
                model: Vlp16,
                lasers: puck_lite_laser_params(),
                distance_resolution: *consts::puck_lite::DISTANCE_RESOLUTION,
                return_type: DynamicReturn::from(return_mode),
            }
        }
    }

    impl Vlp32_Last_Config {
        pub fn vlp_32c_last_return() -> Self {
            Config {
                model: Vlp32,
                lasers: vlp_32c_laser_params(),
                distance_resolution: *consts::vlp_32c::DISTANCE_RESOLUTION,
                return_type: LastReturn,
            }
        }
    }

    impl Vlp32_Strongest_Config {
        pub fn vlp_32c_strongest_return() -> Self {
            Config {
                model: Vlp32,
                lasers: vlp_32c_laser_params(),
                distance_resolution: *consts::vlp_32c::DISTANCE_RESOLUTION,
                return_type: StrongestReturn,
            }
        }
    }

    impl Vlp32_Dual_Config {
        pub fn vlp_32c_dual_return() -> Self {
            Config {
                model: Vlp32,
                lasers: vlp_32c_laser_params(),
                distance_resolution: *consts::vlp_32c::DISTANCE_RESOLUTION,
                return_type: DualReturn,
            }
        }
    }

    impl Vlp32_Dynamic_Config {
        pub fn vlp_32c_dynamic_return(return_mode: ReturnMode) -> Self {
            Config {
                model: Vlp32,
                lasers: vlp_32c_laser_params(),
                distance_resolution: *consts::vlp_32c::DISTANCE_RESOLUTION,
                return_type: DynamicReturn::from(return_mode),
            }
        }
    }
}

mod params {
    use super::*;

    #[derive(Debug, Clone)]
    pub struct LaserParameter {
        pub elevation_angle: Angle,
        pub azimuth_offset: Angle,
        pub vertical_offset: Length,
        pub horizontal_offset: Length,
    }

    pub fn vlp_16_laser_params() -> [LaserParameter; 16] {
        let mut params: [MaybeUninit<LaserParameter>; 16] =
            unsafe { MaybeUninit::uninit().assume_init() };
        izip!(
            params.iter_mut(),
            consts::vlp_16::ELEVAION_DEGREES.iter(),
            consts::vlp_16::VERTICAL_OFFSETS.iter(),
            consts::vlp_16::HORIZONTAL_OFFSETS.iter(),
            consts::vlp_16::AZIMUTH_OFFSETS.iter(),
        )
        .for_each(
            |(param, &elevation_angle, &vertical_offset, &horizontal_offset, &azimuth_offset)| {
                *param = MaybeUninit::new(LaserParameter {
                    elevation_angle: Angle::from_degrees(elevation_angle),
                    vertical_offset: Length::from_millimeters(vertical_offset),
                    horizontal_offset: Length::from_millimeters(horizontal_offset),
                    azimuth_offset: Angle::from_degrees(azimuth_offset),
                });
            },
        );

        unsafe { mem::transmute::<_, [LaserParameter; 16]>(params) }
    }

    pub fn puck_hires_laser_params() -> [LaserParameter; 16] {
        let mut params: [MaybeUninit<LaserParameter>; 16] =
            unsafe { MaybeUninit::uninit().assume_init() };
        izip!(
            params.iter_mut(),
            consts::puck_hires::ELEVAION_DEGREES.iter(),
            consts::puck_hires::VERTICAL_OFFSETS.iter(),
            consts::puck_hires::HORIZONTAL_OFFSETS.iter(),
            consts::puck_hires::AZIMUTH_OFFSETS.iter(),
        )
        .for_each(
            |(param, &elevation_angle, &vertical_offset, &horizontal_offset, &azimuth_offset)| {
                *param = MaybeUninit::new(LaserParameter {
                    elevation_angle: Angle::from_degrees(elevation_angle),
                    vertical_offset: Length::from_millimeters(vertical_offset),
                    horizontal_offset: Length::from_millimeters(horizontal_offset),
                    azimuth_offset: Angle::from_degrees(azimuth_offset),
                });
            },
        );

        unsafe { mem::transmute::<_, [LaserParameter; 16]>(params) }
    }

    pub fn puck_lite_laser_params() -> [LaserParameter; 16] {
        let mut params: [MaybeUninit<LaserParameter>; 16] =
            unsafe { MaybeUninit::uninit().assume_init() };
        izip!(
            params.iter_mut(),
            consts::puck_lite::ELEVAION_DEGREES.iter(),
            consts::puck_lite::VERTICAL_OFFSETS.iter(),
            consts::puck_lite::HORIZONTAL_OFFSETS.iter(),
            consts::puck_lite::AZIMUTH_OFFSETS.iter(),
        )
        .for_each(
            |(param, &elevation_angle, &vertical_offset, &horizontal_offset, &azimuth_offset)| {
                *param = MaybeUninit::new(LaserParameter {
                    elevation_angle: Angle::from_degrees(elevation_angle),
                    vertical_offset: Length::from_millimeters(vertical_offset),
                    horizontal_offset: Length::from_millimeters(horizontal_offset),
                    azimuth_offset: Angle::from_degrees(azimuth_offset),
                });
            },
        );

        unsafe { mem::transmute::<_, [LaserParameter; 16]>(params) }
    }

    pub fn vlp_32c_laser_params() -> [LaserParameter; 32] {
        let mut params: [MaybeUninit<LaserParameter>; 32] =
            unsafe { MaybeUninit::uninit().assume_init() };
        izip!(
            params.iter_mut(),
            consts::vlp_32c::ELEVAION_DEGREES.iter(),
            consts::vlp_32c::VERTICAL_OFFSETS.iter(),
            consts::vlp_32c::HORIZONTAL_OFFSETS.iter(),
            consts::vlp_32c::AZIMUTH_OFFSETS.iter(),
        )
        .for_each(
            |(param, &elevation_angle, &vertical_offset, &horizontal_offset, &azimuth_offset)| {
                *param = MaybeUninit::new(LaserParameter {
                    elevation_angle: Angle::from_degrees(elevation_angle),
                    vertical_offset: Length::from_millimeters(vertical_offset),
                    horizontal_offset: Length::from_millimeters(horizontal_offset),
                    azimuth_offset: Angle::from_degrees(azimuth_offset),
                });
            },
        );

        unsafe { mem::transmute::<_, [LaserParameter; 32]>(params) }
    }
}

mod param_config {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    #[serde(try_from = "ParamsConfigUnchecked", into = "ParamsConfigUnchecked")]
    pub struct ParamsConfig {
        lasers: Vec<LaserConfig>,
        num_lasers: usize,
        distance_resolution: R64,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    struct ParamsConfigUnchecked {
        pub lasers: Vec<LaserConfig>,
        pub num_lasers: usize,
        pub distance_resolution: R64,
    }

    #[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct LaserConfig {
        pub dist_correction: R64,
        pub dist_correction_x: R64,
        pub dist_correction_y: R64,
        pub focal_distance: R64,
        pub focal_slope: R64,
        pub horiz_offset_correction: Option<R64>,
        pub laser_id: usize,
        pub rot_correction: R64,
        pub vert_correction: R64,
        pub vert_offset_correction: R64,
    }

    impl ParamsConfig {
        pub fn open_yaml<P>(path: P) -> Result<Self>
        where
            P: AsRef<Path>,
        {
            let mut reader = BufReader::new(File::open(path)?);
            let config = Self::from_reader_yaml(&mut reader)?;
            Ok(config)
        }

        pub fn from_reader_yaml<R>(reader: &mut R) -> Result<Self>
        where
            R: Read,
        {
            let mut text = String::new();
            reader.read_to_string(&mut text)?;
            let config = serde_yaml::from_str(&text)?;
            Ok(config)
        }

        /// Get a reference to the params config's lasers.
        pub fn lasers(&self) -> &[LaserConfig] {
            self.lasers.as_ref()
        }

        /// Get the params config's distance resolution.
        pub fn distance_resolution(&self) -> R64 {
            self.distance_resolution
        }

        /// Get the params config's num lasers.
        pub fn num_lasers(&self) -> usize {
            self.num_lasers
        }
    }

    impl TryFrom<ParamsConfigUnchecked> for ParamsConfig {
        type Error = Error;

        fn try_from(from: ParamsConfigUnchecked) -> Result<Self, Self::Error> {
            let ParamsConfigUnchecked {
                lasers,
                num_lasers,
                distance_resolution,
            } = from;

            ensure!(
                from.distance_resolution > 0.0,
                "distance_resolution must be positive"
            );
            ensure!(
                num_lasers == lasers.len(),
                "the number of element in lasers field does not match num_layers"
            );
            ensure!(
                {
                    lasers
                        .iter()
                        .enumerate()
                        .all(|(idx, params)| idx == params.laser_id)
                },
                "the laser_id in lasers field must be consecutively counted from 1"
            );

            Ok(Self {
                lasers,
                num_lasers,
                distance_resolution,
            })
        }
    }

    impl From<ParamsConfig> for ParamsConfigUnchecked {
        fn from(from: ParamsConfig) -> Self {
            let ParamsConfig {
                lasers,
                num_lasers,
                distance_resolution,
            } = from;

            Self {
                lasers,
                num_lasers,
                distance_resolution,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use anyhow::Result;

    #[test]
    fn buildin_config_test() -> Result<()> {
        let _: Vlp16_Last_Config = Config::vlp_16_last_return();
        let _: Vlp16_Strongest_Config = Config::vlp_16_strongest_return();
        let _: Vlp16_Dual_Config = Config::vlp_16_dual_return();

        let _: Vlp16_Last_Config = Config::puck_hires_last_return();
        let _: Vlp16_Strongest_Config = Config::puck_hires_strongest_return();
        let _: Vlp16_Dual_Config = Config::puck_hires_dual_return();

        let _: Vlp16_Last_Config = Config::puck_lite_last_return();
        let _: Vlp16_Strongest_Config = Config::puck_lite_strongest_return();
        let _: Vlp16_Dual_Config = Config::puck_lite_dual_return();

        let _: Vlp32_Last_Config = Config::vlp_32c_last_return();
        let _: Vlp32_Strongest_Config = Config::vlp_32c_strongest_return();
        let _: Vlp32_Dual_Config = Config::vlp_32c_dual_return();

        Ok(())
    }

    #[test]
    fn load_yaml_params_test() -> Result<()> {
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/32db.yaml"
        ))?;
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/64e_s2.1-sztaki.yaml"
        ))?;
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/64e_s3-xiesc.yaml"
        ))?;
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/64e_utexas.yaml"
        ))?;
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/VeloView-VLP-32C.yaml"
        ))?;
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/VLP16db.yaml"
        ))?;
        ParamsConfig::open_yaml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/config/velodyne/VLP16_hires_db.yaml"
        ))?;
        Ok(())
    }
}
