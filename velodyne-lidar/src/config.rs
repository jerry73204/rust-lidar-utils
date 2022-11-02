//! Defines a set of Velodyne LiDAR configurations.

use crate::{
    common::*,
    consts,
    converter::ConverterKind,
    firing::types::FiringFormat,
    frame_xyz::batcher::FrameXyzBatcherKind,
    packet::{ProductID, ReturnMode},
};

pub use config_::*;
mod config_ {
    use super::*;

    // type

    /// Config type for Velodyne LiDARs.
    #[derive(Debug, Clone)]
    pub struct Config {
        pub lasers: Vec<LaserParameter>,
        pub return_mode: ReturnMode,
        pub product_id: ProductID,
        pub distance_resolution: Length,
    }

    // impls

    impl Config {
        pub fn firing_format(&self) -> Option<FiringFormat> {
            FiringFormat::new(self.product_id, self.return_mode)
        }
    }

    impl Config {
        pub fn build_converter(self) -> Result<ConverterKind> {
            ConverterKind::from_config(self)
        }

        pub fn build_frame_xyz_batcher(&self) -> Result<FrameXyzBatcherKind> {
            FrameXyzBatcherKind::from_config(self)
        }

        pub fn new_vlp_16_last() -> Self {
            Self {
                lasers: LaserParameter::vlp_16().to_vec(),
                return_mode: ReturnMode::Last,
                product_id: ProductID::VLP16,
                distance_resolution: *consts::vlp_16::DISTANCE_RESOLUTION,
            }
        }

        pub fn new_vlp_16_strongest() -> Self {
            Self {
                lasers: LaserParameter::vlp_16().to_vec(),
                return_mode: ReturnMode::Strongest,
                product_id: ProductID::VLP16,
                distance_resolution: *consts::vlp_16::DISTANCE_RESOLUTION,
            }
        }

        pub fn new_vlp_16_dual() -> Self {
            Self {
                lasers: LaserParameter::vlp_16().to_vec(),
                return_mode: ReturnMode::Dual,
                product_id: ProductID::VLP16,
                distance_resolution: *consts::vlp_16::DISTANCE_RESOLUTION,
            }
        }

        pub fn new_puck_hires_last() -> Self {
            Self {
                lasers: LaserParameter::puck_hires().to_vec(),
                return_mode: ReturnMode::Last,
                product_id: ProductID::PuckHiRes,
                distance_resolution: *consts::puck_hires::DISTANCE_RESOLUTION,
            }
        }

        pub fn new_puck_hires_strongest() -> Self {
            Self {
                lasers: LaserParameter::puck_hires().to_vec(),
                return_mode: ReturnMode::Strongest,
                product_id: ProductID::PuckHiRes,
                distance_resolution: *consts::puck_hires::DISTANCE_RESOLUTION,
            }
        }

        pub fn new_puck_hires_dual() -> Self {
            Self {
                lasers: LaserParameter::puck_hires().to_vec(),
                return_mode: ReturnMode::Dual,
                product_id: ProductID::PuckHiRes,
                distance_resolution: *consts::puck_hires::DISTANCE_RESOLUTION,
            }
        }

        pub fn new_puck_lite_last() -> Self {
            Self {
                lasers: LaserParameter::puck_lite().to_vec(),
                return_mode: ReturnMode::Last,
                product_id: ProductID::PuckLite,
                distance_resolution: *consts::puck_lite::DISTANCE_RESOLUTION,
            }
        }

        pub fn new_puck_lite_strongest() -> Self {
            Self {
                lasers: LaserParameter::puck_lite().to_vec(),
                return_mode: ReturnMode::Strongest,
                product_id: ProductID::PuckLite,
                distance_resolution: *consts::puck_lite::DISTANCE_RESOLUTION,
            }
        }

        pub fn new_puck_lite_dual() -> Self {
            Self {
                lasers: LaserParameter::puck_lite().to_vec(),
                return_mode: ReturnMode::Dual,
                product_id: ProductID::PuckLite,
                distance_resolution: *consts::puck_lite::DISTANCE_RESOLUTION,
            }
        }

        pub fn new_vlp_32c_last() -> Self {
            Self {
                lasers: LaserParameter::vlp_32c().to_vec(),
                return_mode: ReturnMode::Last,
                product_id: ProductID::VLP32C,
                distance_resolution: *consts::vlp_32c::DISTANCE_RESOLUTION,
            }
        }

        pub fn new_vlp_32c_strongest() -> Self {
            Self {
                lasers: LaserParameter::vlp_32c().to_vec(),
                return_mode: ReturnMode::Strongest,
                product_id: ProductID::VLP32C,
                distance_resolution: *consts::vlp_32c::DISTANCE_RESOLUTION,
            }
        }

        pub fn new_vlp_32c_dual() -> Self {
            Self {
                lasers: LaserParameter::vlp_32c().to_vec(),
                return_mode: ReturnMode::Dual,
                product_id: ProductID::VLP32C,
                distance_resolution: *consts::vlp_32c::DISTANCE_RESOLUTION,
            }
        }
    }
}

pub use params::*;
mod params {
    use super::*;

    #[derive(Debug, Clone)]
    pub struct LaserParameter {
        pub elevation: Angle,
        pub azimuth_offset: Angle,
        pub vertical_offset: Length,
        pub horizontal_offset: Length,
    }

    impl LaserParameter {
        pub fn vlp_16() -> [LaserParameter; 16] {
            let params: Vec<_> = izip!(
                consts::vlp_16::ELEVAION_DEGREES,
                consts::vlp_16::VERTICAL_OFFSETS,
                consts::vlp_16::HORIZONTAL_OFFSETS,
                consts::vlp_16::AZIMUTH_OFFSETS,
            )
            .map(
                |(elevation, vertical_offset, horizontal_offset, azimuth_offset)| LaserParameter {
                    elevation: Angle::from_degrees(elevation),
                    vertical_offset: Length::from_millimeters(vertical_offset),
                    horizontal_offset: Length::from_millimeters(horizontal_offset),
                    azimuth_offset: Angle::from_degrees(azimuth_offset),
                },
            )
            .collect();

            params.try_into().unwrap_or_else(|_| unreachable!())
        }

        pub fn puck_hires() -> [LaserParameter; 16] {
            let params: Vec<_> = izip!(
                consts::puck_hires::ELEVAION_DEGREES,
                consts::puck_hires::VERTICAL_OFFSETS,
                consts::puck_hires::HORIZONTAL_OFFSETS,
                consts::puck_hires::AZIMUTH_OFFSETS,
            )
            .map(
                |(elevation, vertical_offset, horizontal_offset, azimuth_offset)| LaserParameter {
                    elevation: Angle::from_degrees(elevation),
                    vertical_offset: Length::from_millimeters(vertical_offset),
                    horizontal_offset: Length::from_millimeters(horizontal_offset),
                    azimuth_offset: Angle::from_degrees(azimuth_offset),
                },
            )
            .collect();

            params.try_into().unwrap_or_else(|_| unreachable!())
        }

        pub fn puck_lite() -> [LaserParameter; 16] {
            let params: Vec<_> = izip!(
                consts::puck_lite::ELEVAION_DEGREES,
                consts::puck_lite::VERTICAL_OFFSETS,
                consts::puck_lite::HORIZONTAL_OFFSETS,
                consts::puck_lite::AZIMUTH_OFFSETS,
            )
            .map(
                |(elevation, vertical_offset, horizontal_offset, azimuth_offset)| LaserParameter {
                    elevation: Angle::from_degrees(elevation),
                    vertical_offset: Length::from_millimeters(vertical_offset),
                    horizontal_offset: Length::from_millimeters(horizontal_offset),
                    azimuth_offset: Angle::from_degrees(azimuth_offset),
                },
            )
            .collect();

            params.try_into().unwrap_or_else(|_| unreachable!())
        }

        pub fn vlp_32c() -> [LaserParameter; 32] {
            let params: Vec<_> = izip!(
                consts::vlp_32c::ELEVAION_DEGREES,
                consts::vlp_32c::VERTICAL_OFFSETS,
                consts::vlp_32c::HORIZONTAL_OFFSETS,
                consts::vlp_32c::AZIMUTH_OFFSETS,
            )
            .map(
                |(elevation, vertical_offset, horizontal_offset, azimuth_offset)| LaserParameter {
                    elevation: Angle::from_degrees(elevation),
                    vertical_offset: Length::from_millimeters(vertical_offset),
                    horizontal_offset: Length::from_millimeters(horizontal_offset),
                    azimuth_offset: Angle::from_degrees(azimuth_offset),
                },
            )
            .collect();

            params.try_into().unwrap_or_else(|_| unreachable!())
        }
    }
}

pub use param_config::*;
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
