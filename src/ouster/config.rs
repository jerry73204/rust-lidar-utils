//! Configuration types for Ouster LiDARs.

use super::{
    consts::{OS_1_BEAM_ALTITUDE_DEGREES, OS_1_BEAM_AZIMUTH_DEGREE_CORRECTIONS, PIXELS_PER_COLUMN},
    enums::LidarMode,
};
use crate::common::*;
pub use serde_big_array::big_array;

// TODO: This workaround handles large array for serde.
//       We'll remove is it once the const generics is introduced.
big_array! { BigArray; }

/// A serializable struct that represents a Ouster sensor configuration.
#[derive(Clone, PartialEq, Eq, Hash, Serialize, Deserialize, Derivative)]
#[derivative(Debug)]
pub struct Config {
    #[serde(with = "BigArray")]
    pub beam_altitude_angles: [R64; PIXELS_PER_COLUMN],
    #[serde(with = "BigArray", rename = "beam_azimuth_angles")]
    pub beam_azimuth_angle_corrections: [R64; PIXELS_PER_COLUMN],
    pub lidar_mode: LidarMode,
}

impl Config {
    /// Creates new config.
    pub fn new(
        beam_altitude_angles: [f64; PIXELS_PER_COLUMN],
        beam_azimuth_angle_corrections: [f64; PIXELS_PER_COLUMN],
        lidar_mode: LidarMode,
    ) -> Config {
        Config {
            beam_altitude_angles: unsafe { mem::transmute(beam_altitude_angles) },
            beam_azimuth_angle_corrections: unsafe {
                mem::transmute(beam_azimuth_angle_corrections)
            },
            lidar_mode,
        }
    }

    /// Loads config JSON file from path.
    pub fn from_path<P: AsRef<Path>>(path: P) -> Result<Config> {
        let file = File::open(path.as_ref())?;
        let ret = Self::from_reader(file)?;
        Ok(ret)
    }

    /// Loads config JSON data from reader with [Read](std::io::Read) trait.
    pub fn from_reader<R: Read>(reader: R) -> Result<Config> {
        let ret = serde_json::de::from_reader(reader)?;
        Ok(ret)
    }

    /// Parses from JSON string.
    pub fn from_json_str(data: &str) -> Result<Config> {
        let ret = serde_json::from_str(data)?;
        Ok(ret)
    }

    /// Sets `beam_azimuth_angle_corrections` field.
    pub fn beam_azimuth_angle_corrections(
        &mut self,
        beam_azimuth_angle_corrections: [f64; PIXELS_PER_COLUMN],
    ) {
        self.beam_azimuth_angle_corrections =
            unsafe { mem::transmute(beam_azimuth_angle_corrections) };
    }

    /// Sets `beam_altitude_angles` field.
    pub fn beam_altitude_angles(&mut self, beam_altitude_angles: [f64; PIXELS_PER_COLUMN]) {
        self.beam_altitude_angles = unsafe { mem::transmute(beam_altitude_angles) };
    }

    /// Sets `lidar_mode` field.
    pub fn lidar_mode(&mut self, lidar_mode: LidarMode) {
        self.lidar_mode = lidar_mode;
    }

    /// Create default configuration for Ouster OS-1.
    pub fn os_1_config() -> Self {
        // From firmware 1.12.0
        let beam_altitude_angles = OS_1_BEAM_ALTITUDE_DEGREES;
        let beam_azimuth_angle_corrections = OS_1_BEAM_AZIMUTH_DEGREE_CORRECTIONS;

        Self {
            beam_altitude_angles: unsafe { mem::transmute(beam_altitude_angles) },
            beam_azimuth_angle_corrections: unsafe {
                mem::transmute(beam_azimuth_angle_corrections)
            },
            lidar_mode: LidarMode::Mode1024x10,
        }
    }
}
