use crate::{
    error::Error,
    serialized::{SerializedLaser, SerializedVelodyneParams},
};
use itertools::Itertools;
use measurements::Length;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(
    try_from = "SerializedVelodyneParams",
    into = "SerializedVelodyneParams"
)]
pub struct VelodyneParams {
    pub distance_resolution: Length,
    pub lasers: Vec<Laser>,
}

impl TryFrom<SerializedVelodyneParams> for VelodyneParams {
    type Error = Error;

    fn try_from(orig: SerializedVelodyneParams) -> Result<Self, Self::Error> {
        let SerializedVelodyneParams {
            num_lasers,
            distance_resolution,
            lasers,
        } = orig;

        if num_lasers != lasers.len() {
            return Err(Error::invalid_params(format!(
                "The number of lasers mismatches.\n\
                 Get num_lasers = {num_lasers}, but `lasers` field has {} items.",
                lasers.len()
            )));
        }

        if !distance_resolution.is_finite() || distance_resolution <= 0.0 {
            return Err(Error::invalid_params(format!(
                "Distance_resolution must be finite positive, but get {distance_resolution}"
            )));
        }

        let lasers: Vec<_> = lasers.into_iter().map(Laser::try_from).try_collect()?;

        Ok(Self {
            distance_resolution: Length::from_meters(distance_resolution),
            lasers,
        })
    }
}

impl From<VelodyneParams> for SerializedVelodyneParams {
    fn from(orig: VelodyneParams) -> Self {
        let VelodyneParams {
            distance_resolution,
            lasers,
        } = orig;
        let lasers: Vec<_> = lasers.into_iter().map(SerializedLaser::from).collect();

        Self {
            num_lasers: lasers.len(),
            distance_resolution: distance_resolution.as_meters(),
            lasers,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(try_from = "SerializedLaser", into = "SerializedLaser")]
pub struct Laser {
    pub laser_id: u32,
    pub dist_correction: f64,
    pub dist_correction_x: f64,
    pub dist_correction_y: f64,
    pub focal_distance: f64,
    pub focal_slope: f64,
    pub horiz_offset_correction: f64,
    pub rot_correction: f64,
    pub vert_correction: f64,
    pub vert_offset_correction: f64,
}

impl TryFrom<SerializedLaser> for Laser {
    type Error = Error;

    fn try_from(orig: SerializedLaser) -> Result<Self, Self::Error> {
        let SerializedLaser {
            dist_correction,
            dist_correction_x,
            dist_correction_y,
            focal_distance,
            focal_slope,
            horiz_offset_correction,
            laser_id,
            rot_correction,
            vert_correction,
            vert_offset_correction,
        } = orig;

        Ok(Self {
            laser_id,
            dist_correction,
            dist_correction_x,
            dist_correction_y,
            focal_distance,
            focal_slope,
            horiz_offset_correction,
            rot_correction,
            vert_correction,
            vert_offset_correction,
        })
    }
}

impl From<Laser> for SerializedLaser {
    fn from(orig: Laser) -> Self {
        let Laser {
            dist_correction,
            dist_correction_x,
            dist_correction_y,
            focal_distance,
            focal_slope,
            horiz_offset_correction,
            laser_id,
            rot_correction,
            vert_correction,
            vert_offset_correction,
        } = orig;

        Self {
            laser_id,
            dist_correction,
            dist_correction_x,
            dist_correction_y,
            focal_distance,
            focal_slope,
            horiz_offset_correction,
            rot_correction,
            vert_correction,
            vert_offset_correction,
        }
    }
}
