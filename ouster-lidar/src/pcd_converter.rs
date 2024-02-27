//! Provides the converter type that converts packets to points.

use super::{
    config::Config,
    consts::PIXELS_PER_COLUMN,
    packet::{Column, Packet},
};
use crate::utils::AngleExt as _;
use anyhow::{ensure, Result};
use itertools::izip;
use measurements::{Angle, Length};
use num_traits::Float;
use std::{f64::consts::PI, time::Duration};

fn spherical_to_xyz(range: Length, azimuth_angle: Angle, altitude_angle: Angle) -> [Length; 3] {
    let x = range * altitude_angle.sin() * azimuth_angle.cos();
    let y = range * altitude_angle.sin() * azimuth_angle.sin();
    let z = range * altitude_angle.cos();
    [x, y, z]
}

#[derive(Clone, Debug)]
pub struct Point {
    pub timestamp: Duration,
    pub azimuth_angle: Angle,
    pub distance: Length,
    pub reflectivity: u16,
    pub signal_photons: u16,
    pub noise_photons: u16,
    pub laser_id: u32,
    pub point: [Length; 3],
}

/// A conversion tool that transforms [Column](Column) raw sensor data
/// into point clouds.
#[derive(Debug, Clone)]
pub struct PointCloudConverter {
    altitude_angles: [Angle; PIXELS_PER_COLUMN],
    azimuth_angle_corrections: [Angle; PIXELS_PER_COLUMN],
    columns_per_revolution: u16,
}

impl PointCloudConverter {
    /// Create a converter from config.
    pub fn from_config(config: Config) -> Self {
        let Config {
            beam_azimuth_angle_corrections,
            beam_altitude_angles,
            lidar_mode,
        } = config;

        let altitude_angles = {
            let mut array = [Angle::from_radians(0.0); PIXELS_PER_COLUMN];
            debug_assert_eq!(array.len(), beam_altitude_angles.len());

            for idx in 0..(array.len()) {
                let angle =
                    std::f64::consts::FRAC_PI_2 - beam_altitude_angles[idx].to_radians().raw();
                array[idx] = Angle::from_radians(angle);
            }
            array
        };

        let azimuth_angle_corrections = {
            let mut array = [Angle::from_radians(0.0); PIXELS_PER_COLUMN];
            debug_assert_eq!(array.len(), beam_azimuth_angle_corrections.len());

            for idx in 0..(array.len()) {
                let angle = beam_azimuth_angle_corrections[idx].to_radians();
                array[idx] = Angle::from_radians(angle.raw());
            }
            array
        };

        let columns_per_revolution = lidar_mode.columns_per_revolution();

        Self {
            altitude_angles,
            azimuth_angle_corrections,
            columns_per_revolution,
        }
    }

    /// Get lidar scene width by its mode. For example,
    /// [LidarMode](super::enums::LidarMode) mode results
    /// in 1024.
    pub fn columns_per_revolution(&self) -> u16 {
        self.columns_per_revolution
    }

    /// Compute point locations from column returned from lidar.
    ///
    /// The method takes [Column.measurement_id](Column.measurement_id) as column index.
    /// It returns error if the index is out of bound.
    pub(crate) fn column_to_points(&self, column: &Column) -> Result<Vec<Point>> {
        // sanity check
        let col_index = column.measurement_id;
        ensure!(
            col_index < self.columns_per_revolution,
            "measurement_id {} is exceeds the upper bound {}. Is the lidar_mode configured correctly?",
            col_index,
            self.columns_per_revolution,
        );

        // return empty list if the column is not valid
        if !column.valid() {
            return Ok(vec![]);
        }

        let pixels_iter = column.pixels.iter();

        let points = izip!(
            pixels_iter,
            self.altitude_angles.iter(),
            self.azimuth_angle_corrections.iter(),
            0..
        )
        .map(
            |(pixel, altitude_angle, azimuth_angle_correction, laser_id)| {
                // add correction according to manual
                let clockwise_azimuth_angle = column.azimuth() + *azimuth_angle_correction;
                let counter_clockwise_azimuth_angle =
                    Angle::from_radians(PI * 2.0) - clockwise_azimuth_angle;
                let distance = pixel.distance();
                let timestamp = column.time();
                let point =
                    spherical_to_xyz(distance, counter_clockwise_azimuth_angle, *altitude_angle);

                Point {
                    timestamp,
                    reflectivity: pixel.reflectivity,
                    signal_photons: pixel.signal_photons,
                    noise_photons: pixel.noise_photons,
                    azimuth_angle: clockwise_azimuth_angle,
                    distance,
                    laser_id,
                    point,
                }
            },
        )
        .collect::<Vec<_>>();
        Ok(points)
    }

    /// Compute point positions from a packet.
    pub fn convert<P>(&self, packet: P) -> Result<Vec<Point>>
    where
        P: AsRef<Packet>,
    {
        let points: Vec<_> = packet
            .as_ref()
            .columns
            .iter()
            .map(|col| self.column_to_points(col))
            .collect::<Result<Vec<_>>>()?
            .into_iter()
            .flatten()
            .collect();
        Ok(points)
    }
}
