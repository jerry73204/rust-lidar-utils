//! Provides the converter type that converts packets to points.

use super::{
    config::Config,
    consts::PIXELS_PER_COLUMN,
    packet::{Column, Packet},
};
use crate::common::spherical_to_xyz;
use failure::{ensure, Fallible};
use itertools::izip;
use uom::si::{
    angle::radian,
    f64::{Angle as F64Angle, Length as F64Length, Time as F64Time},
};

#[derive(Clone, Debug)]
pub struct Point {
    pub timestamp: F64Time,
    pub azimuth_angle: F64Angle,
    pub distance: F64Length,
    pub reflectivity: u16,
    pub signal_photons: u16,
    pub noise_photons: u16,
    pub laser_id: u32,
    pub point: [F64Length; 3],
}

/// A conversion tool that transforms [Column](Column) raw sensor data
/// into point clouds.
#[derive(Clone)]
pub struct PointCloudConverter {
    altitude_angles: [F64Angle; PIXELS_PER_COLUMN],
    azimuth_angle_corrections: [F64Angle; PIXELS_PER_COLUMN],
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
            let mut array = [F64Angle::new::<radian>(0.0); PIXELS_PER_COLUMN];
            debug_assert_eq!(array.len(), beam_altitude_angles.len());

            for idx in 0..(array.len()) {
                let angle = std::f64::consts::FRAC_PI_2 - beam_altitude_angles[idx].to_radians();
                array[idx] = F64Angle::new::<radian>(angle);
            }
            array
        };

        let azimuth_angle_corrections = {
            let mut array = [F64Angle::new::<radian>(0.0); PIXELS_PER_COLUMN];
            debug_assert_eq!(array.len(), beam_azimuth_angle_corrections.len());

            for idx in 0..(array.len()) {
                let angle = beam_azimuth_angle_corrections[idx].to_radians();
                array[idx] = F64Angle::new::<radian>(angle);
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
    /// [Mode1024x10](LidarMode::Mode1024x10) mode results
    /// in 1024.
    pub fn columns_per_revolution(&self) -> u16 {
        self.columns_per_revolution
    }

    /// Compute point locations from column returned from lidar.
    ///
    /// The method takes [Column.measurement_id](Column.measurement_id) as column index.
    /// It returns error if the index is out of bound.
    pub(crate) fn column_to_points(&self, column: &Column) -> Fallible<Vec<Point>> {
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
                let clockwise_azimuth_angle = column.azimuth_angle() + *azimuth_angle_correction;
                let counter_clockwise_azimuth_angle =
                    F64Angle::new::<radian>(std::f64::consts::PI * 2.0) - clockwise_azimuth_angle;
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

    pub fn convert<P>(&self, packet: P) -> Fallible<Vec<Point>>
    where
        P: AsRef<Packet>,
    {
        let points = packet
            .as_ref()
            .columns
            .iter()
            .map(|col| self.column_to_points(col))
            .collect::<Fallible<Vec<_>>>()?
            .into_iter()
            .flat_map(|points| points)
            .collect::<Vec<_>>();
        Ok(points)
    }
}
