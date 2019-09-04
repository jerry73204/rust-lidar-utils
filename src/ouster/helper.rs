use super::{Column, LidarMode, PIXELS_PER_COLUMN};
use failure::Fallible;
use ndarray::{Array3, Axis};
use serde::{Deserialize, Serialize};
use std::{
    fmt::{Debug, Error as FormatError, Formatter},
    fs::File,
    io::Read,
    path::Path,
};

// TODO: This workaround handles large array for serde.
//       We'll remove is it once the const generics is introduced.
big_array! { BigArray; }

#[derive(Clone, Serialize, Deserialize, Derivative)]
#[derivative(Debug)]
pub struct Config {
    #[serde(with = "BigArray")]
    #[derivative(Debug(format_with = "self::large_array_fmt"))]
    pub beam_altitude_angles: [f64; PIXELS_PER_COLUMN],
    #[serde(with = "BigArray")]
    #[derivative(Debug(format_with = "self::large_array_fmt"))]
    pub beam_azimuth_angles: [f64; PIXELS_PER_COLUMN],
    pub lidar_mode: LidarMode,
}

impl Config {
    /// Create new config.
    pub fn new(
        beam_altitude_angles: [f64; PIXELS_PER_COLUMN],
        beam_azimuth_angles: [f64; PIXELS_PER_COLUMN],
        lidar_mode: LidarMode,
    ) -> Config {
        Config {
            beam_altitude_angles,
            beam_azimuth_angles,
            lidar_mode,
        }
    }

    /// Load config JSON file from path.
    pub fn from_path<P: AsRef<Path>>(path: P) -> Fallible<Config> {
        let file = File::open(path.as_ref())?;
        let ret = Self::from_reader(file)?;
        Ok(ret)
    }

    /// Load config JSON data from reader with [Read](std::io::Read) trait.
    pub fn from_reader<R: Read>(reader: R) -> Fallible<Config> {
        let ret = serde_json::de::from_reader(reader)?;
        Ok(ret)
    }

    /// Parse from JSON string.
    pub fn from_str(data: &str) -> Fallible<Config> {
        let ret = serde_json::from_str(data)?;
        Ok(ret)
    }
}

impl Default for Config {
    fn default() -> Config {
        // From firmare 1.12.0
        let beam_altitude_angles = [
            17.042,
            16.427,
            15.872,
            15.324,
            14.851,
            14.269,
            13.733,
            13.18,
            12.713,
            12.136,
            11.599,
            11.067,
            10.587,
            10.046,
            9.503,
            8.965999999999999,
            8.504,
            7.952,
            7.414,
            6.869,
            6.417,
            5.866,
            5.331,
            4.792,
            4.329,
            3.791,
            3.248,
            2.699,
            2.26,
            1.709,
            1.17,
            0.62,
            0.177,
            -0.37,
            -0.916,
            -1.466,
            -1.924,
            -2.454,
            -3.001,
            -3.551,
            -3.997,
            -4.545,
            -5.088,
            -5.64,
            -6.08,
            -6.638,
            -7.17,
            -7.736,
            -8.196999999999999,
            -8.728,
            -9.282,
            -9.853999999999999,
            -10.299,
            -10.833,
            -11.386,
            -11.965,
            -12.422,
            -12.957,
            -13.525,
            -14.109,
            -14.576,
            -15.133,
            -15.691,
            -16.3,
        ];

        let beam_azimuth_angles = [
            3.073,
            0.922,
            -1.238,
            -3.386,
            3.057,
            0.915,
            -1.214,
            -3.321,
            3.06,
            0.9370000000000001,
            -1.174,
            -3.284,
            3.051,
            0.953,
            -1.154,
            -3.242,
            3.05,
            0.958,
            -1.126,
            -3.198,
            3.053,
            0.981,
            -1.104,
            -3.177,
            3.082,
            1.001,
            -1.079,
            -3.141,
            3.101,
            1.025,
            -1.048,
            -3.124,
            3.115,
            1.041,
            -1.028,
            -3.1,
            3.135,
            1.066,
            -1.0,
            -3.077,
            3.177,
            1.093,
            -0.981,
            -3.06,
            3.213,
            1.117,
            -0.963,
            -3.048,
            3.249,
            1.158,
            -0.948,
            -3.047,
            3.297,
            1.2,
            -0.913,
            -3.023,
            3.345,
            1.231,
            -0.881,
            -3.022,
            3.425,
            1.267,
            -0.872,
            -3.024,
        ];

        Config {
            beam_altitude_angles,
            beam_azimuth_angles,
            lidar_mode: LidarMode::Mode512x10,
        }
    }
}

impl From<Helper> for Config {
    fn from(config: Helper) -> Config {
        Config {
            beam_altitude_angles: config.beam_altitude_angles,
            beam_azimuth_angles: config.beam_azimuth_angles,
            lidar_mode: config.lidar_mode,
        }
    }
}

#[derive(Clone, Derivative)]
#[derivative(Debug)]
pub struct Helper {
    #[derivative(Debug(format_with = "self::large_array_fmt"))]
    beam_altitude_angles: [f64; PIXELS_PER_COLUMN],
    #[derivative(Debug(format_with = "self::large_array_fmt"))]
    beam_azimuth_angles: [f64; PIXELS_PER_COLUMN],
    lidar_mode: LidarMode,
    num_columns: usize,
    spherical_projection: Array3<f64>,
}

impl Helper {
    pub fn new(
        beam_altitude_angles: [f64; PIXELS_PER_COLUMN],
        beam_azimuth_angles: [f64; PIXELS_PER_COLUMN],
        lidar_mode: LidarMode,
    ) -> Helper {
        Config::new(beam_altitude_angles, beam_azimuth_angles, lidar_mode).into()
    }

    pub fn from_config(config: Config) -> Helper {
        config.into()
    }

    pub fn beam_altitude_angles(&self) -> &[f64; PIXELS_PER_COLUMN] {
        &self.beam_altitude_angles
    }

    pub fn beam_azimuth_angles(&self) -> &[f64; PIXELS_PER_COLUMN] {
        &self.beam_azimuth_angles
    }

    pub fn lidar_mode(&self) -> LidarMode {
        self.lidar_mode
    }

    /// Get lidar scene width by its mode.
    pub fn num_columns(&self) -> usize {
        self.num_columns
    }

    /// Compute spherical projection on unit circle for each laser beam.
    ///
    /// It returns a three dimensional array indexed by column index,
    /// row index and component index. The first dimension size depends on
    /// [Helper::num_columns](Helper::num_columns). The second index size is fixed
    /// [PIXELS_PER_COLUMN](PIXELS_PER_COLUMN). The last dimension corresponds
    /// to x, y, z components.
    pub fn spherical_projection(&self) -> &Array3<f64> {
        &self.spherical_projection
    }

    /// Compute point locations from column returned from lidar.
    ///
    /// The method takes [Column.measurement_id](Column.measurement_id) as column index.
    /// It returns error if the index is out of bound.
    pub fn column_to_points(&self, column: &Column) -> Fallible<Vec<(f64, f64, f64)>> {
        let col_index = column.measurement_id as usize;
        ensure!(
            col_index < self.spherical_projection.shape()[0],
            "measurement_id is out of bound"
        );

        let sub_projection = self.spherical_projection.index_axis(Axis(0), col_index);

        let points = column
            .pixels
            .iter()
            .enumerate()
            .map(|(row_index, pixel)| {
                let x = sub_projection[(row_index, 0)];
                let y = sub_projection[(row_index, 1)];
                let z = sub_projection[(row_index, 2)];
                let range = pixel.range() as f64;
                let rx = x as f64 * range;
                let ry = y as f64 * range;
                let rz = z as f64 * range;
                (rx, ry, rz)
            })
            .collect::<Vec<_>>();

        Ok(points)
    }
}

impl From<Config> for Helper {
    fn from(ser_config: Config) -> Helper {
        let num_columns = {
            use LidarMode::*;
            match ser_config.lidar_mode {
                Mode512x10 | Mode512x20 => 512,
                Mode1024x10 | Mode1024x20 => 1024,
                Mode2048x10 => 2048,
            }
        };

        let spherical_projection = {
            use std::f64::consts::PI;
            let deg2rad = |deg: f64| deg * PI / 180.0;

            let mut projection = Array3::<f64>::zeros((num_columns, PIXELS_PER_COLUMN, 3));

            (0..num_columns).into_iter().for_each(|col| {
                let azimuth_angle_base = 2.0 * PI * col as f64 / num_columns as f64;

                ser_config
                    .beam_azimuth_angles
                    .iter()
                    .zip(ser_config.beam_altitude_angles.iter())
                    .enumerate()
                    .for_each(|(row, (azimuth_deg_off, altitude_deg))| {
                        let azimuth_angle = deg2rad(*azimuth_deg_off) + azimuth_angle_base;
                        let altitude_angle = deg2rad(*altitude_deg);

                        let x = altitude_angle.cos() * azimuth_angle.cos();
                        let y = altitude_angle.cos() * azimuth_angle.sin();
                        let z = altitude_angle.sin();

                        projection[(col, row, 0)] = x;
                        projection[(col, row, 1)] = y;
                        projection[(col, row, 2)] = z;
                    });
            });

            projection
        };

        Helper {
            beam_altitude_angles: ser_config.beam_altitude_angles,
            beam_azimuth_angles: ser_config.beam_azimuth_angles,
            lidar_mode: ser_config.lidar_mode,
            num_columns,
            spherical_projection,
        }
    }
}

impl Default for Helper {
    fn default() -> Helper {
        Config::default().into()
    }
}

fn large_array_fmt<T: Debug>(
    array: &[T; PIXELS_PER_COLUMN],
    formatter: &mut Formatter,
) -> Result<(), FormatError> {
    write!(formatter, "{:?}", array as &[_])
}
