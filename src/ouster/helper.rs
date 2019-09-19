use super::{Column, LidarMode, ENCODER_TICKS_PER_REV, PIXELS_PER_COLUMN};
use failure::Fallible;
use serde::{Deserialize, Serialize};
use std::{
    cmp::Ordering,
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
    /// Creates new config.
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

    /// Loads config JSON file from path.
    pub fn from_path<P: AsRef<Path>>(path: P) -> Fallible<Config> {
        let file = File::open(path.as_ref())?;
        let ret = Self::from_reader(file)?;
        Ok(ret)
    }

    /// Loads config JSON data from reader with [Read](std::io::Read) trait.
    pub fn from_reader<R: Read>(reader: R) -> Fallible<Config> {
        let ret = serde_json::de::from_reader(reader)?;
        Ok(ret)
    }

    /// Parses from JSON string.
    pub fn from_str(data: &str) -> Fallible<Config> {
        let ret = serde_json::from_str(data)?;
        Ok(ret)
    }

    /// Sets `beam_azimuth_angles` field.
    pub fn beam_azimuth_angles(&mut self, beam_azimuth_angles: [f64; PIXELS_PER_COLUMN]) {
        self.beam_azimuth_angles = beam_azimuth_angles;
    }

    /// Sets `beam_altitude_angles` field.
    pub fn beam_altitude_angles(&mut self, beam_altitude_angles: [f64; PIXELS_PER_COLUMN]) {
        self.beam_altitude_angles = beam_altitude_angles;
    }

    /// Sets `lidar_mode` field.
    pub fn lidar_mode(&mut self, lidar_mode: LidarMode) {
        self.lidar_mode = lidar_mode;
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

/// A conversion tool that transforms [Column](Column) raw sensor data
/// into point clouds.
#[derive(Clone, Debug)]
pub struct PointCloudConverter {
    config: Config,
    num_columns: u16,
}

impl PointCloudConverter {
    /// Create a converter from config.
    pub fn from_config(config: Config) -> Self {
        config.into()
    }

    /// Get internal configuration.
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// Get lidar scene width by its mode. For example,
    /// [Mode1024x10](LidarMode::Mode1024x10) mode results
    /// in 1024.
    pub fn num_columns(&self) -> u16 {
        self.num_columns
    }

    /// Compute point locations from column returned from lidar.
    ///
    /// The method takes [Column.measurement_id](Column.measurement_id) as column index.
    /// It returns error if the index is out of bound.
    pub fn column_to_points(&self, column: &Column) -> Fallible<Vec<(f64, f64, f64)>> {
        let col_index = column.measurement_id;
        ensure!(
            col_index < self.num_columns,
            "measurement_id is out of bound"
        );

        let points = column
            .pixels
            .iter()
            .enumerate()
            .map(|(row_index, pixel)| {
                use std::f64::consts::PI;
                let azimuth_angle = 2.0
                    * PI
                    * (column.encoder_ticks as f64 / ENCODER_TICKS_PER_REV as f64
                        + self.config.beam_azimuth_angles[row_index] / 360.0);
                let altitude_angle = 2.0 * PI * self.config.beam_altitude_angles[row_index] / 360.0;
                let range = pixel.range() as f64;
                let x = range * azimuth_angle.cos() * altitude_angle.cos();
                let y = -range * azimuth_angle.sin() * altitude_angle.cos();
                let z = range * altitude_angle.sin();
                (x, y, z)
            })
            .collect::<Vec<_>>();

        Ok(points)
    }
}

impl From<Config> for PointCloudConverter {
    fn from(config: Config) -> Self {
        let num_columns = {
            use LidarMode::*;
            match config.lidar_mode {
                Mode512x10 | Mode512x20 => 512,
                Mode1024x10 | Mode1024x20 => 1024,
                Mode2048x10 => 2048,
            }
        };

        Self {
            config,
            num_columns,
        }
    }
}

/// A frame is a collection of points gathered in one
/// LIDAR rotation.
#[derive(Debug, Clone)]
pub struct Frame {
    /// The ID marked by [FrameConverter](FrameConverter).
    pub frame_id: u16,
    /// The IDs of dropped frames before this frame comes in.
    pub skipped_frame_ids: Vec<u16>,
    /// Stands for missing columns in this frame.
    pub skipped_measurement_ids: Vec<u16>,
    /// Point cloud data.
    pub points: Vec<(f64, f64, f64)>,
}

/// It reads [columns](Column) of sensor data, and
/// gathers points into sequence of frames.
///
/// It internally computes point cloud using
/// [PointCloudConverter](PointCloudConverter).
/// The columns must be pushed in the same order
/// of LIDAR output. It keeps track of skipped
/// columns and dropped frames.
#[derive(Debug, Clone)]
pub struct FrameConverter {
    pcd_converter: PointCloudConverter,
    state: Option<FrameConverterState>,
}

impl FrameConverter {
    /// Creates converter from config.
    pub fn from_config(config: Config) -> Self {
        config.into()
    }

    /// Pushes new [column](Column) to converter.
    /// Make sure the columns are pushed in the same
    /// order of LIDAR output.
    pub fn push(&mut self, column: &Column) -> Fallible<Vec<Frame>> {
        if !column.valid() {
            return Ok(vec![]);
        }

        let curr_points = self.pcd_converter.column_to_points(&column)?;
        let curr_fid = column.frame_id;
        let curr_mid = column.measurement_id;

        match self.state.take() {
            Some(state) => {
                match state.last_fid.cmp(&curr_fid) {
                    Ordering::Less => {
                        // Produce frame if new frame ID is not expected
                        let first_frame_opt = if state.expect_new_fid {
                            None
                        } else {
                            let skipped_mids = {
                                let mut skipped_mids = state.skipped_mids;
                                skipped_mids
                                    .extend((state.last_mid + 1)..self.pcd_converter.num_columns());
                                skipped_mids
                            };
                            let frame = Frame {
                                frame_id: state.last_fid,
                                skipped_frame_ids: state.skipped_fids,
                                skipped_measurement_ids: skipped_mids,
                                points: state.points,
                            };
                            Some(frame)
                        };

                        // Produce frame if measurement ID is exactly the latest ID of frame
                        let (second_frame_opt, new_state) = {
                            let skipped_fids = ((state.last_fid + 1)..curr_fid).collect();
                            let skipped_mids = (0..curr_mid).collect();

                            if curr_mid + 1 == self.pcd_converter.num_columns {
                                let second_frame = Frame {
                                    frame_id: curr_fid,
                                    skipped_frame_ids: skipped_fids,
                                    skipped_measurement_ids: skipped_mids,
                                    points: curr_points,
                                };

                                let new_state = FrameConverterState {
                                    expect_new_fid: true,
                                    last_mid: curr_mid,
                                    last_fid: curr_fid,
                                    skipped_fids: vec![],
                                    skipped_mids: vec![],
                                    points: vec![],
                                };

                                (Some(second_frame), new_state)
                            } else {
                                let new_state = FrameConverterState {
                                    expect_new_fid: false,
                                    last_mid: curr_mid,
                                    last_fid: curr_fid,
                                    skipped_fids,
                                    skipped_mids,
                                    points: curr_points,
                                };

                                (None, new_state)
                            }
                        };

                        // Update and return
                        self.state = Some(new_state);

                        let output_frames = first_frame_opt
                            .into_iter()
                            .chain(second_frame_opt.into_iter())
                            .collect();

                        return Ok(output_frames);
                    }
                    Ordering::Equal => {
                        if state.last_mid >= curr_mid {
                            let error = format_err!(
                                "Input measurement ID is less than that of last column"
                            );
                            return Err(error);
                        }

                        let skipped_fids = state.skipped_fids;

                        let mut skipped_mids = state.skipped_mids;
                        skipped_mids.extend((state.last_mid + 1)..curr_mid);

                        let mut points = state.points;
                        points.extend(curr_points);

                        // Produce frame if measurement ID is the latest one in frame
                        let (frame_opt, new_state) =
                            if curr_mid + 1 == self.pcd_converter.num_columns() {
                                let frame = Frame {
                                    frame_id: curr_fid,
                                    skipped_frame_ids: skipped_fids,
                                    skipped_measurement_ids: skipped_mids,
                                    points,
                                };

                                let new_state = FrameConverterState {
                                    expect_new_fid: true,
                                    last_mid: curr_mid,
                                    last_fid: curr_fid,
                                    skipped_fids: vec![],
                                    skipped_mids: vec![],
                                    points: vec![],
                                };

                                (Some(frame), new_state)
                            } else {
                                let new_state = FrameConverterState {
                                    expect_new_fid: false,
                                    last_mid: curr_mid,
                                    last_fid: curr_fid,
                                    skipped_fids,
                                    skipped_mids,
                                    points,
                                };

                                (None, new_state)
                            };

                        let output_frames = frame_opt.into_iter().collect();
                        self.state = Some(new_state);
                        return Ok(output_frames);
                    }
                    Ordering::Greater => {
                        let error = format_err!("Input frame ID is less than that of last column");
                        return Err(error);
                    }
                }
            }
            None => {
                if curr_mid + 1 == self.pcd_converter.num_columns() {
                    let new_state = FrameConverterState {
                        expect_new_fid: true,
                        last_mid: curr_mid,
                        last_fid: curr_fid,
                        skipped_fids: vec![],
                        skipped_mids: vec![],
                        points: vec![],
                    };
                    self.state = Some(new_state);
                    let output_frame = Frame {
                        frame_id: curr_fid,
                        skipped_frame_ids: vec![],
                        skipped_measurement_ids: (0..(column.measurement_id)).collect(),
                        points: curr_points,
                    };
                    return Ok(vec![output_frame]);
                } else {
                    let new_state = FrameConverterState {
                        expect_new_fid: false,
                        last_mid: curr_mid,
                        last_fid: curr_fid,
                        skipped_fids: vec![],
                        skipped_mids: (0..(column.measurement_id)).collect(),
                        points: curr_points,
                    };
                    self.state = Some(new_state);
                    return Ok(vec![]);
                }
            }
        }
    }

    /// Consumes the instance and outputs last maybe
    /// incomplete frame.
    pub fn finish(mut self) -> Option<Frame> {
        match self.state.take() {
            Some(state) => {
                if state.expect_new_fid {
                    None
                } else {
                    let mut skipped_mids = state.skipped_mids;
                    skipped_mids.extend((state.last_mid + 1)..(self.pcd_converter.num_columns()));

                    let frame = Frame {
                        frame_id: state.last_fid,
                        skipped_frame_ids: state.skipped_fids,
                        skipped_measurement_ids: skipped_mids,
                        points: state.points,
                    };
                    Some(frame)
                }
            }
            None => None,
        }
    }
}

impl From<Config> for FrameConverter {
    fn from(config: Config) -> Self {
        let pcd_converter = PointCloudConverter::from(config);

        Self {
            pcd_converter,
            state: None,
        }
    }
}

#[derive(Clone, Debug)]
struct FrameConverterState {
    expect_new_fid: bool,
    skipped_fids: Vec<u16>,
    skipped_mids: Vec<u16>,
    last_mid: u16,
    last_fid: u16,
    points: Vec<(f64, f64, f64)>,
}

fn large_array_fmt<T: Debug>(
    array: &[T; PIXELS_PER_COLUMN],
    formatter: &mut Formatter,
) -> Result<(), FormatError> {
    write!(formatter, "{:?}", array as &[_])
}
