//! Provides a set of tools convert raw packets from Ouster sensors.

use super::{
    config::Config,
    consts::COLUMNS_PER_PACKET,
    packet::{Column, Packet},
    pcd_converter::{Point, PointCloudConverter},
};
use crate::common::*;

/// A frame is a collection of points gathered in one
/// LIDAR rotation.
#[derive(Debug, Clone)]
pub struct Frame {
    /// The ID marked by [FrameConverter](FrameConverter).
    pub frame_id: u16,
    /// The IDs of dropped frames before this frame comes in.
    pub skipped_frame_ids: Range<u16>,
    /// Pairs of `(measurement_id, timestamp)`.
    pub timestamps: Vec<(u16, u64)>,
    /// Point cloud data.
    pub points: Vec<Point>,
}

/// It reads [columns](Column) of sensor data, and
/// gathers points into sequence of frames.
///
/// It internally computes point cloud using
/// [PointCloudConverter](PointCloudConverter).
/// The columns must be pushed in the same order
/// of LIDAR output. It keeps track of skipped
/// columns and dropped frames.
#[derive(Debug)]
pub struct FrameConverter {
    pcd_converter: PointCloudConverter,
    state: Option<FrameConverterState>,
}

impl FrameConverter {
    /// Creates converter from config.
    pub fn from_config(config: Config) -> Self {
        Self {
            pcd_converter: PointCloudConverter::from_config(config),
            state: None,
        }
    }

    /// Returns the resolution in `(width, height)` pair.
    pub fn resolution(&self) -> (u16, u16) {
        let width = self.pcd_converter.columns_per_revolution();
        (width, 64)
    }

    /// Returns the number of columns per revolution.
    pub fn columns_per_revolution(&self) -> u16 {
        self.pcd_converter.columns_per_revolution()
    }

    /// Pushes new [Column] to converter.
    pub fn push_column(&mut self, column: &Column) -> Result<Vec<Frame>> {
        let curr_fid = column.frame_id;
        let curr_mid = column.measurement_id;
        let curr_ts = column.timestamp;
        let curr_points = self.pcd_converter.column_to_points(&column)?;

        // If received column is not valid, update last_{fid,mid} only
        if !column.valid() {
            let (frame_opt, new_state) = match self.state.take() {
                Some(mut state) => {
                    let frame_opt = match state.last_fid.cmp(&curr_fid) {
                        Ordering::Less => state.frame.take(),
                        Ordering::Equal => None,
                        Ordering::Greater => {
                            bail!(
                                "Measurement ID of received column is less than that of previous column"
                            );
                        }
                    };

                    state.last_fid = curr_fid;
                    state.last_mid = curr_mid;
                    (frame_opt, state)
                }
                None => {
                    let new_state = FrameConverterState {
                        last_fid: curr_fid,
                        last_mid: curr_mid,
                        frame: None,
                    };
                    (None, new_state)
                }
            };

            self.state = Some(new_state);
            return Ok(frame_opt.into_iter().collect());
        }

        let (new_state, output_frames) = match self.state.take() {
            Some(mut state) => {
                match state.last_fid.cmp(&curr_fid) {
                    Ordering::Less => {
                        // Case: New frame ID
                        // Pop out saved frame and conditionally save or output second frame

                        let first_frame_opt = state.frame.take();
                        let second_frame = Frame {
                            frame_id: curr_fid,
                            skipped_frame_ids: (state.last_fid + 1)..curr_fid,
                            timestamps: {
                                let mut timestamps = Vec::with_capacity(COLUMNS_PER_PACKET);
                                timestamps.push((curr_mid, curr_ts));
                                timestamps
                            },
                            points: curr_points,
                        };
                        let mut new_state = FrameConverterState {
                            last_mid: curr_mid,
                            last_fid: curr_fid,
                            frame: None,
                        };

                        // Produce frame if measurement ID is exactly the latest ID of frame
                        let (second_frame_opt, new_state) =
                            if curr_mid + 1 == self.pcd_converter.columns_per_revolution() {
                                (Some(second_frame), new_state)
                            } else {
                                new_state.frame = Some(second_frame);
                                (None, new_state)
                            };

                        let output_frames = first_frame_opt
                            .into_iter()
                            .chain(second_frame_opt.into_iter())
                            .collect();

                        (new_state, output_frames)
                    }
                    Ordering::Equal => {
                        if state.last_mid >= curr_mid {
                            let error = format_err!(
                                "Measurement ID of received column is less than that of previous column"
                            );
                            return Err(error);
                        }

                        // Conditionally produce frame if measurement ID is the latest one
                        let mut new_state = FrameConverterState {
                            last_mid: curr_mid,
                            last_fid: curr_fid,
                            frame: None,
                        };
                        let frame = {
                            let mut frame = state.frame.take().unwrap_or_else(|| {
                                unreachable!("Please report bug to upstream");
                            });
                            frame.timestamps.push((curr_mid, curr_ts));
                            frame.points.extend(curr_points);
                            frame
                        };

                        let (frame_opt, new_state) =
                            if curr_mid + 1 == self.pcd_converter.columns_per_revolution() {
                                (Some(frame), new_state)
                            } else {
                                new_state.frame = Some(frame);
                                (None, new_state)
                            };

                        let output_frames = frame_opt.into_iter().collect();
                        (new_state, output_frames)
                    }
                    Ordering::Greater => {
                        let error = format_err!(
                            "Frame ID of received column is less than that of previous column"
                        );
                        return Err(error);
                    }
                }
            }
            None => {
                let frame = Frame {
                    frame_id: curr_fid,
                    skipped_frame_ids: curr_fid..curr_fid,
                    timestamps: {
                        let mut timestamps = Vec::with_capacity(COLUMNS_PER_PACKET);
                        timestamps.push((curr_mid, curr_ts));
                        timestamps
                    },
                    points: curr_points,
                };
                let mut new_state = FrameConverterState {
                    last_mid: curr_mid,
                    last_fid: curr_fid,
                    frame: None,
                };

                let frame_opt = if curr_mid + 1 == self.pcd_converter.columns_per_revolution() {
                    Some(frame)
                } else {
                    new_state.frame = Some(frame);
                    None
                };

                (new_state, frame_opt.into_iter().collect())
            }
        };

        self.state = Some(new_state);
        Ok(output_frames)
    }

    /// Pushes new [Packet] to converter.
    pub fn push_packet<P>(&mut self, packet: P) -> Result<Vec<Frame>>
    where
        P: AsRef<Packet>,
    {
        let mut frames = vec![];
        for column in packet.as_ref().columns.iter() {
            frames.extend(self.push_column(&column)?);
        }
        Ok(frames)
    }

    /// Consumes the instance and outputs last maybe
    /// incomplete frame.
    pub fn finish(mut self) -> Option<Frame> {
        self.state
            .take()
            .map(|mut state| state.frame.take())
            .unwrap_or(None)
    }
}

#[derive(Clone, Debug)]
struct FrameConverterState {
    last_mid: u16,
    last_fid: u16,
    frame: Option<Frame>,
}
