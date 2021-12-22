//! Point data types.

use crate::common::*;

pub use dual_return_point::*;
pub use dynamic_return_points::*;
pub use single_return_point::*;

/// Generic point from Velodyne LiDAR device.

pub trait VelodynePoint {
    fn laser_id(&self) -> u32;
    fn timestamp(&self) -> Time;
    fn original_azimuth_angle(&self) -> Angle;
    fn corrected_azimuth_angle(&self) -> Angle;
    fn set_col_idx(&mut self, id: usize);
    fn col_idx(&self) -> usize;
    fn row_idx(&self) -> usize;
}

// Todo
pub trait LidarFrame {}

/// Point in strongest or last return mode.
#[derive(Debug, Clone, Copy)]
pub struct PointData {
    pub distance: Length,
    pub intensity: u8,
    pub position: [Length; 3],
}
#[derive(Debug, Clone, Copy)]
pub struct LidarFrameEntry {
    //Index of channel, from bottom(-25 degree) to top (15 degree)
    pub row_idx: usize,
    // Index of line in a frame
    pub col_idx: usize,
}

mod single_return_point {
    use super::*;

    /// Point in strongest or last return mode.
    #[derive(Debug, Clone, Copy)]
    pub struct SingleReturnPoint {
        pub laser_id: u32,
        pub timestamp: Time,
        pub original_azimuth_angle: Angle,
        pub corrected_azimuth_angle: Angle,
        pub data: PointData,
        pub lidar_frame_entry: LidarFrameEntry,
    }

    impl VelodynePoint for SingleReturnPoint {
        fn laser_id(&self) -> u32 {
            self.laser_id
        }

        fn timestamp(&self) -> Time {
            self.timestamp
        }

        fn original_azimuth_angle(&self) -> Angle {
            self.original_azimuth_angle
        }

        fn corrected_azimuth_angle(&self) -> Angle {
            self.corrected_azimuth_angle
        }
        fn set_col_idx(&mut self, id: usize) {
            self.lidar_frame_entry.col_idx = id;
        }
        fn col_idx(&self) -> usize {
            self.lidar_frame_entry.col_idx
        }

        fn row_idx(&self) -> usize {
            self.lidar_frame_entry.row_idx
        }
    }
}

mod dual_return_point {
    use super::*;

    /// Point in dual return mode.
    #[derive(Debug, Clone, Copy)]
    pub struct DualReturnPoint {
        pub laser_id: u32,
        pub timestamp: Time,
        pub original_azimuth_angle: Angle,
        pub corrected_azimuth_angle: Angle,
        pub strongest_return_data: PointData,
        pub last_return_data: PointData,
        pub lidar_frame_entry: LidarFrameEntry,
    }

    impl DualReturnPoint {
        pub fn try_from_pair(
            strongest_return_point: SingleReturnPoint,
            last_return_point: SingleReturnPoint,
        ) -> Result<Self> {
            let SingleReturnPoint {
                laser_id: laser_id_strongest,
                timestamp: timestamp_strongest,
                original_azimuth_angle: original_azimuth_angle_strongest,
                corrected_azimuth_angle: corrected_azimuth_angle_strongest,
                data: strongest_return_data,
                lidar_frame_entry,
            } = strongest_return_point;

            let SingleReturnPoint {
                laser_id: laser_id_last,
                timestamp: timestamp_last,
                original_azimuth_angle: original_azimuth_angle_last,
                corrected_azimuth_angle: corrected_azimuth_angle_last,
                data: last_return_data,
                lidar_frame_entry,
            } = last_return_point;

            ensure!(
                laser_id_strongest == laser_id_last,
                "laser ID does not match"
            );
            ensure!(
                timestamp_strongest == timestamp_last,
                "timestamp does not match"
            );
            ensure!(
                original_azimuth_angle_strongest == original_azimuth_angle_last,
                "original azimuth angle does not match"
            );
            ensure!(
                corrected_azimuth_angle_strongest == corrected_azimuth_angle_last,
                "corrected azimuth angle does not match"
            );

            let dual_return_point = DualReturnPoint {
                laser_id: laser_id_strongest,
                timestamp: timestamp_strongest,
                original_azimuth_angle: original_azimuth_angle_strongest,
                corrected_azimuth_angle: corrected_azimuth_angle_strongest,
                strongest_return_data,
                last_return_data,
                lidar_frame_entry,
            };

            Ok(dual_return_point)
        }
    }

    impl VelodynePoint for DualReturnPoint {
        fn laser_id(&self) -> u32 {
            self.laser_id
        }

        fn timestamp(&self) -> Time {
            self.timestamp
        }

        fn original_azimuth_angle(&self) -> Angle {
            self.original_azimuth_angle
        }

        fn corrected_azimuth_angle(&self) -> Angle {
            self.corrected_azimuth_angle
        }
        fn set_col_idx(&mut self, id: usize) {
            self.lidar_frame_entry.col_idx = id;
        }
        fn col_idx(&self) -> usize {
            self.lidar_frame_entry.col_idx
        }

        fn row_idx(&self) -> usize {
            self.lidar_frame_entry.row_idx
        }
    }
}

mod dynamic_return_points {
    use super::*;

    /// Collection of points in either single return or dual return mode.
    #[derive(Debug, Clone)]
    pub enum DynamicReturnPoints {
        Single(Vec<SingleReturnPoint>),
        Dual(Vec<DualReturnPoint>),
    }

    impl DynamicReturnPoints {
        pub fn is_empty(&self) -> bool {
            match self {
                Self::Single(points) => points.is_empty(),
                Self::Dual(points) => points.is_empty(),
            }
        }

        pub(crate) fn empty_like(&self) -> Self {
            match self {
                Self::Single(_) => Self::Single(vec![]),
                Self::Dual(_) => Self::Dual(vec![]),
            }
        }
    }

    impl IntoIterator for DynamicReturnPoints {
        type Item = DynamicReturnPoint;
        type IntoIter = DynamicReturnPointsIter;

        fn into_iter(self) -> Self::IntoIter {
            let iter: Box<dyn Iterator<Item = DynamicReturnPoint> + Sync + Send> = match self {
                Self::Single(points) => {
                    Box::new(points.into_iter().map(DynamicReturnPoint::Single))
                }
                Self::Dual(points) => Box::new(points.into_iter().map(DynamicReturnPoint::Dual)),
            };
            Self::IntoIter { iter }
        }
    }

    impl From<Vec<SingleReturnPoint>> for DynamicReturnPoints {
        fn from(points: Vec<SingleReturnPoint>) -> Self {
            Self::Single(points)
        }
    }

    impl From<Vec<DualReturnPoint>> for DynamicReturnPoints {
        fn from(points: Vec<DualReturnPoint>) -> Self {
            Self::Dual(points)
        }
    }

    /// Collection of points in either single return or dual return mode.
    #[derive(Derivative)]
    #[derivative(Debug)]
    pub struct DynamicReturnPointsIter {
        #[derivative(Debug = "ignore")]
        iter: Box<dyn Iterator<Item = DynamicReturnPoint> + Sync + Send>,
    }

    impl Iterator for DynamicReturnPointsIter {
        type Item = DynamicReturnPoint;

        fn next(&mut self) -> Option<Self::Item> {
            self.iter.next()
        }
    }

    /// collection of points in either single return or dual return mode.
    #[derive(Debug, Clone, Copy)]
    pub enum DynamicReturnPoint {
        Single(SingleReturnPoint),
        Dual(DualReturnPoint),
    }

    impl DynamicReturnPoint {
        pub fn laser_id(&self) -> u32 {
            match self {
                Self::Single(point) => point.laser_id,
                Self::Dual(point) => point.laser_id,
            }
        }

        pub fn timestamp(&self) -> Time {
            match self {
                Self::Single(point) => point.timestamp,
                Self::Dual(point) => point.timestamp,
            }
        }

        pub fn original_azimuth_angle(&self) -> Angle {
            match self {
                Self::Single(point) => point.original_azimuth_angle,
                Self::Dual(point) => point.original_azimuth_angle,
            }
        }

        pub fn corrected_azimuth_angle(&self) -> Angle {
            match self {
                Self::Single(point) => point.corrected_azimuth_angle,
                Self::Dual(point) => point.corrected_azimuth_angle,
            }
        }
    }
}
