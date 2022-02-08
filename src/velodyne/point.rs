//! Point data types.

use crate::common::*;

/// Generic point from Velodyne LiDAR device.

pub trait VelodynePoint {
    fn laser_id(&self) -> u32;
    fn timestamp(&self) -> Duration;
    fn original_azimuth(&self) -> Angle;
    fn corrected_azimuth(&self) -> Angle;
}

/// Point in strongest or last return mode.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Measurement {
    pub distance: Length,
    pub intensity: u8,
    pub xyz: [Length; 3],
}

pub trait LidarFrameMsg {
    fn set_row_idx(&mut self, id: usize);
    fn row_idx(&self) -> usize;
    fn set_col_idx(&mut self, id: usize);
    fn col_idx(&self) -> usize;
}

#[derive(Debug, Clone, Copy)]
pub struct LidarFrameEntry {
    //Index of channel, from bottom(-25 degree) to top (15 degree)
    pub row_idx: usize,
    // Index of line in a frame
    pub col_idx: usize,
}

impl LidarFrameMsg for LidarFrameEntry {
    fn set_row_idx(&mut self, id: usize) {
        self.row_idx = id;
    }
    fn row_idx(&self) -> usize {
        self.row_idx
    }
    fn set_col_idx(&mut self, id: usize) {
        self.col_idx = id;
    }
    fn col_idx(&self) -> usize {
        self.col_idx
    }
}

pub use point_::*;
mod point_ {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct PointSingle {
        pub laser_id: usize,
        pub time: Duration,
        pub azimuth: Angle,
        pub measurement: Measurement,
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct PointDual {
        pub laser_id: usize,
        pub time: Duration,
        pub azimuth: Angle,
        pub measurement_strongest: Measurement,
        pub measurement_last: Measurement,
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct FiringXyzSingle16 {
        pub time: Duration,
        pub azimuth_count: u16,
        pub azimuth_range: Range<Angle>,
        pub points: [PointSingle; 16],
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct FiringXyzSingle32 {
        pub time: Duration,
        pub azimuth_count: u16,
        pub azimuth_range: Range<Angle>,
        pub points: [PointSingle; 32],
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct FiringXyzDual16 {
        pub time: Duration,
        pub azimuth_count: u16,
        pub azimuth_range: Range<Angle>,
        pub points: [PointDual; 16],
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct FiringXyzDual32 {
        pub time: Duration,
        pub azimuth_count: u16,
        pub azimuth_range: Range<Angle>,
        pub points: [PointDual; 32],
    }
}

pub use single_return_point::*;
mod single_return_point {
    use super::*;

    /// Point in strongest or last return mode.
    #[derive(Debug, Clone)]
    pub struct SinglePoint {
        pub laser_id: u32,
        pub timestamp: Duration,
        pub original_azimuth_angle: Angle,
        pub corrected_azimuth_angle: Angle,
        pub data: Measurement,
        pub lidar_frame_entry: LidarFrameEntry,
    }

    impl VelodynePoint for SinglePoint {
        fn laser_id(&self) -> u32 {
            self.laser_id
        }

        fn timestamp(&self) -> Duration {
            self.timestamp
        }

        fn original_azimuth(&self) -> Angle {
            self.original_azimuth_angle
        }

        fn corrected_azimuth(&self) -> Angle {
            self.corrected_azimuth_angle
        }
    }

    impl LidarFrameMsg for SinglePoint {
        fn set_row_idx(&mut self, id: usize) {
            self.lidar_frame_entry.row_idx = id;
        }
        fn row_idx(&self) -> usize {
            self.lidar_frame_entry.row_idx
        }
        fn set_col_idx(&mut self, id: usize) {
            self.lidar_frame_entry.col_idx = id;
        }
        fn col_idx(&self) -> usize {
            self.lidar_frame_entry.col_idx
        }
    }
}

pub use dual_return_point::*;
mod dual_return_point {
    use super::*;

    /// Point in dual return mode.
    #[derive(Debug, Clone)]
    pub struct DualPoint {
        pub laser_id: u32,
        pub timestamp: Duration,
        pub original_azimuth_angle: Angle,
        pub corrected_azimuth_angle: Angle,
        pub strongest_return_data: Measurement,
        pub last_return_data: Measurement,
        pub lidar_frame_entry: LidarFrameEntry,
    }

    impl DualPoint {
        pub fn try_from_pair(
            strongest_return_point: SinglePoint,
            last_return_point: SinglePoint,
        ) -> Result<Self> {
            let SinglePoint {
                laser_id: laser_id_strongest,
                timestamp: timestamp_strongest,
                original_azimuth_angle: original_azimuth_angle_strongest,
                corrected_azimuth_angle: corrected_azimuth_angle_strongest,
                data: strongest_return_data,
                ..
            } = strongest_return_point;

            let SinglePoint {
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

            let dual_return_point = DualPoint {
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

    impl VelodynePoint for DualPoint {
        fn laser_id(&self) -> u32 {
            self.laser_id
        }

        fn timestamp(&self) -> Duration {
            self.timestamp
        }

        fn original_azimuth(&self) -> Angle {
            self.original_azimuth_angle
        }

        fn corrected_azimuth(&self) -> Angle {
            self.corrected_azimuth_angle
        }
    }

    impl LidarFrameMsg for DualPoint {
        fn set_row_idx(&mut self, id: usize) {
            self.lidar_frame_entry.row_idx = id;
        }
        fn row_idx(&self) -> usize {
            self.lidar_frame_entry.row_idx
        }
        fn set_col_idx(&mut self, id: usize) {
            self.lidar_frame_entry.col_idx = id;
        }
        fn col_idx(&self) -> usize {
            self.lidar_frame_entry.col_idx
        }
    }
}

pub use dynamic_return_points::*;
mod dynamic_return_points {
    use crate::velodyne::PcdFrame;

    use super::*;

    // Convert dynamic return points to frame
    #[derive(Debug, Clone)]
    pub enum DynamicReturnFrame {
        Single(PcdFrame<SinglePoint>),
        Dual(PcdFrame<DualPoint>),
    }

    impl DynamicReturnFrame {
        pub fn is_empty(&self) -> bool {
            match self {
                Self::Single(points) => points.data.is_empty(),
                Self::Dual(points) => points.data.is_empty(),
            }
        }
    }

    impl IntoIterator for DynamicReturnFrame {
        type Item = DPoint;
        type IntoIter = DPointsIter;

        fn into_iter(self) -> Self::IntoIter {
            let iter: Box<dyn Iterator<Item = DPoint> + Sync + Send> = match self {
                Self::Single(points) => Box::new(points.data.into_iter().map(DPoint::Single)),
                Self::Dual(points) => Box::new(points.data.into_iter().map(DPoint::Dual)),
            };
            Self::IntoIter { iter }
        }
    }

    /// Collection of points in either single return or dual return mode.
    #[derive(Debug, Clone)]
    pub enum DPoints {
        Single(Vec<SinglePoint>),
        Dual(Vec<DualPoint>),
    }

    impl DPoints {
        pub fn is_empty(&self) -> bool {
            match self {
                Self::Single(points) => points.is_empty(),
                Self::Dual(points) => points.is_empty(),
            }
        }

        // pub(crate) fn empty_like(&self) -> Self {
        //     match self {
        //         Self::Single(_) => Self::Single(vec![]),
        //         Self::Dual(_) => Self::Dual(vec![]),
        //     }
        // }
    }

    impl IntoIterator for DPoints {
        type Item = DPoint;
        type IntoIter = DPointsIter;

        fn into_iter(self) -> Self::IntoIter {
            let iter: Box<dyn Iterator<Item = DPoint> + Sync + Send> = match self {
                Self::Single(points) => Box::new(points.into_iter().map(DPoint::Single)),
                Self::Dual(points) => Box::new(points.into_iter().map(DPoint::Dual)),
            };
            Self::IntoIter { iter }
        }
    }

    impl From<Vec<SinglePoint>> for DPoints {
        fn from(points: Vec<SinglePoint>) -> Self {
            Self::Single(points)
        }
    }

    impl From<Vec<DualPoint>> for DPoints {
        fn from(points: Vec<DualPoint>) -> Self {
            Self::Dual(points)
        }
    }

    /// Collection of points in either single return or dual return mode.
    #[derive(Derivative)]
    #[derivative(Debug)]
    pub struct DPointsIter {
        #[derivative(Debug = "ignore")]
        iter: Box<dyn Iterator<Item = DPoint> + Sync + Send>,
    }

    impl Iterator for DPointsIter {
        type Item = DPoint;

        fn next(&mut self) -> Option<Self::Item> {
            self.iter.next()
        }
    }

    /// collection of points in either single return or dual return mode.
    #[derive(Debug, Clone)]
    pub enum DPoint {
        Single(SinglePoint),
        Dual(DualPoint),
    }

    impl DPoint {
        pub fn laser_id(&self) -> u32 {
            match self {
                Self::Single(point) => point.laser_id,
                Self::Dual(point) => point.laser_id,
            }
        }

        pub fn timestamp(&self) -> Duration {
            match self {
                Self::Single(point) => point.timestamp,
                Self::Dual(point) => point.timestamp,
            }
        }

        pub fn original_azimuth(&self) -> Angle {
            match self {
                Self::Single(point) => point.original_azimuth_angle,
                Self::Dual(point) => point.original_azimuth_angle,
            }
        }

        pub fn corrected_azimuth(&self) -> Angle {
            match self {
                Self::Single(point) => point.corrected_azimuth_angle,
                Self::Dual(point) => point.corrected_azimuth_angle,
            }
        }
    }
}
