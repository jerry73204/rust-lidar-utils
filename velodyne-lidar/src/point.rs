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

/// Point in strongest or last return mode.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeasurementDual {
    pub strongest: Measurement,
    pub last: Measurement,
}

/// Point in strongest or last return mode.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MeasurementKind {
    Single(Measurement),
    Dual(MeasurementDual),
}

impl From<Measurement> for MeasurementKind {
    fn from(from: Measurement) -> Self {
        Self::Single(from)
    }
}

impl From<MeasurementDual> for MeasurementKind {
    fn from(from: MeasurementDual) -> Self {
        Self::Dual(from)
    }
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

pub use point_types::*;
mod point_types {
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
        pub measurements: MeasurementDual,
    }
}

pub use point_kind::*;
mod point_kind {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct PointKind {
        pub laser_id: usize,
        pub time: Duration,
        pub azimuth: Angle,
        pub measurement: MeasurementKind,
    }

    impl From<PointSingle> for PointKind {
        fn from(from: PointSingle) -> Self {
            let PointSingle {
                laser_id,
                time,
                azimuth,
                measurement,
            } = from;
            Self {
                laser_id,
                time,
                azimuth,
                measurement: measurement.into(),
            }
        }
    }

    impl From<PointDual> for PointKind {
        fn from(from: PointDual) -> Self {
            let PointDual {
                laser_id,
                time,
                azimuth,
                measurements,
            } = from;
            Self {
                laser_id,
                time,
                azimuth,
                measurement: measurements.into(),
            }
        }
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum PointKindRef<'a> {
        Single(&'a PointSingle),
        Dual(&'a PointDual),
    }

    impl<'a> PointKindRef<'a> {
        pub fn laser_id(&self) -> usize {
            match self {
                Self::Single(point) => point.laser_id,
                Self::Dual(point) => point.laser_id,
            }
        }

        pub fn time(&self) -> Duration {
            match self {
                Self::Single(point) => point.time,
                Self::Dual(point) => point.time,
            }
        }

        pub fn azimuth(&self) -> Angle {
            match self {
                Self::Single(point) => point.azimuth,
                Self::Dual(point) => point.azimuth,
            }
        }

        pub fn try_into_single(self) -> Result<&'a PointSingle, Self> {
            if let Self::Single(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_dual(self) -> Result<&'a PointDual, Self> {
            if let Self::Dual(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn as_single(&self) -> Option<&&'a PointSingle> {
            if let Self::Single(v) = self {
                Some(v)
            } else {
                None
            }
        }

        pub fn as_dual(&self) -> Option<&&'a PointDual> {
            if let Self::Dual(v) = self {
                Some(v)
            } else {
                None
            }
        }
    }

    impl<'a> From<&'a PointDual> for PointKindRef<'a> {
        fn from(v: &'a PointDual) -> Self {
            Self::Dual(v)
        }
    }

    impl<'a> From<&'a PointSingle> for PointKindRef<'a> {
        fn from(v: &'a PointSingle) -> Self {
            Self::Single(v)
        }
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
