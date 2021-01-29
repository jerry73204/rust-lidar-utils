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
}

/// Point in strongest or last return mode.
#[derive(Debug, Clone)]
pub struct PointData {
    pub distance: Length,
    pub intensity: u8,
    pub position: [Length; 3],
}

mod single_return_point {
    use super::*;

    /// Point in strongest or last return mode.
    #[derive(Debug, Clone)]
    pub struct SingleReturnPoint {
        pub laser_id: u32,
        pub timestamp: Time,
        pub original_azimuth_angle: Angle,
        pub corrected_azimuth_angle: Angle,
        pub data: PointData,
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
    }
}

mod dual_return_point {
    use super::*;

    /// Point in dual return mode.
    #[derive(Debug, Clone)]
    pub struct DualReturnPoint {
        pub laser_id: u32,
        pub timestamp: Time,
        pub original_azimuth_angle: Angle,
        pub corrected_azimuth_angle: Angle,
        pub strongest_return_data: PointData,
        pub last_return_data: PointData,
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
            } = strongest_return_point;

            let SingleReturnPoint {
                laser_id: laser_id_last,
                timestamp: timestamp_last,
                original_azimuth_angle: original_azimuth_angle_last,
                corrected_azimuth_angle: corrected_azimuth_angle_last,
                data: last_return_data,
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
}
