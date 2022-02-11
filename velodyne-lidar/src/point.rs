//! Point data types.

use crate::common::*;

pub use measurement::*;
mod measurement {
    use super::*;

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

    impl PointDual {
        pub fn measurement_strongest(&self) -> &Measurement {
            &self.measurements.strongest
        }

        pub fn measurement_last(&self) -> &Measurement {
            &self.measurements.last
        }
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
