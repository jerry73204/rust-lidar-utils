use crate::{
    common::*,
    velodyne::point::{PointDual, PointKind, PointSingle},
};

pub(crate) use firing_trait::*;
mod firing_trait {
    use super::*;

    pub trait FiringXyz {
        fn azimuth_count(&self) -> u16;
    }

    impl FiringXyz for FiringXyzSingle16 {
        fn azimuth_count(&self) -> u16 {
            self.azimuth_count
        }
    }

    impl FiringXyz for FiringXyzSingle32 {
        fn azimuth_count(&self) -> u16 {
            self.azimuth_count
        }
    }

    impl FiringXyz for FiringXyzDual16 {
        fn azimuth_count(&self) -> u16 {
            self.azimuth_count
        }
    }

    impl FiringXyz for FiringXyzDual32 {
        fn azimuth_count(&self) -> u16 {
            self.azimuth_count
        }
    }
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

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringXyzKind {
    pub time: Duration,
    pub azimuth_count: u16,
    pub azimuth_range: Range<Angle>,
    pub points: Vec<PointKind>,
}

impl From<FiringXyzSingle16> for FiringXyzKind {
    fn from(from: FiringXyzSingle16) -> Self {
        let FiringXyzSingle16 {
            time,
            azimuth_count,
            azimuth_range,
            points,
        } = from;

        Self {
            time,
            azimuth_count,
            azimuth_range,
            points: points.into_iter().map(Into::into).collect(),
        }
    }
}

impl From<FiringXyzSingle32> for FiringXyzKind {
    fn from(from: FiringXyzSingle32) -> Self {
        let FiringXyzSingle32 {
            time,
            azimuth_count,
            azimuth_range,
            points,
        } = from;

        Self {
            time,
            azimuth_count,
            azimuth_range,
            points: points.into_iter().map(Into::into).collect(),
        }
    }
}

impl From<FiringXyzDual16> for FiringXyzKind {
    fn from(from: FiringXyzDual16) -> Self {
        let FiringXyzDual16 {
            time,
            azimuth_count,
            azimuth_range,
            points,
        } = from;

        Self {
            time,
            azimuth_count,
            azimuth_range,
            points: points.into_iter().map(Into::into).collect(),
        }
    }
}

impl From<FiringXyzDual32> for FiringXyzKind {
    fn from(from: FiringXyzDual32) -> Self {
        let FiringXyzDual32 {
            time,
            azimuth_count,
            azimuth_range,
            points,
        } = from;

        Self {
            time,
            azimuth_count,
            azimuth_range,
            points: points.into_iter().map(Into::into).collect(),
        }
    }
}
