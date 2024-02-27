//! Point types for laser measurements.
use crate::types::measurements::{Measurement, MeasurementDual};
use measurements::Angle;
use std::time::Duration;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PointS {
    pub laser_id: usize,
    pub toh: Duration,
    pub azimuth: Angle,
    pub measurement: Measurement,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PointD {
    pub laser_id: usize,
    pub toh: Duration,
    pub azimuth: Angle,
    pub measurements: MeasurementDual,
}

impl PointD {
    pub fn measurement_strongest(&self) -> &Measurement {
        &self.measurements.strongest
    }

    pub fn measurement_last(&self) -> &Measurement {
        &self.measurements.last
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Point {
    Single(PointS),
    Dual(PointD),
}

impl Point {
    pub fn laser_id(&self) -> usize {
        match self {
            Self::Single(point) => point.laser_id,
            Self::Dual(point) => point.laser_id,
        }
    }

    pub fn time(&self) -> Duration {
        match self {
            Self::Single(point) => point.toh,
            Self::Dual(point) => point.toh,
        }
    }

    pub fn azimuth(&self) -> Angle {
        match self {
            Self::Single(point) => point.azimuth,
            Self::Dual(point) => point.azimuth,
        }
    }

    pub fn try_into_single(self) -> Result<PointS, Self> {
        if let Self::Single(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_dual(self) -> Result<PointD, Self> {
        if let Self::Dual(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn as_single(&self) -> Option<&PointS> {
        if let Self::Single(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_dual(&self) -> Option<&PointD> {
        if let Self::Dual(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl From<PointD> for Point {
    fn from(v: PointD) -> Self {
        Self::Dual(v)
    }
}

impl From<PointS> for Point {
    fn from(v: PointS) -> Self {
        Self::Single(v)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PointRef<'a> {
    Single(&'a PointS),
    Dual(&'a PointD),
}

impl<'a> PointRef<'a> {
    pub fn laser_id(&self) -> usize {
        match self {
            Self::Single(point) => point.laser_id,
            Self::Dual(point) => point.laser_id,
        }
    }

    pub fn time(&self) -> Duration {
        match self {
            Self::Single(point) => point.toh,
            Self::Dual(point) => point.toh,
        }
    }

    pub fn azimuth(&self) -> Angle {
        match self {
            Self::Single(point) => point.azimuth,
            Self::Dual(point) => point.azimuth,
        }
    }

    pub fn as_single(&self) -> Option<&'a PointS> {
        if let Self::Single(v) = self {
            Some(v)
        } else {
            None
        }
    }

    pub fn as_dual(&self) -> Option<&'a PointD> {
        if let Self::Dual(v) = self {
            Some(v)
        } else {
            None
        }
    }
}

impl<'a> From<&'a PointD> for PointRef<'a> {
    fn from(v: &'a PointD) -> Self {
        Self::Dual(v)
    }
}

impl<'a> From<&'a PointS> for PointRef<'a> {
    fn from(v: &'a PointS) -> Self {
        Self::Single(v)
    }
}
