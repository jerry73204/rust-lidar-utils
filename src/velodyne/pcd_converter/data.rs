use uom::si::f64::{Angle as F64Angle, Length as F64Length, Time as F64Time};

/// Point in strongest or last return mode.
#[derive(Debug, Clone)]
pub struct SingleReturnPoint {
    pub timestamp: F64Time,
    pub azimuth_angle: F64Angle,
    pub distance: F64Length,
    pub intensity: u8,
    pub laser_id: u32,
    pub point: [F64Length; 3],
}

impl SingleReturnPoint {
    pub fn timestamp(&self) -> F64Time {
        self.timestamp
    }

    pub fn azimuth_angle(&self) -> F64Angle {
        self.azimuth_angle
    }
}

/// Point in dual return mode.
#[derive(Debug, Clone)]
pub struct DualReturnPoint {
    pub strongest_return: SingleReturnPoint,
    pub last_return: SingleReturnPoint,
}

impl DualReturnPoint {
    pub fn timestamp(&self) -> F64Time {
        assert_eq!(self.strongest_return.timestamp, self.last_return.timestamp);
        self.strongest_return.timestamp
    }

    pub fn azimuth_angle(&self) -> F64Angle {
        assert_eq!(
            self.strongest_return.azimuth_angle,
            self.last_return.azimuth_angle
        );
        self.strongest_return.azimuth_angle
    }
}

/// A point type can be in strongest, last or dual return mode.
#[derive(Debug, Clone)]
pub enum DynamicReturnPoint {
    SingleReturn(SingleReturnPoint),
    DualReturn(DualReturnPoint),
}

impl DynamicReturnPoint {
    pub fn timestamp(&self) -> F64Time {
        use DynamicReturnPoint::*;
        match self {
            SingleReturn(point) => point.timestamp(),
            DualReturn(point) => point.timestamp(),
        }
    }

    pub fn azimuth_angle(&self) -> F64Angle {
        use DynamicReturnPoint::*;
        match self {
            SingleReturn(point) => point.azimuth_angle(),
            DualReturn(point) => point.azimuth_angle(),
        }
    }
}

impl From<SingleReturnPoint> for DynamicReturnPoint {
    fn from(point: SingleReturnPoint) -> Self {
        Self::SingleReturn(point)
    }
}

impl From<DualReturnPoint> for DynamicReturnPoint {
    fn from(point: DualReturnPoint) -> Self {
        Self::DualReturn(point)
    }
}
