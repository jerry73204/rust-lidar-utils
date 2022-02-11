use crate::{
    common::*,
    point::{PointDual, PointSingle},
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
pub enum FiringXyzKind {
    Single16(FiringXyzSingle16),
    Single32(FiringXyzSingle32),
    Dual16(FiringXyzDual16),
    Dual32(FiringXyzDual32),
}

impl FiringXyzKind {
    pub fn time(&self) -> Duration {
        match self {
            FiringXyzKind::Single16(me) => me.time,
            FiringXyzKind::Single32(me) => me.time,
            FiringXyzKind::Dual16(me) => me.time,
            FiringXyzKind::Dual32(me) => me.time,
        }
    }

    pub fn azimuth_count(&self) -> u16 {
        match self {
            FiringXyzKind::Single16(me) => me.azimuth_count,
            FiringXyzKind::Single32(me) => me.azimuth_count,
            FiringXyzKind::Dual16(me) => me.azimuth_count,
            FiringXyzKind::Dual32(me) => me.azimuth_count,
        }
    }

    pub fn azimuth_range(&self) -> &Range<Angle> {
        match self {
            FiringXyzKind::Single16(me) => &me.azimuth_range,
            FiringXyzKind::Single32(me) => &me.azimuth_range,
            FiringXyzKind::Dual16(me) => &me.azimuth_range,
            FiringXyzKind::Dual32(me) => &me.azimuth_range,
        }
    }

    pub fn try_into_single16(self) -> Result<FiringXyzSingle16, Self> {
        if let Self::Single16(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_single32(self) -> Result<FiringXyzSingle32, Self> {
        if let Self::Single32(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_dual16(self) -> Result<FiringXyzDual16, Self> {
        if let Self::Dual16(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_dual32(self) -> Result<FiringXyzDual32, Self> {
        if let Self::Dual32(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }
}

impl From<FiringXyzDual32> for FiringXyzKind {
    fn from(v: FiringXyzDual32) -> Self {
        Self::Dual32(v)
    }
}

impl From<FiringXyzDual16> for FiringXyzKind {
    fn from(v: FiringXyzDual16) -> Self {
        Self::Dual16(v)
    }
}

impl From<FiringXyzSingle32> for FiringXyzKind {
    fn from(v: FiringXyzSingle32) -> Self {
        Self::Single32(v)
    }
}

impl From<FiringXyzSingle16> for FiringXyzKind {
    fn from(v: FiringXyzSingle16) -> Self {
        Self::Single16(v)
    }
}
