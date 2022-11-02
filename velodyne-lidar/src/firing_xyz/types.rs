use crate::{
    common::*,
    point::types::{PointDual, PointSingle},
};

pub(crate) use firing_trait::*;
mod firing_trait {
    use super::*;

    pub trait FiringXyz {
        fn azimuth(&self) -> Angle;
    }

    impl FiringXyz for FiringXyzSingle16 {
        fn azimuth(&self) -> Angle {
            self.azimuth_range.start
        }
    }

    impl FiringXyz for FiringXyzSingle32 {
        fn azimuth(&self) -> Angle {
            self.azimuth_range.start
        }
    }

    impl FiringXyz for FiringXyzDual16 {
        fn azimuth(&self) -> Angle {
            self.azimuth_range.start
        }
    }

    impl FiringXyz for FiringXyzDual32 {
        fn azimuth(&self) -> Angle {
            self.azimuth_range.start
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringXyzSingle16 {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub points: [PointSingle; 16],
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringXyzSingle32 {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub points: [PointSingle; 32],
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringXyzDual16 {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub points: [PointDual; 16],
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringXyzDual32 {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub points: [PointDual; 32],
}

pub use kind::*;
mod kind {
    use super::*;

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

        pub fn azimuth(&self) -> Angle {
            self.azimuth_range().start
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
}

pub use ref_kind::*;
mod ref_kind {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum FiringXyzRefKind<'a> {
        Single16(&'a FiringXyzSingle16),
        Single32(&'a FiringXyzSingle32),
        Dual16(&'a FiringXyzDual16),
        Dual32(&'a FiringXyzDual32),
    }

    impl<'a> FiringXyzRefKind<'a> {
        pub fn time(&self) -> Duration {
            match self {
                FiringXyzRefKind::Single16(me) => me.time,
                FiringXyzRefKind::Single32(me) => me.time,
                FiringXyzRefKind::Dual16(me) => me.time,
                FiringXyzRefKind::Dual32(me) => me.time,
            }
        }

        pub fn azimuth(&self) -> Angle {
            self.azimuth_range().start
        }

        pub fn azimuth_range(&self) -> &Range<Angle> {
            match self {
                FiringXyzRefKind::Single16(me) => &me.azimuth_range,
                FiringXyzRefKind::Single32(me) => &me.azimuth_range,
                FiringXyzRefKind::Dual16(me) => &me.azimuth_range,
                FiringXyzRefKind::Dual32(me) => &me.azimuth_range,
            }
        }

        pub fn to_single16(&self) -> Option<&'a FiringXyzSingle16> {
            if let Self::Single16(v) = self {
                Some(v)
            } else {
                None
            }
        }

        pub fn to_single32(&self) -> Option<&'a FiringXyzSingle32> {
            if let Self::Single32(v) = self {
                Some(v)
            } else {
                None
            }
        }

        pub fn to_dual16(&self) -> Option<&'a FiringXyzDual16> {
            if let Self::Dual16(v) = self {
                Some(v)
            } else {
                None
            }
        }

        pub fn to_dual32(&self) -> Option<&'a FiringXyzDual32> {
            if let Self::Dual32(v) = self {
                Some(v)
            } else {
                None
            }
        }
    }

    impl<'a> From<&'a FiringXyzDual32> for FiringXyzRefKind<'a> {
        fn from(v: &'a FiringXyzDual32) -> Self {
            Self::Dual32(v)
        }
    }

    impl<'a> From<&'a FiringXyzDual16> for FiringXyzRefKind<'a> {
        fn from(v: &'a FiringXyzDual16) -> Self {
            Self::Dual16(v)
        }
    }

    impl<'a> From<&'a FiringXyzSingle32> for FiringXyzRefKind<'a> {
        fn from(v: &'a FiringXyzSingle32) -> Self {
            Self::Single32(v)
        }
    }

    impl<'a> From<&'a FiringXyzSingle16> for FiringXyzRefKind<'a> {
        fn from(v: &'a FiringXyzSingle16) -> Self {
            Self::Single16(v)
        }
    }
}
