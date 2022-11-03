use crate::{
    common::*,
    kinds::FormatKind,
    point::types::{PointD, PointS},
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringXyzS16 {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub points: [PointS; 16],
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringXyzS32 {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub points: [PointS; 32],
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringXyzD16 {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub points: [PointD; 16],
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringXyzD32 {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub points: [PointD; 32],
}

pub use kind::*;
mod kind {
    use super::*;

    pub type FiringXyz = FormatKind<FiringXyzS16, FiringXyzS32, FiringXyzD16, FiringXyzD32>;

    impl FiringXyz {
        pub fn time(&self) -> Duration {
            match self {
                FiringXyz::Single16(me) => me.time,
                FiringXyz::Single32(me) => me.time,
                FiringXyz::Dual16(me) => me.time,
                FiringXyz::Dual32(me) => me.time,
            }
        }
    }

    impl From<FiringXyzD32> for FiringXyz {
        fn from(v: FiringXyzD32) -> Self {
            Self::Dual32(v)
        }
    }

    impl From<FiringXyzD16> for FiringXyz {
        fn from(v: FiringXyzD16) -> Self {
            Self::Dual16(v)
        }
    }

    impl From<FiringXyzS32> for FiringXyz {
        fn from(v: FiringXyzS32) -> Self {
            Self::Single32(v)
        }
    }

    impl From<FiringXyzS16> for FiringXyz {
        fn from(v: FiringXyzS16) -> Self {
            Self::Single16(v)
        }
    }
}

pub use ref_kind::*;
mod ref_kind {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum FiringXyzRef<'a> {
        Single16(&'a FiringXyzS16),
        Single32(&'a FiringXyzS32),
        Dual16(&'a FiringXyzD16),
        Dual32(&'a FiringXyzD32),
    }

    impl<'a> FiringXyzRef<'a> {
        pub fn time(&self) -> Duration {
            match self {
                FiringXyzRef::Single16(me) => me.time,
                FiringXyzRef::Single32(me) => me.time,
                FiringXyzRef::Dual16(me) => me.time,
                FiringXyzRef::Dual32(me) => me.time,
            }
        }
    }

    impl<'a> From<&'a FiringXyzD32> for FiringXyzRef<'a> {
        fn from(v: &'a FiringXyzD32) -> Self {
            Self::Dual32(v)
        }
    }

    impl<'a> From<&'a FiringXyzD16> for FiringXyzRef<'a> {
        fn from(v: &'a FiringXyzD16) -> Self {
            Self::Dual16(v)
        }
    }

    impl<'a> From<&'a FiringXyzS32> for FiringXyzRef<'a> {
        fn from(v: &'a FiringXyzS32) -> Self {
            Self::Single32(v)
        }
    }

    impl<'a> From<&'a FiringXyzS16> for FiringXyzRef<'a> {
        fn from(v: &'a FiringXyzS16) -> Self {
            Self::Single16(v)
        }
    }
}
