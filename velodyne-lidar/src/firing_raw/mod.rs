//! Firings of blocks.

use crate::{kinds::FormatKind, packet::Channel};
use std::{ops::Range, time::Duration};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringRawS16 {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub channels: [Channel; 16],
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringRawS32 {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub channels: [Channel; 32],
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringRawD16 {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub channels_strongest: [Channel; 16],
    pub channels_last: [Channel; 16],
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringRawD32 {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub channels_strongest: [Channel; 32],
    pub channels_last: [Channel; 32],
}

pub use kind::*;
mod kind {
    use super::*;

    pub type FiringRaw = FormatKind<FiringRawS16, FiringRawS32, FiringRawD16, FiringRawD32>;

    impl FiringRaw {
        pub fn time(&self) -> Duration {
            match self {
                FiringRaw::Single16(me) => me.time,
                FiringRaw::Single32(me) => me.time,
                FiringRaw::Dual16(me) => me.time,
                FiringRaw::Dual32(me) => me.time,
            }
        }
    }

    impl From<FiringRawD32> for FiringRaw {
        fn from(v: FiringRawD32) -> Self {
            Self::Dual32(v)
        }
    }

    impl From<FiringRawD16> for FiringRaw {
        fn from(v: FiringRawD16) -> Self {
            Self::Dual16(v)
        }
    }

    impl From<FiringRawS32> for FiringRaw {
        fn from(v: FiringRawS32) -> Self {
            Self::Single32(v)
        }
    }

    impl From<FiringRawS16> for FiringRaw {
        fn from(v: FiringRawS16) -> Self {
            Self::Single16(v)
        }
    }
}

use measurements::Angle;
pub use ref_kind::*;
mod ref_kind {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum FiringRawRef<'a> {
        Single16(&'a FiringRawS16),
        Single32(&'a FiringRawS32),
        Dual16(&'a FiringRawD16),
        Dual32(&'a FiringRawD32),
    }

    impl<'a> FiringRawRef<'a> {
        pub fn time(&self) -> Duration {
            match self {
                FiringRawRef::Single16(me) => me.time,
                FiringRawRef::Single32(me) => me.time,
                FiringRawRef::Dual16(me) => me.time,
                FiringRawRef::Dual32(me) => me.time,
            }
        }
    }

    impl<'a> From<&'a FiringRawD32> for FiringRawRef<'a> {
        fn from(v: &'a FiringRawD32) -> Self {
            Self::Dual32(v)
        }
    }

    impl<'a> From<&'a FiringRawD16> for FiringRawRef<'a> {
        fn from(v: &'a FiringRawD16) -> Self {
            Self::Dual16(v)
        }
    }

    impl<'a> From<&'a FiringRawS32> for FiringRawRef<'a> {
        fn from(v: &'a FiringRawS32) -> Self {
            Self::Single32(v)
        }
    }

    impl<'a> From<&'a FiringRawS16> for FiringRawRef<'a> {
        fn from(v: &'a FiringRawS16) -> Self {
            Self::Single16(v)
        }
    }
}
