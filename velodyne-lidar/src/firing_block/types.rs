use crate::{
    common::*,
    packet::{Block, Channel, ProductID, ReturnMode},
};

pub use firing_format::*;
mod firing_format {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub enum FiringFormat {
        Single16,
        Dual16,
        Single32,
        Dual32,
    }

    impl FiringFormat {
        pub fn new(product_id: ProductID, return_mode: ReturnMode) -> Option<FiringFormat> {
            use FiringFormat::*;
            use ProductID::*;
            use ReturnMode::*;

            Some(match (product_id, return_mode) {
                (HDL32E | VLP32C, Strongest | Last) => Single32,
                (HDL32E | VLP32C, Dual) => Dual32,
                (VLP16 | PuckLite | PuckHiRes, Strongest | Last) => Single16,
                (VLP16 | PuckLite | PuckHiRes, Dual) => Dual16,
                (Velarray, Strongest | Last) => return None,
                (Velarray, Dual) => return None,
                (VLS128, Strongest | Last) => return None,
                (VLS128, Dual) => return None,
            })
        }
    }
}

pub use firing_types::*;
mod firing_types {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct FiringBlockS16<'a> {
        pub time: Duration,
        pub azimuth_range: Range<Angle>,
        pub block: &'a Block,
        pub channels: &'a [Channel; 16],
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct FiringBlockD16<'a> {
        pub time: Duration,
        pub azimuth_range: Range<Angle>,
        pub block_strongest: &'a Block,
        pub block_last: &'a Block,
        pub channels_strongest: &'a [Channel; 16],
        pub channels_last: &'a [Channel; 16],
    }

    impl<'a> FiringBlockD16<'a> {
        pub fn strongest_part(&self) -> FiringBlockS16<'a> {
            let Self {
                time,
                ref azimuth_range,
                block_strongest: block,
                channels_strongest: channels,
                ..
            } = *self;

            FiringBlockS16 {
                time,
                azimuth_range: azimuth_range.clone(),
                block,
                channels,
            }
        }

        pub fn last_part(&self) -> FiringBlockS16<'a> {
            let Self {
                time,
                ref azimuth_range,
                block_last: block,
                channels_last: channels,
                ..
            } = *self;

            FiringBlockS16 {
                time,
                azimuth_range: azimuth_range.clone(),
                block,
                channels,
            }
        }
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct FiringBlockS32<'a> {
        pub time: Duration,
        pub azimuth_range: Range<Angle>,
        pub block: &'a Block,
        pub channels: &'a [Channel; 32],
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub struct FiringBlockD32<'a> {
        pub time: Duration,
        pub azimuth_range: Range<Angle>,
        pub block_strongest: &'a Block,
        pub block_last: &'a Block,
        pub channels_strongest: &'a [Channel; 32],
        pub channels_last: &'a [Channel; 32],
    }

    impl<'a> FiringBlockD32<'a> {
        pub fn strongest_part(&self) -> FiringBlockS32<'a> {
            let Self {
                time,
                ref azimuth_range,
                block_strongest: block,
                channels_strongest: channels,
                ..
            } = *self;

            FiringBlockS32 {
                time,
                azimuth_range: azimuth_range.clone(),
                block,
                channels,
            }
        }

        pub fn last_part(&self) -> FiringBlockS32<'a> {
            let Self {
                time,
                ref azimuth_range,
                block_last: block,
                channels_last: channels,
                ..
            } = *self;

            FiringBlockS32 {
                time,
                azimuth_range: azimuth_range.clone(),
                block,
                channels,
            }
        }
    }
}

pub use firing_kind::*;
mod firing_kind {
    use super::*;

    pub enum FiringBlock<'a> {
        Single16(FiringBlockS16<'a>),
        Single32(FiringBlockS32<'a>),
        Dual16(FiringBlockD16<'a>),
        Dual32(FiringBlockD32<'a>),
    }

    impl<'a> FiringBlock<'a> {
        pub fn try_into_single16(self) -> Result<FiringBlockS16<'a>, Self> {
            if let Self::Single16(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_single32(self) -> Result<FiringBlockS32<'a>, Self> {
            if let Self::Single32(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_dual16(self) -> Result<FiringBlockD16<'a>, Self> {
            if let Self::Dual16(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_dual32(self) -> Result<FiringBlockD32<'a>, Self> {
            if let Self::Dual32(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }
    }

    impl<'a> From<FiringBlockD32<'a>> for FiringBlock<'a> {
        fn from(v: FiringBlockD32<'a>) -> Self {
            Self::Dual32(v)
        }
    }

    impl<'a> From<FiringBlockD16<'a>> for FiringBlock<'a> {
        fn from(v: FiringBlockD16<'a>) -> Self {
            Self::Dual16(v)
        }
    }

    impl<'a> From<FiringBlockS32<'a>> for FiringBlock<'a> {
        fn from(v: FiringBlockS32<'a>) -> Self {
            Self::Single32(v)
        }
    }

    impl<'a> From<FiringBlockS16<'a>> for FiringBlock<'a> {
        fn from(v: FiringBlockS16<'a>) -> Self {
            Self::Single16(v)
        }
    }
}
