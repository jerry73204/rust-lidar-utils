use crate::{
    common::*,
    convert::functions::{
        firing_block_to_xyz_d16, firing_block_to_xyz_d32, firing_block_to_xyz_s16,
        firing_block_to_xyz_s32,
    },
    firing_xyz::{FiringXyz, FiringXyzD16, FiringXyzD32, FiringXyzS16, FiringXyzS32},
    kinds::FormatKind,
    packet::{Block, Channel},
    Config, Config16, Config32,
};
use anyhow::anyhow;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringBlockS16<'a> {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub block: &'a Block,
    pub channels: &'a [Channel; 16],
}

impl<'a> FiringBlockS16<'a> {
    pub fn to_firing_xyz(&self, beams: &Config16) -> FiringXyzS16 {
        firing_block_to_xyz_s16(self, beams)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FiringBlockS32<'a> {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub block: &'a Block,
    pub channels: &'a [Channel; 32],
}

impl<'a> FiringBlockS32<'a> {
    pub fn to_firing_xyz(&self, beams: &Config32) -> FiringXyzS32 {
        firing_block_to_xyz_s32(self, beams)
    }
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
    pub fn to_firing_xyz(&self, beams: &Config16) -> FiringXyzD16 {
        firing_block_to_xyz_d16(self, beams)
    }

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
pub struct FiringBlockD32<'a> {
    pub time: Duration,
    pub azimuth_range: Range<Angle>,
    pub block_strongest: &'a Block,
    pub block_last: &'a Block,
    pub channels_strongest: &'a [Channel; 32],
    pub channels_last: &'a [Channel; 32],
}

impl<'a> FiringBlockD32<'a> {
    pub fn to_firing_xyz(&self, beams: &Config32) -> FiringXyzD32 {
        firing_block_to_xyz_d32(self, beams)
    }

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

pub type FiringBlock<'a> =
    FormatKind<FiringBlockS16<'a>, FiringBlockS32<'a>, FiringBlockD16<'a>, FiringBlockD32<'a>>;

impl<'a> FiringBlock<'a> {
    pub fn to_firing_xyz(&self, beams: &Config) -> Result<FiringXyz> {
        let err = || anyhow!("TODO");

        use FormatKind as F;

        let output = match self {
            F::Single16(inner) => {
                let beams: Config16 = beams.clone().try_into().map_err(|_| err())?;
                inner.to_firing_xyz(&beams).into()
            }
            F::Dual16(inner) => {
                let beams: Config16 = beams.clone().try_into().map_err(|_| err())?;
                inner.to_firing_xyz(&beams).into()
            }
            F::Single32(inner) => {
                let beams: Config32 = beams.clone().try_into().map_err(|_| err())?;
                inner.to_firing_xyz(&beams).into()
            }
            F::Dual32(inner) => {
                let beams: Config32 = beams.clone().try_into().map_err(|_| err())?;
                inner.to_firing_xyz(&beams).into()
            }
        };

        Ok(output)
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
