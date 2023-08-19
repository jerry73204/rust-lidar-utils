//! Frames with raw sensor values.

use super::{
    channel_array::ChannelArrayD,
    firing_raw::{FiringRaw, FiringRawRef},
};
use crate::{
    common::*,
    packet::Channel,
    traits::{BoxIterator, PointField},
    types::{
        channel::{ChannelD, ChannelKind, ChannelRefD},
        firing_raw::{FiringRawD16, FiringRawD32, FiringRawS16, FiringRawS32},
        format::FormatKind,
    },
};

pub type FrameRaw = FormatKind<FrameRawS16, FrameRawS32, FrameRawD16, FrameRawD32>;

impl FrameRaw {
    pub fn firing_iter(&self) -> impl Iterator<Item = FiringRawRef<'_>> + Clone + Sync + Send {
        match self {
            FrameRaw::Single16(me) => FormatKind::from_s16(me.firings.iter()),
            FrameRaw::Single32(me) => FormatKind::from_s32(me.firings.iter()),
            FrameRaw::Dual16(me) => FormatKind::from_d16(me.firings.iter()),
            FrameRaw::Dual32(me) => FormatKind::from_d32(me.firings.iter()),
        }
    }

    pub fn into_firing_iter(self) -> impl Iterator<Item = FiringRaw> + Clone + Sync + Send {
        match self {
            FrameRaw::Single16(me) => FormatKind::from_s16(me.firings.into_iter()),
            FrameRaw::Single32(me) => FormatKind::from_s32(me.firings.into_iter()),
            FrameRaw::Dual16(me) => FormatKind::from_d16(me.firings.into_iter()),
            FrameRaw::Dual32(me) => FormatKind::from_d32(me.firings.into_iter()),
        }
    }

    pub fn into_channel_iter(self) -> BoxIterator<'static, ChannelKind> {
        match self {
            Self::Single16(frame) => Box::new(frame.into_channel_iter().map(ChannelKind::from)),
            Self::Single32(frame) => Box::new(frame.into_channel_iter().map(ChannelKind::from)),
            Self::Dual16(frame) => Box::new(frame.into_channel_iter().map(ChannelKind::from)),
            Self::Dual32(frame) => Box::new(frame.into_channel_iter().map(ChannelKind::from)),
        }
    }

    pub fn into_indexed_channel_iter(self) -> BoxIterator<'static, ((usize, usize), ChannelKind)> {
        match self {
            Self::Single16(frame) => Box::new(
                frame
                    .into_indexed_channel_iter()
                    .map(|(index, point)| (index, ChannelKind::from(point))),
            ),
            Self::Single32(frame) => Box::new(
                frame
                    .into_indexed_channel_iter()
                    .map(|(index, point)| (index, ChannelKind::from(point))),
            ),
            Self::Dual16(frame) => Box::new(
                frame
                    .into_indexed_channel_iter()
                    .map(|(index, point)| (index, ChannelKind::from(point))),
            ),
            Self::Dual32(frame) => Box::new(
                frame
                    .into_indexed_channel_iter()
                    .map(|(index, point)| (index, ChannelKind::from(point))),
            ),
        }
    }
}

impl From<FrameRawD16> for FrameRaw {
    fn from(v: FrameRawD16) -> Self {
        Self::Dual16(v)
    }
}

impl From<FrameRawD32> for FrameRaw {
    fn from(v: FrameRawD32) -> Self {
        Self::Dual32(v)
    }
}

impl From<FrameRawS32> for FrameRaw {
    fn from(v: FrameRawS32) -> Self {
        Self::Single32(v)
    }
}

impl From<FrameRawS16> for FrameRaw {
    fn from(v: FrameRawS16) -> Self {
        Self::Single16(v)
    }
}

macro_rules! declare_type_single {
    ($name:ident, $firing:ident, $nrows:expr) => {
        #[derive(Debug, Clone)]
        pub struct $name {
            pub firings: Vec<$firing>,
        }

        impl PointField for $name {
            type Point<'a> = &'a Channel;

            fn nrows(&self) -> usize {
                $nrows
            }

            fn ncols(&self) -> usize {
                self.firings.len()
            }

            fn point_at(&self, row: usize, col: usize) -> Option<Self::Point<'_>> {
                self.firings.get(col)?.channels.get(row)
            }
        }

        impl $name {
            pub fn into_channel_iter(self) -> impl Iterator<Item = Channel> + Clone + Sync + Send {
                self.firings.into_iter().flat_map(|firing| firing.channels)
            }

            pub fn into_indexed_channel_iter(
                self,
            ) -> impl Iterator<Item = ((usize, usize), Channel)> + Clone + Sync + Send {
                self.firings
                    .into_iter()
                    .enumerate()
                    .flat_map(|(col, firing)| {
                        firing
                            .channels
                            .into_iter()
                            .enumerate()
                            .map(move |(row, point)| ((row, col), point))
                    })
            }
        }
    };
}

macro_rules! declare_type_dual {
    ($name:ident, $firing:ident, $nrows:expr) => {
        #[derive(Debug, Clone)]
        pub struct $name {
            pub firings: Vec<$firing>,
        }

        impl PointField for $name {
            type Point<'a> = ChannelRefD<'a>;

            fn nrows(&self) -> usize {
                $nrows
            }

            fn ncols(&self) -> usize {
                self.firings.len()
            }

            fn point_at(&self, row: usize, col: usize) -> Option<Self::Point<'_>> {
                let firing = self.firings.get(col)?;
                let strongest = firing.channels.strongest.get(row)?;
                let last = firing.channels.last.get(row)?;
                Some(ChannelRefD { strongest, last })
            }
        }

        impl $name {
            pub fn into_channel_iter(self) -> impl Iterator<Item = ChannelD> + Clone + Sync + Send {
                self.firings.into_iter().flat_map(|firing| {
                    let ChannelArrayD { strongest, last } = firing.channels;
                    izip!(strongest, last).map(|(strongest, last)| ChannelD { strongest, last })
                })
            }

            pub fn into_indexed_channel_iter(
                self,
            ) -> impl Iterator<Item = ((usize, usize), ChannelD)> + Clone + Sync + Send {
                self.firings
                    .into_iter()
                    .enumerate()
                    .flat_map(|(col, firing)| {
                        let ChannelArrayD { strongest, last } = firing.channels;
                        izip!(strongest, last)
                            .enumerate()
                            .map(move |(row, (strongest, last))| {
                                let pair = ChannelD { strongest, last };
                                ((row, col), pair)
                            })
                    })
            }
        }
    };
}

declare_type_single!(FrameRawS16, FiringRawS16, 16);
declare_type_single!(FrameRawS32, FiringRawS32, 32);
declare_type_dual!(FrameRawD16, FiringRawD16, 16);
declare_type_dual!(FrameRawD32, FiringRawD32, 32);
