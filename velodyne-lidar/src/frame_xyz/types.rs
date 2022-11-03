use crate::{
    common::*,
    firing_xyz::{
        iter::{
            FiringXyzIter, FiringXyzIterD16, FiringXyzIterD32, FiringXyzIterS16, FiringXyzIterS32,
            FiringXyzRefIter, FiringXyzRefIterD16, FiringXyzRefIterD32, FiringXyzRefIterS16,
            FiringXyzRefIterS32,
        },
        types::{FiringXyzD16, FiringXyzD32, FiringXyzS16, FiringXyzS32},
    },
    kinds::FormatKind,
    point::{
        iter::PointIter,
        types::{Point, PointD, PointS},
    },
    traits::PointField,
};

pub use frame_kind::*;
mod frame_kind {

    use super::*;

    pub type FrameXyz = FormatKind<FrameXyzS16, FrameXyzS32, FrameXyzD16, FrameXyzD32>;

    impl FrameXyz {
        pub fn firing_iter<'a>(
            &'a self,
        ) -> FiringXyzRefIter<
            impl Iterator<Item = &'a FiringXyzS16>,
            impl Iterator<Item = &'a FiringXyzS32>,
            impl Iterator<Item = &'a FiringXyzD16>,
            impl Iterator<Item = &'a FiringXyzD32>,
        > {
            match self {
                FrameXyz::Single16(me) => FiringXyzRefIterS16(me.firings.iter()).into(),
                FrameXyz::Single32(me) => FiringXyzRefIterS32(me.firings.iter()).into(),
                FrameXyz::Dual16(me) => FiringXyzRefIterD16(me.firings.iter()).into(),
                FrameXyz::Dual32(me) => FiringXyzRefIterD32(me.firings.iter()).into(),
            }
        }

        pub fn into_firing_iter(
            self,
        ) -> FiringXyzIter<
            impl Iterator<Item = FiringXyzS16>,
            impl Iterator<Item = FiringXyzS32>,
            impl Iterator<Item = FiringXyzD16>,
            impl Iterator<Item = FiringXyzD32>,
        > {
            match self {
                FrameXyz::Single16(me) => FiringXyzIterS16(me.firings.into_iter()).into(),
                FrameXyz::Single32(me) => FiringXyzIterS32(me.firings.into_iter()).into(),
                FrameXyz::Dual16(me) => FiringXyzIterD16(me.firings.into_iter()).into(),
                FrameXyz::Dual32(me) => FiringXyzIterD32(me.firings.into_iter()).into(),
            }
        }

        pub fn into_point_iter(
            self,
        ) -> PointIter<
            impl Iterator<Item = PointS>,
            impl Iterator<Item = PointS>,
            impl Iterator<Item = PointD>,
            impl Iterator<Item = PointD>,
        > {
            match self {
                Self::Single16(frame) => PointIter::Single16(frame.into_point_iter()),
                Self::Single32(frame) => PointIter::Single32(frame.into_point_iter()),
                Self::Dual16(frame) => PointIter::Dual16(frame.into_point_iter()),
                Self::Dual32(frame) => PointIter::Dual32(frame.into_point_iter()),
            }
        }

        pub fn into_indexed_point_iter(self) -> Box<dyn Iterator<Item = ((usize, usize), Point)>> {
            match self {
                Self::Single16(frame) => Box::new(
                    frame
                        .into_indexed_point_iter()
                        .map(|(index, point)| (index, Point::from(point))),
                ),
                Self::Single32(frame) => Box::new(
                    frame
                        .into_indexed_point_iter()
                        .map(|(index, point)| (index, Point::from(point))),
                ),
                Self::Dual16(frame) => Box::new(
                    frame
                        .into_indexed_point_iter()
                        .map(|(index, point)| (index, Point::from(point))),
                ),
                Self::Dual32(frame) => Box::new(
                    frame
                        .into_indexed_point_iter()
                        .map(|(index, point)| (index, Point::from(point))),
                ),
            }
        }
    }

    impl From<FrameXyzD16> for FrameXyz {
        fn from(v: FrameXyzD16) -> Self {
            Self::Dual16(v)
        }
    }

    impl From<FrameXyzD32> for FrameXyz {
        fn from(v: FrameXyzD32) -> Self {
            Self::Dual32(v)
        }
    }

    impl From<FrameXyzS32> for FrameXyz {
        fn from(v: FrameXyzS32) -> Self {
            Self::Single32(v)
        }
    }

    impl From<FrameXyzS16> for FrameXyz {
        fn from(v: FrameXyzS16) -> Self {
            Self::Single16(v)
        }
    }
}

pub use frame_types::*;
mod frame_types {
    use super::*;

    macro_rules! declare_type {
        ($name:ident, $firing:ident, $nrows:expr, $point:ident) => {
            #[derive(Debug, Clone)]
            pub struct $name {
                pub firings: Vec<$firing>,
            }

            impl PointField for $name {
                type Point<'a> = &'a $point;

                fn nrows(&self) -> usize {
                    $nrows
                }

                fn ncols(&self) -> usize {
                    self.firings.len()
                }

                fn point_at(&self, row: usize, col: usize) -> Option<Self::Point<'_>> {
                    self.firings.get(col)?.points.get(row)
                }
            }

            impl $name {
                pub fn into_point_iter(self) -> impl Iterator<Item = $point> {
                    self.firings.into_iter().flat_map(|firing| firing.points)
                }

                pub fn into_indexed_point_iter(
                    self,
                ) -> impl Iterator<Item = ((usize, usize), $point)> {
                    self.firings
                        .into_iter()
                        .enumerate()
                        .flat_map(|(col, firing)| {
                            firing
                                .points
                                .into_iter()
                                .enumerate()
                                .map(move |(row, point)| ((row, col), point))
                        })
                }
            }
        };
    }

    declare_type!(FrameXyzS16, FiringXyzS16, 16, PointS);
    declare_type!(FrameXyzS32, FiringXyzS32, 32, PointS);
    declare_type!(FrameXyzD16, FiringXyzD16, 16, PointD);
    declare_type!(FrameXyzD32, FiringXyzD32, 32, PointD);
}
