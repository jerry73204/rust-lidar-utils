use crate::{
    common::*,
    firing_xyz::{
        iter::{
            FiringXyzD16Iter, FiringXyzD32Iter, FiringXyzDual16RefIter, FiringXyzDual32RefIter,
            FiringXyzIter, FiringXyzRefIter, FiringXyzS16Iter, FiringXyzS32Iter,
            FiringXyzSingle16RefIter, FiringXyzSingle32RefIter,
        },
        types::{FiringXyzD16, FiringXyzD32, FiringXyzS16, FiringXyzS32},
    },
    point::{
        iter::{PointIter, PointRefIter},
        types::{Point, PointD, PointRef, PointS},
    },
};

pub use frame_kind::*;
mod frame_kind {
    use super::*;

    pub enum FrameXyz {
        Single16(FrameXyzS16),
        Single32(FrameXyzS32),
        Dual16(FrameXyzD16),
        Dual32(FrameXyzD32),
    }

    impl FrameXyz {
        pub fn nrows(&self) -> usize {
            match self {
                Self::Single16(frame) => frame.nrows(),
                Self::Single32(frame) => frame.nrows(),
                Self::Dual16(frame) => frame.nrows(),
                Self::Dual32(frame) => frame.nrows(),
            }
        }

        pub fn ncols(&self) -> usize {
            match self {
                Self::Single16(frame) => frame.ncols(),
                Self::Single32(frame) => frame.ncols(),
                Self::Dual16(frame) => frame.ncols(),
                Self::Dual32(frame) => frame.ncols(),
            }
        }

        pub fn point_at(&self, row: usize, col: usize) -> Option<PointRef<'_>> {
            Some(match self {
                Self::Single16(frame) => frame.point_at(row, col)?.into(),
                Self::Single32(frame) => frame.point_at(row, col)?.into(),
                Self::Dual16(frame) => frame.point_at(row, col)?.into(),
                Self::Dual32(frame) => frame.point_at(row, col)?.into(),
            })
        }

        pub fn firing_iter<'a>(
            &'a self,
        ) -> FiringXyzRefIter<
            impl Iterator<Item = &'a FiringXyzS16>,
            impl Iterator<Item = &'a FiringXyzS32>,
            impl Iterator<Item = &'a FiringXyzD16>,
            impl Iterator<Item = &'a FiringXyzD32>,
        > {
            match self {
                FrameXyz::Single16(me) => FiringXyzSingle16RefIter(me.firings.iter()).into(),
                FrameXyz::Single32(me) => FiringXyzSingle32RefIter(me.firings.iter()).into(),
                FrameXyz::Dual16(me) => FiringXyzDual16RefIter(me.firings.iter()).into(),
                FrameXyz::Dual32(me) => FiringXyzDual32RefIter(me.firings.iter()).into(),
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
                FrameXyz::Single16(me) => FiringXyzS16Iter(me.firings.into_iter()).into(),
                FrameXyz::Single32(me) => FiringXyzS32Iter(me.firings.into_iter()).into(),
                FrameXyz::Dual16(me) => FiringXyzD16Iter(me.firings.into_iter()).into(),
                FrameXyz::Dual32(me) => FiringXyzD32Iter(me.firings.into_iter()).into(),
            }
        }

        pub fn point_iter<'a>(
            &'a self,
        ) -> PointRefIter<
            impl Iterator<Item = &'a PointS>,
            impl Iterator<Item = &'a PointS>,
            impl Iterator<Item = &'a PointD>,
            impl Iterator<Item = &'a PointD>,
        > {
            match self {
                Self::Single16(frame) => PointRefIter::Single16(frame.point_iter()),
                Self::Single32(frame) => PointRefIter::Single32(frame.point_iter()),
                Self::Dual16(frame) => PointRefIter::Dual16(frame.point_iter()),
                Self::Dual32(frame) => PointRefIter::Dual32(frame.point_iter()),
            }
        }

        pub fn indexed_point_iter<'a>(
            &'a self,
        ) -> Box<dyn Iterator<Item = ((usize, usize), PointRef<'a>)> + 'a> {
            match self {
                Self::Single16(frame) => Box::new(
                    frame
                        .indexed_point_iter()
                        .map(|(index, point)| (index, PointRef::from(point))),
                ),
                Self::Single32(frame) => Box::new(
                    frame
                        .indexed_point_iter()
                        .map(|(index, point)| (index, PointRef::from(point))),
                ),
                Self::Dual16(frame) => Box::new(
                    frame
                        .indexed_point_iter()
                        .map(|(index, point)| (index, PointRef::from(point))),
                ),
                Self::Dual32(frame) => Box::new(
                    frame
                        .indexed_point_iter()
                        .map(|(index, point)| (index, PointRef::from(point))),
                ),
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

            impl $name {
                pub fn nrows(&self) -> usize {
                    $nrows
                }

                pub fn ncols(&self) -> usize {
                    self.firings.len()
                }

                pub fn point_at(&self, row: usize, col: usize) -> Option<&$point> {
                    self.firings.get(col)?.points.get(row)
                }

                pub fn point_iter(&self) -> impl Iterator<Item = &$point> {
                    self.firings.iter().flat_map(|firing| &firing.points)
                }

                pub fn indexed_point_iter(
                    &self,
                ) -> impl Iterator<Item = ((usize, usize), &$point)> {
                    self.firings.iter().enumerate().flat_map(|(col, firing)| {
                        firing
                            .points
                            .iter()
                            .enumerate()
                            .map(move |(row, point)| ((row, col), point))
                    })
                }

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
