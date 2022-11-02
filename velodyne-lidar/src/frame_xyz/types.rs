use crate::{
    common::*,
    firing_xyz::{
        iter::{
            FiringXyzDual16Iter, FiringXyzDual16RefIter, FiringXyzDual32Iter,
            FiringXyzDual32RefIter, FiringXyzKindIter, FiringXyzKindRefIter, FiringXyzSingle16Iter,
            FiringXyzSingle16RefIter, FiringXyzSingle32Iter, FiringXyzSingle32RefIter,
        },
        types::{FiringXyzDual16, FiringXyzDual32, FiringXyzSingle16, FiringXyzSingle32},
    },
    point::{
        iter::{PointIter, PointRefIter},
        types::{PointDual, PointKind, PointKindRef, PointSingle},
    },
};

pub use frame_kind::*;
mod frame_kind {
    use super::*;

    pub enum FrameXyzKind {
        Single16(FrameXyzSingle16),
        Single32(FrameXyzSingle32),
        Dual16(FrameXyzDual16),
        Dual32(FrameXyzDual32),
    }

    impl FrameXyzKind {
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

        pub fn point_at(&self, row: usize, col: usize) -> Option<PointKindRef<'_>> {
            Some(match self {
                Self::Single16(frame) => frame.point_at(row, col)?.into(),
                Self::Single32(frame) => frame.point_at(row, col)?.into(),
                Self::Dual16(frame) => frame.point_at(row, col)?.into(),
                Self::Dual32(frame) => frame.point_at(row, col)?.into(),
            })
        }

        pub fn firing_iter<'a>(
            &'a self,
        ) -> FiringXyzKindRefIter<
            impl Iterator<Item = &'a FiringXyzSingle16>,
            impl Iterator<Item = &'a FiringXyzSingle32>,
            impl Iterator<Item = &'a FiringXyzDual16>,
            impl Iterator<Item = &'a FiringXyzDual32>,
        > {
            match self {
                FrameXyzKind::Single16(me) => FiringXyzSingle16RefIter(me.firings.iter()).into(),
                FrameXyzKind::Single32(me) => FiringXyzSingle32RefIter(me.firings.iter()).into(),
                FrameXyzKind::Dual16(me) => FiringXyzDual16RefIter(me.firings.iter()).into(),
                FrameXyzKind::Dual32(me) => FiringXyzDual32RefIter(me.firings.iter()).into(),
            }
        }

        pub fn into_firing_iter(
            self,
        ) -> FiringXyzKindIter<
            impl Iterator<Item = FiringXyzSingle16>,
            impl Iterator<Item = FiringXyzSingle32>,
            impl Iterator<Item = FiringXyzDual16>,
            impl Iterator<Item = FiringXyzDual32>,
        > {
            match self {
                FrameXyzKind::Single16(me) => FiringXyzSingle16Iter(me.firings.into_iter()).into(),
                FrameXyzKind::Single32(me) => FiringXyzSingle32Iter(me.firings.into_iter()).into(),
                FrameXyzKind::Dual16(me) => FiringXyzDual16Iter(me.firings.into_iter()).into(),
                FrameXyzKind::Dual32(me) => FiringXyzDual32Iter(me.firings.into_iter()).into(),
            }
        }

        pub fn point_iter<'a>(
            &'a self,
        ) -> PointRefIter<
            impl Iterator<Item = &'a PointSingle>,
            impl Iterator<Item = &'a PointSingle>,
            impl Iterator<Item = &'a PointDual>,
            impl Iterator<Item = &'a PointDual>,
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
        ) -> Box<dyn Iterator<Item = ((usize, usize), PointKindRef<'a>)> + 'a> {
            match self {
                Self::Single16(frame) => Box::new(
                    frame
                        .indexed_point_iter()
                        .map(|(index, point)| (index, PointKindRef::from(point))),
                ),
                Self::Single32(frame) => Box::new(
                    frame
                        .indexed_point_iter()
                        .map(|(index, point)| (index, PointKindRef::from(point))),
                ),
                Self::Dual16(frame) => Box::new(
                    frame
                        .indexed_point_iter()
                        .map(|(index, point)| (index, PointKindRef::from(point))),
                ),
                Self::Dual32(frame) => Box::new(
                    frame
                        .indexed_point_iter()
                        .map(|(index, point)| (index, PointKindRef::from(point))),
                ),
            }
        }

        pub fn into_point_iter(
            self,
        ) -> PointIter<
            impl Iterator<Item = PointSingle>,
            impl Iterator<Item = PointSingle>,
            impl Iterator<Item = PointDual>,
            impl Iterator<Item = PointDual>,
        > {
            match self {
                Self::Single16(frame) => PointIter::Single16(frame.into_point_iter()),
                Self::Single32(frame) => PointIter::Single32(frame.into_point_iter()),
                Self::Dual16(frame) => PointIter::Dual16(frame.into_point_iter()),
                Self::Dual32(frame) => PointIter::Dual32(frame.into_point_iter()),
            }
        }

        pub fn into_indexed_point_iter(
            self,
        ) -> Box<dyn Iterator<Item = ((usize, usize), PointKind)>> {
            match self {
                Self::Single16(frame) => Box::new(
                    frame
                        .into_indexed_point_iter()
                        .map(|(index, point)| (index, PointKind::from(point))),
                ),
                Self::Single32(frame) => Box::new(
                    frame
                        .into_indexed_point_iter()
                        .map(|(index, point)| (index, PointKind::from(point))),
                ),
                Self::Dual16(frame) => Box::new(
                    frame
                        .into_indexed_point_iter()
                        .map(|(index, point)| (index, PointKind::from(point))),
                ),
                Self::Dual32(frame) => Box::new(
                    frame
                        .into_indexed_point_iter()
                        .map(|(index, point)| (index, PointKind::from(point))),
                ),
            }
        }
    }

    impl From<FrameXyzDual16> for FrameXyzKind {
        fn from(v: FrameXyzDual16) -> Self {
            Self::Dual16(v)
        }
    }

    impl From<FrameXyzDual32> for FrameXyzKind {
        fn from(v: FrameXyzDual32) -> Self {
            Self::Dual32(v)
        }
    }

    impl From<FrameXyzSingle32> for FrameXyzKind {
        fn from(v: FrameXyzSingle32) -> Self {
            Self::Single32(v)
        }
    }

    impl From<FrameXyzSingle16> for FrameXyzKind {
        fn from(v: FrameXyzSingle16) -> Self {
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

    declare_type!(FrameXyzSingle16, FiringXyzSingle16, 16, PointSingle);
    declare_type!(FrameXyzSingle32, FiringXyzSingle32, 32, PointSingle);
    declare_type!(FrameXyzDual16, FiringXyzDual16, 16, PointDual);
    declare_type!(FrameXyzDual32, FiringXyzDual32, 32, PointDual);
}
