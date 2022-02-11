use crate::{
    common::*,
    firing_xyz::{FiringXyzDual16, FiringXyzDual32, FiringXyzSingle16, FiringXyzSingle32},
    firing_xyz_iter::{
        FiringXyzDual16Iter, FiringXyzDual32Iter, FiringXyzIter, FiringXyzSingle16Iter,
        FiringXyzSingle32Iter,
    },
    point::{PointDual, PointKind, PointKindRef, PointSingle},
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

        pub fn into_firing_iter(
            self,
        ) -> FiringXyzIter<
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

pub use point_iter::*;
pub mod point_iter {
    use super::*;

    pub enum PointIter<A, B, C, D>
    where
        A: Iterator<Item = PointSingle>,
        B: Iterator<Item = PointSingle>,
        C: Iterator<Item = PointDual>,
        D: Iterator<Item = PointDual>,
    {
        Single16(A),
        Single32(B),
        Dual16(C),
        Dual32(D),
    }

    impl<A, B, C, D> Iterator for PointIter<A, B, C, D>
    where
        A: Iterator<Item = PointSingle>,
        B: Iterator<Item = PointSingle>,
        C: Iterator<Item = PointDual>,
        D: Iterator<Item = PointDual>,
    {
        type Item = PointKind;

        fn next(&mut self) -> Option<Self::Item> {
            Some(match self {
                Self::Single16(iter) => iter.next()?.into(),
                Self::Single32(iter) => iter.next()?.into(),
                Self::Dual16(iter) => iter.next()?.into(),
                Self::Dual32(iter) => iter.next()?.into(),
            })
        }
    }
}

pub use point_ref_iter::*;
pub mod point_ref_iter {
    use super::*;

    pub enum PointRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a PointSingle>,
        B: Iterator<Item = &'a PointSingle>,
        C: Iterator<Item = &'a PointDual>,
        D: Iterator<Item = &'a PointDual>,
    {
        Single16(A),
        Single32(B),
        Dual16(C),
        Dual32(D),
    }

    impl<'a, A, B, C, D> Iterator for PointRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a PointSingle>,
        B: Iterator<Item = &'a PointSingle>,
        C: Iterator<Item = &'a PointDual>,
        D: Iterator<Item = &'a PointDual>,
    {
        type Item = PointKindRef<'a>;

        fn next(&mut self) -> Option<Self::Item> {
            Some(match self {
                Self::Single16(iter) => iter.next()?.into(),
                Self::Single32(iter) => iter.next()?.into(),
                Self::Dual16(iter) => iter.next()?.into(),
                Self::Dual32(iter) => iter.next()?.into(),
            })
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
