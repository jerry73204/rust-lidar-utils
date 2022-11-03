pub use frame_kind::*;
mod frame_kind {
    use super::{FrameXyzD16, FrameXyzD32, FrameXyzS16, FrameXyzS32};
    use crate::{
        firing_xyz::types::{FiringXyzD16, FiringXyzD32, FiringXyzS16, FiringXyzS32},
        kinds::FormatKind,
        point::types::Point,
        traits::BoxIterator,
    };

    pub type FrameXyz = FormatKind<FrameXyzS16, FrameXyzS32, FrameXyzD16, FrameXyzD32>;

    impl FrameXyz {
        pub fn firing_iter<'a>(
            &'a self,
        ) -> FormatKind<
            impl Iterator<Item = &'a FiringXyzS16> + Clone + Sync + Send,
            impl Iterator<Item = &'a FiringXyzS32> + Clone + Sync + Send,
            impl Iterator<Item = &'a FiringXyzD16> + Clone + Sync + Send,
            impl Iterator<Item = &'a FiringXyzD32> + Clone + Sync + Send,
        > {
            match self {
                FrameXyz::Single16(me) => FormatKind::from_s16(me.firings.iter()),
                FrameXyz::Single32(me) => FormatKind::from_s32(me.firings.iter()),
                FrameXyz::Dual16(me) => FormatKind::from_d16(me.firings.iter()),
                FrameXyz::Dual32(me) => FormatKind::from_d32(me.firings.iter()),
            }
        }

        pub fn into_firing_iter(
            self,
        ) -> FormatKind<
            impl Iterator<Item = FiringXyzS16> + Clone + Sync + Send,
            impl Iterator<Item = FiringXyzS32> + Clone + Sync + Send,
            impl Iterator<Item = FiringXyzD16> + Clone + Sync + Send,
            impl Iterator<Item = FiringXyzD32> + Clone + Sync + Send + Clone + Sync + Send,
        > {
            match self {
                FrameXyz::Single16(me) => FormatKind::from_s16(me.firings.into_iter()),
                FrameXyz::Single32(me) => FormatKind::from_s32(me.firings.into_iter()),
                FrameXyz::Dual16(me) => FormatKind::from_d16(me.firings.into_iter()),
                FrameXyz::Dual32(me) => FormatKind::from_d32(me.firings.into_iter()),
            }
        }

        pub fn into_point_iter(self) -> BoxIterator<'static, Point> {
            match self {
                Self::Single16(frame) => Box::new(frame.into_point_iter().map(Point::from)),
                Self::Single32(frame) => Box::new(frame.into_point_iter().map(Point::from)),
                Self::Dual16(frame) => Box::new(frame.into_point_iter().map(Point::from)),
                Self::Dual32(frame) => Box::new(frame.into_point_iter().map(Point::from)),
            }
        }

        pub fn into_indexed_point_iter(self) -> BoxIterator<'static, ((usize, usize), Point)> {
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
    use crate::{
        common::*,
        firing_xyz::types::{FiringXyzD16, FiringXyzD32, FiringXyzS16, FiringXyzS32},
        point::types::{PointD, PointS},
        traits::PointField,
    };

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
                pub fn into_point_iter(self) -> impl Iterator<Item = $point> + Clone + Sync + Send {
                    self.firings.into_iter().flat_map(|firing| firing.points)
                }

                pub fn into_indexed_point_iter(
                    self,
                ) -> impl Iterator<Item = ((usize, usize), $point)> + Clone + Sync + Send {
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
