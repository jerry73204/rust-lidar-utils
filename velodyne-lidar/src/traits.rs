pub type BoxIterator<'a, T> = Box<dyn Iterator<Item = T> + Sync + Send + 'a>;

pub use azimuth::*;
mod azimuth {
    use crate::{
        firing_block::{FiringBlockD16, FiringBlockD32, FiringBlockS16, FiringBlockS32},
        firing_xyz::{FiringXyzD16, FiringXyzD32, FiringXyzS16, FiringXyzS32},
        frame_xyz::{FrameXyzD16, FrameXyzD32, FrameXyzS16, FrameXyzS32},
    };
    use measurements::Angle;
    use std::ops::Range;

    pub trait AzimuthRange {
        fn azimuth_range(&self) -> Range<Angle>;

        fn start_azimuth(&self) -> Angle {
            self.azimuth_range().start
        }

        fn end_azimuth(&self) -> Angle {
            self.azimuth_range().end
        }
    }

    impl<'a> AzimuthRange for FiringBlockS16<'a> {
        fn azimuth_range(&self) -> Range<Angle> {
            self.azimuth_range.clone()
        }
    }

    impl<'a> AzimuthRange for FiringBlockS32<'a> {
        fn azimuth_range(&self) -> Range<Angle> {
            self.azimuth_range.clone()
        }
    }

    impl<'a> AzimuthRange for FiringBlockD16<'a> {
        fn azimuth_range(&self) -> Range<Angle> {
            self.azimuth_range.clone()
        }
    }

    impl<'a> AzimuthRange for FiringBlockD32<'a> {
        fn azimuth_range(&self) -> Range<Angle> {
            self.azimuth_range.clone()
        }
    }

    impl AzimuthRange for FiringXyzS16 {
        fn azimuth_range(&self) -> Range<Angle> {
            self.azimuth_range.clone()
        }
    }

    impl AzimuthRange for FiringXyzS32 {
        fn azimuth_range(&self) -> Range<Angle> {
            self.azimuth_range.clone()
        }
    }

    impl AzimuthRange for FiringXyzD16 {
        fn azimuth_range(&self) -> Range<Angle> {
            self.azimuth_range.clone()
        }
    }

    impl AzimuthRange for FiringXyzD32 {
        fn azimuth_range(&self) -> Range<Angle> {
            self.azimuth_range.clone()
        }
    }

    impl AzimuthRange for FrameXyzS16 {
        fn azimuth_range(&self) -> Range<Angle> {
            let start = self.firings[0].azimuth_range().start;
            let end = self.firings.last().unwrap().azimuth_range().end;
            start..end
        }
    }

    impl AzimuthRange for FrameXyzS32 {
        fn azimuth_range(&self) -> Range<Angle> {
            let start = self.firings[0].azimuth_range().start;
            let end = self.firings.last().unwrap().azimuth_range().end;
            start..end
        }
    }

    impl AzimuthRange for FrameXyzD16 {
        fn azimuth_range(&self) -> Range<Angle> {
            let start = self.firings[0].azimuth_range().start;
            let end = self.firings.last().unwrap().azimuth_range().end;
            start..end
        }
    }

    impl AzimuthRange for FrameXyzD32 {
        fn azimuth_range(&self) -> Range<Angle> {
            let start = self.firings[0].azimuth_range().start;
            let end = self.firings.last().unwrap().azimuth_range().end;
            start..end
        }
    }
}

pub use point_field::*;
mod point_field {
    use itertools::iproduct;

    use super::BoxIterator;

    pub trait PointField {
        type Point<'a>
        where
            Self: 'a;

        fn nrows(&self) -> usize;

        fn ncols(&self) -> usize;

        fn point_at<'a>(&'a self, row: usize, col: usize) -> Option<Self::Point<'a>>;

        fn indexed_point_iter<'a>(&'a self) -> BoxIterator<'a, ((usize, usize), Self::Point<'a>)>
        where
            Self: Sync,
        {
            Box::new(
                iproduct!(0..self.nrows(), 0..self.ncols())
                    .map(|(row, col)| ((row, col), self.point_at(row, col).unwrap())),
            )
        }

        fn point_iter<'a>(&'a self) -> BoxIterator<'a, Self::Point<'a>>
        where
            Self: Sync,
        {
            Box::new(self.indexed_point_iter().map(|(_index, point)| point))
        }
    }
}
