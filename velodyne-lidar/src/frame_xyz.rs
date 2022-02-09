use super::{
    firing_xyz::{FiringXyzDual16, FiringXyzDual32, FiringXyzSingle16, FiringXyzSingle32},
    point::{PointDual, PointSingle},
};
use crate::common::*;

pub use frame_kind::*;
mod frame_kind {
    use super::*;

    pub enum FrameXyzKind {
        Single16(FrameXyzSingle16),
        Single32(FrameXyzSingle32),
        Dual16(FrameXyzDual16),
        Dual32(FrameXyzDual32),
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
            }
        };
    }

    declare_type!(FrameXyzSingle16, FiringXyzSingle16, 16, PointSingle);
    declare_type!(FrameXyzSingle32, FiringXyzSingle32, 32, PointSingle);
    declare_type!(FrameXyzDual16, FiringXyzDual16, 16, PointDual);
    declare_type!(FrameXyzDual32, FiringXyzDual32, 32, PointDual);
}

// pub fn firings_to_frames_single_16<I>(firings: I) -> impl Iterator<Item = FrameXyzSingle16>
// where
//     I: IntoIterator<Item = FiringXyzSingle16>,
// {
//     let buffer: Vec<FiringXyzSingle16> = vec![];

//     itertools::unfold((firings.into_iter(), buffer), |(firings, buffer)| loop {
//         if let Some(curr) = firings.next() {
//             let wrap =
//                 matches!(buffer.last(), Some(prev) if prev.azimuth_count > curr.azimuth_count);

//             if wrap {
//                 let output = mem::replace(buffer, vec![curr]);
//                 break Some(output);
//             } else {
//                 buffer.push(curr);
//             }
//         } else {
//             break (!buffer.is_empty()).then(|| mem::take(buffer));
//         }
//     })
//     .map(|firings| FrameXyzSingle16 { firings })
// }

// pub fn firings_to_frames_single_32<I>(firings: I) -> impl Iterator<Item = FrameXyzSingle32>
// where
//     I: IntoIterator<Item = FiringXyzSingle32>,
// {
//     let buffer: Vec<FiringXyzSingle32> = vec![];

//     itertools::unfold((firings.into_iter(), buffer), |(firings, buffer)| loop {
//         if let Some(curr) = firings.next() {
//             let wrap =
//                 matches!(buffer.last(), Some(prev) if prev.azimuth_count > curr.azimuth_count);

//             if wrap {
//                 let output = mem::replace(buffer, vec![curr]);
//                 break Some(output);
//             } else {
//                 buffer.push(curr);
//             }
//         } else {
//             break (!buffer.is_empty()).then(|| mem::take(buffer));
//         }
//     })
//     .map(|firings| FrameXyzSingle32 { firings })
// }

// pub fn firings_to_frames_dual_16<I>(firings: I) -> impl Iterator<Item = FrameXyzDual16>
// where
//     I: IntoIterator<Item = FiringXyzDual16>,
// {
//     let buffer: Vec<FiringXyzDual16> = vec![];

//     itertools::unfold((firings.into_iter(), buffer), |(firings, buffer)| loop {
//         if let Some(curr) = firings.next() {
//             let wrap =
//                 matches!(buffer.last(), Some(prev) if prev.azimuth_count > curr.azimuth_count);

//             if wrap {
//                 let output = mem::replace(buffer, vec![curr]);
//                 break Some(output);
//             } else {
//                 buffer.push(curr);
//             }
//         } else {
//             break (!buffer.is_empty()).then(|| mem::take(buffer));
//         }
//     })
//     .map(|firings| FrameXyzDual16 { firings })
// }

// pub fn firings_to_frames_dual_32<I>(firings: I) -> impl Iterator<Item = FrameXyzDual32>
// where
//     I: IntoIterator<Item = FiringXyzDual32>,
// {
//     let buffer: Vec<FiringXyzDual32> = vec![];

//     itertools::unfold((firings.into_iter(), buffer), |(firings, buffer)| loop {
//         if let Some(curr) = firings.next() {
//             let wrap =
//                 matches!(buffer.last(), Some(prev) if prev.azimuth_count > curr.azimuth_count);

//             if wrap {
//                 let output = mem::replace(buffer, vec![curr]);
//                 break Some(output);
//             } else {
//                 buffer.push(curr);
//             }
//         } else {
//             break (!buffer.is_empty()).then(|| mem::take(buffer));
//         }
//     })
//     .map(|firings| FrameXyzDual32 { firings })
// }
