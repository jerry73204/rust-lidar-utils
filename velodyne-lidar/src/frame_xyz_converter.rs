use super::{
    firing_xyz::{
        FiringXyz, FiringXyzDual16, FiringXyzDual32, FiringXyzSingle16, FiringXyzSingle32,
    },
    frame_xyz::{FrameXyzDual16, FrameXyzDual32, FrameXyzSingle16, FrameXyzSingle32},
};
use crate::common::*;

macro_rules! declare_converter {
    ($name:ident, $firing:ident, $frame:ident) => {
        pub struct $name {
            buffer: Vec<$firing>,
        }

        impl $name {
            pub fn new() -> Self {
                Self { buffer: vec![] }
            }

            pub fn push_one(&mut self, firing: $firing) -> Option<$frame> {
                let firings = push_one(&mut self.buffer, firing)?;
                Some($frame { firings })
            }

            // pub fn push_many<'a, I>(&'a mut self, firings: I) -> impl Iterator<Item = $frame> + 'a
            // where
            //     I: IntoIterator<Item = $firing> + 'a,
            // {
            //     push_many(&mut self.buffer, firings).map(|firings| $frame { firings })
            // }

            pub fn take(&mut self) -> Option<$frame> {
                let firings = mem::take(&mut self.buffer);
                (!firings.is_empty()).then(|| $frame { firings })
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self::new()
            }
        }
    };
}

declare_converter!(
    FrameXyzConverterSingle16,
    FiringXyzSingle16,
    FrameXyzSingle16
);
declare_converter!(
    FrameXyzConverterSingle32,
    FiringXyzSingle32,
    FrameXyzSingle32
);
declare_converter!(FrameXyzConverterDual16, FiringXyzDual16, FrameXyzDual16);
declare_converter!(FrameXyzConverterDual32, FiringXyzDual32, FrameXyzDual32);

fn push_one<F: FiringXyz>(buffer: &mut Vec<F>, curr: F) -> Option<Vec<F>> {
    let wrap = matches!(buffer.last(), Some(prev) if prev.azimuth_count() > curr.azimuth_count());

    if wrap {
        let output = mem::replace(buffer, vec![curr]);
        Some(output)
    } else {
        buffer.push(curr);
        None
    }
}

// fn push_many<'a, F: FiringXyz, I>(
//     buffer: &'a mut Vec<F>,
//     iter: I,
// ) -> impl Iterator<Item = Vec<F>> + 'a
// where
//     I: IntoIterator<Item = F>,
//     I::IntoIter: 'a,
// {
//     iter.into_iter()
//         .scan(buffer, |buffer, firing| Some(push_one(buffer, firing)))
//         .flatten()
// }
