pub use crate::velodyne::frame_xyz::{
    FrameXyzDual16, FrameXyzDual32, FrameXyzSingle16, FrameXyzSingle32,
};

macro_rules! declare_iter {
    ($name:ident, $item:ident) => {
        #[derive(Debug, Clone, PartialEq, Eq)]
        pub struct $name<I>(pub(crate) I)
        where
            I: Iterator<Item = $item>;

        impl<I> Iterator for $name<I>
        where
            I: Iterator<Item = $item>,
        {
            type Item = $item;

            fn next(&mut self) -> Option<Self::Item> {
                self.0.next()
            }

            fn size_hint(&self) -> (usize, Option<usize>) {
                self.0.size_hint()
            }
        }
    };
}

declare_iter!(FrameXyzSingle16Iter, FrameXyzSingle16);
declare_iter!(FrameXyzSingle32Iter, FrameXyzSingle32);
declare_iter!(FrameXyzDual16Iter, FrameXyzDual16);
declare_iter!(FrameXyzDual32Iter, FrameXyzDual32);
