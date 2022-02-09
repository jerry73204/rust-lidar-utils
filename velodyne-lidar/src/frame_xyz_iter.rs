use crate::frame_xyz::FrameXyzKind;
pub use crate::frame_xyz::{FrameXyzDual16, FrameXyzDual32, FrameXyzSingle16, FrameXyzSingle32};

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

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzSingle16>,
    B: Iterator<Item = FrameXyzSingle32>,
    C: Iterator<Item = FrameXyzDual16>,
    D: Iterator<Item = FrameXyzDual32>,
{
    Single16(FrameXyzSingle16Iter<A>),
    Single32(FrameXyzSingle32Iter<B>),
    Dual16(FrameXyzDual16Iter<C>),
    Dual32(FrameXyzDual32Iter<D>),
}

impl<A, B, C, D> From<FrameXyzDual32Iter<D>> for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzSingle16>,
    B: Iterator<Item = FrameXyzSingle32>,
    C: Iterator<Item = FrameXyzDual16>,
    D: Iterator<Item = FrameXyzDual32>,
{
    fn from(v: FrameXyzDual32Iter<D>) -> Self {
        Self::Dual32(v)
    }
}

impl<A, B, C, D> From<FrameXyzDual16Iter<C>> for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzSingle16>,
    B: Iterator<Item = FrameXyzSingle32>,
    C: Iterator<Item = FrameXyzDual16>,
    D: Iterator<Item = FrameXyzDual32>,
{
    fn from(v: FrameXyzDual16Iter<C>) -> Self {
        Self::Dual16(v)
    }
}

impl<A, B, C, D> From<FrameXyzSingle32Iter<B>> for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzSingle16>,
    B: Iterator<Item = FrameXyzSingle32>,
    C: Iterator<Item = FrameXyzDual16>,
    D: Iterator<Item = FrameXyzDual32>,
{
    fn from(v: FrameXyzSingle32Iter<B>) -> Self {
        Self::Single32(v)
    }
}

impl<A, B, C, D> From<FrameXyzSingle16Iter<A>> for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzSingle16>,
    B: Iterator<Item = FrameXyzSingle32>,
    C: Iterator<Item = FrameXyzDual16>,
    D: Iterator<Item = FrameXyzDual32>,
{
    fn from(v: FrameXyzSingle16Iter<A>) -> Self {
        Self::Single16(v)
    }
}

impl<A, B, C, D> Iterator for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzSingle16>,
    B: Iterator<Item = FrameXyzSingle32>,
    C: Iterator<Item = FrameXyzDual16>,
    D: Iterator<Item = FrameXyzDual32>,
{
    type Item = FrameXyzKind;

    fn next(&mut self) -> Option<Self::Item> {
        Some(match self {
            Self::Single16(iter) => iter.next()?.into(),
            Self::Single32(iter) => iter.next()?.into(),
            Self::Dual16(iter) => iter.next()?.into(),
            Self::Dual32(iter) => iter.next()?.into(),
        })
    }
}
