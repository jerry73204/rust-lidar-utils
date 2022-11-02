use crate::frame_xyz::types::FrameXyz;
pub use crate::frame_xyz::types::{FrameXyzD16, FrameXyzD32, FrameXyzS16, FrameXyzS32};

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

declare_iter!(FrameXyzS16Iter, FrameXyzS16);
declare_iter!(FrameXyzS32Iter, FrameXyzS32);
declare_iter!(FrameXyzD16Iter, FrameXyzD16);
declare_iter!(FrameXyzD32Iter, FrameXyzD32);

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzS16>,
    B: Iterator<Item = FrameXyzS32>,
    C: Iterator<Item = FrameXyzD16>,
    D: Iterator<Item = FrameXyzD32>,
{
    Single16(FrameXyzS16Iter<A>),
    Single32(FrameXyzS32Iter<B>),
    Dual16(FrameXyzD16Iter<C>),
    Dual32(FrameXyzD32Iter<D>),
}

impl<A, B, C, D> From<FrameXyzD32Iter<D>> for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzS16>,
    B: Iterator<Item = FrameXyzS32>,
    C: Iterator<Item = FrameXyzD16>,
    D: Iterator<Item = FrameXyzD32>,
{
    fn from(v: FrameXyzD32Iter<D>) -> Self {
        Self::Dual32(v)
    }
}

impl<A, B, C, D> From<FrameXyzD16Iter<C>> for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzS16>,
    B: Iterator<Item = FrameXyzS32>,
    C: Iterator<Item = FrameXyzD16>,
    D: Iterator<Item = FrameXyzD32>,
{
    fn from(v: FrameXyzD16Iter<C>) -> Self {
        Self::Dual16(v)
    }
}

impl<A, B, C, D> From<FrameXyzS32Iter<B>> for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzS16>,
    B: Iterator<Item = FrameXyzS32>,
    C: Iterator<Item = FrameXyzD16>,
    D: Iterator<Item = FrameXyzD32>,
{
    fn from(v: FrameXyzS32Iter<B>) -> Self {
        Self::Single32(v)
    }
}

impl<A, B, C, D> From<FrameXyzS16Iter<A>> for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzS16>,
    B: Iterator<Item = FrameXyzS32>,
    C: Iterator<Item = FrameXyzD16>,
    D: Iterator<Item = FrameXyzD32>,
{
    fn from(v: FrameXyzS16Iter<A>) -> Self {
        Self::Single16(v)
    }
}

impl<A, B, C, D> Iterator for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzS16>,
    B: Iterator<Item = FrameXyzS32>,
    C: Iterator<Item = FrameXyzD16>,
    D: Iterator<Item = FrameXyzD32>,
{
    type Item = FrameXyz;

    fn next(&mut self) -> Option<Self::Item> {
        Some(match self {
            Self::Single16(iter) => iter.next()?.into(),
            Self::Single32(iter) => iter.next()?.into(),
            Self::Dual16(iter) => iter.next()?.into(),
            Self::Dual32(iter) => iter.next()?.into(),
        })
    }
}
