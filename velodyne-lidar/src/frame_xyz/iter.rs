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

declare_iter!(FrameXyzIterS16, FrameXyzS16);
declare_iter!(FrameXyzIterS32, FrameXyzS32);
declare_iter!(FrameXyzIterD16, FrameXyzD16);
declare_iter!(FrameXyzIterD32, FrameXyzD32);

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzS16>,
    B: Iterator<Item = FrameXyzS32>,
    C: Iterator<Item = FrameXyzD16>,
    D: Iterator<Item = FrameXyzD32>,
{
    Single16(FrameXyzIterS16<A>),
    Single32(FrameXyzIterS32<B>),
    Dual16(FrameXyzIterD16<C>),
    Dual32(FrameXyzIterD32<D>),
}

impl<A, B, C, D> From<FrameXyzIterD32<D>> for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzS16>,
    B: Iterator<Item = FrameXyzS32>,
    C: Iterator<Item = FrameXyzD16>,
    D: Iterator<Item = FrameXyzD32>,
{
    fn from(v: FrameXyzIterD32<D>) -> Self {
        Self::Dual32(v)
    }
}

impl<A, B, C, D> From<FrameXyzIterD16<C>> for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzS16>,
    B: Iterator<Item = FrameXyzS32>,
    C: Iterator<Item = FrameXyzD16>,
    D: Iterator<Item = FrameXyzD32>,
{
    fn from(v: FrameXyzIterD16<C>) -> Self {
        Self::Dual16(v)
    }
}

impl<A, B, C, D> From<FrameXyzIterS32<B>> for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzS16>,
    B: Iterator<Item = FrameXyzS32>,
    C: Iterator<Item = FrameXyzD16>,
    D: Iterator<Item = FrameXyzD32>,
{
    fn from(v: FrameXyzIterS32<B>) -> Self {
        Self::Single32(v)
    }
}

impl<A, B, C, D> From<FrameXyzIterS16<A>> for FrameXyzIter<A, B, C, D>
where
    A: Iterator<Item = FrameXyzS16>,
    B: Iterator<Item = FrameXyzS32>,
    C: Iterator<Item = FrameXyzD16>,
    D: Iterator<Item = FrameXyzD32>,
{
    fn from(v: FrameXyzIterS16<A>) -> Self {
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
