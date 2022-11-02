use super::types::{FiringBlock, FiringBlockD16, FiringBlockD32, FiringBlockS16, FiringBlockS32};

macro_rules! declare_iter {
    ($name:ident, $item:ident) => {
        #[derive(Debug, Clone, PartialEq, Eq)]
        pub struct $name<'a, I>(pub(crate) I)
        where
            I: Iterator<Item = $item<'a>>;

        impl<'a, I> Iterator for $name<'a, I>
        where
            I: Iterator<Item = $item<'a>>,
        {
            type Item = $item<'a>;

            fn next(&mut self) -> Option<Self::Item> {
                self.0.next()
            }

            fn size_hint(&self) -> (usize, Option<usize>) {
                self.0.size_hint()
            }
        }
    };
}

declare_iter!(FiringBlockS16Iter, FiringBlockS16);
declare_iter!(FiringBlockS32Iter, FiringBlockS32);
declare_iter!(FiringBlockD16Iter, FiringBlockD16);
declare_iter!(FiringBlockD32Iter, FiringBlockD32);

pub use firing_iter_kind::*;
mod firing_iter_kind {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum FiringBlockIter<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringBlockS16<'a>>,
        B: Iterator<Item = FiringBlockS32<'a>>,
        C: Iterator<Item = FiringBlockD16<'a>>,
        D: Iterator<Item = FiringBlockD32<'a>>,
    {
        Single16(FiringBlockS16Iter<'a, A>),
        Single32(FiringBlockS32Iter<'a, B>),
        Dual16(FiringBlockD16Iter<'a, C>),
        Dual32(FiringBlockD32Iter<'a, D>),
    }

    impl<'a, A, B, C, D> From<FiringBlockD32Iter<'a, D>> for FiringBlockIter<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringBlockS16<'a>>,
        B: Iterator<Item = FiringBlockS32<'a>>,
        C: Iterator<Item = FiringBlockD16<'a>>,
        D: Iterator<Item = FiringBlockD32<'a>>,
    {
        fn from(v: FiringBlockD32Iter<'a, D>) -> Self {
            Self::Dual32(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringBlockD16Iter<'a, C>> for FiringBlockIter<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringBlockS16<'a>>,
        B: Iterator<Item = FiringBlockS32<'a>>,
        C: Iterator<Item = FiringBlockD16<'a>>,
        D: Iterator<Item = FiringBlockD32<'a>>,
    {
        fn from(v: FiringBlockD16Iter<'a, C>) -> Self {
            Self::Dual16(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringBlockS32Iter<'a, B>> for FiringBlockIter<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringBlockS16<'a>>,
        B: Iterator<Item = FiringBlockS32<'a>>,
        C: Iterator<Item = FiringBlockD16<'a>>,
        D: Iterator<Item = FiringBlockD32<'a>>,
    {
        fn from(v: FiringBlockS32Iter<'a, B>) -> Self {
            Self::Single32(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringBlockS16Iter<'a, A>> for FiringBlockIter<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringBlockS16<'a>>,
        B: Iterator<Item = FiringBlockS32<'a>>,
        C: Iterator<Item = FiringBlockD16<'a>>,
        D: Iterator<Item = FiringBlockD32<'a>>,
    {
        fn from(v: FiringBlockS16Iter<'a, A>) -> Self {
            Self::Single16(v)
        }
    }

    impl<'a, A, B, C, D> Iterator for FiringBlockIter<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringBlockS16<'a>>,
        B: Iterator<Item = FiringBlockS32<'a>>,
        C: Iterator<Item = FiringBlockD16<'a>>,
        D: Iterator<Item = FiringBlockD32<'a>>,
    {
        type Item = FiringBlock<'a>;

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
