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

declare_iter!(FiringBlockIterS16, FiringBlockS16);
declare_iter!(FiringBlockIterS32, FiringBlockS32);
declare_iter!(FiringBlockIterD16, FiringBlockD16);
declare_iter!(FiringBlockIterD32, FiringBlockD32);

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
        Single16(FiringBlockIterS16<'a, A>),
        Single32(FiringBlockIterS32<'a, B>),
        Dual16(FiringBlockIterD16<'a, C>),
        Dual32(FiringBlockIterD32<'a, D>),
    }

    impl<'a, A, B, C, D> From<FiringBlockIterD32<'a, D>> for FiringBlockIter<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringBlockS16<'a>>,
        B: Iterator<Item = FiringBlockS32<'a>>,
        C: Iterator<Item = FiringBlockD16<'a>>,
        D: Iterator<Item = FiringBlockD32<'a>>,
    {
        fn from(v: FiringBlockIterD32<'a, D>) -> Self {
            Self::Dual32(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringBlockIterD16<'a, C>> for FiringBlockIter<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringBlockS16<'a>>,
        B: Iterator<Item = FiringBlockS32<'a>>,
        C: Iterator<Item = FiringBlockD16<'a>>,
        D: Iterator<Item = FiringBlockD32<'a>>,
    {
        fn from(v: FiringBlockIterD16<'a, C>) -> Self {
            Self::Dual16(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringBlockIterS32<'a, B>> for FiringBlockIter<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringBlockS16<'a>>,
        B: Iterator<Item = FiringBlockS32<'a>>,
        C: Iterator<Item = FiringBlockD16<'a>>,
        D: Iterator<Item = FiringBlockD32<'a>>,
    {
        fn from(v: FiringBlockIterS32<'a, B>) -> Self {
            Self::Single32(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringBlockIterS16<'a, A>> for FiringBlockIter<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringBlockS16<'a>>,
        B: Iterator<Item = FiringBlockS32<'a>>,
        C: Iterator<Item = FiringBlockD16<'a>>,
        D: Iterator<Item = FiringBlockD32<'a>>,
    {
        fn from(v: FiringBlockIterS16<'a, A>) -> Self {
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
