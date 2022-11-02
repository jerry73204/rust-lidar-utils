use super::types::{FiringDual16, FiringDual32, FiringKind, FiringSingle16, FiringSingle32};

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

declare_iter!(FiringSingle16Iter, FiringSingle16);
declare_iter!(FiringSingle32Iter, FiringSingle32);
declare_iter!(FiringDual16Iter, FiringDual16);
declare_iter!(FiringDual32Iter, FiringDual32);

pub use firing_iter_kind::*;
mod firing_iter_kind {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum FiringIterKind<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringSingle16<'a>>,
        B: Iterator<Item = FiringSingle32<'a>>,
        C: Iterator<Item = FiringDual16<'a>>,
        D: Iterator<Item = FiringDual32<'a>>,
    {
        Single16(FiringSingle16Iter<'a, A>),
        Single32(FiringSingle32Iter<'a, B>),
        Dual16(FiringDual16Iter<'a, C>),
        Dual32(FiringDual32Iter<'a, D>),
    }

    impl<'a, A, B, C, D> From<FiringDual32Iter<'a, D>> for FiringIterKind<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringSingle16<'a>>,
        B: Iterator<Item = FiringSingle32<'a>>,
        C: Iterator<Item = FiringDual16<'a>>,
        D: Iterator<Item = FiringDual32<'a>>,
    {
        fn from(v: FiringDual32Iter<'a, D>) -> Self {
            Self::Dual32(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringDual16Iter<'a, C>> for FiringIterKind<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringSingle16<'a>>,
        B: Iterator<Item = FiringSingle32<'a>>,
        C: Iterator<Item = FiringDual16<'a>>,
        D: Iterator<Item = FiringDual32<'a>>,
    {
        fn from(v: FiringDual16Iter<'a, C>) -> Self {
            Self::Dual16(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringSingle32Iter<'a, B>> for FiringIterKind<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringSingle16<'a>>,
        B: Iterator<Item = FiringSingle32<'a>>,
        C: Iterator<Item = FiringDual16<'a>>,
        D: Iterator<Item = FiringDual32<'a>>,
    {
        fn from(v: FiringSingle32Iter<'a, B>) -> Self {
            Self::Single32(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringSingle16Iter<'a, A>> for FiringIterKind<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringSingle16<'a>>,
        B: Iterator<Item = FiringSingle32<'a>>,
        C: Iterator<Item = FiringDual16<'a>>,
        D: Iterator<Item = FiringDual32<'a>>,
    {
        fn from(v: FiringSingle16Iter<'a, A>) -> Self {
            Self::Single16(v)
        }
    }

    impl<'a, A, B, C, D> Iterator for FiringIterKind<'a, A, B, C, D>
    where
        A: Iterator<Item = FiringSingle16<'a>>,
        B: Iterator<Item = FiringSingle32<'a>>,
        C: Iterator<Item = FiringDual16<'a>>,
        D: Iterator<Item = FiringDual32<'a>>,
    {
        type Item = FiringKind<'a>;

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
