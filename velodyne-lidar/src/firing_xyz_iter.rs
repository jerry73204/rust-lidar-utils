use crate::{
    common::*,
    firing_xyz::{
        FiringXyzDual16, FiringXyzDual32, FiringXyzKind, FiringXyzRefKind, FiringXyzSingle16,
        FiringXyzSingle32,
    },
    frame_xyz::{FrameXyzDual16, FrameXyzDual32, FrameXyzSingle16, FrameXyzSingle32},
    frame_xyz_batcher::{
        FrameXyzBatcherDual16, FrameXyzBatcherDual32, FrameXyzBatcherSingle16,
        FrameXyzBatcherSingle32,
    },
    frame_xyz_iter::{
        FrameXyzDual16Iter, FrameXyzDual32Iter, FrameXyzSingle16Iter, FrameXyzSingle32Iter,
    },
};

macro_rules! declare_iter {
    ($name:ident, $item:ident, $frame_conv:ident, $frame:ident, $frame_iter:ident $(,)?) => {
        #[derive(Debug, Clone, PartialEq, Eq)]
        pub struct $name<I>(pub(crate) I)
        where
            I: Iterator<Item = $item>;

        impl<I> $name<I>
        where
            I: Iterator<Item = $item>,
        {
            pub fn into_frame_iter(self) -> $frame_iter<impl Iterator<Item = $frame>> {
                let conv = $frame_conv::new();

                let iter = itertools::unfold(Some((self, conv)), |state| {
                    if let Some((iter, conv)) = state {
                        Some(if let Some(firing) = iter.next() {
                            conv.push_one(firing)
                        } else {
                            let output = conv.take();
                            *state = None;
                            output
                        })
                    } else {
                        None
                    }
                })
                .flatten();

                $frame_iter(iter)
            }
        }

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

declare_iter!(
    FiringXyzSingle16Iter,
    FiringXyzSingle16,
    FrameXyzBatcherSingle16,
    FrameXyzSingle16,
    FrameXyzSingle16Iter
);
declare_iter!(
    FiringXyzSingle32Iter,
    FiringXyzSingle32,
    FrameXyzBatcherSingle32,
    FrameXyzSingle32,
    FrameXyzSingle32Iter
);
declare_iter!(
    FiringXyzDual16Iter,
    FiringXyzDual16,
    FrameXyzBatcherDual16,
    FrameXyzDual16,
    FrameXyzDual16Iter
);
declare_iter!(
    FiringXyzDual32Iter,
    FiringXyzDual32,
    FrameXyzBatcherDual32,
    FrameXyzDual32,
    FrameXyzDual32Iter
);

macro_rules! declare_ref_iter {
    ($name:ident, $item:ident $(,)?) => {
        #[derive(Debug, Clone, PartialEq, Eq)]
        pub struct $name<'a, I>(pub(crate) I)
        where
            I: Iterator<Item = &'a $item>;

        impl<'a, I> Iterator for $name<'a, I>
        where
            I: Iterator<Item = &'a $item>,
        {
            type Item = &'a $item;

            fn next(&mut self) -> Option<Self::Item> {
                self.0.next()
            }

            fn size_hint(&self) -> (usize, Option<usize>) {
                self.0.size_hint()
            }
        }
    };
}

declare_ref_iter!(FiringXyzSingle16RefIter, FiringXyzSingle16,);
declare_ref_iter!(FiringXyzSingle32RefIter, FiringXyzSingle32,);
declare_ref_iter!(FiringXyzDual16RefIter, FiringXyzDual16,);
declare_ref_iter!(FiringXyzDual32RefIter, FiringXyzDual32,);

pub use firing_xyz_kind_iter::*;
mod firing_xyz_kind_iter {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum FiringXyzKindIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzSingle16>,
        B: Iterator<Item = FiringXyzSingle32>,
        C: Iterator<Item = FiringXyzDual16>,
        D: Iterator<Item = FiringXyzDual32>,
    {
        Single16(FiringXyzSingle16Iter<A>),
        Single32(FiringXyzSingle32Iter<B>),
        Dual16(FiringXyzDual16Iter<C>),
        Dual32(FiringXyzDual32Iter<D>),
    }

    impl<A, B, C, D> From<FiringXyzDual32Iter<D>> for FiringXyzKindIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzSingle16>,
        B: Iterator<Item = FiringXyzSingle32>,
        C: Iterator<Item = FiringXyzDual16>,
        D: Iterator<Item = FiringXyzDual32>,
    {
        fn from(v: FiringXyzDual32Iter<D>) -> Self {
            Self::Dual32(v)
        }
    }

    impl<A, B, C, D> From<FiringXyzDual16Iter<C>> for FiringXyzKindIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzSingle16>,
        B: Iterator<Item = FiringXyzSingle32>,
        C: Iterator<Item = FiringXyzDual16>,
        D: Iterator<Item = FiringXyzDual32>,
    {
        fn from(v: FiringXyzDual16Iter<C>) -> Self {
            Self::Dual16(v)
        }
    }

    impl<A, B, C, D> From<FiringXyzSingle32Iter<B>> for FiringXyzKindIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzSingle16>,
        B: Iterator<Item = FiringXyzSingle32>,
        C: Iterator<Item = FiringXyzDual16>,
        D: Iterator<Item = FiringXyzDual32>,
    {
        fn from(v: FiringXyzSingle32Iter<B>) -> Self {
            Self::Single32(v)
        }
    }

    impl<A, B, C, D> From<FiringXyzSingle16Iter<A>> for FiringXyzKindIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzSingle16>,
        B: Iterator<Item = FiringXyzSingle32>,
        C: Iterator<Item = FiringXyzDual16>,
        D: Iterator<Item = FiringXyzDual32>,
    {
        fn from(v: FiringXyzSingle16Iter<A>) -> Self {
            Self::Single16(v)
        }
    }

    impl<A, B, C, D> Iterator for FiringXyzKindIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzSingle16>,
        B: Iterator<Item = FiringXyzSingle32>,
        C: Iterator<Item = FiringXyzDual16>,
        D: Iterator<Item = FiringXyzDual32>,
    {
        type Item = FiringXyzKind;

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

pub use firing_xyz_kind_ref_iter::*;
mod firing_xyz_kind_ref_iter {

    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum FiringXyzKindRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzSingle16>,
        B: Iterator<Item = &'a FiringXyzSingle32>,
        C: Iterator<Item = &'a FiringXyzDual16>,
        D: Iterator<Item = &'a FiringXyzDual32>,
    {
        Single16(FiringXyzSingle16RefIter<'a, A>),
        Single32(FiringXyzSingle32RefIter<'a, B>),
        Dual16(FiringXyzDual16RefIter<'a, C>),
        Dual32(FiringXyzDual32RefIter<'a, D>),
    }

    impl<'a, A, B, C, D> From<FiringXyzDual32RefIter<'a, D>> for FiringXyzKindRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzSingle16>,
        B: Iterator<Item = &'a FiringXyzSingle32>,
        C: Iterator<Item = &'a FiringXyzDual16>,
        D: Iterator<Item = &'a FiringXyzDual32>,
    {
        fn from(v: FiringXyzDual32RefIter<'a, D>) -> Self {
            Self::Dual32(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringXyzDual16RefIter<'a, C>> for FiringXyzKindRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzSingle16>,
        B: Iterator<Item = &'a FiringXyzSingle32>,
        C: Iterator<Item = &'a FiringXyzDual16>,
        D: Iterator<Item = &'a FiringXyzDual32>,
    {
        fn from(v: FiringXyzDual16RefIter<'a, C>) -> Self {
            Self::Dual16(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringXyzSingle32RefIter<'a, B>> for FiringXyzKindRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzSingle16>,
        B: Iterator<Item = &'a FiringXyzSingle32>,
        C: Iterator<Item = &'a FiringXyzDual16>,
        D: Iterator<Item = &'a FiringXyzDual32>,
    {
        fn from(v: FiringXyzSingle32RefIter<'a, B>) -> Self {
            Self::Single32(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringXyzSingle16RefIter<'a, A>> for FiringXyzKindRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzSingle16>,
        B: Iterator<Item = &'a FiringXyzSingle32>,
        C: Iterator<Item = &'a FiringXyzDual16>,
        D: Iterator<Item = &'a FiringXyzDual32>,
    {
        fn from(v: FiringXyzSingle16RefIter<'a, A>) -> Self {
            Self::Single16(v)
        }
    }

    impl<'a, A, B, C, D> Iterator for FiringXyzKindRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzSingle16>,
        B: Iterator<Item = &'a FiringXyzSingle32>,
        C: Iterator<Item = &'a FiringXyzDual16>,
        D: Iterator<Item = &'a FiringXyzDual32>,
    {
        type Item = FiringXyzRefKind<'a>;

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
