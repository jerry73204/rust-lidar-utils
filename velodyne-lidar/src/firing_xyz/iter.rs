use crate::{
    common::*,
    firing_xyz::types::{
        FiringXyzD16, FiringXyzD32, FiringXyzKind, FiringXyzRef, FiringXyzS16, FiringXyzS32,
    },
    frame_xyz::{
        batcher::{FrameXyzBatcherD16, FrameXyzBatcherD32, FrameXyzBatcherS16, FrameXyzBatcherS32},
        iter::{FrameXyzD16Iter, FrameXyzD32Iter, FrameXyzS16Iter, FrameXyzS32Iter},
        types::{FrameXyzD16, FrameXyzD32, FrameXyzS16, FrameXyzS32},
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
    FiringXyzS16Iter,
    FiringXyzS16,
    FrameXyzBatcherS16,
    FrameXyzS16,
    FrameXyzS16Iter
);
declare_iter!(
    FiringXyzS32Iter,
    FiringXyzS32,
    FrameXyzBatcherS32,
    FrameXyzS32,
    FrameXyzS32Iter
);
declare_iter!(
    FiringXyzD16Iter,
    FiringXyzD16,
    FrameXyzBatcherD16,
    FrameXyzD16,
    FrameXyzD16Iter
);
declare_iter!(
    FiringXyzD32Iter,
    FiringXyzD32,
    FrameXyzBatcherD32,
    FrameXyzD32,
    FrameXyzD32Iter
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

declare_ref_iter!(FiringXyzSingle16RefIter, FiringXyzS16,);
declare_ref_iter!(FiringXyzSingle32RefIter, FiringXyzS32,);
declare_ref_iter!(FiringXyzDual16RefIter, FiringXyzD16,);
declare_ref_iter!(FiringXyzDual32RefIter, FiringXyzD32,);

pub use firing_xyz_kind_iter::*;
mod firing_xyz_kind_iter {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum FiringXyzKindIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzS16>,
        B: Iterator<Item = FiringXyzS32>,
        C: Iterator<Item = FiringXyzD16>,
        D: Iterator<Item = FiringXyzD32>,
    {
        Single16(FiringXyzS16Iter<A>),
        Single32(FiringXyzS32Iter<B>),
        Dual16(FiringXyzD16Iter<C>),
        Dual32(FiringXyzD32Iter<D>),
    }

    impl<A, B, C, D> From<FiringXyzD32Iter<D>> for FiringXyzKindIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzS16>,
        B: Iterator<Item = FiringXyzS32>,
        C: Iterator<Item = FiringXyzD16>,
        D: Iterator<Item = FiringXyzD32>,
    {
        fn from(v: FiringXyzD32Iter<D>) -> Self {
            Self::Dual32(v)
        }
    }

    impl<A, B, C, D> From<FiringXyzD16Iter<C>> for FiringXyzKindIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzS16>,
        B: Iterator<Item = FiringXyzS32>,
        C: Iterator<Item = FiringXyzD16>,
        D: Iterator<Item = FiringXyzD32>,
    {
        fn from(v: FiringXyzD16Iter<C>) -> Self {
            Self::Dual16(v)
        }
    }

    impl<A, B, C, D> From<FiringXyzS32Iter<B>> for FiringXyzKindIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzS16>,
        B: Iterator<Item = FiringXyzS32>,
        C: Iterator<Item = FiringXyzD16>,
        D: Iterator<Item = FiringXyzD32>,
    {
        fn from(v: FiringXyzS32Iter<B>) -> Self {
            Self::Single32(v)
        }
    }

    impl<A, B, C, D> From<FiringXyzS16Iter<A>> for FiringXyzKindIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzS16>,
        B: Iterator<Item = FiringXyzS32>,
        C: Iterator<Item = FiringXyzD16>,
        D: Iterator<Item = FiringXyzD32>,
    {
        fn from(v: FiringXyzS16Iter<A>) -> Self {
            Self::Single16(v)
        }
    }

    impl<A, B, C, D> Iterator for FiringXyzKindIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzS16>,
        B: Iterator<Item = FiringXyzS32>,
        C: Iterator<Item = FiringXyzD16>,
        D: Iterator<Item = FiringXyzD32>,
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
        A: Iterator<Item = &'a FiringXyzS16>,
        B: Iterator<Item = &'a FiringXyzS32>,
        C: Iterator<Item = &'a FiringXyzD16>,
        D: Iterator<Item = &'a FiringXyzD32>,
    {
        Single16(FiringXyzSingle16RefIter<'a, A>),
        Single32(FiringXyzSingle32RefIter<'a, B>),
        Dual16(FiringXyzDual16RefIter<'a, C>),
        Dual32(FiringXyzDual32RefIter<'a, D>),
    }

    impl<'a, A, B, C, D> From<FiringXyzDual32RefIter<'a, D>> for FiringXyzKindRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzS16>,
        B: Iterator<Item = &'a FiringXyzS32>,
        C: Iterator<Item = &'a FiringXyzD16>,
        D: Iterator<Item = &'a FiringXyzD32>,
    {
        fn from(v: FiringXyzDual32RefIter<'a, D>) -> Self {
            Self::Dual32(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringXyzDual16RefIter<'a, C>> for FiringXyzKindRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzS16>,
        B: Iterator<Item = &'a FiringXyzS32>,
        C: Iterator<Item = &'a FiringXyzD16>,
        D: Iterator<Item = &'a FiringXyzD32>,
    {
        fn from(v: FiringXyzDual16RefIter<'a, C>) -> Self {
            Self::Dual16(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringXyzSingle32RefIter<'a, B>> for FiringXyzKindRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzS16>,
        B: Iterator<Item = &'a FiringXyzS32>,
        C: Iterator<Item = &'a FiringXyzD16>,
        D: Iterator<Item = &'a FiringXyzD32>,
    {
        fn from(v: FiringXyzSingle32RefIter<'a, B>) -> Self {
            Self::Single32(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringXyzSingle16RefIter<'a, A>> for FiringXyzKindRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzS16>,
        B: Iterator<Item = &'a FiringXyzS32>,
        C: Iterator<Item = &'a FiringXyzD16>,
        D: Iterator<Item = &'a FiringXyzD32>,
    {
        fn from(v: FiringXyzSingle16RefIter<'a, A>) -> Self {
            Self::Single16(v)
        }
    }

    impl<'a, A, B, C, D> Iterator for FiringXyzKindRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzS16>,
        B: Iterator<Item = &'a FiringXyzS32>,
        C: Iterator<Item = &'a FiringXyzD16>,
        D: Iterator<Item = &'a FiringXyzD32>,
    {
        type Item = FiringXyzRef<'a>;

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
