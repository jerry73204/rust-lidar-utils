use crate::{
    common::*,
    firing_xyz::types::{
        FiringXyz, FiringXyzD16, FiringXyzD32, FiringXyzRef, FiringXyzS16, FiringXyzS32,
    },
    frame_xyz::{
        batcher::{FrameXyzBatcherD16, FrameXyzBatcherD32, FrameXyzBatcherS16, FrameXyzBatcherS32},
        iter::{FrameXyzIterD16, FrameXyzIterD32, FrameXyzIterS16, FrameXyzIterS32},
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
    FiringXyzIterS16,
    FiringXyzS16,
    FrameXyzBatcherS16,
    FrameXyzS16,
    FrameXyzIterS16
);
declare_iter!(
    FiringXyzIterS32,
    FiringXyzS32,
    FrameXyzBatcherS32,
    FrameXyzS32,
    FrameXyzIterS32
);
declare_iter!(
    FiringXyzIterD16,
    FiringXyzD16,
    FrameXyzBatcherD16,
    FrameXyzD16,
    FrameXyzIterD16
);
declare_iter!(
    FiringXyzIterD32,
    FiringXyzD32,
    FrameXyzBatcherD32,
    FrameXyzD32,
    FrameXyzIterD32
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

declare_ref_iter!(FiringXyzRefIterS16, FiringXyzS16);
declare_ref_iter!(FiringXyzRefIterS32, FiringXyzS32);
declare_ref_iter!(FiringXyzRefIterD16, FiringXyzD16);
declare_ref_iter!(FiringXyzRefIterD32, FiringXyzD32);

pub use firing_xyz_kind_iter::*;
mod firing_xyz_kind_iter {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    pub enum FiringXyzIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzS16>,
        B: Iterator<Item = FiringXyzS32>,
        C: Iterator<Item = FiringXyzD16>,
        D: Iterator<Item = FiringXyzD32>,
    {
        Single16(FiringXyzIterS16<A>),
        Single32(FiringXyzIterS32<B>),
        Dual16(FiringXyzIterD16<C>),
        Dual32(FiringXyzIterD32<D>),
    }

    impl<A, B, C, D> From<FiringXyzIterD32<D>> for FiringXyzIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzS16>,
        B: Iterator<Item = FiringXyzS32>,
        C: Iterator<Item = FiringXyzD16>,
        D: Iterator<Item = FiringXyzD32>,
    {
        fn from(v: FiringXyzIterD32<D>) -> Self {
            Self::Dual32(v)
        }
    }

    impl<A, B, C, D> From<FiringXyzIterD16<C>> for FiringXyzIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzS16>,
        B: Iterator<Item = FiringXyzS32>,
        C: Iterator<Item = FiringXyzD16>,
        D: Iterator<Item = FiringXyzD32>,
    {
        fn from(v: FiringXyzIterD16<C>) -> Self {
            Self::Dual16(v)
        }
    }

    impl<A, B, C, D> From<FiringXyzIterS32<B>> for FiringXyzIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzS16>,
        B: Iterator<Item = FiringXyzS32>,
        C: Iterator<Item = FiringXyzD16>,
        D: Iterator<Item = FiringXyzD32>,
    {
        fn from(v: FiringXyzIterS32<B>) -> Self {
            Self::Single32(v)
        }
    }

    impl<A, B, C, D> From<FiringXyzIterS16<A>> for FiringXyzIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzS16>,
        B: Iterator<Item = FiringXyzS32>,
        C: Iterator<Item = FiringXyzD16>,
        D: Iterator<Item = FiringXyzD32>,
    {
        fn from(v: FiringXyzIterS16<A>) -> Self {
            Self::Single16(v)
        }
    }

    impl<A, B, C, D> Iterator for FiringXyzIter<A, B, C, D>
    where
        A: Iterator<Item = FiringXyzS16>,
        B: Iterator<Item = FiringXyzS32>,
        C: Iterator<Item = FiringXyzD16>,
        D: Iterator<Item = FiringXyzD32>,
    {
        type Item = FiringXyz;

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
    pub enum FiringXyzRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzS16>,
        B: Iterator<Item = &'a FiringXyzS32>,
        C: Iterator<Item = &'a FiringXyzD16>,
        D: Iterator<Item = &'a FiringXyzD32>,
    {
        Single16(FiringXyzRefIterS16<'a, A>),
        Single32(FiringXyzRefIterS32<'a, B>),
        Dual16(FiringXyzRefIterD16<'a, C>),
        Dual32(FiringXyzRefIterD32<'a, D>),
    }

    impl<'a, A, B, C, D> From<FiringXyzRefIterD32<'a, D>> for FiringXyzRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzS16>,
        B: Iterator<Item = &'a FiringXyzS32>,
        C: Iterator<Item = &'a FiringXyzD16>,
        D: Iterator<Item = &'a FiringXyzD32>,
    {
        fn from(v: FiringXyzRefIterD32<'a, D>) -> Self {
            Self::Dual32(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringXyzRefIterD16<'a, C>> for FiringXyzRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzS16>,
        B: Iterator<Item = &'a FiringXyzS32>,
        C: Iterator<Item = &'a FiringXyzD16>,
        D: Iterator<Item = &'a FiringXyzD32>,
    {
        fn from(v: FiringXyzRefIterD16<'a, C>) -> Self {
            Self::Dual16(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringXyzRefIterS32<'a, B>> for FiringXyzRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzS16>,
        B: Iterator<Item = &'a FiringXyzS32>,
        C: Iterator<Item = &'a FiringXyzD16>,
        D: Iterator<Item = &'a FiringXyzD32>,
    {
        fn from(v: FiringXyzRefIterS32<'a, B>) -> Self {
            Self::Single32(v)
        }
    }

    impl<'a, A, B, C, D> From<FiringXyzRefIterS16<'a, A>> for FiringXyzRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a FiringXyzS16>,
        B: Iterator<Item = &'a FiringXyzS32>,
        C: Iterator<Item = &'a FiringXyzD16>,
        D: Iterator<Item = &'a FiringXyzD32>,
    {
        fn from(v: FiringXyzRefIterS16<'a, A>) -> Self {
            Self::Single16(v)
        }
    }

    impl<'a, A, B, C, D> Iterator for FiringXyzRefIter<'a, A, B, C, D>
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
