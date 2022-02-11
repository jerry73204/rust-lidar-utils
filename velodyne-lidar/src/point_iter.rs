use crate::point::{PointDual, PointKind, PointKindRef, PointSingle};

pub use point_iter_::*;
pub mod point_iter_ {
    use super::*;

    pub enum PointIter<A, B, C, D>
    where
        A: Iterator<Item = PointSingle>,
        B: Iterator<Item = PointSingle>,
        C: Iterator<Item = PointDual>,
        D: Iterator<Item = PointDual>,
    {
        Single16(A),
        Single32(B),
        Dual16(C),
        Dual32(D),
    }

    impl<A, B, C, D> Iterator for PointIter<A, B, C, D>
    where
        A: Iterator<Item = PointSingle>,
        B: Iterator<Item = PointSingle>,
        C: Iterator<Item = PointDual>,
        D: Iterator<Item = PointDual>,
    {
        type Item = PointKind;

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

pub use point_ref_iter::*;
pub mod point_ref_iter {
    use super::*;

    pub enum PointRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a PointSingle>,
        B: Iterator<Item = &'a PointSingle>,
        C: Iterator<Item = &'a PointDual>,
        D: Iterator<Item = &'a PointDual>,
    {
        Single16(A),
        Single32(B),
        Dual16(C),
        Dual32(D),
    }

    impl<'a, A, B, C, D> Iterator for PointRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a PointSingle>,
        B: Iterator<Item = &'a PointSingle>,
        C: Iterator<Item = &'a PointDual>,
        D: Iterator<Item = &'a PointDual>,
    {
        type Item = PointKindRef<'a>;

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
