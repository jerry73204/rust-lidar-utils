use crate::point::types::{Point, PointD, PointRef, PointS};

pub use point_iter_::*;
pub mod point_iter_ {
    use super::*;

    pub enum PointIter<A, B, C, D>
    where
        A: Iterator<Item = PointS>,
        B: Iterator<Item = PointS>,
        C: Iterator<Item = PointD>,
        D: Iterator<Item = PointD>,
    {
        Single16(A),
        Single32(B),
        Dual16(C),
        Dual32(D),
    }

    impl<A, B, C, D> Iterator for PointIter<A, B, C, D>
    where
        A: Iterator<Item = PointS>,
        B: Iterator<Item = PointS>,
        C: Iterator<Item = PointD>,
        D: Iterator<Item = PointD>,
    {
        type Item = Point;

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
        A: Iterator<Item = &'a PointS>,
        B: Iterator<Item = &'a PointS>,
        C: Iterator<Item = &'a PointD>,
        D: Iterator<Item = &'a PointD>,
    {
        Single16(A),
        Single32(B),
        Dual16(C),
        Dual32(D),
    }

    impl<'a, A, B, C, D> Iterator for PointRefIter<'a, A, B, C, D>
    where
        A: Iterator<Item = &'a PointS>,
        B: Iterator<Item = &'a PointS>,
        C: Iterator<Item = &'a PointD>,
        D: Iterator<Item = &'a PointD>,
    {
        type Item = PointRef<'a>;

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
