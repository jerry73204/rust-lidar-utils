use super::BoxIterator;
use itertools::iproduct;

/// Rectangular random accessible point array.
pub trait PointField {
    type Point<'a>
    where
        Self: 'a;

    fn nrows(&self) -> usize;

    fn ncols(&self) -> usize;

    fn point_at(&self, row: usize, col: usize) -> Option<Self::Point<'_>>;

    fn indexed_point_iter(&self) -> BoxIterator<'_, ((usize, usize), Self::Point<'_>)>
    where
        Self: Sync,
    {
        Box::new(
            iproduct!(0..self.nrows(), 0..self.ncols())
                .map(|(row, col)| ((row, col), self.point_at(row, col).unwrap())),
        )
    }

    fn point_iter(&self) -> BoxIterator<'_, Self::Point<'_>>
    where
        Self: Sync,
    {
        Box::new(self.indexed_point_iter().map(|(_index, point)| point))
    }
}
