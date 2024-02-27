use crate::consts::CHANNEL_PERIOD;
use std::time::Duration;

pub trait FiringLike {
    type Point<'a>
    where
        Self: 'a;

    fn start_toh(&self) -> Duration;

    fn num_points(&self) -> usize;

    fn point_at(&self, index: usize) -> Option<Self::Point<'_>>;

    fn time_iter(&self) -> TimeIterator {
        TimeIterator {
            index: 0,
            len: self.num_points(),
            value: self.start_toh(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct TimeIterator {
    index: usize,
    len: usize,
    value: Duration,
}

impl Iterator for TimeIterator {
    type Item = Duration;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index == self.len {
            return None;
        }

        let value = self.value;
        self.index += 1;
        self.value += CHANNEL_PERIOD;
        Some(value)
    }
}
