use crate::{
    common::*,
    firing_block::{FiringBlockD16, FiringBlockD32, FiringBlockS16, FiringBlockS32},
    firing_xyz::{FiringXyzD16, FiringXyzD32, FiringXyzS16, FiringXyzS32},
    kinds::FormatKind,
    traits::AzimuthRange,
};

#[derive(Debug, Clone)]
pub struct Batcher<E>
where
    E: AzimuthRange,
{
    buffer: Vec<E>,
}

impl<E> Batcher<E>
where
    E: AzimuthRange,
{
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push_one(&mut self, firing: E) -> Option<Vec<E>> {
        let buffer = &mut self.buffer;
        let wrap =
            matches!(buffer.last(), Some(prev) if prev.start_azimuth() > firing.start_azimuth());

        if wrap {
            let output = mem::replace(buffer, vec![firing]);
            Some(output)
        } else {
            buffer.push(firing);
            None
        }
    }

    pub fn push_many<'a, I>(&'a mut self, firings: I) -> impl Iterator<Item = Vec<E>> + 'a
    where
        I: IntoIterator<Item = E> + 'a,
    {
        firings
            .into_iter()
            .filter_map(|firing| self.push_one(firing))
    }

    pub fn take(&mut self) -> Option<Vec<E>> {
        let firings = mem::take(&mut self.buffer);
        (!firings.is_empty()).then_some(firings)
    }

    pub fn with_iter<I>(self, firings: I) -> impl Iterator<Item = Vec<E>>
    where
        I: IntoIterator<Item = E>,
    {
        itertools::unfold(Some((firings.into_iter(), self)), |state| {
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
        .flatten()
    }
}

impl<E> Default for Batcher<E>
where
    E: AzimuthRange,
{
    fn default() -> Self {
        Self::new()
    }
}

pub type FiringBlockBatcherS16<'a> = Batcher<FiringBlockS16<'a>>;
pub type FiringBlockBatcherS32<'a> = Batcher<FiringBlockS32<'a>>;
pub type FiringBlockBatcherD16<'a> = Batcher<FiringBlockD16<'a>>;
pub type FiringBlockBatcherD32<'a> = Batcher<FiringBlockD32<'a>>;

pub type FiringBlockBatcher<'a> = FormatKind<
    FiringBlockBatcherS16<'a>,
    FiringBlockBatcherS32<'a>,
    FiringBlockBatcherD16<'a>,
    FiringBlockBatcherD32<'a>,
>;

pub type FiringXyzBatcher =
    FormatKind<FiringXyzBatcherS16, FiringXyzBatcherS32, FiringXyzBatcherD16, FiringXyzBatcherD32>;

pub type FiringXyzBatcherS16 = Batcher<FiringXyzS16>;
pub type FiringXyzBatcherS32 = Batcher<FiringXyzS32>;
pub type FiringXyzBatcherD16 = Batcher<FiringXyzD16>;
pub type FiringXyzBatcherD32 = Batcher<FiringXyzD32>;
