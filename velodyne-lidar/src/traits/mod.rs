//! Trait definitions.

pub(crate) type BoxIterator<'a, T> = Box<dyn Iterator<Item = T> + Sync + Send + 'a>;

pub use azimuth_range::*;
mod azimuth_range;

pub use point_field::*;
mod point_field;

pub use firing_like::*;
mod firing_like;
