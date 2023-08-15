//! Velodyne packet format types, configs and converters.

pub mod batcher;
mod common;
pub mod config;
pub mod consts;
mod convert;
pub mod iter;
pub mod packet;
#[cfg(feature = "parallel")]
pub mod par_iter;
pub mod traits;
pub mod types;
mod utils;

pub use config::*;
pub use packet::{DataPacket, Packet, PositionPacket, ProductID, ReturnMode};

pub mod prelude {
    pub use crate::traits::{AzimuthRange, FiringLike, PointField};
}
