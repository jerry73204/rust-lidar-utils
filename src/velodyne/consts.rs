//! Useful constants for Velodyne data structures and calculations.

pub const DATA_PORT: u16 = 2368;
// pub const CHANNEL_PER_FIRING: usize = 16;
pub const COLUMNS_PER_PACKET: usize = 12;
pub const AZIMUTH_COUNT_PER_REV: usize = 36001; // Extra last tick overlaps with first tick
pub const CHANNEL_PERIOD: f64 = 2.304; // microseconds
pub const FIRING_PERIOD: f64 = 55.296; // microseconds
pub const FIRINGS_PER_COLUMN: usize = 2;
