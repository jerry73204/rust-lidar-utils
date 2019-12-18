//! Useful constants for Velodyne data structures and calculations.

pub const DATA_PORT: u16 = 2368;
pub const CHANNELS_PER_BLOCK: usize = 32;
pub const BLOCKS_PER_PACKET: usize = 12;
pub const AZIMUTH_COUNT_PER_REV: usize = 36001; // Extra last tick overlaps with first tick
pub const CHANNEL_PERIOD: f64 = 2.304; // microseconds
pub const FIRING_PERIOD: f64 = 55.296; // microseconds
pub const FIRINGS_PER_COLUMN: usize = 2;

pub const VLP_16_ELEVAION_DEGREES: [f64; 16] = [
    -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0,
];
pub const VLP_16_AZIMUTH_OFFSET: [f64; 16] = [
    11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, 5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7, -11.2,
];

pub const PUKE_LITE_ELEVAION_DEGREES: [f64; 16] = [
    -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0,
];
pub const PUKE_LITE_AZIMUTH_OFFSET: [f64; 16] = [
    11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, 5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7, -11.2,
];

pub const PUKE_HI_RES_ELEVAION_DEGREES: [f64; 16] = [
    -10.00, 0.67, -8.67, 2.00, -7.33, 3.33, -6.00, 4.67, -4.67, 6.00, -3.33, 7.33, -2.00, 8.67,
    -0.67, 10.00,
];
pub const PUKE_HI_RES_AZIMUTH_OFFSET: [f64; 16] = [
    7.4, -0.9, 6.5, -1.8, 5.5, -2.7, 4.6, -3.7, 3.7, -4.6, 2.7, -5.5, 1.8, -6.5, 0.9, -7.4,
];

pub const VLP_32C_ELEVAION_DEGREES: [f64; 32] = [
    -25.0, -1.0, -1.667, -15.639, -11.31, 0.0, -0.667, -8.843, -7.254, 0.333, -0.333, -6.148,
    -5.333, 1.333, 0.667, -4.0, -4.667, 1.667, 1.0, -3.667, -3.333, 3.333, 2.333, -2.667, -3.0,
    7.0, 4.667, -2.333, -2.0, 15.0, 10.333, -1.333,
];

pub const VLP_32C_AZIMUTH_OFFSET: [f64; 32] = [
    1.4, -4.2, 1.4, -1.4, 1.4, -1.4, 4.2, -1.4, 1.4, -4.2, 1.4, -1.4, 4.2, -1.4, 4.2, -1.4, 1.4,
    -4.2, 1.4, -4.2, 4.2, -1.4, 1.4, -1.4, 1.4, -1.4, 1.4, -4.2, 4.2, -1.4, 1.4, -1.4,
];
