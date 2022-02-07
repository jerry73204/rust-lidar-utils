//! Useful constants for Velodyne data structures and calculations.

use measurements::Length;
use once_cell::sync::Lazy;
use std::time::Duration;

/// Default UDP data port used by Velodyne LiDARs.
pub const DATA_PORT: u16 = 2368;

/// Number of channels in one block, where each channel represents a laser return.
pub const CHANNELS_PER_BLOCK: usize = 32;

/// Number of blocks in one packet, where each block represents a series of laser returns.
pub const BLOCKS_PER_PACKET: usize = 12;

/// Number azimuth _ticks_ of in one revolution.
pub const AZIMUTH_COUNT_PER_REV: usize = 36001; // Extra last tick overlaps with first tick

/// Period of one laser return in microseconds.
const CHANNEL_PERIOD_MICROS: f64 = 2.304; // microseconds

/// Period of one laser return.
pub const CHANNEL_PERIOD: Duration = Duration::from_nanos((CHANNEL_PERIOD_MICROS * 1e3) as u64);

/// Period of one vertical scan in microseconds.
const FIRING_PERIOD_MICROS: f64 = 55.296; // microseconds

/// Period of one vertical scan.
pub const FIRING_PERIOD: Duration = Duration::from_nanos((FIRING_PERIOD_MICROS * 1e3) as u64);

// VLP-16 parameters

pub mod vlp_16 {
    use super::*;

    pub const DISTANCE_RESOLUTION_MILLIS: f64 = 2.0;

    pub static DISTANCE_RESOLUTION: Lazy<Length> =
        Lazy::new(|| Length::from_millimeters(DISTANCE_RESOLUTION_MILLIS));

    /// Elevaion angles of VLP-16.
    pub const ELEVAION_DEGREES: [f64; 16] = [
        -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0,
        15.0,
    ];

    /// VLP-16 correspond index
    pub const ELEVAION_INDEX: [usize; 16] = [15, 13, 11, 9, 7, 5, 3, 1, 14, 12, 10, 8, 6, 4, 2, 0];

    /// The correction distance added to point position along vertical axis for VLP-16.
    pub const VERTICAL_OFFSETS: [f64; 16] = [
        11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, 5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7, -11.2,
    ];

    pub const AZIMUTH_OFFSETS: [f64; 16] = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ];

    pub const HORIZONTAL_OFFSETS: [f64; 16] = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ];
}

// Puck-Lite parameters

pub mod puck_lite {
    use super::*;

    pub const DISTANCE_RESOLUTION_MILLIS: f64 = 2.0;

    pub static DISTANCE_RESOLUTION: Lazy<Length> =
        Lazy::new(|| Length::from_millimeters(DISTANCE_RESOLUTION_MILLIS));

    /// Elevaion angles of Puck Lite.
    pub const ELEVAION_DEGREES: [f64; 16] = [
        -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0,
        15.0,
    ];

    /// The correction distance added to point position along vertical axis for Puck Lite.
    pub const VERTICAL_OFFSETS: [f64; 16] = [
        11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, 5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7, -11.2,
    ];
    pub const AZIMUTH_OFFSETS: [f64; 16] = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ];

    pub const HORIZONTAL_OFFSETS: [f64; 16] = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ];
}

// Puck Hi-Res parameters
pub mod puck_hires {
    use super::*;

    pub const DISTANCE_RESOLUTION_MILLIS: f64 = 2.0;

    pub static DISTANCE_RESOLUTION: Lazy<Length> =
        Lazy::new(|| Length::from_millimeters(DISTANCE_RESOLUTION_MILLIS));

    /// Elevaion angles of Puck Hi-Res.
    pub const ELEVAION_DEGREES: [f64; 16] = [
        -10.00, 0.67, -8.67, 2.00, -7.33, 3.33, -6.00, 4.67, -4.67, 6.00, -3.33, 7.33, -2.00, 8.67,
        -0.67, 10.00,
    ];

    /// The correction distance added to point position along vertical axis for Puck Hi-Res.
    pub const VERTICAL_OFFSETS: [f64; 16] = [
        7.4, -0.9, 6.5, -1.8, 5.5, -2.7, 4.6, -3.7, 3.7, -4.6, 2.7, -5.5, 1.8, -6.5, 0.9, -7.4,
    ];
    pub const AZIMUTH_OFFSETS: [f64; 16] = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ];

    pub const HORIZONTAL_OFFSETS: [f64; 16] = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ];
}

// VLP-32C parameters

pub mod vlp_32c {
    use super::*;

    pub const DISTANCE_RESOLUTION_MILLIS: f64 = 4.0;

    pub static DISTANCE_RESOLUTION: Lazy<Length> =
        Lazy::new(|| Length::from_millimeters(DISTANCE_RESOLUTION_MILLIS));

    /// Elevaion angles of VLP-32C.
    pub const ELEVAION_DEGREES: [f64; 32] = [
        -25.0, -1.0, -1.667, -15.639, -11.31, 0.0, -0.667, -8.843, -7.254, 0.333, -0.333, -6.148,
        -5.333, 1.333, 0.667, -4.0, -4.667, 1.667, 1.0, -3.667, -3.333, 3.333, 2.333, -2.667, -3.0,
        7.0, 4.667, -2.333, -2.0, 15.0, 10.333, -1.333,
    ];

    /// VLP-32C correspond index
    pub const ELEVAION_INDEX: [usize; 32] = [
        29, 30, 25, 26, 21, 22, 17, 13, 18, 14, 9, 5, 10, 6, 1, 31, 2, 28, 27, 23, 24, 20, 19, 15,
        16, 12, 11, 8, 7, 4, 3, 0,
    ];

    /// The correction distance added to point position along vertical axis for VLP-32C.
    pub const VERTICAL_OFFSETS: [f64; 32] = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ];

    pub const AZIMUTH_OFFSETS: [f64; 32] = [
        1.4, -4.2, 1.4, -1.4, 1.4, -1.4, 4.2, -1.4, 1.4, -4.2, 1.4, -1.4, 4.2, -1.4, 4.2, -1.4,
        1.4, -4.2, 1.4, -4.2, 4.2, -1.4, 1.4, -1.4, 1.4, -1.4, 1.4, -4.2, 4.2, -1.4, 1.4, -1.4,
    ];

    pub const HORIZONTAL_OFFSETS: [f64; 32] = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    ];
}
