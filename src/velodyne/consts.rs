pub const DATA_PORT: u16 = 2368;
pub const LASER_PER_FIRING: usize = 32;
pub const FIRING_PER_PACKET: usize = 12;
pub const ENCODER_TICKS_PER_REV: usize = 36001; // Extra last tick overlaps with first tick
pub const DEFAULT_ALTITUDE_DEGREES: [f64; LASER_PER_FIRING] = [
    -30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999, -25.33, -4.0, -24.0,
    -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16.0,
    5.3299999, -14.67, 6.6700001, -13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67,
];
