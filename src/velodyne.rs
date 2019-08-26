use failure::Fallible;
use std::mem::size_of;

pub const DATA_PORT: u16 = 2368;
pub const LASER_PER_FIRING: usize = 32;
pub const FIRING_PER_PACKET: usize = 12;
pub const ENCODER_TICKS_PER_REV: usize = 36001; // Extra last tick overlaps with first tick
pub const VERTICAL_CORRECTIONS: [f64; LASER_PER_FIRING] = [
    -30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999, -25.33, -4.0, -24.0,
    -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16.0,
    5.3299999, -14.67, 6.6700001, -13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67,
];

#[repr(u16)]
#[derive(Debug, Clone, Copy)]
pub enum BlockIdentifier {
    Block0To31 = 0xeeff,
    Block32To63 = 0xddff,
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct LaserReturn {
    pub distance: u16,
    pub intensity: u8,
}

impl LaserReturn {
    pub fn meter_distance(&self) -> f64 {
        self.distance as f64 * 0.002
    }
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct FiringData {
    pub block_identifier: BlockIdentifier,
    pub encoder_ticks: u16,
    pub laster_returns: [LaserReturn; LASER_PER_FIRING],
}

impl FiringData {
    pub fn azimuth_angle(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.encoder_ticks as f64 / (ENCODER_TICKS_PER_REV - 1) as f64
    }
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct Packet {
    pub firing_data: [FiringData; FIRING_PER_PACKET],
    pub gps_timestamp: u32,
    pub mode: u8,
    pub sensor_type: u8,
}

impl Packet {
    pub fn from_buffer(buffer: [u8; size_of::<Packet>()]) -> Packet {
        unsafe { std::mem::transmute::<_, Packet>(buffer) }
    }

    pub fn from_slice<'a>(buffer: &'a [u8]) -> Fallible<&'a Packet> {
        ensure!(
            buffer.len() == size_of::<Packet>(),
            "Requre the slice length to be {}, but get {}",
            size_of::<Packet>(),
            buffer.len(),
        );
        let packet = unsafe { &*(buffer.as_ptr() as *const Packet) };
        Ok(packet)
    }
}

// References
// https://github.com/PointCloudLibrary/pcl/blob/b2212ef2466ba734bcd675427f6d982a15fd780a/io/src/hdl_grabber.cpp#L312
// https://github.com/PointCloudLibrary/pcl/blob/b2212ef2466ba734bcd675427f6d982a15fd780a/io/src/hdl_grabber.cpp#L396
