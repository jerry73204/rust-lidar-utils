use failure::Fallible;
use ndarray::Array3;
#[cfg(feature = "enable-pcap")]
use pcap::Packet as PcapPacket;
use std::mem::size_of;

pub const DATA_PORT: u16 = 2368;
pub const LASER_PER_FIRING: usize = 32;
pub const FIRING_PER_PACKET: usize = 12;
pub const ENCODER_TICKS_PER_REV: usize = 36001; // Extra last tick overlaps with first tick
pub const DEFAULT_ALTITUDE_DEGREES: [f64; LASER_PER_FIRING] = [
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
    /// The raw distance of laser return. The distance in meter is the raw distance times 0.002.
    pub distance: u16,
    /// The intensity of laser return.
    pub intensity: u8,
}

impl LaserReturn {
    /// Compute distance in meters from sensor data.
    pub fn meter_distance(&self) -> f64 {
        self.distance as f64 * 0.002
    }
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct Firing {
    /// Valid if either 0xeeff or 0xddff, corresponding to range from 0 to 31, or range from 32 to 63.
    pub block_identifier: BlockIdentifier,
    /// Encoder count of rotation motor ranging from 0 to 36000 (inclusive).
    pub encoder_ticks: u16,
    /// Array of laser returns.
    pub laster_returns: [LaserReturn; LASER_PER_FIRING],
}

impl Firing {
    /// Compute azimuth angle in radian from encoder ticks.
    pub fn azimuth_angle(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.encoder_ticks as f64 / (ENCODER_TICKS_PER_REV - 1) as f64
    }
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct Packet {
    pub firings: [Firing; FIRING_PER_PACKET],
    pub gps_timestamp: u32,
    pub mode: u8,
    pub sensor_type: u8,
}

impl Packet {
    /// Construct packet from [pcap::Packet](pcap::Packet).
    #[cfg(feature = "enable-pcap")]
    pub fn from_pcap(packet: &PcapPacket) -> Fallible<Packet> {
        let packet_header_size = 42;

        ensure!(
            packet.header.len as usize - packet_header_size == size_of::<Packet>(),
            "Input pcap packet is not a valid Velodyne Lidar packet",
        );

        let mut buffer = Box::new([0u8; size_of::<Packet>()]);
        buffer.copy_from_slice(&packet.data[packet_header_size..]);
        Ok(Self::from_buffer(*buffer))
    }

    /// Construct packet from binary buffer.
    pub fn from_buffer(buffer: [u8; size_of::<Packet>()]) -> Packet {
        unsafe { std::mem::transmute::<_, Packet>(buffer) }
    }

    /// Construct packet from slice of bytes. Fail if the slice size is not correct.
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

pub struct Helper {
    altitude_degrees: [f64; LASER_PER_FIRING],
    spherical_projection: Array3<f64>,
}

impl Helper {
    /// Construct helper from altitude degrees for each laser beam.
    pub fn new(altitude_degrees: [f64; LASER_PER_FIRING]) -> Helper {
        let num_columns = ENCODER_TICKS_PER_REV - 1;
        let num_rows = altitude_degrees.len();
        let mut spherical_projection = Array3::<f64>::zeros((num_columns, num_rows, 3));

        (0..num_columns).into_iter().for_each(|col_index| {
            let azimuth_angle = 2.0 * std::f64::consts::PI * col_index as f64 / num_columns as f64;

            altitude_degrees
                .iter()
                .enumerate()
                .for_each(|(row_index, altitude_deg)| {
                    let altitude_angle = std::f64::consts::PI * altitude_deg / 180.0;

                    // TODO: the formula is different from upstream
                    // https://github.com/PointCloudLibrary/pcl/blob/master/io/src/hdl_grabber.cpp#L396
                    let x = altitude_angle.cos() * azimuth_angle.cos();
                    let y = altitude_angle.cos() * azimuth_angle.sin();
                    let z = altitude_angle.sin();

                    spherical_projection[(col_index, row_index, 0)] = x;
                    spherical_projection[(col_index, row_index, 1)] = y;
                    spherical_projection[(col_index, row_index, 2)] = z;
                })
        });

        let helper = Helper {
            altitude_degrees,
            spherical_projection,
        };

        helper
    }

    pub fn altitude_degrees(&self) -> &[f64; LASER_PER_FIRING] {
        &self.altitude_degrees
    }

    pub fn spherical_projection(&self) -> &Array3<f64> {
        &self.spherical_projection
    }

    /// Compute point locations from firing data from sensor.
    pub fn firing_to_points(&self, firing: &Firing) -> Fallible<Vec<(f64, f64, f64)>> {
        ensure!(
            (firing.encoder_ticks as usize) < ENCODER_TICKS_PER_REV,
            "encoder_ticks is out of bound"
        );
        let azimuth_angle = firing.azimuth_angle();
        let points = firing
            .laster_returns
            .iter()
            .zip(self.altitude_degrees.iter())
            .map(|(laster_return, altitude_deg)| {
                let altitude_angle = std::f64::consts::PI * altitude_deg / 180.0;
                let distance = laster_return.meter_distance();

                // TODO: the formula is different from upstream
                // https://github.com/PointCloudLibrary/pcl/blob/master/io/src/hdl_grabber.cpp#L396
                let x = distance * altitude_angle.cos() * azimuth_angle.cos();
                let y = distance * altitude_angle.cos() * azimuth_angle.sin();
                let z = distance * altitude_angle.sin();

                (x, y, z)
            })
            .collect::<Vec<_>>();
        Ok(points)
    }
}

impl Default for Helper {
    fn default() -> Helper {
        Helper::new(DEFAULT_ALTITUDE_DEGREES)
    }
}

// References
// https://github.com/PointCloudLibrary/pcl/blob/b2212ef2466ba734bcd675427f6d982a15fd780a/io/src/hdl_grabber.cpp#L312
// https://github.com/PointCloudLibrary/pcl/blob/b2212ef2466ba734bcd675427f6d982a15fd780a/io/src/hdl_grabber.cpp#L396
