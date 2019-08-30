use chrono::NaiveDateTime;
use failure::Fallible;
use ndarray::{Array3, Axis};
#[cfg(feature = "enable-pcap")]
use pcap::Packet as PcapPacket;
use serde::{Deserialize, Serialize};
use std::{
    fmt::{Debug, Formatter, Result as FormatResult},
    fs::File,
    io::Read,
    mem::size_of,
    path::Path,
};

pub const ENCODER_TICKS_PER_REV: u32 = 90112;
pub const PIXELS_PER_COLUMN: usize = 64;
pub const COLUMNS_PER_PACKET: usize = 16;

// TODO: This workaround handles large array for serde.
//       We'll remove is it once the const generics is introduced.
big_array! { BigArray; }

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct Pixel {
    /// The least significant 20 bits form distance in millimeters.
    pub raw_range: u32,
    pub reflectivity: u16,
    pub signal_photons: u16,
    pub noise_photons: u16,
    _pad: u16,
}

impl Pixel {
    /// Extract distance in millimeters from raw_range field.
    pub fn range(&self) -> u32 {
        self.raw_range & 0x000fffff
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct Column {
    /// Unix timestamp.
    pub timestamp: u64,
    /// The column index.
    pub measurement_id: u16,
    /// The frame index.
    pub frame_id: u16,
    /// Encoder count of rotation motor ranging from 0 to 90111 (inclusive).
    pub encoder_ticks: u32,
    /// Array of pixels.
    pub pixels: [Pixel; PIXELS_PER_COLUMN],
    /// Packet validility mark. True if value is 0xffffffff.
    pub raw_valid: u32,
}

impl Column {
    /// Construct [NaiveDateTime](chrono::NaiveDateTime) object from column timestamp.
    pub fn datetime(&self) -> NaiveDateTime {
        let secs = self.timestamp / 1_000_000_000;
        let nsecs = self.timestamp % 1_000_000_000;
        NaiveDateTime::from_timestamp(secs as i64, nsecs as u32)
    }

    /// Compute azimuth angle in radian from encoder ticks.
    pub fn azimuth_angle(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.encoder_ticks as f64 / ENCODER_TICKS_PER_REV as f64
    }

    /// Return if this packet is marked valid.
    pub fn valid(&self) -> bool {
        self.raw_valid == 0xffffffff
    }
}

impl Debug for Column {
    fn fmt(&self, formatter: &mut Formatter) -> FormatResult {
        let timestamp = self.timestamp;
        let measurement_id = self.measurement_id;
        let frame_id = self.frame_id;
        let encoder_ticks = self.encoder_ticks;
        let raw_valid = self.raw_valid;

        write!(
            formatter,
            "Column {{
    timestamp: {},
    measurement_id: {},
    frame_id: {},
    encoder_ticks: {},
    pixels: [...{} elemnts],
    raw_valid: 0x{:x},
}}",
            timestamp, measurement_id, frame_id, encoder_ticks, PIXELS_PER_COLUMN, raw_valid
        )
    }
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct Packet {
    pub columns: [Column; COLUMNS_PER_PACKET],
}

impl Packet {
    /// Construct packet from [pcap's Packet](pcap::Packet).
    #[cfg(feature = "enable-pcap")]
    pub fn from_pcap(packet: &PcapPacket) -> Fallible<Packet> {
        let packet_header_size = 42;

        ensure!(
            packet.header.len as usize - packet_header_size == size_of::<Packet>(),
            "Input pcap packet is not a valid Ouster Lidar packet",
        );

        let mut buffer = Box::new([0u8; size_of::<Packet>()]);
        buffer.copy_from_slice(&packet.data[packet_header_size..]);
        Ok(Self::from_buffer(*buffer))
    }

    /// Construct packet from binary buffer.
    pub fn from_buffer(buffer: [u8; size_of::<Packet>()]) -> Packet {
        unsafe { std::mem::transmute::<_, Packet>(buffer) }
    }

    /// Construct packet from slice of bytes. Error if the slice size is not correct.
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum LidarMode {
    #[serde(rename = "512x10")]
    Mode512x10,
    #[serde(rename = "512x20")]
    Mode512x20,
    #[serde(rename = "1024x10")]
    Mode1024x10,
    #[serde(rename = "1024x20")]
    Mode1024x20,
    #[serde(rename = "2048x10")]
    Mode2048x10,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct Config {
    #[serde(with = "BigArray")]
    pub beam_altitude_angles: [f64; PIXELS_PER_COLUMN],
    #[serde(with = "BigArray")]
    pub beam_azimuth_angles: [f64; PIXELS_PER_COLUMN],
    pub lidar_mode: LidarMode,
}

impl Config {
    /// Create new config.
    pub fn new(
        beam_altitude_angles: [f64; PIXELS_PER_COLUMN],
        beam_azimuth_angles: [f64; PIXELS_PER_COLUMN],
        lidar_mode: LidarMode,
    ) -> Config {
        Config {
            beam_altitude_angles,
            beam_azimuth_angles,
            lidar_mode,
        }
    }

    /// Load config JSON file from path.
    pub fn from_path<P: AsRef<Path>>(path: P) -> Fallible<Config> {
        let file = File::open(path.as_ref())?;
        let ret = Self::from_reader(file)?;
        Ok(ret)
    }

    /// Load config JSON data from reader with [Read](std::io::Read) trait.
    pub fn from_reader<R: Read>(reader: R) -> Fallible<Config> {
        let ret = serde_json::de::from_reader(reader)?;
        Ok(ret)
    }

    /// Parse from JSON string.
    pub fn from_str(data: &str) -> Fallible<Config> {
        let ret = serde_json::from_str(data)?;
        Ok(ret)
    }
}

impl Debug for Config {
    fn fmt(&self, formatter: &mut Formatter) -> FormatResult {
        write!(
            formatter,
            "Config {{
    altitude_angles: [...{} elemnts],
    azimuth_angle_offsets: [...{} elemnts],
    lidar_mode: {:?},
}}",
            PIXELS_PER_COLUMN, PIXELS_PER_COLUMN, self.lidar_mode
        )
    }
}

impl Default for Config {
    fn default() -> Config {
        let beam_altitude_angles = [
            16.611, 16.084, 15.557, 15.029, 14.502, 13.975, 13.447, 12.920, 12.393, 11.865, 11.338,
            10.811, 10.283, 9.756, 9.229, 8.701, 8.174, 7.646, 7.119, 6.592, 6.064, 5.537, 5.010,
            4.482, 3.955, 3.428, 2.900, 2.373, 1.846, 1.318, 0.791, 0.264, -0.264, -0.791, -1.318,
            -1.846, -2.373, -2.900, -3.428, -3.955, -4.482, -5.010, -5.537, -6.064, -6.592, -7.119,
            -7.646, -8.174, -8.701, -9.229, -9.756, -10.283, -10.811, -11.338, -11.865, -12.393,
            -12.920, -13.447, -13.975, -14.502, -15.029, -15.557, -16.084, -16.611,
        ];

        let beam_azimuth_angles = [
            3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055,
            -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055,
            -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164,
            1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
            3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055,
            -3.164, 3.164, 1.055, -1.055, -3.164, 3.164, 1.055, -1.055, -3.164,
        ];

        Config {
            beam_altitude_angles,
            beam_azimuth_angles,
            lidar_mode: LidarMode::Mode512x10,
        }
    }
}

impl From<Helper> for Config {
    fn from(config: Helper) -> Config {
        Config {
            beam_altitude_angles: config.beam_altitude_angles,
            beam_azimuth_angles: config.beam_azimuth_angles,
            lidar_mode: config.lidar_mode,
        }
    }
}

#[derive(Clone)]
pub struct Helper {
    beam_altitude_angles: [f64; PIXELS_PER_COLUMN],
    beam_azimuth_angles: [f64; PIXELS_PER_COLUMN],
    lidar_mode: LidarMode,
    num_columns: usize,
    spherical_projection: Array3<f64>,
}

impl Helper {
    pub fn new(
        beam_altitude_angles: [f64; PIXELS_PER_COLUMN],
        beam_azimuth_angles: [f64; PIXELS_PER_COLUMN],
        lidar_mode: LidarMode,
    ) -> Helper {
        Config::new(beam_altitude_angles, beam_azimuth_angles, lidar_mode).into()
    }

    pub fn from_config(config: Config) -> Helper {
        config.into()
    }

    pub fn beam_altitude_angles(&self) -> &[f64; PIXELS_PER_COLUMN] {
        &self.beam_altitude_angles
    }

    pub fn beam_azimuth_angles(&self) -> &[f64; PIXELS_PER_COLUMN] {
        &self.beam_azimuth_angles
    }

    pub fn lidar_mode(&self) -> LidarMode {
        self.lidar_mode
    }

    /// Get lidar scene width by its mode.
    pub fn num_columns(&self) -> usize {
        self.num_columns
    }

    /// Compute spherical projection on unit circle for each laser beam.
    ///
    /// It returns a three dimensional array indexed by column index,
    /// row index and component index. The first dimension size depends on
    /// [Helper::num_columns](Helper::num_columns). The second index size is fixed
    /// [PIXELS_PER_COLUMN](PIXELS_PER_COLUMN). The last dimension corresponds
    /// to x, y, z components.
    pub fn spherical_projection(&self) -> &Array3<f64> {
        &self.spherical_projection
    }

    /// Compute point locations from column returned from lidar.
    ///
    /// The method takes [Column.measurement_id](Column.measurement_id) as column index.
    /// It returns error if the index is out of bound.
    pub fn column_to_points(&self, column: &Column) -> Fallible<Vec<(f64, f64, f64)>> {
        let col_index = column.measurement_id as usize;
        ensure!(
            col_index < self.spherical_projection.shape()[0],
            "measurement_id is out of bound"
        );

        let sub_projection = self.spherical_projection.index_axis(Axis(0), col_index);

        let points = column
            .pixels
            .iter()
            .enumerate()
            .map(|(row_index, pixel)| {
                let x = sub_projection[(row_index, 0)];
                let y = sub_projection[(row_index, 1)];
                let z = sub_projection[(row_index, 2)];
                let range = pixel.range() as f64;
                let rx = x as f64 * range;
                let ry = y as f64 * range;
                let rz = z as f64 * range;
                (rx, ry, rz)
            })
            .collect::<Vec<_>>();

        Ok(points)
    }
}

impl From<Config> for Helper {
    fn from(ser_config: Config) -> Helper {
        let num_columns = {
            use LidarMode::*;
            match ser_config.lidar_mode {
                Mode512x10 | Mode512x20 => 512,
                Mode1024x10 | Mode1024x20 => 1024,
                Mode2048x10 => 2048,
            }
        };

        let spherical_projection = {
            use std::f64::consts::PI;
            let deg2rad = |deg: f64| deg * PI / 180.0;

            let mut projection = Array3::<f64>::zeros((num_columns, PIXELS_PER_COLUMN, 3));

            (0..num_columns).into_iter().for_each(|col| {
                let azimuth_angle_base = 2.0 * PI * col as f64 / num_columns as f64;

                ser_config
                    .beam_azimuth_angles
                    .iter()
                    .zip(ser_config.beam_altitude_angles.iter())
                    .enumerate()
                    .for_each(|(row, (azimuth_deg_off, altitude_deg))| {
                        let azimuth_angle = deg2rad(*azimuth_deg_off) + azimuth_angle_base;
                        let altitude_angle = deg2rad(*altitude_deg);

                        let x = altitude_angle.cos() * azimuth_angle.cos();
                        let y = altitude_angle.cos() * azimuth_angle.sin();
                        let z = altitude_angle.sin();

                        projection[(col, row, 0)] = x;
                        projection[(col, row, 1)] = y;
                        projection[(col, row, 2)] = z;
                    });
            });

            projection
        };

        Helper {
            beam_altitude_angles: ser_config.beam_altitude_angles,
            beam_azimuth_angles: ser_config.beam_azimuth_angles,
            lidar_mode: ser_config.lidar_mode,
            num_columns,
            spherical_projection,
        }
    }
}

impl Debug for Helper {
    fn fmt(&self, formatter: &mut Formatter) -> FormatResult {
        write!(
            formatter,
            "Helper {{
    altitude_angles: [...{} elemnts],
    azimuth_angle_offsets: [...{} elemnts],
    lidar_mode: {:?},
}}",
            PIXELS_PER_COLUMN, PIXELS_PER_COLUMN, self.lidar_mode
        )
    }
}

impl Default for Helper {
    fn default() -> Helper {
        Config::default().into()
    }
}
