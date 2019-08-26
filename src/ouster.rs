use chrono::NaiveDateTime;
use failure::Fallible;
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
    pub raw_range: u32,
    pub reflectivity: u16,
    pub signal_photons: u16,
    pub noise_photons: u16,
    _pad: u16,
}

impl Pixel {
    pub fn range(&self) -> u32 {
        self.raw_range & 0x000fffff
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct Column {
    pub timestamp: u64,
    pub measurement_id: u16,
    pub frame_id: u16,
    pub encoder_ticks: u32,
    pub pixels: [Pixel; PIXELS_PER_COLUMN],
    pub raw_valid: u32,
}

impl Column {
    pub fn datetime(&self) -> NaiveDateTime {
        let secs = self.timestamp / 1_000_000_000;
        let nsecs = self.timestamp % 1_000_000_000;
        NaiveDateTime::from_timestamp(secs as i64, nsecs as u32)
    }

    pub fn azimuth_angle(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.encoder_ticks as f64 / ENCODER_TICKS_PER_REV as f64
    }

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

#[derive(Debug, Clone, Serialize, Deserialize)]
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
    pub fn from_path<P: AsRef<Path>>(path: P) -> Fallible<Config> {
        let file = File::open(path.as_ref())?;
        let ret = Self::from_reader(file)?;
        Ok(ret)
    }

    pub fn from_reader<R: Read>(reader: R) -> Fallible<Config> {
        let ret = serde_json::de::from_reader(reader)?;
        Ok(ret)
    }

    pub fn from_str(data: &str) -> Fallible<Config> {
        let ret = serde_json::from_str(data)?;
        Ok(ret)
    }

    pub fn width(&self) -> usize {
        use LidarMode::*;
        match self.lidar_mode {
            Mode512x10 | Mode512x20 => 512,
            Mode1024x10 | Mode1024x20 => 1024,
            Mode2048x10 => 2048,
        }
    }

    pub fn xyz_lut(&self) -> Vec<Vec<(f64, f64, f64)>> {
        use std::f64::consts::PI;

        let deg2rad = |deg: f64| deg * PI / 180.0;
        let width = self.width();
        let points = (0..width)
            .into_iter()
            .map(|col| {
                let azimuth_angle_base = 2.0 * PI * col as f64 / width as f64;

                let row_points = self
                    .beam_azimuth_angles
                    .iter()
                    .zip(self.beam_altitude_angles.iter())
                    .map(|(azimuth_deg_off, altitude_deg)| {
                        let azimuth_angle = deg2rad(*azimuth_deg_off) + azimuth_angle_base;
                        let altitude_angle = deg2rad(*altitude_deg);

                        let x = altitude_angle.cos() * azimuth_angle.cos();
                        let y = altitude_angle.cos() * azimuth_angle.sin();
                        let z = altitude_angle.sin();

                        (x, y, z)
                    })
                    .collect::<Vec<_>>();

                row_points
            })
            .collect::<Vec<_>>();

        points
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
