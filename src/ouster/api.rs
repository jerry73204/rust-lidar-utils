use super::{LidarMode, MultipurposeIoMode, Polarity, TimestampMode, PIXELS_PER_COLUMN};
use failure::Fallible;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::{
    fmt::{Debug, Display, Error as FormatError, Formatter},
    io::{prelude::*, BufReader, LineWriter, Lines},
    net::{Ipv4Addr, SocketAddr, TcpStream, ToSocketAddrs},
};

// TODO: This workaround handles large array for serde.
//       We'll remove is it once the const generics is introduced.
big_array! { BigArray; }

#[derive(Debug, Serialize, Deserialize)]
pub struct ConfigText {
    #[serde(
        deserialize_with = "de_bool_from_int",
        serialize_with = "ser_bool_to_int"
    )]
    pub auto_start_flag: bool,
    pub lidar_mode: LidarMode,
    pub multipurpose_io_mode: MultipurposeIoMode,
    pub sync_pulse_in_polarity: Polarity,
    pub sync_pulse_out_angle: u64,
    pub sync_pulse_out_frequency: u64,
    pub sync_pulse_out_polarity: Polarity,
    pub sync_pulse_out_pulse_width: u64,
    pub timestamp_mode: TimestampMode,
    pub udp_ip: String,
    pub udp_port_imu: u64,
    pub udp_port_lidar: u64,
    #[serde(
        deserialize_with = "de_bool_from_int",
        serialize_with = "ser_bool_to_int"
    )]
    pub window_rejection_enable: bool,
}

#[derive(Serialize, Deserialize, Derivative)]
#[derivative(Debug)]
pub struct BeamIntrinsics {
    #[serde(with = "BigArray")]
    #[derivative(Debug(format_with = "self::large_array_fmt"))]
    pub beam_altitude_angles: [f64; PIXELS_PER_COLUMN],
    #[serde(with = "BigArray")]
    #[derivative(Debug(format_with = "self::large_array_fmt"))]
    pub beam_azimuth_angles: [f64; PIXELS_PER_COLUMN],
}

pub struct CommandClient<A: ToSocketAddrs> {
    address: A,
    reader: Lines<BufReader<TcpStream>>,
    writer: LineWriter<TcpStream>,
}

impl<A: ToSocketAddrs> CommandClient<A> {
    pub fn connect(address: A) -> Fallible<CommandClient<A>> {
        let stream = TcpStream::connect(&address)?;
        let reader = BufReader::new(stream.try_clone()?).lines();
        let writer = LineWriter::new(stream);
        let client = CommandClient {
            reader,
            writer,
            address,
        };
        Ok(client)
    }

    pub fn get_config_txt(&mut self) -> Fallible<ConfigText> {
        self.writer.write_all(b"get_config_txt\n")?;
        let line = self
            .reader
            .next()
            .ok_or(format_err!("Unexpected end of stream"))??;
        let config = serde_json::from_str(&line)?;
        Ok(config)
    }

    pub fn get_beam_intrinsics(&mut self) -> Fallible<BeamIntrinsics> {
        self.writer.write_all(b"get_beam_intrinsics\n")?;
        let line = self
            .reader
            .next()
            .ok_or(format_err!("Unexpected end of stream"))??;
        let config = serde_json::from_str(&line)?;
        Ok(config)
    }

    pub fn reinitialize(&mut self) -> Fallible<()> {
        self.writer.write_all(b"reinitialize\n")?;
        let line = self
            .reader
            .next()
            .ok_or(format_err!("Unexpected end of stream"))??;
        ensure!(line == "reinitialize", "Unexpected response {:?}", line);

        // Re-establish TCP connection to prevent blocking the buggy LIDAR server
        let stream = TcpStream::connect(&self.address)?;
        self.reader = BufReader::new(stream.try_clone()?).lines();
        self.writer = LineWriter::new(stream);

        Ok(())
    }

    pub fn set_udp_ip(&mut self, ip: Ipv4Addr) -> Fallible<()> {
        self.set_config_param("udp_ip", ip)?;
        Ok(())
    }

    pub fn set_udp_port_lidar(&mut self, port: u16) -> Fallible<()> {
        self.set_config_param("udp_port_lidar", port)?;
        Ok(())
    }

    pub fn set_udp_port_imu(&mut self, port: u16) -> Fallible<()> {
        self.set_config_param("udp_port_imu", port)?;
        Ok(())
    }

    pub fn set_lidar_mode(&mut self, mode: LidarMode) -> Fallible<()> {
        self.set_config_param("lidar_mode", mode)?;
        Ok(())
    }

    pub fn set_timestamp_mode(&mut self, mode: TimestampMode) -> Fallible<()> {
        self.set_config_param("timestamp_mode", mode)?;
        Ok(())
    }

    pub fn set_sync_pulse_in_polarity(&mut self, polarity: Polarity) -> Fallible<()> {
        self.set_config_param("sync_pulse_in_polarity", polarity)?;
        Ok(())
    }

    pub fn set_nmea_in_polarity(&mut self, polarity: Polarity) -> Fallible<()> {
        self.set_config_param("nmea_in_polarity", polarity)?;
        Ok(())
    }

    fn set_config_param<T: Display>(&mut self, param: &str, arg: T) -> Fallible<()> {
        let command = format!("set_config_param {} {}\n", param, arg);
        self.writer.write_all(command.as_bytes())?;
        let line = self
            .reader
            .next()
            .ok_or(format_err!("Unexpected end of stream"))??;
        ensure!(line == "set_config_param", "Unexpected response {:?}", line);
        Ok(())
    }
}

fn ser_bool_to_int<S>(value: &bool, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    match value {
        true => serializer.serialize_u64(1),
        false => serializer.serialize_u64(0),
    }
}

fn de_bool_from_int<'de, D>(deserializer: D) -> Result<bool, D::Error>
where
    D: Deserializer<'de>,
{
    match u64::deserialize(deserializer)? {
        1 => Ok(true),
        0 => Ok(false),
        other => {
            use serde::de::{Error, Unexpected};
            let error = Error::invalid_value(Unexpected::Unsigned(other), &"Expect 0 or 1");
            Err(error)
        }
    }
}

fn large_array_fmt<T: Debug>(
    array: &[T; PIXELS_PER_COLUMN],
    formatter: &mut Formatter,
) -> Result<(), FormatError> {
    write!(formatter, "{:?}", array as &[_])
}
