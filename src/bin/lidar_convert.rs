use failure::{bail, format_err, Fallible};
use lidar_utils::{
    ouster::{
        packet::Packet as OusterPacket,
        utils::{Config as OusterConfig, FrameConverter as OusterFrameConverter},
    },
    velodyne::{
        packet::Packet as VelodynePacket,
        utils::{
            Config as VelodyneConfig, FrameConverter as VelodyneFrameConverter, VelodynePointList,
        },
    },
    Point as CartesianPoint,
};
use pcap::{Capture, Error as PcapError};
use pcd_rs::{
    meta::DataKind,
    seq_writer::{SeqWriterBuilder, SeqWriterBuilderEx, SeqWriterEx},
    PCDRecordWrite,
};
use std::path::{Path, PathBuf};

const UDP_HEADER_SIZE: usize = 42;

#[derive(Debug, Clone, PartialEq, PCDRecordWrite)]
struct PCDPoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub timestamp: f64,
}

#[derive(Debug, Clone, PartialEq)]
enum Model {
    OusterOs1,
    VelodyneVlp16,
}

fn main() -> Fallible<()> {
    // Parse arguments
    let args_config = clap::load_yaml!("lidar_convert.args.yaml");

    let arg_matches = clap::App::from_yaml(args_config).get_matches();
    let input_path = {
        let path = arg_matches.value_of("INPUT").unwrap();
        PathBuf::from(path)
    };
    let output_dir = {
        let dir = arg_matches.value_of("output_directory").unwrap();
        let path = PathBuf::from(dir);
        std::fs::create_dir_all(&path)?;
        path
    };
    let ouster_config_opt = arg_matches
        .value_of("ouster_config")
        .map(|path| PathBuf::from(path));
    let velodyne_rpm_opt = arg_matches
        .value_of("velodyne_rpm")
        .map(|text| text.parse::<u64>())
        .transpose()?;
    let model = {
        use Model::*;
        let name = arg_matches.value_of("model").unwrap();
        match name {
            "OUSTER_OS_1" => OusterOs1,
            "VELODYNE_VLP_16" => VelodyneVlp16,
            _ => bail!("model name {:?} is not recognized", name),
        }
    };

    match model {
        Model::OusterOs1 => {
            convert_ouster_os1(input_path, output_dir, ouster_config_opt)?;
        }
        Model::VelodyneVlp16 => {
            let rpm = velodyne_rpm_opt.ok_or(format_err!(
                "--velodyne-rpm is required for Velodyne sensors"
            ))?;
            convert_vel_vlp_16(input_path, output_dir, rpm)?;
        }
    }

    Ok(())
}

fn convert_ouster_os1<P: AsRef<Path>>(
    input_path: P,
    output_dir: P,
    config_path_opt: Option<P>,
) -> Fallible<()> {
    let mut cap = Capture::from_file(input_path)?;
    let packet_size = std::mem::size_of::<OusterPacket>() + UDP_HEADER_SIZE;
    cap.filter(&format!(
        "udp && less {} && greater {}",
        packet_size, packet_size
    ))?;

    let config = match config_path_opt {
        Some(path) => OusterConfig::from_path(path.as_ref())?,
        None => OusterConfig::os_1_config(),
    };
    let mut frame_converter = OusterFrameConverter::new(config);

    loop {
        let udp_packet = match cap.next() {
            Ok(packet) => packet,
            Err(PcapError::NoMorePackets) => break,
            Err(error) => return Err(error.into()),
        };
        let packet = OusterPacket::from_pcap(&udp_packet)?;

        frame_converter
            .push_packet(&packet)?
            .into_iter()
            .map(|frame| {
                let points = frame
                    .points
                    .into_iter()
                    .map(|point| {
                        let CartesianPoint { x, y, z } = point.value.cartesian;
                        let timestamp = point.timestamp_ns;
                        PCDPoint {
                            x: x as f32,
                            y: y as f32,
                            z: z as f32,
                            timestamp: timestamp as f64,
                        }
                    })
                    .collect::<Vec<_>>();

                let output_path = output_dir.as_ref().join(format!("{}.pcd", frame.frame_id));
                let mut writer = SeqWriterBuilder::<PCDPoint, _>::new(
                    points.len() as u64,
                    1,
                    Default::default(),
                    DataKind::ASCII,
                )?
                .create(output_path)?;

                for point in points.into_iter() {
                    writer.push(&point)?;
                }

                Ok(())
            })
            .collect::<Fallible<Vec<_>>>()?;
    }

    Ok(())
}

fn convert_vel_vlp_16<P: AsRef<Path>>(input_path: P, output_dir: P, rpm: u64) -> Fallible<()> {
    let mut cap = Capture::from_file(input_path)?;
    let packet_size = std::mem::size_of::<VelodynePacket>() + UDP_HEADER_SIZE;
    cap.filter(&format!(
        "udp && less {} && greater {}",
        packet_size, packet_size
    ))?;

    let config = VelodyneConfig::vlp_16_config();
    let mut frame_converter = VelodyneFrameConverter::new(rpm, config)?;

    loop {
        let udp_packet = match cap.next() {
            Ok(packet) => packet,
            Err(PcapError::NoMorePackets) => break,
            Err(error) => return Err(error.into()),
        };
        let packet = VelodynePacket::from_pcap(&udp_packet)?;

        frame_converter
            .push_packet(&packet)?
            .into_iter()
            .map(|frame| {
                match frame.points {
                    VelodynePointList::LastReturn(points) => {
                        let n_points = points.len() as u64;

                        let pcd_points = points
                            .into_iter()
                            .map(|point| {
                                let CartesianPoint { x, y, z } = point.value.cartesian;
                                let timestamp = point.timestamp_ns;
                                PCDPoint {
                                    x: x as f32,
                                    y: y as f32,
                                    z: z as f32,
                                    timestamp: timestamp as f64,
                                }
                            })
                            .collect::<Vec<_>>();

                        if let Some(first_point) = pcd_points.first() {
                            let filename = format!("{}-last_return.pcd", first_point.timestamp);
                            let output_path = output_dir.as_ref().join(&filename);
                            let mut writer = SeqWriterBuilder::<PCDPoint, _>::new(
                                n_points,
                                1,
                                Default::default(),
                                DataKind::ASCII,
                            )?
                            .create(output_path)?;

                            for point in pcd_points.into_iter() {
                                writer.push(&point)?;
                            }
                        }
                    }
                    VelodynePointList::Strongest(points) => {
                        let n_points = points.len() as u64;
                        let pcd_points = points
                            .into_iter()
                            .map(|point| {
                                let CartesianPoint { x, y, z } = point.value.cartesian;
                                let timestamp = point.timestamp_ns;
                                PCDPoint {
                                    x: x as f32,
                                    y: y as f32,
                                    z: z as f32,
                                    timestamp: timestamp as f64,
                                }
                            })
                            .collect::<Vec<_>>();

                        if let Some(first_point) = pcd_points.first() {
                            let filename = format!("{}-strongest.pcd", first_point.timestamp);
                            let output_path = output_dir.as_ref().join(&filename);
                            let mut writer = SeqWriterBuilder::<PCDPoint, _>::new(
                                n_points,
                                1,
                                Default::default(),
                                DataKind::ASCII,
                            )?
                            .create(output_path)?;

                            for point in pcd_points.into_iter() {
                                writer.push(&point)?;
                            }
                        }
                    }
                    VelodynePointList::DualReturn(points) => {
                        let mut last_return_points = vec![];
                        let mut strongest_points = vec![];

                        points.into_iter().for_each(|point| {
                            let (last_return_pt, strongest_pt) = point.value;
                            let timestamp = point.timestamp_ns;

                            last_return_points.push({
                                let CartesianPoint { x, y, z } = last_return_pt.cartesian;
                                PCDPoint {
                                    x: x as f32,
                                    y: y as f32,
                                    z: z as f32,
                                    timestamp: timestamp as f64,
                                }
                            });
                            strongest_points.push({
                                let CartesianPoint { x, y, z } = strongest_pt.cartesian;
                                PCDPoint {
                                    x: x as f32,
                                    y: y as f32,
                                    z: z as f32,
                                    timestamp: timestamp as f64,
                                }
                            });
                        });

                        let n_last_return_points = last_return_points.len() as u64;
                        let n_strongest_points = strongest_points.len() as u64;

                        if let (Some(first_last_return_pt), Some(first_strongest_pt)) =
                            (last_return_points.first(), strongest_points.first())
                        {
                            {
                                let filename =
                                    format!("{}-last_return.pcd", first_last_return_pt.timestamp);
                                let output_path = output_dir.as_ref().join(&filename);
                                let mut writer = SeqWriterBuilder::<PCDPoint, _>::new(
                                    n_last_return_points,
                                    1,
                                    Default::default(),
                                    DataKind::ASCII,
                                )?
                                .create(output_path)?;

                                for point in last_return_points.into_iter() {
                                    writer.push(&point)?;
                                }
                            }

                            {
                                let filename =
                                    format!("{}-strongest.pcd", first_strongest_pt.timestamp);
                                let output_path = output_dir.as_ref().join(&filename);
                                let mut writer = SeqWriterBuilder::<PCDPoint, _>::new(
                                    n_strongest_points,
                                    1,
                                    Default::default(),
                                    DataKind::ASCII,
                                )?
                                .create(output_path)?;

                                for point in strongest_points.into_iter() {
                                    writer.push(&point)?;
                                }
                            }
                        }
                    }
                }
                Ok(())
            })
            .collect::<Fallible<Vec<_>>>()?;
    }

    Ok(())
}
