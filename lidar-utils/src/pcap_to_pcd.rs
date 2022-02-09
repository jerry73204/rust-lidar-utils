use crate::opts;
use anyhow::Result;
use futures::stream::{self, StreamExt as _};
use par_stream::{ParStreamExt as _, TryParStreamExt as _};
use pcap::Capture;
use pcd_rs::PcdSerialize;
use std::{
    fs,
    path::{Path, PathBuf},
    sync::Arc,
};
use tokio::runtime::Runtime;
use velodyne_lidar::{config::Config, DataPacket};

const UDP_HEADER_SIZE: usize = 42;

#[derive(PcdSerialize)]
struct PcdPoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub fn pcap_to_pcd(args: opts::Convert) -> Result<()> {
    let opts::Convert {
        input_file,
        output_dir,
        parallel,
    } = args;

    if parallel {
        convert_parallel(input_file, output_dir)?;
    } else {
        convert(&input_file, &output_dir)?;
    }
    Ok(())
}

fn convert(input_file: &Path, output_dir: &Path) -> Result<()> {
    let mut cap = Capture::from_file(input_file)?;
    cap.filter("udp", true)?;

    let packets = itertools::unfold(cap, |cap| {
        Some(loop {
            let packet = cap.next().ok()?;
            let slice = &packet.data[UDP_HEADER_SIZE..];

            if let Ok(packet) = DataPacket::from_slice(slice) {
                break *packet;
            }
        })
    });
    let converter = Config::new_vlp_32c_strongest()
        .build_converter()
        .unwrap()
        .into_single32();

    fs::create_dir_all(&output_dir)?;

    converter
        .packet_iter_to_frame_xyz_iter(packets)
        .enumerate()
        .try_for_each(|(index, frame)| {
            let file = output_dir.join(format!("{:05}.pcd", index));

            let mut writer = pcd_rs::WriterInit {
                height: frame.nrows() as u64,
                width: frame.ncols() as u64,
                viewpoint: Default::default(),
                data_kind: pcd_rs::DataKind::Binary,
                schema: None,
            }
            .create(file)?;

            let points = frame.into_point_iter().map(|point| {
                let [x, y, z] = point.measurement.xyz;

                PcdPoint {
                    x: x.as_meters() as f32,
                    y: y.as_meters() as f32,
                    z: z.as_meters() as f32,
                }
            });

            for point in points {
                writer.push(&point)?;
            }

            writer.finish()?;

            anyhow::Ok(())
        })?;

    Ok(())
}

fn convert_parallel(input_file: PathBuf, output_dir: PathBuf) -> Result<()> {
    let runtime = Runtime::new()?;

    runtime.block_on(async move {
        let mut cap = Capture::from_file(input_file)?;
        cap.filter("udp", true)?;

        let packets = itertools::unfold(cap, |cap| {
            Some(loop {
                let packet = cap.next().ok()?;
                let slice = &packet.data[UDP_HEADER_SIZE..];

                if let Ok(packet) = DataPacket::from_slice(slice) {
                    break *packet;
                }
            })
        });

        fs::create_dir_all(&output_dir)?;
        let output_dir = Arc::new(output_dir);

        let config = Config::new_vlp_32c_strongest();
        let batcher = config.build_frame_xyz_batcher()?.into_single32();
        let converter = config.build_converter()?.into_single32();

        let converter = Arc::new(converter);

        let firing_stream = par_stream::iter_blocking(None, packets)
            .par_map(None, move |packet| {
                let converter = converter.clone();

                move || {
                    let firings: Vec<_> = converter.packet_to_firing_xyz_iter(&packet).collect();
                    stream::iter(firings)
                }
            })
            .flatten();
        let frame_stream = batcher.with_stream(firing_stream);

        frame_stream
            .enumerate()
            .map(Ok)
            .try_par_for_each_blocking(None, move |(index, frame)| {
                let output_dir = output_dir.clone();

                move || {
                    let file = output_dir.join(format!("{:05}.pcd", index));

                    let mut writer = pcd_rs::WriterInit {
                        height: frame.nrows() as u64,
                        width: frame.ncols() as u64,
                        viewpoint: Default::default(),
                        data_kind: pcd_rs::DataKind::Binary,
                        schema: None,
                    }
                    .create(file)?;

                    let points = frame.into_point_iter().map(|point| {
                        let [x, y, z] = point.measurement.xyz;

                        PcdPoint {
                            x: x.as_meters() as f32,
                            y: y.as_meters() as f32,
                            z: z.as_meters() as f32,
                        }
                    });

                    for point in points {
                        writer.push(&point)?;
                    }

                    writer.finish()?;

                    anyhow::Ok(())
                }
            })
            .await?;

        anyhow::Ok(())
    })
}
