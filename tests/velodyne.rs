extern crate failure;
extern crate lidar_utils;
extern crate pcap;
extern crate pretty_env_logger;
extern crate serde_json;

use failure::{ensure, Fallible};
use lidar_utils::velodyne::{
    packet::Packet as VelodynePacket,
    utils::{Config, FrameConverter, PointCloudConverter},
};
use pcap::Capture;

#[test]
#[cfg(feature = "enable-pcap")]
fn velodyne_pcap_file() -> Fallible<()> {
    let mut packets = vec![];

    let mut cap = Capture::from_file("test_files/velodyne_example.pcap")?;
    cap.filter("udp")?;

    while let Ok(packet) = cap.next() {
        let lidar_packet = VelodynePacket::from_pcap(&packet)?;

        packets.push(lidar_packet);
    }

    let mut prev_timestamp = None;

    for packet in packets.iter() {
        if let Some(prev) = prev_timestamp {
            let curr = packet.timestamp;
            assert!(curr > prev);
        }
        prev_timestamp = Some(packet.timestamp);
    }

    Ok(())
}

#[test]
#[cfg(feature = "enable-pcap")]
fn velodyne_scan() -> Fallible<()> {
    use std::f64::consts::PI;

    let config = Config::vlp_16_config();
    let converter = PointCloudConverter::new(config);

    let mut cap = Capture::from_file("test_files/velodyne_example.pcap")?;
    cap.filter("udp")?;

    let mut prev_timestamp = None;

    while let Ok(packet) = cap.next() {
        let lidar_packet = VelodynePacket::from_pcap(&packet)?;

        for point in converter.packet_to_points(&lidar_packet)?.into_iter() {
            let timestamp = point.timestamp_ns();
            let azimuth_angle = point.azimuth_angle();

            ensure!(
                azimuth_angle >= 0.0 && azimuth_angle <= 2.0 * PI,
                "azimuth angle is out of range"
            );

            if let Some(prev) = prev_timestamp {
                ensure!(timestamp > prev, "Points are not ordered by timestamp");
            }
            prev_timestamp = Some(timestamp);
        }
    }

    Ok(())
}

#[test]
#[cfg(feature = "enable-pcap")]
fn velodyne_frames() -> Fallible<()> {
    use std::f64::consts::PI;

    let config = Config::vlp_16_config();
    let mut converter = FrameConverter::new(300, config)?;

    let mut cap = Capture::from_file("test_files/velodyne_example.pcap")?;
    cap.filter("udp")?;

    while let Ok(packet) = cap.next() {
        let lidar_packet = match VelodynePacket::from_pcap(&packet) {
            Ok(packet) => packet,
            Err(_) => continue,
        };
        let frames = converter.push_packet(&lidar_packet)?;

        let mut prev_timestamp = None;

        for frame in frames.into_iter() {
            for point in frame.points.into_iter() {
                let timestamp = point.timestamp_ns();
                let azimuth_angle = point.azimuth_angle();

                ensure!(
                    azimuth_angle >= 0.0 && azimuth_angle <= 2.0 * PI,
                    "azimuth angle is out of range"
                );

                if let Some(prev) = prev_timestamp {
                    ensure!(timestamp > prev, "points are not ordered by timestamp");
                }

                prev_timestamp = Some(timestamp);
            }
        }
    }

    Ok(())
}
