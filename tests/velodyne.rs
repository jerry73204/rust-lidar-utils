#![cfg(feature = "velodyne-test")]

use anyhow::Result;
use lidar_utils::velodyne::{
    config::ConfigBuilder,
    packet::Packet as VelodynePacket,
    pcd_converter::{PointCloudConverter, PointCloudConverterInterface, PointInterface},
};
use pcap::Capture;

#[test]
#[cfg(feature = "pcap")]
fn velodyne_vlp_16_pcap_file() -> Result<()> {
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
#[cfg(feature = "pcap")]
fn velodyne_vlp_16_scan() -> Result<()> {
    let config = ConfigBuilder::vlp_16_strongest_return();
    let mut converter = PointCloudConverter::from_config(config);

    let mut cap = Capture::from_file("test_files/velodyne_example.pcap")?;
    cap.filter("udp")?;

    let mut prev_timestamp = None;
    let mut prev_azimuth_angle = None;
    let mut points_per_frame = 0;

    while let Ok(packet) = cap.next() {
        let lidar_packet = VelodynePacket::from_pcap(&packet)?;

        for point in converter.convert(lidar_packet)?.into_iter() {
            let curr_timestamp = point.timestamp();
            if let Some(prev) = prev_timestamp {
                assert!(curr_timestamp > prev, "Points are not ordered by timestamp");
            }
            prev_timestamp = Some(curr_timestamp);

            let curr_azimuth_angle = point.original_azimuth_angle();
            if let Some(true) = prev_azimuth_angle.map(|prev| curr_azimuth_angle >= prev) {
                points_per_frame += 1;
            } else {
                eprintln!("# of points in frame: {}", points_per_frame);
                points_per_frame = 0;
            }
            prev_azimuth_angle = Some(curr_azimuth_angle);
        }
    }

    let _ = points_per_frame;

    Ok(())
}

#[test]
#[cfg(feature = "pcap")]
fn velodyne_vlp_32_pcap_file() -> Result<()> {
    let mut packets = vec![];

    let mut cap = Capture::from_file("test_files/hdl32_example.pcap")?;
    cap.filter("udp")?;

    while let Ok(packet) = cap.next() {
        let lidar_packet = match VelodynePacket::from_pcap(&packet) {
            Ok(packet) => packet,
            Err(_) => continue,
        };
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
#[cfg(feature = "pcap")]
fn velodyne_vlp_32c_scan() -> Result<()> {
    let config = ConfigBuilder::vlp_32c_strongest_return();
    let mut converter = PointCloudConverter::from_config(config);

    let mut cap = Capture::from_file("test_files/hdl32_example.pcap")?;
    cap.filter("udp")?;

    // let mut prev_timestamp = None;
    let mut prev_azimuth_angle = None;
    let mut points_per_frame = 0;

    while let Ok(packet) = cap.next() {
        let lidar_packet = match VelodynePacket::from_pcap(&packet) {
            Ok(packet) => packet,
            Err(_) => continue,
        };

        for point in converter.convert(lidar_packet)?.into_iter() {
            let curr_azimuth_angle = point.original_azimuth_angle();
            if let Some(true) = prev_azimuth_angle.map(|prev| curr_azimuth_angle >= prev) {
                points_per_frame += 1;
            } else {
                eprintln!("# of points in frame: {}", points_per_frame);
                points_per_frame = 0;
            }
            prev_azimuth_angle = Some(curr_azimuth_angle);
        }
    }

    let _ = points_per_frame;

    Ok(())
}
