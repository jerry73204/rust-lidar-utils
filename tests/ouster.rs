#[macro_use]
extern crate failure;
#[macro_use]
extern crate log;

use failure::Fallible;
use lidar_utils::ouster::{
    packet::Packet as OusterPacket,
    utils::{Config, FrameConverter, PointCloudConverter},
};
use pcap::Capture;

#[test]
#[cfg(feature = "enable-pcap")]
fn ouster_create_packet() -> Fallible<()> {
    let mut packets = vec![];

    let mut cap = Capture::from_file("test_files/ouster_example.pcap")?;
    cap.filter("udp")?;

    while let Ok(packet) = cap.next() {
        let lidar_packet = OusterPacket::from_pcap(&packet)?;
        packets.push(lidar_packet);
    }

    let mut prev_timestamp = None;

    for packet in packets.iter() {
        let timestamp = packet.columns[0].timestamp;
        if let Some(prev) = prev_timestamp {
            ensure!(timestamp > prev, "packets are not ordered by timestsamp");
        }
        prev_timestamp = Some(timestamp);
    }

    Ok(())
}

#[test]
#[cfg(feature = "enable-pcap")]
fn ouster_pcd_converter() -> Fallible<()> {
    // Load config
    let config = Config::from_path("test_files/ouster_example.json")?;
    let pcd_converter = PointCloudConverter::new(config);

    // Load pcap file
    let mut cap = Capture::from_file("test_files/ouster_example.pcap")?;
    cap.filter("udp")?;

    let mut last_fid_opt = None;

    while let Ok(packet) = cap.next() {
        let lidar_packet = OusterPacket::from_pcap(&packet)?;

        for column in lidar_packet.columns.iter() {
            // Skip invalid columns
            if !column.valid() {
                warn!("Invalid column detected");
                continue;
            }

            // Check if frame ID rewinds
            ensure!(
                last_fid_opt
                    .map(|last_fid| last_fid <= column.frame_id)
                    .unwrap_or(true),
                "Column with inconsecutive frame id detected. Please report this bug."
            );

            // Construct point cloud
            last_fid_opt = Some(column.frame_id);
            let _column_points = pcd_converter.column_to_points(&column)?;
        }
    }

    Ok(())
}

#[test]
#[cfg(feature = "enable-pcap")]
fn ouster_frame_converter() -> Fallible<()> {
    // Load config
    let config = Config::from_path("test_files/ouster_example.json")?;
    let mut frame_converter = FrameConverter::new(config);

    // Load pcap file
    let mut cap = Capture::from_file("/home/jerry73204/Downloads/lombard_street_OS1.pcap")?;
    cap.filter("udp")?;

    let mut frames = vec![];

    while let Ok(packet) = cap.next() {
        let lidar_packet = OusterPacket::from_pcap(&packet)?;
        let new_frames = frame_converter.push_packet(&lidar_packet)?;
        frames.extend(new_frames);
    }

    if let Some(frame) = frame_converter.finish() {
        frames.push(frame);
    }

    let mut prev_frame_id_opt = None;
    let mut prev_timestamp_opt = None;
    for frame in frames {
        if let Some(prev_frame_id) = prev_frame_id_opt {
            ensure!(prev_frame_id < frame.frame_id, "Frame ID is not ordered");
        }
        prev_frame_id_opt = Some(frame.frame_id);

        let mut prev_measurement_id_opt = None;
        for (measurement_id, timestamp) in frame.timestamps.iter() {
            if let Some(prev_timestamp) = prev_timestamp_opt {
                ensure!(prev_timestamp < *timestamp, "Timestamp is not ordered");
            }
            prev_timestamp_opt = Some(*timestamp);

            if let Some(prev_measurement_id) = prev_measurement_id_opt {
                ensure!(
                    prev_measurement_id < *measurement_id,
                    "Measurement ID is not ordered"
                );
            }
            prev_measurement_id_opt = Some(*measurement_id);
        }
    }

    Ok(())
}
