#![cfg(feature = "ouster-test")]

use anyhow::Result;
use lidar_utils::ouster::{
    config::Config, frame_converter::FrameConverter, packet::Packet as OusterPacket,
    pcd_converter::PointCloudConverter,
};
use pcap::Capture;

#[test]
#[cfg(feature = "pcap")]
fn ouster_create_packet() -> Result<()> {
    let mut packets = vec![];

    let mut cap = Capture::from_file("test_files/ouster_example.pcap")?;
    cap.filter("udp", true)?;

    while let Ok(packet) = cap.next() {
        let lidar_packet = OusterPacket::from_pcap(&packet)?;
        packets.push(lidar_packet);
    }

    let mut prev_timestamp = None;

    for packet in packets.iter() {
        let timestamp = packet.columns[0].timestamp;
        if let Some(prev) = prev_timestamp {
            assert!(timestamp > prev, "packets are not ordered by timestsamp");
        }
        prev_timestamp = Some(timestamp);
    }

    Ok(())
}

#[test]
#[cfg(feature = "pcap")]
fn ouster_pcd_converter() -> Result<()> {
    // Load config
    let config = Config::from_path("test_files/ouster_example.json")?;
    let pcd_converter = PointCloudConverter::from_config(config);

    // Load pcap file
    let mut cap = Capture::from_file("test_files/ouster_example.pcap")?;
    cap.filter("udp", true)?;

    while let Ok(packet) = cap.next() {
        let lidar_packet = OusterPacket::from_pcap(&packet)?;
        let points = pcd_converter.convert(lidar_packet)?;
        assert!(points.len() as u16 == pcd_converter.columns_per_revolution());
    }

    Ok(())
}

#[test]
#[cfg(feature = "pcap")]
fn ouster_frame_converter() -> Result<()> {
    // Load config
    let config = Config::from_path("test_files/ouster_example.json")?;
    let mut frame_converter = FrameConverter::from_config(config);

    // Load pcap file
    let mut cap = Capture::from_file("test_files/ouster_example.pcap")?;
    cap.filter("udp", true)?;

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
            assert!(prev_frame_id < frame.frame_id, "Frame ID is not ordered");
        }
        prev_frame_id_opt = Some(frame.frame_id);

        let mut prev_measurement_id_opt = None;
        for (measurement_id, timestamp) in frame.timestamps.iter() {
            if let Some(prev_timestamp) = prev_timestamp_opt {
                assert!(prev_timestamp < *timestamp, "Timestamp is not ordered");
            }
            prev_timestamp_opt = Some(*timestamp);

            if let Some(prev_measurement_id) = prev_measurement_id_opt {
                assert!(
                    prev_measurement_id < *measurement_id,
                    "Measurement ID is not ordered"
                );
            }
            prev_measurement_id_opt = Some(*measurement_id);
        }
    }

    Ok(())
}
