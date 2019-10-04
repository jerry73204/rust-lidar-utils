extern crate failure;
extern crate lidar_utils;
extern crate pcap;
extern crate pretty_env_logger;
extern crate serde_json;

use failure::Fallible;
use lidar_utils::velodyne::{Packet as VelodynePacket, PointCloudConverter};
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

    for (idx, packet) in packets.iter().enumerate() {
        let ts = packet.gps_timestamp;
        println!("No. {}, gps_timestamp = {}", idx, ts);
    }

    Ok(())
}

#[test]
#[cfg(feature = "enable-pcap")]
fn velodyne_scan() -> Fallible<()> {
    let converter = PointCloudConverter::default();

    let mut cap = Capture::from_file("test_files/velodyne_example.pcap")?;
    cap.filter("udp")?;

    while let Ok(packet) = cap.next() {
        let lidar_packet = VelodynePacket::from_pcap(&packet)?;

        let mut frame_points = vec![];
        for firing in lidar_packet.firings.iter() {
            let mut column_points = converter.firing_to_points(&firing)?;
            frame_points.append(&mut column_points);
        }
    }

    Ok(())
}
