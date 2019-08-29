#[macro_use]
extern crate failure;
extern crate lidar_buffer;
extern crate pcap;
extern crate serde_json;
#[macro_use]
extern crate log;
extern crate pretty_env_logger;

use failure::Fallible;
use lidar_buffer::velodyne::{Packet as VelodynePacket, ALTITUDE_DEGREES};
use pcap::Capture;
use std::mem::size_of;

const PACKET_HEADER_SIZE: usize = 42; // Ethernet + IPv4 header size

#[test]
fn velodyne_test() -> Fallible<()> {
    let mut packets = vec![];

    let mut cap = Capture::from_file("test_files/velodyne_example.pcap")?;
    cap.filter("udp")?;

    while let Ok(packet) = cap.next() {
        let buffer_len = packet.header.len as usize - PACKET_HEADER_SIZE;
        if buffer_len != size_of::<VelodynePacket>() {
            continue;
        }

        let mut buffer = Box::new([0u8; size_of::<VelodynePacket>()]);
        buffer.copy_from_slice(&packet.data[PACKET_HEADER_SIZE..]);
        let lidar_packet = VelodynePacket::from_buffer(*buffer);

        packets.push(lidar_packet);
    }

    for (idx, packet) in packets.iter().enumerate() {
        let ts = packet.gps_timestamp;
        println!("No. {}, gps_timestamp = {}", idx, ts);
    }

    Ok(())
}

#[test]
fn velodyne_scan() -> Fallible<()> {
    let mut cap = Capture::from_file("test_files/velodyne_example.pcap")?;
    cap.filter("udp")?;

    while let Ok(packet) = cap.next() {
        let buffer_len = packet.header.len as usize - PACKET_HEADER_SIZE;
        if buffer_len != size_of::<VelodynePacket>() {
            continue;
        }

        let mut buffer = Box::new([0u8; size_of::<VelodynePacket>()]);
        buffer.copy_from_slice(&packet.data[PACKET_HEADER_SIZE..]);
        let lidar_packet = VelodynePacket::from_buffer(*buffer);

        let points = lidar_packet.firings
            .iter()
            .map(|firing| {
                let azimuth_angle = firing.azimuth_angle();
                let column_points = firing.laster_returns
                    .iter()
                    .zip(ALTITUDE_DEGREES.iter())
                    .map(|(laster_return, altitude_deg)| {
                        let altitude_angle = std::f64::consts::PI * altitude_deg / 180.0;
                        let distance = laster_return.meter_distance();

                        // TODO: check formula
                        // https://github.com/PointCloudLibrary/pcl/blob/master/io/src/hdl_grabber.cpp#L396
                        let x = distance * altitude_angle.cos() * azimuth_angle.cos();
                        let y = distance * altitude_angle.cos() * azimuth_angle.sin();
                        let z = distance * altitude_angle.sin();

                        (x, y, z)
                    })
                    .collect::<Vec<_>>();
                column_points
            })
            .collect::<Vec<_>>();
        // TODO
    }

    Ok(())
}
