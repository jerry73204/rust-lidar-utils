extern crate failure;
extern crate lidar_buffer;
extern crate pcap;
extern crate serde_json;
#[macro_use]
extern crate log;
extern crate pretty_env_logger;

use failure::Fallible;
use lidar_buffer::ouster::{Config, Helper, Packet as OusterPacket};
use pcap::Capture;

#[test]
#[cfg(feature = "enable-pcap")]
fn ouster_pcap_file() -> Fallible<()> {
    let mut packets = vec![];

    let mut cap = Capture::from_file("test_files/ouster_example.pcap")?;
    cap.filter("udp")?;

    while let Ok(packet) = cap.next() {
        let lidar_packet = OusterPacket::from_pcap(&packet)?;
        packets.push(lidar_packet);
    }

    for (idx, packet) in packets.iter().enumerate() {
        let ts = packet.columns[0].timestamp;
        println!("No. {}, timestamp = {}", idx, ts);
    }

    Ok(())
}

#[test]
#[cfg(feature = "enable-pcap")]
fn ouster_scan() -> Fallible<()> {
    pretty_env_logger::init();

    // Load config
    let config = Config::from_path("test_files/ouster_example.json")?;
    let helper = Helper::from_config(config);

    // Load pcap file
    let mut cap = Capture::from_file("/home/jerry73204/Downloads/lombard_street_OS1.pcap")?;
    cap.filter("udp")?;

    let mut curr_fid = None;
    let mut expect_mid = None;

    let mut frames = vec![];
    let mut frame_points = vec![];

    while let Ok(packet) = cap.next() {
        let lidar_packet = OusterPacket::from_pcap(&packet)?;

        for column in lidar_packet.columns.iter() {
            // Skip invalid columns
            if !column.valid() {
                warn!("Invalid column detected");
                continue;
            }

            // Skip columns with late frame ids
            if let Some(orig_fid) = curr_fid {
                if column.frame_id < orig_fid {
                    warn!("Column with inconsecutive frame id detected");
                    continue;
                }
            }

            // Update frame id and expected measurement id
            let new_fid = column.frame_id;
            let new_mid = column.measurement_id;

            match curr_fid {
                Some(orig_fid) => {
                    if orig_fid == new_fid {
                        expect_mid = match expect_mid {
                            Some(orig_mid) => Some(orig_mid + 1),
                            None => Some(new_mid),
                        };
                    } else {
                        if orig_fid + 1 != new_fid {
                            warn!("Skipped frame id detected");
                        }
                        frames.push((orig_fid, frame_points));
                        frame_points = vec![];
                        curr_fid = Some(new_fid);
                        expect_mid = Some(0);
                    }
                }
                None => {
                    curr_fid = Some(column.frame_id);
                    expect_mid = Some(new_mid);
                }
            };

            // Check measurement id
            match expect_mid {
                Some(mid) => {
                    if mid != new_mid {
                        warn!("Unordered measurement id detected");
                    }
                }
                None => unreachable!(),
            }

            // Construct point cloud
            let mut column_points = helper.column_to_points(&column)?;
            frame_points.append(&mut column_points);
        }
    }

    Ok(())
}
