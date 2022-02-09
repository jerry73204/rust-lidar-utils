use anyhow::{ensure, Result};
use itertools::izip;
use pcap::Capture;
use velodyne_lidar::{config::Config, consts, pcap::PcapFileReader, DataPacket};

const UDP_HEADER_SIZE: usize = 42;

#[test]
fn velodyne_vlp_16_pcap_file() -> Result<()> {
    let data_packets: Vec<_> = PcapFileReader::open("test_files/velodyne_vlp16.pcap")?
        .filter_map(|packet| packet.try_into_data().ok())
        .collect();

    // timestamp test
    {
        let is_timestamp_valid = izip!(data_packets.iter(), data_packets.iter().skip(1))
            .all(|(former, latter)| former.timestamp < latter.timestamp);

        ensure!(is_timestamp_valid, "invalid timestamp detected");
    }

    //check if elevation angle is in order
    {
        let original = consts::vlp_16::ELEVAION_DEGREES;
        let mut sort = consts::vlp_16::ELEVAION_DEGREES;
        sort.sort_by(|a, b| b.partial_cmp(a).unwrap());

        let idx_order = consts::vlp_16::ELEVAION_INDEX;

        for i in 0..idx_order.len() {
            assert!(sort[i] == original[idx_order[i]]);
        }
    }

    // convert to point cloud
    {
        let converter = Config::new_vlp_16_strongest()
            .build_converter()
            .unwrap()
            .into_single16();
        let _: Vec<_> = data_packets
            .iter()
            .flat_map(|packet| converter.packet_to_firing_xyz_iter(packet))
            .collect();
    }

    // convert to frames
    // {
    //     let beam_num = 16;
    //     let mut converter = Config::new_vlp_16_strongest().unwrap()
    //         .build_frame_converter()
    //         .into_single16();
    //     data_packets.iter().try_for_each(|packet| -> Result<_> {
    //         let frame_return = converter.convert(packet);
    //         if let Some(frame) = frame_return {
    //             // check if azimuth is in order
    //             for i in 1..((frame.data.len() / beam_num) - 1) {
    //                 assert!(
    //                     frame.data[i * beam_num].original_azimuth_angle
    //                         < frame.data[(i + 1) * beam_num].original_azimuth_angle
    //                 )
    //             }
    //             //check if elevion(laser id) is in order
    //             let deg = consts::vlp_16::ELEVAION_INDEX;
    //             for i in 0..(frame.data.len() - 1) {
    //                 assert!(
    //                     (deg[frame.data[i].laser_id as usize]
    //                         < deg[frame.data[i + 1].laser_id as usize])
    //                         || (deg[frame.data[i].laser_id as usize] == 15
    //                             && deg[frame.data[i + 1].laser_id as usize] == 0)
    //                 );
    //             }
    //         }

    //         Ok(())
    //     })?;
    // }

    Ok(())
}

#[test]
fn velodyne_vlp_32_pcap_file() -> Result<()> {
    let mut cap = Capture::from_file("test_files/velodyne_vlp32.pcap")?;
    cap.filter("udp", true)?;

    let data_packets: Vec<_> = itertools::unfold(cap, |cap| {
        Some(loop {
            let packet = cap.next().ok()?;
            let slice = &packet.data[UDP_HEADER_SIZE..];

            if let Ok(packet) = DataPacket::from_slice(slice) {
                break *packet;
            }
        })
    })
    .collect();

    // timestamp test
    {
        let is_timestamp_valid = izip!(data_packets.iter(), data_packets.iter().skip(1))
            .all(|(former, latter)| former.timestamp < latter.timestamp);

        ensure!(is_timestamp_valid, "invalid timestamp detected");
    }

    //check if elevation angle is in order
    {
        let original = consts::vlp_32c::ELEVAION_DEGREES;
        let mut sort = consts::vlp_32c::ELEVAION_DEGREES;
        sort.sort_by(|a, b| b.partial_cmp(a).unwrap());

        let idx_order = consts::vlp_32c::ELEVAION_INDEX;

        for i in 0..idx_order.len() {
            assert!(sort[i] == original[idx_order[i]]);
        }
    }

    // convert to point cloud
    {
        let converter = Config::new_vlp_32c_strongest().build_converter().unwrap();
        converter
            .packet_iter_to_frame_xyz_iter(data_packets)
            .count();
    }

    // convert to frames
    // {
    //     let beam_num = 32;
    //     let mut converter = Config::vlp_32c_strongest().build_frame_converter();
    //     data_packets.iter().try_for_each(|packet| -> Result<_> {
    //         let frame_return = converter.convert(packet);

    //         if let Some(frame) = frame_return {
    //             // check if azimuth is in order
    //             for i in 1..((frame.data.len() / beam_num) - 1) {
    //                 assert!(
    //                     frame.data[i * beam_num].original_azimuth_angle
    //                         < frame.data[(i + 1) * beam_num].original_azimuth_angle
    //                 )
    //             }
    //             //check if elevion(laser id) is in order
    //             let deg = consts::vlp_32c::ELEVAION_INDEX;
    //             for i in 0..(frame.data.len() - 1) {
    //                 assert!(
    //                     (deg[frame.data[i].laser_id as usize]
    //                         < deg[frame.data[i + 1].laser_id as usize])
    //                         || (deg[frame.data[i].laser_id as usize] == 31
    //                             && deg[frame.data[i + 1].laser_id as usize] == 0)
    //                 );
    //             }
    //         }

    //         Ok(())
    //     })?;
    // }

    Ok(())
}
