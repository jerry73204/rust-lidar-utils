// #![cfg(feature = "velodyne-test")]

use anyhow::{ensure, Result};
use itertools::izip;
use lidar_utils::velodyne::consts;
use lidar_utils::velodyne::{
    Config, DataPacket, FrameConverter, PointCloudConverter, PositionPacket,
    Vlp16_Strongest_FrameConverter, Vlp16_Strongest_PcdConverter, Vlp32_Strongest_FrameConverter,
    Vlp32_Strongest_PcdConverter,
};
use pcap::Capture;
use std::mem;

const UDP_HEADER_SIZE: usize = 42;

#[test]
#[cfg(feature = "pcap")]
fn velodyne_vlp_16_pcap_file() -> Result<()> {
    let mut cap = Capture::from_file("test_files/velodyne_vlp16.pcap")?;
    cap.filter("udp", true)?;

    let mut data_packets = vec![];
    let mut position_packets = vec![];

    while let Ok(packet) = cap.next() {
        if packet.data.len() == mem::size_of::<DataPacket>() + UDP_HEADER_SIZE {
            data_packets.push(DataPacket::from_pcap(&packet)?);
        } else if packet.data.len() == mem::size_of::<PositionPacket>() + UDP_HEADER_SIZE {
            position_packets.push(PositionPacket::from_pcap(&packet)?);
        }
    }

    // timestamp test
    {
        let is_timestamp_valid = izip!(data_packets.iter(), data_packets.iter().skip(1))
            .all(|(former, latter)| former.timestamp < latter.timestamp);

        ensure!(is_timestamp_valid, "invalid timestamp detected");
    }

    // convert to point cloud
    {
        let config = Config::vlp_16_strongest_return();
        let mut converter = Vlp16_Strongest_PcdConverter::from_config(config);
        data_packets.iter().try_for_each(|packet| -> Result<_> {
            converter.convert(packet)?;
            Ok(())
        })?;
    }

    // convert to frames
    {
        let beam_num = 16;
        let config = Config::vlp_16_strongest_return();
        let mut converter = Vlp16_Strongest_FrameConverter::from_config(config);
        data_packets.iter().try_for_each(|packet| -> Result<_> {
            let frame_return = converter.convert(packet).unwrap();
            if frame_return.len() >= 1 {
                frame_return.iter().for_each(|frame| {
                    // check if azimuth is in order
                    for i in 1..(frame.len() / beam_num) - 1 {
                        assert!(
                            frame[i * beam_num].original_azimuth_angle
                                < frame[(i + 1) * beam_num].original_azimuth_angle
                        )
                    }
                });

                //check if elevion(laser id) is in order
                let deg = consts::VLP_16_ELEVAION_INDEX;
                frame_return.iter().for_each(|frame| {
                    for i in 0..frame.len() - 1 {
                        assert!(
                            (deg[frame[i].laser_id as usize] < deg[frame[i + 1].laser_id as usize])
                                || (deg[frame[i].laser_id as usize] == 15
                                    && deg[frame[i + 1].laser_id as usize] == 0)
                        );
                    }
                });
            }
            Ok(())
        })?;
    }

    Ok(())
}

#[test]
#[cfg(feature = "pcap")]
fn velodyne_vlp_32_pcap_file() -> Result<()> {
    let mut cap = Capture::from_file("test_files/velodyne_vlp32.pcap")?;
    cap.filter("udp", true)?;

    let mut data_packets = vec![];
    let mut position_packets = vec![];

    while let Ok(packet) = cap.next() {
        if packet.data.len() == mem::size_of::<DataPacket>() + UDP_HEADER_SIZE {
            data_packets.push(DataPacket::from_pcap(&packet)?);
        } else if packet.data.len() == mem::size_of::<PositionPacket>() + UDP_HEADER_SIZE {
            position_packets.push(PositionPacket::from_pcap(&packet)?);
        }
    }

    // timestamp test
    {
        let is_timestamp_valid = izip!(data_packets.iter(), data_packets.iter().skip(1))
            .all(|(former, latter)| former.timestamp < latter.timestamp);

        ensure!(is_timestamp_valid, "invalid timestamp detected");
    }

    // convert to point cloud
    {
        let config = Config::vlp_32c_strongest_return();
        let mut converter = Vlp32_Strongest_PcdConverter::from_config(config);
        data_packets.iter().try_for_each(|packet| -> Result<_> {
            converter.convert(packet)?;
            Ok(())
        })?;
    }

    // convert to frames
    {
        let beam_num = 32;
        let config = Config::vlp_32c_strongest_return();
        let mut converter = Vlp32_Strongest_FrameConverter::from_config(config);
        data_packets.iter().try_for_each(|packet| -> Result<_> {
            let frame_return = converter.convert(packet).unwrap();
            if frame_return.len() >= 1 {
                frame_return.iter().for_each(|frame| {
                    // check if azimuth is in order
                    for i in 1..(frame.len() / beam_num) - 1 {
                        assert!(
                            frame[i * beam_num].original_azimuth_angle
                                < frame[(i + 1) * beam_num].original_azimuth_angle
                        )
                    }
                });

                //check if elevion(laser id) is in order
                let deg = consts::VLP_32C_ELEVAION_INDEX;
                frame_return.iter().for_each(|frame| {
                    for i in 0..frame.len() - 1 {
                        assert!(
                            (deg[frame[i].laser_id as usize] < deg[frame[i + 1].laser_id as usize])
                                || (deg[frame[i].laser_id as usize] == 31
                                    && deg[frame[i + 1].laser_id as usize] == 0)
                        );
                    }
                });
            }
            Ok(())
        })?;
    }

    Ok(())
}
