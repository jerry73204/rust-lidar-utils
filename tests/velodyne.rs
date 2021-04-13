#![cfg(feature = "velodyne-test")]

use anyhow::{ensure, Result};
use itertools::izip;
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
    cap.filter("udp")?;

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
        let config = Config::vlp_16_strongest_return();
        let mut converter = Vlp16_Strongest_FrameConverter::from_config(config);
        data_packets.iter().try_for_each(|packet| -> Result<_> {
            converter.convert(packet)?;
            Ok(())
        })?;
    }

    Ok(())
}

#[test]
#[cfg(feature = "pcap")]
fn velodyne_vlp_32_pcap_file() -> Result<()> {
    let mut cap = Capture::from_file("test_files/velodyne_vlp32.pcap")?;
    cap.filter("udp")?;

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
        let config = Config::vlp_32c_strongest_return();
        let mut converter = Vlp32_Strongest_FrameConverter::from_config(config);
        data_packets.iter().try_for_each(|packet| -> Result<_> {
            converter.convert(packet)?;
            Ok(())
        })?;
    }

    Ok(())
}
