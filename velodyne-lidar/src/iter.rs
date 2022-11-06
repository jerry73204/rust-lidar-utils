use crate::{
    batcher::Batcher,
    firing_xyz::{FiringXyzD16, FiringXyzD32, FiringXyzS16, FiringXyzS32},
    frame_xyz::{FrameXyz, FrameXyzD16, FrameXyzD32, FrameXyzS16, FrameXyzS32},
    kinds::{Format, FormatKind},
    Config, Config16, Config32, Packet,
};
use anyhow::{anyhow, Result};
use log::warn;

pub type FrameXyzIter<'a> = Box<dyn Iterator<Item = FrameXyz> + Send + 'a>;

fn audit_format(packet_format: Option<Format>, config_format: Format) {
    match packet_format {
        Some(packet_format) => {
            if packet_format != config_format {
                warn!(
                    "format mismatch: received a packet with format {:?}, but is treated as {:?}",
                    packet_format, config_format
                );
            }
        }
        None => warn!(
            "unable to determine the format of a packet, assume the format to be {:?}",
            config_format
        ),
    }
}

pub fn packet_to_frame_xyz_iter<'a, I>(config: Config, packets: I) -> Result<FrameXyzIter<'a>>
where
    I: IntoIterator<Item = Packet> + 'a,
    I::IntoIter: Send,
{
    use FormatKind as K;

    let config_kinds = config
        .try_into_kind()
        .map_err(|_| anyhow!("invalid configuration"))?;

    let iter: FrameXyzIter = match config_kinds {
        K::Single16(config) => {
            Box::new(packet_to_frame_xyz_iter_s16(config, packets).map(K::from_s16))
        }
        K::Single32(config) => {
            Box::new(packet_to_frame_xyz_iter_s32(config, packets).map(K::from_s32))
        }
        K::Dual16(config) => {
            Box::new(packet_to_frame_xyz_iter_d16(config, packets).map(K::from_d16))
        }
        K::Dual32(config) => {
            Box::new(packet_to_frame_xyz_iter_d32(config, packets).map(K::from_d32))
        }
    };

    Ok(iter)
}

pub fn packet_to_frame_xyz_iter_s16<I>(
    config: Config16,
    packets: I,
) -> impl Iterator<Item = FrameXyzS16> + Send
where
    I: IntoIterator<Item = Packet>,
    I::IntoIter: Send,
{
    let batcher = Batcher::new();

    packets
        .into_iter()
        .filter_map(|packet| packet.try_into_data().ok())
        .map(move |packet| {
            audit_format(packet.try_format(), config.format());

            let firings: Vec<FiringXyzS16> = packet
                .firing_block_iter_s16()
                .map(|block| block.to_firing_xyz(&config))
                .collect();
            firings
        })
        .scan(batcher, |batcher, firings| {
            let frames: Vec<_> = batcher
                .push_many(firings)
                .map(|firings| FrameXyzS16 { firings })
                .collect();
            Some(frames)
        })
        .flatten()
}

pub fn packet_to_frame_xyz_iter_s32<I>(
    config: Config32,
    packets: I,
) -> impl Iterator<Item = FrameXyzS32> + Send
where
    I: IntoIterator<Item = Packet>,
    I::IntoIter: Send,
{
    let batcher = Batcher::new();

    packets
        .into_iter()
        .filter_map(|packet| packet.try_into_data().ok())
        .map(move |packet| {
            audit_format(packet.try_format(), config.format());

            let firings: Vec<FiringXyzS32> = packet
                .firing_block_iter_s32()
                .map(|block| block.to_firing_xyz(&config))
                .collect();
            firings
        })
        .scan(batcher, |batcher, firings| {
            let frames: Vec<_> = batcher
                .push_many(firings)
                .map(|firings| FrameXyzS32 { firings })
                .collect();
            Some(frames)
        })
        .flatten()
}

pub fn packet_to_frame_xyz_iter_d16<I>(
    config: Config16,
    packets: I,
) -> impl Iterator<Item = FrameXyzD16> + Send
where
    I: IntoIterator<Item = Packet>,
    I::IntoIter: Send,
{
    let batcher = Batcher::new();

    packets
        .into_iter()
        .filter_map(|packet| packet.try_into_data().ok())
        .map(move |packet| {
            audit_format(packet.try_format(), config.format());

            let firings: Vec<FiringXyzD16> = packet
                .firing_block_iter_d16()
                .map(|block| block.to_firing_xyz(&config))
                .collect();
            firings
        })
        .scan(batcher, |batcher, firings| {
            let frames: Vec<_> = batcher
                .push_many(firings)
                .map(|firings| FrameXyzD16 { firings })
                .collect();
            Some(frames)
        })
        .flatten()
}

pub fn packet_to_frame_xyz_iter_d32<I>(
    config: Config32,
    packets: I,
) -> impl Iterator<Item = FrameXyzD32> + Send
where
    I: IntoIterator<Item = Packet>,
    I::IntoIter: Send,
{
    let batcher = Batcher::new();

    packets
        .into_iter()
        .filter_map(|packet| packet.try_into_data().ok())
        .map(move |packet| {
            audit_format(packet.try_format(), config.format());

            let firings: Vec<FiringXyzD32> = packet
                .firing_block_iter_d32()
                .map(|block| block.to_firing_xyz(&config))
                .collect();
            firings
        })
        .scan(batcher, |batcher, firings| {
            let frames: Vec<_> = batcher
                .push_many(firings)
                .map(|firings| FrameXyzD32 { firings })
                .collect();
            Some(frames)
        })
        .flatten()
}
