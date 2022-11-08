//! Iterator conversion functions.

use crate::{
    batcher::Batcher,
    firing_xyz::{FiringXyzD16, FiringXyzD32, FiringXyzS16, FiringXyzS32},
    frame_xyz::{FrameXyz, FrameXyzD16, FrameXyzD32, FrameXyzS16, FrameXyzS32},
    kinds::{Format, FormatKind},
    Config, Config16, Config32, DataPacket,
};
use anyhow::{anyhow, Result};
use log::warn;

pub(crate) type FrameXyzIter<'a> = Box<dyn Iterator<Item = FrameXyz> + Send + 'a>;

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

/// Converts an iterator of packets to an iterator of [FrameXyz].
pub fn packet_to_frame_xyz<'a, I>(config: Config, packets: I) -> Result<FrameXyzIter<'a>>
where
    I: IntoIterator<Item = DataPacket> + 'a,
    I::IntoIter: Send,
{
    use FormatKind as K;

    let config_kinds = config
        .try_into_kind()
        .map_err(|_| anyhow!("invalid configuration"))?;

    let iter: FrameXyzIter = match config_kinds {
        K::Single16(config) => Box::new(packet_to_frame_xyz_s16(config, packets).map(K::from_s16)),
        K::Single32(config) => Box::new(packet_to_frame_xyz_s32(config, packets).map(K::from_s32)),
        K::Dual16(config) => Box::new(packet_to_frame_xyz_d16(config, packets).map(K::from_d16)),
        K::Dual32(config) => Box::new(packet_to_frame_xyz_d32(config, packets).map(K::from_d32)),
    };

    Ok(iter)
}

macro_rules! declare_packet_to_frame_xyz_fn {
    ($name:ident, $config:ident, $firing:ident, $frame:ident, $iter_fn:ident) => {
        pub fn $name<I>(config: $config, packets: I) -> impl Iterator<Item = $frame> + Send
        where
            I: IntoIterator<Item = DataPacket>,
            I::IntoIter: Send,
        {
            let batcher = Batcher::new();

            packets
                .into_iter()
                .map(move |packet| {
                    audit_format(packet.try_format(), config.format());

                    let firings: Vec<$firing> = packet
                        .$iter_fn()
                        .map(|block| block.to_firing_xyz(&config))
                        .collect();
                    firings
                })
                .scan(batcher, |batcher, firings| {
                    let frames: Vec<_> = batcher
                        .push_many(firings)
                        .map(|firings| $frame { firings })
                        .collect();
                    Some(frames)
                })
                .flatten()
        }
    };
}

declare_packet_to_frame_xyz_fn!(
    packet_to_frame_xyz_s16,
    Config16,
    FiringXyzS16,
    FrameXyzS16,
    firing_block_iter_s16
);
declare_packet_to_frame_xyz_fn!(
    packet_to_frame_xyz_s32,
    Config32,
    FiringXyzS32,
    FrameXyzS32,
    firing_block_iter_s32
);
declare_packet_to_frame_xyz_fn!(
    packet_to_frame_xyz_d16,
    Config16,
    FiringXyzD16,
    FrameXyzD16,
    firing_block_iter_d16
);
declare_packet_to_frame_xyz_fn!(
    packet_to_frame_xyz_d32,
    Config32,
    FiringXyzD32,
    FrameXyzD32,
    firing_block_iter_d32
);
