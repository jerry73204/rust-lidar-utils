use crate::{
    common::*,
    utils::{AngleExt as _, DurationExt as _},
    velodyne::{
        config::{Config, LaserParameter},
        consts::{CHANNEL_PERIOD, FIRING_PERIOD},
        firing::{FiringDual16, FiringDual32, FiringFormat, FiringSingle16, FiringSingle32},
        firing_xyz::{FiringXyzDual16, FiringXyzDual32, FiringXyzSingle16, FiringXyzSingle32},
        firing_xyz_iter::{
            FiringXyzDual16Iter, FiringXyzDual32Iter, FiringXyzIter, FiringXyzSingle16Iter,
            FiringXyzSingle32Iter,
        },
        packet::DataPacket,
        point::{Measurement, MeasurementDual, PointDual, PointSingle},
    },
};

macro_rules! declare_converter {
    ($name:ident, $size:expr, $firing:ident, $firing_xyz:ident, $firing_xyz_iter:ident, $convert_fn:ident, $firing_method:ident) => {
        #[derive(Debug, Clone)]
        pub struct $name {
            pub(crate) lasers: [LaserParameter; $size],
            pub(crate) distance_resolution: Length,
        }

        impl $name {
            pub fn convert_firing<'a>(&'a self, firing: $firing<'a>) -> $firing_xyz {
                $convert_fn(&firing, self.distance_resolution, &self.lasers)
            }

            pub fn convert_firings<'a, I>(
                &'a self,
                firings: I,
            ) -> $firing_xyz_iter<impl Iterator<Item = $firing_xyz> + 'a>
            where
                I: IntoIterator<Item = $firing<'a>>,
                I::IntoIter: 'a,
            {
                let iter = firings
                    .into_iter()
                    .map(|firing| self.convert_firing(firing));
                $firing_xyz_iter(iter)
            }

            pub fn convert_packet<'a>(
                &'a self,
                packet: &'a DataPacket,
            ) -> $firing_xyz_iter<impl Iterator<Item = $firing_xyz> + 'a> {
                self.convert_firings(packet.$firing_method())
            }

            pub fn convert_packets<'a, P, I>(
                &'a self,
                packets: I,
            ) -> $firing_xyz_iter<impl Iterator<Item = $firing_xyz> + 'a>
            where
                I: IntoIterator<Item = P>,
                I::IntoIter: 'a,
                P: Borrow<DataPacket> + 'a,
            {
                let iter = packets.into_iter().flat_map(|packet| {
                    let firings: Vec<_> = self.convert_packet(packet.borrow()).collect();
                    firings
                });
                $firing_xyz_iter(iter)
            }
        }
    };
}

declare_converter!(
    PcdConverterSingle16,
    16,
    FiringSingle16,
    FiringXyzSingle16,
    FiringXyzSingle16Iter,
    firing_single_16_to_xyz,
    single_16_firings
);

declare_converter!(
    PcdConverterSingle32,
    32,
    FiringSingle32,
    FiringXyzSingle32,
    FiringXyzSingle32Iter,
    firing_single_32_to_xyz,
    single_32_firings
);

declare_converter!(
    PcdConverterDual16,
    16,
    FiringDual16,
    FiringXyzDual16,
    FiringXyzDual16Iter,
    firing_dual_16_to_xyz,
    dual_16_firings
);

declare_converter!(
    PcdConverterDual32,
    32,
    FiringDual32,
    FiringXyzDual32,
    FiringXyzDual32Iter,
    firing_dual_32_to_xyz,
    dual_32_firings
);

pub use kind::*;
mod kind {

    use super::*;

    #[derive(Debug, Clone)]
    pub enum PcdConverterKind {
        Single16(PcdConverterSingle16),
        Single32(PcdConverterSingle32),
        Dual16(PcdConverterDual16),
        Dual32(PcdConverterDual32),
    }

    impl From<PcdConverterDual32> for PcdConverterKind {
        fn from(v: PcdConverterDual32) -> Self {
            Self::Dual32(v)
        }
    }

    impl From<PcdConverterDual16> for PcdConverterKind {
        fn from(v: PcdConverterDual16) -> Self {
            Self::Dual16(v)
        }
    }

    impl From<PcdConverterSingle32> for PcdConverterKind {
        fn from(v: PcdConverterSingle32) -> Self {
            Self::Single32(v)
        }
    }

    impl From<PcdConverterSingle16> for PcdConverterKind {
        fn from(v: PcdConverterSingle16) -> Self {
            Self::Single16(v)
        }
    }

    impl PcdConverterKind {
        pub fn firing_format(&self) -> FiringFormat {
            use FiringFormat as F;

            match self {
                Self::Single16(_) => F::Single16,
                Self::Single32(_) => F::Single32,
                Self::Dual16(_) => F::Dual16,
                Self::Dual32(_) => F::Dual32,
            }
        }

        pub fn convert_packet<'a>(
            &'a self,
            packet: &'a DataPacket,
        ) -> FiringXyzIter<
            impl Iterator<Item = FiringXyzSingle16> + 'a,
            impl Iterator<Item = FiringXyzSingle32> + 'a,
            impl Iterator<Item = FiringXyzDual16> + 'a,
            impl Iterator<Item = FiringXyzDual32> + 'a,
        > {
            match self {
                Self::Single16(conv) => conv.convert_packet(packet).into(),
                Self::Single32(conv) => conv.convert_packet(packet).into(),
                Self::Dual16(conv) => conv.convert_packet(packet).into(),
                Self::Dual32(conv) => conv.convert_packet(packet).into(),
            }
        }

        pub fn convert_packets<'a, I>(
            &'a self,
            packets: I,
        ) -> FiringXyzIter<
            impl Iterator<Item = FiringXyzSingle16> + 'a,
            impl Iterator<Item = FiringXyzSingle32> + 'a,
            impl Iterator<Item = FiringXyzDual16> + 'a,
            impl Iterator<Item = FiringXyzDual32> + 'a,
        >
        where
            I: IntoIterator<Item = &'a DataPacket> + 'a,
            I::IntoIter: 'a,
        {
            match self {
                Self::Single16(conv) => conv.convert_packets(packets).into(),
                Self::Single32(conv) => conv.convert_packets(packets).into(),
                Self::Dual16(conv) => conv.convert_packets(packets).into(),
                Self::Dual32(conv) => conv.convert_packets(packets).into(),
            }
        }

        pub fn from_config(config: Config) -> Result<Self> {
            use FiringFormat as F;

            let firing_format = config.firing_format();
            let Config {
                lasers,
                distance_resolution,
                ..
            } = config;

            let err = || format_err!("invalid laser parameters");

            Ok(match firing_format {
                F::Single16 => PcdConverterSingle16 {
                    lasers: lasers.try_into().map_err(|_| err())?,
                    distance_resolution,
                }
                .into(),
                F::Dual16 => PcdConverterDual16 {
                    lasers: lasers.try_into().map_err(|_| err())?,
                    distance_resolution,
                }
                .into(),
                F::Single32 => PcdConverterSingle32 {
                    lasers: lasers.try_into().map_err(|_| err())?,
                    distance_resolution,
                }
                .into(),
                F::Dual32 => PcdConverterDual32 {
                    lasers: lasers.try_into().map_err(|_| err())?,
                    distance_resolution,
                }
                .into(),
            })
        }

        pub fn try_into_single16(self) -> Result<PcdConverterSingle16, Self> {
            if let Self::Single16(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_single32(self) -> Result<PcdConverterSingle32, Self> {
            if let Self::Single32(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_dual16(self) -> Result<PcdConverterDual16, Self> {
            if let Self::Dual16(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn try_into_dual32(self) -> Result<PcdConverterDual32, Self> {
            if let Self::Dual32(v) = self {
                Ok(v)
            } else {
                Err(self)
            }
        }

        pub fn into_single16(self) -> PcdConverterSingle16 {
            self.try_into_single16().unwrap()
        }

        pub fn into_single32(self) -> PcdConverterSingle32 {
            self.try_into_single32().unwrap()
        }

        pub fn into_dual16(self) -> PcdConverterDual16 {
            self.try_into_dual16().unwrap()
        }

        pub fn into_dual32(self) -> PcdConverterDual32 {
            self.try_into_dual32().unwrap()
        }
    }
}

pub use functions::*;
mod functions {
    use super::*;

    pub fn firing_single_16_to_xyz(
        firing: &FiringSingle16,
        distance_resolution: Length,
        lasers: &[LaserParameter; 16],
    ) -> FiringXyzSingle16 {
        let FiringSingle16 {
            time: firing_time,
            ref azimuth_range,
            channels,
            block,
            ..
        } = *firing;

        let channel_times =
            iter::successors(Some(firing_time), |&prev| Some(prev + CHANNEL_PERIOD));

        let points: Vec<_> = izip!(0.., channel_times, channels, lasers)
            .map(move |(laser_id, channel_time, channel, laser)| {
                let ratio = (channel_time - firing_time).div_duration(FIRING_PERIOD);
                let LaserParameter {
                    elevation,
                    azimuth_offset,
                    vertical_offset,
                    horizontal_offset,
                } = *laser;

                // clockwise angle with origin points to front of sensor
                let azimuth = {
                    let azimuth = azimuth_range.start
                        + ((azimuth_range.end - azimuth_range.start) * ratio)
                        + azimuth_offset;
                    azimuth.wrap_to_2pi()
                };
                let distance = distance_resolution * channel.distance as f64;
                let xyz = spherical_to_xyz(
                    distance,
                    elevation,
                    azimuth,
                    vertical_offset,
                    horizontal_offset,
                );

                PointSingle {
                    laser_id,
                    time: channel_time,
                    azimuth,
                    measurement: Measurement {
                        distance,
                        intensity: channel.intensity,
                        xyz,
                    },
                }
            })
            .collect();
        let points: [_; 16] = points.try_into().unwrap_or_else(|_| unreachable!());

        FiringXyzSingle16 {
            time: firing_time,
            azimuth_count: block.azimuth_count,
            azimuth_range: azimuth_range.clone(),
            points,
        }
    }

    pub fn firing_single_32_to_xyz(
        firing: &FiringSingle32,
        distance_resolution: Length,
        lasers: &[LaserParameter; 32],
    ) -> FiringXyzSingle32 {
        let FiringSingle32 {
            time: firing_time,
            ref azimuth_range,
            channels,
            block,
            ..
        } = *firing;

        let channel_times =
            iter::successors(Some(firing_time), |&prev| Some(prev + CHANNEL_PERIOD))
                .flat_map(|time| [time, time]);

        let points: Vec<_> = izip!(0.., channel_times, channels, lasers)
            .map(move |(laser_id, channel_time, channel, laser)| {
                // let timestamp = lower_timestamp + CHANNEL_PERIOD.mul_f64((channel_idx / 2) as f64);

                let ratio = (channel_time - firing_time).div_duration(FIRING_PERIOD);
                let LaserParameter {
                    elevation,
                    azimuth_offset,
                    vertical_offset,
                    horizontal_offset,
                } = *laser;

                // clockwise angle with origin points to front of sensor
                let azimuth = {
                    let azimuth = azimuth_range.start
                        + ((azimuth_range.end - azimuth_range.start) * ratio)
                        + azimuth_offset;
                    azimuth.wrap_to_2pi()
                };
                let distance = distance_resolution * channel.distance as f64;
                let xyz = spherical_to_xyz(
                    distance,
                    elevation,
                    azimuth,
                    vertical_offset,
                    horizontal_offset,
                );

                PointSingle {
                    laser_id,
                    time: channel_time,
                    azimuth,
                    measurement: Measurement {
                        distance,
                        intensity: channel.intensity,
                        xyz,
                    },
                }
            })
            .collect();

        let points = points.try_into().unwrap_or_else(|_| unreachable!());

        FiringXyzSingle32 {
            time: firing_time,
            azimuth_count: block.azimuth_count,
            azimuth_range: azimuth_range.clone(),
            points,
        }
    }

    pub fn firing_dual_16_to_xyz(
        firing: &FiringDual16,
        distance_resolution: Length,
        lasers: &[LaserParameter; 16],
    ) -> FiringXyzDual16 {
        let FiringDual16 {
            time: firing_time,
            ref azimuth_range,
            channels_strongest,
            channels_last,
            block_strongest,
            ..
        } = *firing;

        let channel_times =
            iter::successors(Some(firing_time), |&prev| Some(prev + CHANNEL_PERIOD));

        let points: Vec<_> = izip!(
            0..,
            channel_times,
            channels_strongest,
            channels_last,
            lasers
        )
        .map(
            move |(laser_id, channel_time, channel_strongest, channel_last, laser)| {
                let ratio = (channel_time - firing_time).div_duration(FIRING_PERIOD);
                let LaserParameter {
                    elevation,
                    azimuth_offset,
                    vertical_offset,
                    horizontal_offset,
                } = *laser;

                // clockwise angle with origin points to front of sensor
                let azimuth = {
                    let azimuth = azimuth_range.start
                        + ((azimuth_range.end - azimuth_range.start) * ratio)
                        + azimuth_offset;
                    azimuth.wrap_to_2pi()
                };
                let distance_strongest = distance_resolution * channel_strongest.distance as f64;
                let distance_last = distance_resolution * channel_last.distance as f64;

                let xyz_strongest = spherical_to_xyz(
                    distance_strongest,
                    elevation,
                    azimuth,
                    vertical_offset,
                    horizontal_offset,
                );
                let xyz_last = spherical_to_xyz(
                    distance_last,
                    elevation,
                    azimuth,
                    vertical_offset,
                    horizontal_offset,
                );

                PointDual {
                    laser_id,
                    time: channel_time,
                    azimuth,
                    measurements: MeasurementDual {
                        strongest: Measurement {
                            distance: distance_strongest,
                            intensity: channel_strongest.intensity,
                            xyz: xyz_strongest,
                        },
                        last: Measurement {
                            distance: distance_last,
                            intensity: channel_last.intensity,
                            xyz: xyz_last,
                        },
                    },
                }
            },
        )
        .collect();
        let points: [_; 16] = points.try_into().unwrap_or_else(|_| unreachable!());

        FiringXyzDual16 {
            time: firing_time,
            azimuth_count: block_strongest.azimuth_count,
            azimuth_range: azimuth_range.clone(),
            points,
        }
    }

    pub fn firing_dual_32_to_xyz(
        firing: &FiringDual32,
        distance_resolution: Length,
        lasers: &[LaserParameter; 32],
    ) -> FiringXyzDual32 {
        let FiringDual32 {
            time: firing_time,
            ref azimuth_range,
            channels_strongest,
            channels_last,
            block_strongest,
            ..
        } = *firing;

        let channel_times =
            iter::successors(Some(firing_time), |&prev| Some(prev + CHANNEL_PERIOD))
                .flat_map(|time| [time, time]);

        let points: Vec<_> = izip!(
            0..,
            channel_times,
            channels_strongest,
            channels_last,
            lasers
        )
        .map(
            move |(laser_id, channel_time, channel_strongest, channel_last, laser)| {
                // let timestamp = lower_timestamp + CHANNEL_PERIOD.mul_f64((channel_idx / 2) as f64);

                let ratio = (channel_time - firing_time).div_duration(FIRING_PERIOD);
                let LaserParameter {
                    elevation,
                    azimuth_offset,
                    vertical_offset,
                    horizontal_offset,
                } = *laser;

                // clockwise angle with origin points to front of sensor
                let azimuth = {
                    let azimuth = azimuth_range.start
                        + ((azimuth_range.end - azimuth_range.start) * ratio)
                        + azimuth_offset;
                    azimuth.wrap_to_2pi()
                };
                let distance_strongest = distance_resolution * channel_strongest.distance as f64;
                let distance_last = distance_resolution * channel_last.distance as f64;

                let xyz_strongest = spherical_to_xyz(
                    distance_strongest,
                    elevation,
                    azimuth,
                    vertical_offset,
                    horizontal_offset,
                );
                let xyz_last = spherical_to_xyz(
                    distance_last,
                    elevation,
                    azimuth,
                    vertical_offset,
                    horizontal_offset,
                );

                PointDual {
                    laser_id,
                    time: channel_time,
                    azimuth,
                    measurements: MeasurementDual {
                        strongest: Measurement {
                            distance: distance_strongest,
                            intensity: channel_strongest.intensity,
                            xyz: xyz_strongest,
                        },
                        last: Measurement {
                            distance: distance_last,
                            intensity: channel_last.intensity,
                            xyz: xyz_last,
                        },
                    },
                }
            },
        )
        .collect();

        let points: [_; 32] = points.try_into().unwrap_or_else(|_| unreachable!());

        FiringXyzDual32 {
            time: firing_time,
            azimuth_count: block_strongest.azimuth_count,
            azimuth_range: azimuth_range.clone(),
            points,
        }
    }

    pub fn spherical_to_xyz(
        distance: Length,
        elevation: Angle,
        azimuth: Angle,
        vertical_offset: Length,
        horizontal_offset: Length,
    ) -> [Length; 3] {
        // The origin of elevaion_angle lies on xy plane.
        // The azimuth angle starts from y-axis, rotates clockwise.

        let distance_plane = distance * elevation.cos() - vertical_offset * elevation.sin();
        let x = distance_plane * azimuth.sin() - horizontal_offset * azimuth.cos();
        let y = distance_plane * azimuth.cos() + horizontal_offset * azimuth.sin();
        let z = distance * elevation.sin() + vertical_offset * elevation.cos();
        [x, y, z]
    }
}
